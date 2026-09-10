"""Forge FR3v2 + Franka Hand 驱动。"""

from __future__ import annotations

import logging
import math
import threading
import time
from collections import deque
from collections.abc import Mapping

from forge_common import get_logger
from forge_msgs import JointCommand, JointState
from forge_robot import BaseRobotDriver

from .backend import BackendState, DynamicsFactors, DynamicsLimits, FakeBackend, RobotBackend
from .contract import ACTUATOR_ORDER, ARM_JOINT_ORDER, GRIPPER_MAX_WIDTH_M, load_joint_limits

logger = get_logger(__name__)

# Franka Hand 编码器静止时的量化死区；窗口差分后低于该值的位移视为静止，
# 避免上层 stall 检测把量化噪声当作持续运动。
_GRIPPER_VELOCITY_DEADBAND_M = 5e-6


class FrankaDriverError(RuntimeError):
    """FR3 驱动的可诊断运行时错误。"""


class FrankaConnectionError(FrankaDriverError):
    pass


class FrankaStateError(FrankaDriverError):
    pass


class FrankaCommandError(ValueError, FrankaDriverError):
    pass


class FrankaTimeoutError(TimeoutError, FrankaDriverError):
    pass


def _event(level: int, event: str, **fields: object) -> None:
    logger.log(level, event, extra={"event": event, **fields})


def _exception_chain(exc: BaseException) -> list[dict[str, str]]:
    chain: list[dict[str, str]] = []
    current: BaseException | None = exc
    seen: set[int] = set()
    while current is not None and id(current) not in seen:
        seen.add(id(current))
        chain.append({"type": type(current).__name__, "message": str(current)})
        current = current.__cause__ or current.__context__
    return chain


class FrankaFR3Driver(BaseRobotDriver):
    """带 fail-closed 校验和 bounded latest-only 调度的 FR3 驱动。

    默认把 ``JointCommand.position`` 解释为相对于 connect/recover 时 fresh
    state 的偏移。Franky 最终仍接收绝对关节目标，但该转换完全在驱动内部完成。
    """

    def __init__(
        self,
        *,
        ip: str = "172.16.0.2",
        backend: RobotBackend | None = None,
        dynamics_factor: DynamicsFactors | Mapping[str, object] | float = 0.05,
        action_timeout: float = 0.5,
        state_timeout: float = 0.5,
        motion_timeout: float = 10.0,
        gripper_timeout: float = 5.0,
        homing_timeout: float = 15.0,
        worker_join_timeout: float = 2.0,
        worker_period: float = 0.02,
        min_command_interval: float = 0.05,
        position_margin_rad: float = 0.05,
        max_step_rad: float = 0.05,
        gripper_speed: float = 0.03,
        gripper_force: float = 50.0,
        gripper_velocity_window: float = 0.1,
        require_homing: bool = True,
        position_command_semantics: str = "relative",
        auto_connect: bool = False,
        limits: Mapping[str, Mapping[str, object]] | None = None,
    ) -> None:
        self.ip = ip
        self.backend = backend or FakeBackend()
        self.dynamics_factors = DynamicsFactors.coerce(dynamics_factor)
        self.dynamics_factor = self.dynamics_factors.velocity  # legacy scalar API
        self.action_timeout = self._positive("action_timeout", action_timeout)
        self.state_timeout = self._positive("state_timeout", state_timeout)
        self.motion_timeout = self._positive("motion_timeout", motion_timeout)
        self.gripper_timeout = self._positive("gripper_timeout", gripper_timeout)
        self.homing_timeout = self._positive("homing_timeout", homing_timeout)
        self.worker_join_timeout = self._positive("worker_join_timeout", worker_join_timeout)
        self.worker_period = self._positive("worker_period", worker_period)
        self.min_command_interval = self._positive("min_command_interval", min_command_interval)
        self.position_margin_rad = self._nonnegative("position_margin_rad", position_margin_rad)
        self.max_step_rad = self._positive("max_step_rad", max_step_rad)
        self.gripper_speed = self._positive("gripper_speed", gripper_speed)
        self.gripper_force = self._positive("gripper_force", gripper_force)
        self.gripper_velocity_window = self._positive(
            "gripper_velocity_window", gripper_velocity_window
        )
        self.require_homing = bool(require_homing)
        if position_command_semantics not in ("relative", "absolute"):
            raise ValueError(
                "position_command_semantics 必须是 'relative' 或 'absolute'"
            )
        self.position_command_semantics = position_command_semantics
        self.limits = dict(limits or load_joint_limits())

        self._condition = threading.Condition()
        self._dispatch_lock = threading.Lock()
        self._connected = False
        self._backend_open = False
        self._paused = False
        self._worker_stop = False
        self._worker: threading.Thread | None = None
        self._worker_error: BaseException | None = None
        self._last_state: BackendState | None = None
        self._reference: list[float] | None = None
        self._target: list[float] | None = None
        self._arm_generation = self._gripper_generation = 0
        self._arm_dispatched = self._gripper_dispatched = 0
        self._arm_target_time = self._gripper_target_time = 0.0
        self._arm_active = self._gripper_active = False
        self._arm_started = self._gripper_started = 0.0
        self._last_arm_dispatch = self._last_gripper_dispatch = 0.0
        self._last_robot_time: float | None = None
        self._source_advanced_at = 0.0
        self._gripper_max_width = GRIPPER_MAX_WIDTH_M
        # Franka Hand 的 GripperState 不反馈速度；用 (timestamp, width) 滑动
        # 窗口差分估计，供上层 stall 检测使用。
        self._gripper_width_history: deque[tuple[float, float]] = deque()
        self._dynamics_limits: DynamicsLimits | None = None
        self._validate_limit_margins()
        if auto_connect:
            self.connect()

    @staticmethod
    def _positive(name: str, value: float) -> float:
        number = float(value)
        if not math.isfinite(number) or number <= 0.0:
            raise ValueError(f"{name} 必须是有限正数")
        return number

    @staticmethod
    def _nonnegative(name: str, value: float) -> float:
        number = float(value)
        if not math.isfinite(number) or number < 0.0:
            raise ValueError(f"{name} 必须是有限非负数")
        return number

    def _validate_limit_margins(self) -> None:
        if set(self.limits) != set(ARM_JOINT_ORDER):
            raise ValueError("关节限位必须精确覆盖 FR3 七轴")
        for name in ARM_JOINT_ORDER:
            limit = self.limits[name]["limit"]
            lower, upper = float(limit["lower"]), float(limit["upper"])  # type: ignore[index]
            if (
                not all(math.isfinite(x) for x in (lower, upper))
                or lower + self.position_margin_rad
                >= upper - self.position_margin_rad
            ):
                raise ValueError(f"{name} position margin 使安全区间为空")

    @property
    def joint_order(self) -> list[str]:
        return list(ACTUATOR_ORDER)

    @property
    def connected(self) -> bool:
        with self._condition:
            return self._connected

    def connect(self) -> None:
        with self._condition:
            if self._connected:
                return
        _event(logging.INFO, "fr3_connect_start", backend=type(self.backend).__name__)
        try:
            # 即使 backend.connect() 在部分初始化后失败，也必须进入 disconnect 清理。
            self._backend_open = True
            self.backend.connect()
            dynamics_limits = self.backend.dynamics_limits()
            self._validate_dynamics_limits(dynamics_limits)
            initial = self._read_and_validate_state(reset_source_clock=True, allow_robot_errors=True)
            if self.require_homing and not initial.errors:
                max_width = float(self.backend.home_gripper(self.homing_timeout))
                if not math.isfinite(max_width) or max_width <= 0.0:
                    raise FrankaConnectionError(f"gripper max_width 无效：{max_width}")
                initial = self._read_and_validate_state(reset_source_clock=True, allow_robot_errors=True)
            else:
                max_width = float(initial.gripper_max_width)
            now = time.monotonic()
            with self._condition:
                self._dynamics_limits = dynamics_limits
                self._gripper_max_width = min(max_width, GRIPPER_MAX_WIDTH_M)
                self._initialize_targets(initial, now)
                self._connected = True
                self._paused = False
                self._worker_stop = False
                self._worker_error = (
                    FrankaStateError(f"Franka 报告错误：{list(initial.errors)}")
                    if initial.errors else None
                )
                self._worker = None
                if self._worker_error is None:
                    self._start_worker_locked()
            _event(logging.INFO, "fr3_connect_ok", gripper_max_width_m=self._gripper_max_width,
                   dynamics_factors=self.dynamics_factors.__dict__, recovery_required=bool(initial.errors))
        except BaseException as exc:
            self._cleanup_failed_connect()
            error = (
                exc
                if isinstance(exc, FrankaDriverError)
                else FrankaConnectionError("Franka 连接初始化失败")
            )
            _event(
                logging.ERROR,
                "fr3_connect_failed",
                error_type=type(exc).__name__,
                error=str(exc),
                exception_chain=_exception_chain(exc),
            )
            if error is exc:
                raise
            raise error from exc

    def _cleanup_failed_connect(self) -> None:
        if self._backend_open:
            self._stop_backend(suppress=True)
            try:
                self.backend.disconnect()
            except BaseException:
                logger.exception("fr3_connect_cleanup_failed", extra={"event": "fr3_connect_cleanup_failed"})
        with self._condition:
            self._backend_open = self._connected = False
            self._clear_runtime_state()

    def disconnect(self) -> None:
        with self._condition:
            if not self._backend_open and not self._connected:
                return
            self._connected = False
            self._paused = True
            self._worker_stop = True
            self._condition.notify_all()
            worker = self._worker
        _event(logging.INFO, "fr3_disconnect_start")
        self._stop_backend(suppress=True)
        if worker is not None and worker is not threading.current_thread():
            worker.join(timeout=self.worker_join_timeout)
            if worker.is_alive():
                _event(logging.ERROR, "fr3_worker_join_timeout", timeout_s=self.worker_join_timeout)
        try:
            if self._backend_open:
                self.backend.disconnect()
        finally:
            with self._condition:
                self._backend_open = False
                self._clear_runtime_state()
        _event(logging.INFO, "fr3_disconnect_ok")

    def _clear_runtime_state(self) -> None:
        self._last_state = None
        self._reference = None
        self._target = None
        self._worker = None
        self._worker_error = None
        self._arm_active = self._gripper_active = False
        self._last_robot_time = None
        self._source_advanced_at = 0.0
        self._gripper_width_history.clear()

    def _initialize_targets(self, state: BackendState, now: float) -> None:
        self._last_state = state
        fresh_position = list(state.position) + [state.gripper_width]
        self._reference = list(fresh_position)
        self._target = fresh_position
        self._gripper_width_history.clear()
        self._arm_generation = self._gripper_generation = 0
        self._arm_dispatched = self._gripper_dispatched = 0
        self._arm_target_time = self._gripper_target_time = now
        self._arm_active = self._gripper_active = False
        self._last_arm_dispatch = self._last_gripper_dispatch = 0.0

    def get_state(self) -> JointState:
        self._require_connected()
        try:
            state = self._read_and_validate_state()
        except BaseException as exc:
            error = (
                exc
                if isinstance(exc, FrankaDriverError)
                else FrankaStateError("Franka 状态读取失败")
            )
            self._latch_fault(error, "fr3_state_rejected", cause=exc)
            if error is exc:
                raise
            raise error from exc
        with self._condition:
            self._last_state = state
            gripper_velocity = self._estimate_gripper_velocity_locked(state)
        return JointState(name=list(ACTUATOR_ORDER), position=list(state.position) + [state.gripper_width],
                          velocity=list(state.velocity) + [gripper_velocity],
                          effort=list(state.effort) + [0.0])

    def _estimate_gripper_velocity_locked(self, state: BackendState) -> float:
        """在 ``_condition`` 锁内用宽度滑动窗口差分估计夹爪总开口速度。

        Franka Hand（FCI）不提供夹爪速度反馈；恒报 0 会让上层 controller
        在 ``stall_timeout`` 后无条件把运动中的夹爪误判为 STALLED。窗口内
        位移不超过量化死区时返回 0.0，避免噪声被当作持续运动。
        """
        history = self._gripper_width_history
        now = state.timestamp
        history.append((now, state.gripper_width))
        while history and now - history[0][0] > self.gripper_velocity_window:
            history.popleft()
        oldest_time, oldest_width = history[0]
        elapsed = now - oldest_time
        if elapsed <= 0.0:
            return 0.0
        delta = state.gripper_width - oldest_width
        if abs(delta) <= _GRIPPER_VELOCITY_DEADBAND_M:
            return 0.0
        return delta / elapsed

    def _read_and_validate_state(self, *, reset_source_clock: bool = False,
                                 allow_robot_errors: bool = False) -> BackendState:
        started = time.monotonic()
        state = self.backend.state()
        now = time.monotonic()
        read_elapsed = now - started
        if read_elapsed > self.state_timeout:
            raise FrankaTimeoutError(f"Franka 状态读取耗时 {read_elapsed:.3f} s")
        vectors = (
            state.position + state.velocity + state.effort
            + (state.gripper_width, state.gripper_max_width)
        )
        if len(state.position) != 7 or len(state.velocity) != 7 or len(state.effort) != 7:
            raise FrankaStateError("Franka 状态向量必须为 7 轴")
        if not all(math.isfinite(value) for value in vectors):
            raise FrankaStateError("Franka 状态包含 NaN/Inf")
        age = now - state.timestamp
        if age > self.state_timeout:
            raise FrankaTimeoutError(f"Franka 状态已过期：age={age:.3f} s")
        if state.errors and not allow_robot_errors:
            raise FrankaStateError(f"Franka 报告错误：{list(state.errors)}")
        source = state.robot_time_s
        with self._condition:
            if reset_source_clock:
                self._last_robot_time = source
                self._source_advanced_at = now
            elif source is not None:
                if self._last_robot_time is None:
                    self._last_robot_time, self._source_advanced_at = source, now
                elif source < self._last_robot_time:
                    raise FrankaStateError(f"RobotState.time 倒退：{source} < {self._last_robot_time}")
                elif source > self._last_robot_time:
                    self._last_robot_time, self._source_advanced_at = source, now
                elif now - self._source_advanced_at > self.state_timeout:
                    raise FrankaTimeoutError("RobotState.time 持续未前进")
        return state

    @staticmethod
    def _validate_dynamics_limits(limits: DynamicsLimits) -> None:
        for name, values in (
            ("velocity", limits.velocity),
            ("acceleration", limits.acceleration),
            ("jerk", limits.jerk),
        ):
            if len(values) != 7 or not all(math.isfinite(value) and value > 0.0 for value in values):
                raise FrankaConnectionError(f"Franky {name} limit 无效")

    def set_command(self, command: JointCommand) -> None:
        self._require_connected()
        try:
            self._validate_command_shape(command)
            self.get_state()  # 每个 action 必须建立在 fresh、无错误状态上。
            now = time.monotonic()
            coalesced_arm_generation: int | None = None
            coalesced_gripper_generation: int | None = None
            with self._condition:
                self._raise_worker_error_locked()
                assert self._target is not None
                assert self._reference is not None
                target = list(self._target)
                by_name = dict(zip(command.name, command.position, strict=True))
                arm_touched = any(name in ARM_JOINT_ORDER for name in command.name)
                gripper_touched = "gripper" in by_name
                for index, name in enumerate(ACTUATOR_ORDER):
                    if name not in by_name:
                        continue
                    requested = float(by_name[name])
                    value = (
                        self._reference[index] + requested
                        if self.position_command_semantics == "relative"
                        else requested
                    )
                    if name == "gripper":
                        if not 0.0 <= value <= self._gripper_max_width:
                            raise FrankaCommandError(
                                "gripper 目标换算后超出 "
                                f"[0, {self._gripper_max_width}] m：{value}"
                            )
                    else:
                        limit = self.limits[name]["limit"]
                        lower = float(limit["lower"]) + self.position_margin_rad  # type: ignore[index]
                        upper = float(limit["upper"]) - self.position_margin_rad  # type: ignore[index]
                        if not lower <= value <= upper:
                            raise FrankaCommandError(f"{name} 超出安全区间 [{lower}, {upper}] rad")
                        if abs(value - target[index]) > self.max_step_rad:
                            raise FrankaCommandError(f"{name} 单周期变化超过 {self.max_step_rad} rad")
                    target[index] = value
                was_paused = self._paused
                self._paused = False
                if arm_touched:
                    self._arm_target_time = now
                    if tuple(target[:7]) != tuple(self._target[:7]) or was_paused:
                        self._arm_generation += 1
                        if self._arm_active:
                            coalesced_arm_generation = self._arm_generation
                if gripper_touched:
                    self._gripper_target_time = now
                    if target[7] != self._target[7] or was_paused:
                        self._gripper_generation += 1
                        if self._gripper_active:
                            coalesced_gripper_generation = self._gripper_generation
                self._target = target
                self._condition.notify_all()
            _event(
                logging.DEBUG,
                "fr3_action_accepted",
                joints=list(command.name),
                position_command_semantics=self.position_command_semantics,
            )
            if coalesced_arm_generation is not None:
                _event(
                    logging.DEBUG, "fr3_arm_coalesced",
                    generation=coalesced_arm_generation,
                )
            if coalesced_gripper_generation is not None:
                _event(
                    logging.DEBUG, "fr3_gripper_coalesced",
                    generation=coalesced_gripper_generation,
                )
        except BaseException as exc:
            if isinstance(exc, (FrankaStateError, FrankaTimeoutError)) and self._worker_error is exc:
                raise
            error = exc if isinstance(exc, FrankaDriverError) else FrankaCommandError(str(exc))
            self._latch_fault(error, "fr3_action_rejected", cause=exc, joints=list(command.name))
            if error is exc:
                raise
            raise error from exc

    @staticmethod
    def _validate_command_shape(command: JointCommand) -> None:
        all_values = command.position + command.velocity + command.effort + command.kp + command.kd
        if not all(math.isfinite(float(value)) for value in all_values):
            raise FrankaCommandError("JointCommand 包含 NaN/Inf")
        if command.mode != "position":
            raise FrankaCommandError(f"FR3 驱动只接受 position，收到 {command.mode!r}")
        if command.velocity or command.effort or command.kp or command.kd:
            raise FrankaCommandError("position mode 不接受 velocity/effort/kp/kd 字段")
        if len(command.name) != len(command.position):
            raise FrankaCommandError("JointCommand name 与 position 长度不一致")
        if len(set(command.name)) != len(command.name):
            raise FrankaCommandError("JointCommand 包含重复关节")
        unknown = sorted(set(command.name) - set(ACTUATOR_ORDER))
        if unknown:
            raise FrankaCommandError(f"未知执行器：{unknown}")

    def stop(self) -> None:
        with self._condition:
            if not self._backend_open:
                return
            self._paused = True
            self._arm_dispatched = self._arm_generation
            self._gripper_dispatched = self._gripper_generation
            self._arm_active = self._gripper_active = False
            self._condition.notify_all()
        self._stop_backend(suppress=False)
        _event(logging.INFO, "fr3_stop")

    read_state = get_state
    send_action = set_command
    emergency_stop = stop

    def recover(self) -> bool:
        with self._condition:
            if not self._connected:
                raise FrankaConnectionError("FR3 驱动未连接")
            old_worker = self._worker
            self._worker_stop = True
            self._condition.notify_all()
        self._stop_backend(suppress=True)
        if old_worker is not None and old_worker is not threading.current_thread():
            old_worker.join(self.worker_join_timeout)
        _event(logging.INFO, "fr3_recover_start")
        try:
            if not self.backend.recover():
                raise FrankaDriverError("Franky automatic error recovery 返回 false")
            state = self._read_and_validate_state(reset_source_clock=True)
            if self.require_homing:
                max_width = float(self.backend.home_gripper(self.homing_timeout))
                if not math.isfinite(max_width) or max_width <= 0.0:
                    raise FrankaDriverError(f"gripper max_width 无效：{max_width}")
                self._gripper_max_width = min(max_width, GRIPPER_MAX_WIDTH_M)
                state = self._read_and_validate_state(reset_source_clock=True)
            now = time.monotonic()
            with self._condition:
                self._initialize_targets(state, now)
                self._worker_error = None
                self._paused = False
                self._worker_stop = False
                self._start_worker_locked()
            _event(logging.INFO, "fr3_recover_ok")
            return True
        except BaseException as exc:
            error = exc if isinstance(exc, FrankaDriverError) else FrankaDriverError("Franka recovery 失败")
            self._latch_fault(error, "fr3_recover_failed", cause=exc)
            if error is exc:
                raise
            raise error from exc

    def grasp(self, width: float, *, force: float | None = None, speed: float | None = None) -> bool:
        self._require_connected()
        actual_speed = self.gripper_speed if speed is None else self._positive("speed", speed)
        actual_force = self.gripper_force if force is None else self._positive("force", force)
        if not math.isfinite(width) or not 0.0 <= width <= self._gripper_max_width:
            error = FrankaCommandError(f"gripper 超出 [0, {self._gripper_max_width}] m")
            self._latch_fault(error, "fr3_grasp_rejected")
            raise error
        try:
            return self.backend.grasp(width, actual_speed, actual_force)
        except BaseException as exc:
            error = FrankaDriverError("Franka Hand grasp 失败")
            self._latch_fault(error, "fr3_grasp_failed", cause=exc)
            raise error from exc

    def _require_connected(self) -> None:
        with self._condition:
            if not self._connected:
                raise FrankaConnectionError("FR3 驱动未连接")
            self._raise_worker_error_locked()

    def _start_worker_locked(self) -> None:
        self._worker_stop = False
        self._worker = threading.Thread(
            target=self._worker_loop, name="fr3-motion-worker", daemon=True
        )
        self._worker.start()

    def _raise_worker_error_locked(self) -> None:
        if self._worker_error is not None:
            raise FrankaDriverError("FR3 motion worker 已失败") from self._worker_error

    def _latch_fault(
        self,
        error: BaseException,
        event: str,
        *,
        cause: BaseException | None = None,
        **fields: object,
    ) -> None:
        with self._condition:
            if self._worker_error is None:
                self._worker_error = error
            self._paused = True
            self._arm_dispatched = self._arm_generation
            self._gripper_dispatched = self._gripper_generation
            self._arm_active = self._gripper_active = False
            self._condition.notify_all()
        self._stop_backend(suppress=True)
        actual = cause or error
        _event(
            logging.ERROR,
            event,
            error_type=type(actual).__name__,
            error=str(actual),
            exception_chain=_exception_chain(actual),
            **fields,
        )

    def _stop_backend(self, *, suppress: bool) -> None:
        errors: list[BaseException] = []
        with self._dispatch_lock:
            for operation in (self.backend.stop, self.backend.stop_gripper):
                try:
                    operation()
                except BaseException as exc:
                    errors.append(exc)
                    logger.exception(
                        "fr3_stop_failed", extra={"event": "fr3_stop_failed"}
                    )
        if errors and not suppress:
            raise FrankaDriverError("Franka stop 未完全成功") from errors[0]

    def _worker_loop(self) -> None:
        while True:
            with self._condition:
                self._condition.wait(timeout=self.worker_period)
                if self._worker_stop:
                    return
                if self._paused or self._target is None:
                    continue
                snapshot = (tuple(self._target), self._arm_generation, self._gripper_generation,
                            self._arm_target_time, self._gripper_target_time)
            try:
                self._worker_iteration(*snapshot)
            except BaseException as exc:
                error = (
                    exc
                    if isinstance(exc, FrankaDriverError)
                    else FrankaDriverError("Franka worker SDK 调用失败")
                )
                self._latch_fault(error, "fr3_worker_failed", cause=exc)
                return

    def _worker_iteration(self, target: tuple[float, ...], arm_generation: int,
                          gripper_generation: int, arm_target_time: float,
                          gripper_target_time: float) -> None:
        now = time.monotonic()
        with self._condition:
            if self._paused or self._worker_stop:
                return
            arm_active = self._arm_active
            gripper_active = self._gripper_active
            arm_started = self._arm_started
            gripper_started = self._gripper_started
        if arm_active:
            if now - arm_target_time > self.action_timeout:
                raise FrankaTimeoutError("机械臂 action watchdog 超时")
            if now - arm_started > self.motion_timeout:
                raise FrankaTimeoutError("机械臂 motion 超时")
            arm_active = self.backend.poll_motion()
        if gripper_active:
            if now - gripper_target_time > self.action_timeout:
                raise FrankaTimeoutError("夹爪 action watchdog 超时")
            if now - gripper_started > self.gripper_timeout:
                raise FrankaTimeoutError("夹爪 motion 超时")
            gripper_active = self.backend.poll_gripper()

        # dispatch_lock 把 stop/fault 与 SDK 提交串行化；condition 不跨 backend
        # 调用持有，避免 state() 的 backend→condition 锁序发生反转。
        with self._dispatch_lock:
            with self._condition:
                if self._paused or self._worker_stop:
                    return
                self._arm_active = arm_active
                self._gripper_active = gripper_active
                dispatch_arm = (
                    not self._arm_active
                    and arm_generation != self._arm_dispatched
                    and now - self._last_arm_dispatch >= self.min_command_interval
                )
                dispatch_gripper = (
                    not self._gripper_active
                    and gripper_generation != self._gripper_dispatched
                    and now - self._last_gripper_dispatch
                    >= self.min_command_interval
                )
                if dispatch_arm and now - arm_target_time > self.action_timeout:
                    raise FrankaTimeoutError("待执行机械臂 action 已过期")
                if (
                    dispatch_gripper
                    and now - gripper_target_time > self.action_timeout
                ):
                    raise FrankaTimeoutError("待执行夹爪 action 已过期")
            if dispatch_arm:
                self.backend.command_joints(target[:7], self.dynamics_factors)
                with self._condition:
                    self._arm_dispatched = arm_generation
                    self._arm_active = True
                    self._arm_started = self._last_arm_dispatch = now
                _event(logging.DEBUG, "fr3_arm_dispatched", generation=arm_generation)
            if dispatch_gripper:
                self.backend.command_gripper(
                    target[7], self.gripper_speed, self.gripper_force
                )
                with self._condition:
                    self._gripper_dispatched = gripper_generation
                    self._gripper_active = True
                    self._gripper_started = self._last_gripper_dispatch = now
                _event(
                    logging.DEBUG, "fr3_gripper_dispatched",
                    generation=gripper_generation,
                )
