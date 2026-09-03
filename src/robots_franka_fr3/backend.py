"""Franky 与无硬件 fake 后端；导入模块本身不会加载 Franky wheel。"""

from __future__ import annotations

import math
import os
import platform
import resource
import threading
import time
from dataclasses import dataclass, field
from typing import Any, Mapping, Protocol, Sequence


@dataclass(frozen=True)
class DynamicsFactors:
    """Franky 轨迹速度、加速度和 jerk 的独立安全比例。"""

    velocity: float = 0.05
    acceleration: float = 0.05
    jerk: float = 0.05

    @classmethod
    def coerce(
        cls, value: DynamicsFactors | Mapping[str, object] | float
    ) -> DynamicsFactors:
        if isinstance(value, cls):
            result = value
        elif isinstance(value, Mapping):
            result = cls(
                *(float(value.get(key, 0.05)) for key in (
                    "velocity", "acceleration", "jerk"
                ))
            )
        else:
            scalar = float(value)
            result = cls(scalar, scalar, scalar)
        for name, factor in (
            ("velocity", result.velocity),
            ("acceleration", result.acceleration),
            ("jerk", result.jerk),
        ):
            if not math.isfinite(factor) or not 0.0 < factor <= 1.0:
                raise ValueError(f"{name} dynamics factor 必须位于 (0, 1]")
        return result


@dataclass(frozen=True)
class DynamicsLimits:
    velocity: tuple[float, ...]
    acceleration: tuple[float, ...]
    jerk: tuple[float, ...]


@dataclass(frozen=True)
class BackendState:
    position: tuple[float, ...]
    velocity: tuple[float, ...]
    effort: tuple[float, ...]
    gripper_width: float
    gripper_max_width: float
    timestamp: float
    errors: tuple[str, ...] = ()
    robot_time_s: float | None = None
    gripper_time_s: float | None = None


class RobotBackend(Protocol):
    def connect(self) -> None: ...
    def disconnect(self) -> None: ...
    def state(self) -> BackendState: ...
    def diagnostics(self) -> dict[str, object]: ...
    def dynamics_limits(self) -> DynamicsLimits: ...
    def command_joints(
        self,
        position: Sequence[float],
        dynamics_factor: DynamicsFactors | Mapping[str, object] | float,
    ) -> None: ...
    def poll_motion(self) -> bool: ...
    def stop(self) -> None: ...
    def recover(self) -> bool: ...
    def home_gripper(self, timeout: float = 15.0) -> float: ...
    def command_gripper(
        self, width: float, speed: float, force: float | None = None
    ) -> None: ...
    def poll_gripper(self) -> bool: ...
    def move_gripper(self, width: float, speed: float, force: float | None = None) -> bool: ...
    def grasp(self, width: float, speed: float, force: float) -> bool: ...
    def stop_gripper(self) -> None: ...


def _tuple(value: Any, size: int) -> tuple[float, ...]:
    values = tuple(float(x) for x in value)
    if len(values) != size:
        raise RuntimeError(f"Franka 状态向量长度错误：期望 {size}，实际 {len(values)}")
    return values


def _active_error_names(errors: Any) -> tuple[str, ...]:
    if errors is None:
        return ()
    try:
        return tuple(str(error) for error in errors if error)
    except TypeError:
        pass
    return tuple(
        name
        for name in dir(errors)
        if not name.startswith("_")
        and isinstance(getattr(errors, name, None), bool)
        and getattr(errors, name)
    )


def _duration_seconds(value: Any) -> float | None:
    if value is None:
        return None
    to_sec = getattr(value, "to_sec", None)
    return float(to_sec()) if to_sec is not None else None


class FrankyBackend:
    """对锁定 Franky 2.0 API 的线程安全封装。"""

    def __init__(
        self,
        ip: str,
        *,
        dynamics_factor: DynamicsFactors | Mapping[str, object] | float = 0.05,
        expected_gripper_server_version: int | None = None,
        check_realtime: bool = True,
    ) -> None:
        self.ip = ip
        self.dynamics_factors = DynamicsFactors.coerce(dynamics_factor)
        self.dynamics_factor = self.dynamics_factors.velocity  # legacy inspection
        self.robot: Any | None = None
        self.gripper: Any | None = None
        self.expected_gripper_server_version = expected_gripper_server_version
        self.check_realtime = check_realtime
        self._robot_lock = threading.RLock()
        self._gripper_lock = threading.RLock()
        self._gripper_future: Any | None = None

    def connect(self) -> None:
        if self.check_realtime:
            release = platform.release().lower()
            if "rt" not in release and os.environ.get("FR3_ALLOW_NON_RT") != "1":
                raise RuntimeError(
                    "真机 Franky 控制要求 PREEMPT_RT；"
                    "仅测试可设置 FR3_ALLOW_NON_RT=1"
                )
            soft, _ = resource.getrlimit(resource.RLIMIT_RTPRIO)
            if soft == 0 and os.environ.get("FR3_ALLOW_NON_RT") != "1":
                raise RuntimeError("当前用户没有 rtprio 权限；请配置实时权限")
        try:
            import franky  # type: ignore
        except ImportError as exc:  # pragma: no cover
            raise RuntimeError("未安装 Franky；请按 pyproject.toml 安装 franky-control") from exc
        try:
            robot = franky.Robot(self.ip)
            gripper = franky.Gripper(self.ip)
            if (
                self.expected_gripper_server_version is not None
                and int(gripper.server_version)
                != self.expected_gripper_server_version
            ):
                raise RuntimeError(
                    "Gripper Server 版本不匹配："
                    f"期望 {self.expected_gripper_server_version}，"
                    f"实际 {int(gripper.server_version)}"
                )
            factors = self.dynamics_factors
            robot.relative_dynamics_factor = franky.RelativeDynamicsFactor(
                factors.velocity, factors.acceleration, factors.jerk
            )
        except BaseException:
            self.robot = self.gripper = None
            raise
        self.robot, self.gripper = robot, gripper
        self._gripper_future = None

    def _require(self) -> tuple[Any, Any]:
        if self.robot is None or self.gripper is None:
            raise RuntimeError("Franka backend 尚未连接")
        return self.robot, self.gripper

    def state(self) -> BackendState:
        robot, gripper = self._require()
        with self._robot_lock:
            rs = robot.state
            q_raw, dq_raw = getattr(rs, "q", None), getattr(rs, "dq", None)
            q = _tuple(robot.current_joint_positions if q_raw is None else q_raw, 7)
            dq = _tuple(robot.current_joint_velocities if dq_raw is None else dq_raw, 7)
            tau = _tuple(getattr(rs, "tau_J", (0.0,) * 7), 7)
            errors = _active_error_names(getattr(rs, "current_errors", ()))
            robot_time = _duration_seconds(getattr(rs, "time", None))
        with self._gripper_lock:
            gs = getattr(gripper, "state", None)
            width = float(gripper.width if gs is None or getattr(gs, "width", None) is None else gs.width)
            max_width_raw = (
                gripper.max_width
                if gs is None or getattr(gs, "max_width", None) is None
                else gs.max_width
            )
            max_width = float(max_width_raw)
            gripper_time = _duration_seconds(getattr(gs, "time", None)) if gs is not None else None
        return BackendState(
            q, dq, tau, width, max_width, time.monotonic(), errors,
            robot_time, gripper_time,
        )

    def diagnostics(self) -> dict[str, object]:
        robot, gripper = self._require()
        with self._robot_lock:
            rs = robot.state
            result = {"robot_mode": str(getattr(rs, "robot_mode", "unknown")),
                      "has_errors": bool(getattr(robot, "has_errors", False)),
                      "is_in_control": bool(getattr(robot, "is_in_control", False)),
                      "current_errors": list(_active_error_names(getattr(rs, "current_errors", ()))),
                      "last_motion_errors": list(_active_error_names(getattr(rs, "last_motion_errors", ())))}
        with self._gripper_lock:
            gs = gripper.state
            result.update(gripper_server_version=int(gripper.server_version),
                          gripper_is_grasped=bool(getattr(gs, "is_grasped", False)),
                          gripper_temperature_c=int(getattr(gs, "temperature", 0)))
        return result

    def dynamics_limits(self) -> DynamicsLimits:
        robot, _ = self._require()
        with self._robot_lock:
            return DynamicsLimits(_tuple(robot.joint_velocity_limit.max, 7),
                                  _tuple(robot.joint_acceleration_limit.max, 7),
                                  _tuple(robot.joint_jerk_limit.max, 7))

    def command_joints(
        self,
        position: Sequence[float],
        dynamics_factor: DynamicsFactors | Mapping[str, object] | float,
    ) -> None:
        robot, _ = self._require()
        import franky  # type: ignore
        factors = DynamicsFactors.coerce(dynamics_factor)
        with self._robot_lock:
            robot.relative_dynamics_factor = franky.RelativeDynamicsFactor(
                factors.velocity, factors.acceleration, factors.jerk
            )
            target = franky.JointState([float(x) for x in position])
            robot.move(franky.JointMotion(target), asynchronous=True)

    def poll_motion(self) -> bool:
        robot, _ = self._require()
        with self._robot_lock:
            return bool(robot.poll_motion())

    def stop(self) -> None:
        if self.robot is not None:
            with self._robot_lock:
                self.robot.stop()

    def recover(self) -> bool:
        robot, _ = self._require()
        with self._robot_lock:
            return bool(robot.recover_from_errors())

    @staticmethod
    def _wait_future(future: Any, timeout: float, operation: str) -> bool:
        if not future.wait(timeout):
            raise TimeoutError(f"Franka Hand {operation} 超时（{timeout} s）")
        return bool(future.get())

    def home_gripper(self, timeout: float = 15.0) -> float:
        _, gripper = self._require()
        with self._gripper_lock:
            future = gripper.homing_async()
        if not self._wait_future(future, timeout, "homing"):
            raise RuntimeError("Franka Hand homing 返回 false")
        with self._gripper_lock:
            return float(gripper.max_width)

    def command_gripper(self, width: float, speed: float, force: float | None = None) -> None:
        del force
        _, gripper = self._require()
        with self._gripper_lock:
            if self._gripper_future is not None:
                raise RuntimeError("已有 Franka Hand 命令正在运行")
            self._gripper_future = gripper.move_async(width, speed)

    def poll_gripper(self) -> bool:
        self._require()
        with self._gripper_lock:
            if self._gripper_future is None:
                return False
            if not self._gripper_future.wait(0.0):
                return True
            future, self._gripper_future = self._gripper_future, None
            if not bool(future.get()):
                raise RuntimeError("Franka Hand move 返回 false")
            return False

    def move_gripper(self, width: float, speed: float, force: float | None = None) -> bool:
        del force
        _, gripper = self._require()
        with self._gripper_lock:
            return bool(gripper.move(width, speed))

    def stop_gripper(self) -> None:
        if self.gripper is not None:
            with self._gripper_lock:
                self.gripper.stop()
                self._gripper_future = None

    def grasp(self, width: float, speed: float, force: float) -> bool:
        _, gripper = self._require()
        with self._gripper_lock:
            return bool(gripper.grasp(width, speed, force))

    def disconnect(self) -> None:
        with self._robot_lock, self._gripper_lock:
            self._gripper_future = None
            self.robot = self.gripper = None


@dataclass
class FakeBackend:
    """确定性的 fake backend；所有 CI 测试只使用它或 mock Franky。"""

    position: list[float] = field(default_factory=lambda: [
        0.0, -0.7853981633974483, 0.0, -2.356194490192345,
        0.0, 1.5707963267948966, 0.7853981633974483,
    ])
    gripper_width: float = 0.08
    max_width: float = 0.08
    connected: bool = False
    fail_next: str | None = None
    command_history: list[tuple[float, ...]] = field(default_factory=list)
    dynamics_history: list[DynamicsFactors] = field(default_factory=list)
    gripper_command_history: list[tuple[str, float, float, float | None]] = field(default_factory=list)
    stop_count: int = 0
    robot_stop_count: int = 0
    gripper_stop_count: int = 0
    recover_count: int = 0
    timestamp_offset: float = 0.0
    robot_time_override: float | None = None
    robot_mode: str = "RobotMode.Idle"
    errors: tuple[str, ...] = ()
    last_motion_errors: tuple[str, ...] = ()
    grasp_result: bool = True
    gripper_temperature_c: int = 25
    gripper_is_grasped: bool = False
    motion_polls_remaining: int = 0
    gripper_polls_remaining: int = 0
    _motion_active: bool = False
    _gripper_active: bool = False

    def _fail(self) -> None:
        if self.fail_next:
            message, self.fail_next = self.fail_next, None
            raise RuntimeError(message)

    def connect(self) -> None:
        self._fail()
        self.connected = True

    def _require(self) -> None:
        if not self.connected:
            raise RuntimeError("fake backend 未连接")
        self._fail()

    def disconnect(self) -> None:
        self.connected = self._motion_active = self._gripper_active = False

    def state(self) -> BackendState:
        self._require()
        now = time.monotonic()
        robot_time = now if self.robot_time_override is None else self.robot_time_override
        return BackendState(tuple(self.position), (0.0,) * 7, (0.0,) * 7, self.gripper_width,
                            self.max_width, now - self.timestamp_offset, self.errors, robot_time, now)

    def diagnostics(self) -> dict[str, object]:
        self._require()
        return {"robot_mode": self.robot_mode, "has_errors": bool(self.errors),
                "is_in_control": self._motion_active, "current_errors": list(self.errors),
                "last_motion_errors": list(self.last_motion_errors), "gripper_server_version": 3,
                "gripper_is_grasped": self.gripper_is_grasped,
                "gripper_temperature_c": self.gripper_temperature_c}

    def dynamics_limits(self) -> DynamicsLimits:
        self._require()
        return DynamicsLimits(
            (2.62, 2.62, 2.62, 2.62, 5.26, 4.18, 5.26),
            (10.0,) * 7,
            (5000.0,) * 7,
        )

    def command_joints(
        self,
        position: Sequence[float],
        dynamics_factor: DynamicsFactors | Mapping[str, object] | float,
    ) -> None:
        self._require()
        target = tuple(float(x) for x in position)
        if len(target) != 7:
            raise ValueError("fake 关节目标必须为 7 轴")
        self.position[:] = target
        self.command_history.append(target)
        self.dynamics_history.append(DynamicsFactors.coerce(dynamics_factor))
        self._motion_active = self.motion_polls_remaining > 0

    def poll_motion(self) -> bool:
        self._require()
        if self._motion_active and self.motion_polls_remaining > 0:
            self.motion_polls_remaining -= 1
        self._motion_active = self._motion_active and self.motion_polls_remaining > 0
        return self._motion_active

    def stop(self) -> None:
        self.stop_count += 1
        self.robot_stop_count += 1
        self._motion_active = False

    def recover(self) -> bool:
        self._require()
        self.recover_count += 1
        self.errors = ()
        self.robot_mode = "RobotMode.Idle"
        return True

    def home_gripper(self, timeout: float = 15.0) -> float:
        self._require()
        if timeout <= 0.0:
            raise TimeoutError("fake gripper homing 超时")
        self.gripper_width = self.max_width
        self.gripper_command_history.append(("homing", self.max_width, 0.0, None))
        return self.max_width

    def command_gripper(self, width: float, speed: float, force: float | None = None) -> None:
        self._require()
        if self._gripper_active:
            raise RuntimeError("已有 fake gripper 命令正在运行")
        self.gripper_width = float(width)
        self.gripper_is_grasped = False
        self.gripper_command_history.append(("move", float(width), float(speed), force))
        self._gripper_active = self.gripper_polls_remaining > 0

    def poll_gripper(self) -> bool:
        self._require()
        if self._gripper_active and self.gripper_polls_remaining > 0:
            self.gripper_polls_remaining -= 1
        self._gripper_active = self._gripper_active and self.gripper_polls_remaining > 0
        return self._gripper_active

    def move_gripper(self, width: float, speed: float, force: float | None = None) -> bool:
        self._require()
        self.gripper_width = float(width)
        self.gripper_is_grasped = False
        self.gripper_command_history.append(("move", float(width), float(speed), force))
        return True

    def stop_gripper(self) -> None:
        self.stop_count += 1
        self.gripper_stop_count += 1
        self._gripper_active = False

    def grasp(self, width: float, speed: float, force: float) -> bool:
        self._require()
        self.gripper_width = float(width)
        self.gripper_is_grasped = self.grasp_result
        self.gripper_command_history.append(("grasp", float(width), float(speed), float(force)))
        return self.grasp_result
