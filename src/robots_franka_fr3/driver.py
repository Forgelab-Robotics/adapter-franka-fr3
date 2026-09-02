"""Forge FR3v2 + Franka Hand 驱动。"""

from __future__ import annotations

import math
import threading
import time
from collections.abc import Mapping

from forge_msgs import JointCommand, JointState
from forge_robot import BaseRobotDriver

from .backend import FakeBackend, RobotBackend
from .contract import ACTUATOR_ORDER, ARM_JOINT_ORDER, GRIPPER_MAX_WIDTH_M, load_joint_limits


class FrankaFR3Driver(BaseRobotDriver):
    """带安全校验、最新目标合并和单 worker 抢占的 FR3 驱动。"""

    def __init__(self, *, ip: str = "172.16.0.2", backend: RobotBackend | None = None,
                 dynamics_factor: float = 0.05, action_timeout: float = 0.5,
                 state_timeout: float = 0.5, max_step_rad: float = 0.05,
                 gripper_speed: float = 0.03, gripper_force: float = 50.0,
                 require_homing: bool = True, auto_connect: bool = False,
                 expected_server_version: int | None = None,
                 limits: Mapping[str, Mapping[str, object]] | None = None) -> None:
        self.ip = ip
        self.backend = backend or FakeBackend()
        self.dynamics_factor = float(dynamics_factor)
        self.action_timeout = float(action_timeout)
        self.state_timeout = float(state_timeout)
        self.max_step_rad = float(max_step_rad)
        self.gripper_speed = float(gripper_speed)
        self.gripper_force = float(gripper_force)
        self.require_homing = bool(require_homing)
        self.expected_server_version = expected_server_version
        self.limits = dict(limits or load_joint_limits())
        self._connected = False
        self._last_state = None
        self._target: list[float] | None = None
        self._target_time = 0.0
        self._condition = threading.Condition()
        self._worker: threading.Thread | None = None
        self._worker_stop = False
        self._worker_error: BaseException | None = None
        self._gripper_max_width = GRIPPER_MAX_WIDTH_M

        if auto_connect:
            self.connect()

    @property
    def joint_order(self) -> list[str]:
        return list(ACTUATOR_ORDER)

    @property
    def connected(self) -> bool:
        return self._connected

    def connect(self) -> None:
        if self._connected:
            return
        self.backend.connect()
        state = self.backend.state()
        if self.require_homing:
            self._gripper_max_width = min(float(self.backend.home_gripper()), GRIPPER_MAX_WIDTH_M)
            state = self.backend.state()
        else:
            self._gripper_max_width = min(float(state.gripper_max_width), GRIPPER_MAX_WIDTH_M)
        self._last_state = state
        self._target = list(state.position) + [state.gripper_width]
        self._target_time = time.monotonic()
        self._connected = True
        self._worker_stop = False
        self._worker_error = None
        self._worker = threading.Thread(target=self._worker_loop, name="fr3-motion-worker", daemon=True)
        self._worker.start()

    def disconnect(self) -> None:
        if not self._connected:
            return
        try:
            self.stop()
        finally:
            with self._condition:
                self._worker_stop = True
                self._condition.notify_all()
            if self._worker is not None:
                self._worker.join(timeout=2.0)
            self.backend.stop_gripper()
            self.backend.disconnect()
            self._connected = False
            self._last_state = None
            self._target = None

    def get_state(self) -> JointState:
        self._require_connected()
        state = self.backend.state()
        self._last_state = state
        if time.monotonic() - state.timestamp > self.state_timeout:
            self.stop()
            raise TimeoutError("Franka 状态已过期")
        return JointState(name=list(ACTUATOR_ORDER), position=list(state.position) + [state.gripper_width],
                          velocity=list(state.velocity) + [0.0], effort=list(state.effort) + [0.0])

    def set_command(self, command: JointCommand) -> None:
        self._require_connected()
        if command.mode != "position":
            raise ValueError(f"FR3 驱动只接受 position，收到 {command.mode!r}")
        if len(command.name) != len(command.position):
            raise ValueError("JointCommand name 与 position 长度不一致")
        if len(set(command.name)) != len(command.name):
            raise ValueError("JointCommand 包含重复关节")
        unknown = sorted(set(command.name) - set(ACTUATOR_ORDER))
        if unknown:
            raise ValueError(f"未知执行器：{unknown}")
        assert self._target is not None
        target = list(self._target)
        by_name = dict(zip(command.name, command.position, strict=True))
        current = self.get_state().position
        for index, name in enumerate(ACTUATOR_ORDER):
            if name not in by_name:
                continue
            value = float(by_name[name])
            if not math.isfinite(value):
                raise ValueError(f"{name} action 不是有限数")
            if name == "gripper":
                if not 0.0 <= value <= self._gripper_max_width:
                    raise ValueError(f"gripper 超出 [0, {self._gripper_max_width}] m")
                target[index] = value
                continue
            limit = self.limits[name]["limit"]
            lower, upper = float(limit["lower"]), float(limit["upper"])  # type: ignore[index]
            if not lower <= value <= upper:
                raise ValueError(f"{name} 超出 [{lower}, {upper}] rad")
            if abs(value - current[index]) > self.max_step_rad:
                raise ValueError(f"{name} 单周期变化超过 {self.max_step_rad} rad")
            target[index] = value
        with self._condition:
            self._target = target
            self._target_time = time.monotonic()
            self._condition.notify()

    def stop(self) -> None:
        if self._connected:
            self.backend.stop()

    # SDK/业务脚本中常用的语义别名；Forge 节点仍调用 get_state/set_command。
    read_state = get_state
    send_action = set_command
    emergency_stop = stop

    def recover(self) -> None:
        self._require_connected()
        self.backend.recover()

    def grasp(self, width: float, *, force: float | None = None, speed: float | None = None) -> bool:
        """显式夹持动作；普通 gripper position action 不会隐式触发 grasp。"""
        self._require_connected()
        if not 0.0 <= width <= self._gripper_max_width:
            raise ValueError(f"gripper 超出 [0, {self._gripper_max_width}] m")
        return self.backend.grasp(width, speed or self.gripper_speed, force or self.gripper_force)

    def _require_connected(self) -> None:
        if not self._connected:
            raise RuntimeError("FR3 驱动未连接")
        if self._worker_error is not None:
            raise RuntimeError("FR3 motion worker 已失败") from self._worker_error

    def _worker_loop(self) -> None:
        sent: tuple[float, ...] | None = None
        while True:
            with self._condition:
                self._condition.wait_for(
                    lambda: self._worker_stop
                    or (self._target is not None and tuple(self._target) != sent),
                    timeout=0.05,
                )
                if self._worker_stop:
                    return
                target = tuple(self._target or ())
                target_time = self._target_time
            if time.monotonic() - target_time > self.action_timeout:
                try:
                    self.backend.stop()
                    sent = target
                except BaseException as exc:  # pragma: no cover - defensive
                    self._worker_error = exc
                continue
            try:
                self.backend.command_joints(target[:7], self.dynamics_factor)
                sent = target
                if target[7] != (self._last_state.gripper_width if self._last_state else target[7]):
                    self.backend.move_gripper(target[7], self.gripper_speed, self.gripper_force)
                    self._last_state = self.backend.state()
            except BaseException as exc:
                self._worker_error = exc
                try:
                    self.backend.stop()
                finally:
                    return
