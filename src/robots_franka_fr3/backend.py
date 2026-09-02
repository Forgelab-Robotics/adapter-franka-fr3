"""Franky 与无硬件 fake 后端。

该模块刻意不在导入阶段 import ``franky``，因此资产检查、单元测试和
fake CLI 在没有 Franky 二进制 wheel 的机器上仍然可用。
"""

from __future__ import annotations

import time
import os
import platform
import resource
from dataclasses import dataclass, field
from typing import Any, Protocol, Sequence


@dataclass(frozen=True)
class BackendState:
    position: tuple[float, ...]
    velocity: tuple[float, ...]
    effort: tuple[float, ...]
    gripper_width: float
    gripper_max_width: float
    timestamp: float
    errors: tuple[str, ...] = ()


class RobotBackend(Protocol):
    def connect(self) -> None: ...
    def disconnect(self) -> None: ...
    def state(self) -> BackendState: ...
    def command_joints(self, position: Sequence[float], dynamics_factor: float) -> None: ...
    def stop(self) -> None: ...
    def recover(self) -> None: ...
    def home_gripper(self) -> float: ...
    def move_gripper(self, width: float, speed: float, force: float | None = None) -> bool: ...
    def grasp(self, width: float, speed: float, force: float) -> bool: ...
    def stop_gripper(self) -> None: ...


def _tuple(value: Any, size: int) -> tuple[float, ...]:
    values = tuple(float(x) for x in value)
    if len(values) != size:
        raise RuntimeError(f"Franka 状态向量长度错误：期望 {size}，实际 {len(values)}")
    return values


class FrankyBackend:
    """对 Franky 2.x Python API 的最小、可测试封装。"""

    def __init__(self, ip: str, *, dynamics_factor: float = 0.05,
                 expected_server_version: int | None = None,
                 check_realtime: bool = True) -> None:
        self.ip = ip
        self.dynamics_factor = dynamics_factor
        self.robot: Any | None = None
        self.gripper: Any | None = None
        self.expected_server_version = expected_server_version
        self.check_realtime = check_realtime

    def connect(self) -> None:
        if self.check_realtime:
            release = platform.release().lower()
            if "rt" not in release and os.environ.get("FR3_ALLOW_NON_RT") != "1":
                raise RuntimeError("真机 Franky 控制要求 PREEMPT_RT；仅测试可设置 FR3_ALLOW_NON_RT=1")
            soft, _ = resource.getrlimit(resource.RLIMIT_RTPRIO)
            if soft == 0 and os.environ.get("FR3_ALLOW_NON_RT") != "1":
                raise RuntimeError("当前用户没有 rtprio 权限；请配置实时权限")
        try:
            import franky  # type: ignore
        except ImportError as exc:  # pragma: no cover - depends on deployment
            raise RuntimeError("未安装 Franky；请按 pyproject.toml 安装 franky-control") from exc
        self.robot = franky.Robot(self.ip)
        self.gripper = franky.Gripper(self.ip)
        if self.expected_server_version is not None:
            actual = int(self.gripper.server_version)
            if actual != self.expected_server_version:
                self.robot = None
                self.gripper = None
                raise RuntimeError(f"Robot Server 版本不匹配：期望 {self.expected_server_version}，实际 {actual}")
        self.robot.relative_dynamics_factor = self.dynamics_factor

    def _require(self) -> tuple[Any, Any]:
        if self.robot is None or self.gripper is None:
            raise RuntimeError("Franka backend 尚未连接")
        return self.robot, self.gripper

    def state(self) -> BackendState:
        robot, gripper = self._require()
        rs = robot.state
        q_raw = getattr(rs, "q", None)
        dq_raw = getattr(rs, "dq", None)
        q = _tuple(robot.current_joint_positions if q_raw is None else q_raw, 7)
        dq = _tuple(robot.current_joint_velocities if dq_raw is None else dq_raw, 7)
        tau = _tuple(getattr(rs, "tau_J", (0.0,) * 7), 7)
        gs = getattr(gripper, "state", None)
        width_raw = getattr(gs, "width", None) if gs is not None else None
        max_width_raw = getattr(gs, "max_width", None) if gs is not None else None
        width = float(gripper.width if width_raw is None else width_raw)
        max_width = float(gripper.max_width if max_width_raw is None else max_width_raw)
        errors = getattr(rs, "current_errors", ())
        error_names = tuple(str(x) for x in errors if x)
        return BackendState(q, dq, tau, width, max_width, time.monotonic(), error_names)

    def command_joints(self, position: Sequence[float], dynamics_factor: float) -> None:
        robot, _ = self._require()
        import franky  # type: ignore

        robot.relative_dynamics_factor = dynamics_factor
        # Franky 2.x 的 JointMotion 是 JointPositionMotion 的 Python 暴露名称。
        target = franky.JointState(list(float(x) for x in position))
        robot.move(franky.JointMotion(target), asynchronous=True)

    def stop(self) -> None:
        if self.robot is not None:
            self.robot.stop()

    def recover(self) -> None:
        robot, _ = self._require()
        recover = getattr(robot, "recover_from_errors", None)
        if recover is None:
            raise RuntimeError("当前 Franky 版本不提供 recover_from_errors")
        recover()

    def home_gripper(self) -> float:
        _, gripper = self._require()
        gripper.homing()
        return float(gripper.max_width)

    def move_gripper(self, width: float, speed: float, force: float | None = None) -> bool:
        _, gripper = self._require()
        # open/half/close 使用 move；需要夹持确认时由上层显式调用 grasp。
        return bool(gripper.move(width, speed))

    def stop_gripper(self) -> None:
        if self.gripper is not None:
            self.gripper.stop()

    def grasp(self, width: float, speed: float, force: float) -> bool:
        _, gripper = self._require()
        return bool(gripper.grasp(width, speed, force))

    def disconnect(self) -> None:
        self.robot = None
        self.gripper = None


@dataclass
class FakeBackend:
    """确定性的 fake backend；用于所有无硬件测试。"""

    position: list[float] = field(default_factory=lambda: [0.0, -0.7853981633974483, 0.0, -2.356194490192345, 0.0, 1.5707963267948966, 0.7853981633974483])
    gripper_width: float = 0.08
    max_width: float = 0.08
    connected: bool = False
    fail_next: str | None = None
    command_history: list[tuple[float, ...]] = field(default_factory=list)
    stop_count: int = 0
    timestamp_offset: float = 0.0

    def _fail(self) -> None:
        if self.fail_next:
            message = self.fail_next
            self.fail_next = None
            raise RuntimeError(message)

    def connect(self) -> None:
        self._fail()
        self.connected = True

    def _require(self) -> None:
        if not self.connected:
            raise RuntimeError("fake backend 未连接")
        self._fail()

    def disconnect(self) -> None:
        self.connected = False

    def state(self) -> BackendState:
        self._require()
        return BackendState(tuple(self.position), (0.0,) * 7, (0.0,) * 7, self.gripper_width, self.max_width, time.monotonic() - self.timestamp_offset)

    def command_joints(self, position: Sequence[float], dynamics_factor: float) -> None:
        self._require()
        target = tuple(float(x) for x in position)
        if len(target) != 7:
            raise ValueError("fake 关节目标必须为 7 轴")
        self.position[:] = target
        self.command_history.append(target)

    def stop(self) -> None:
        self.stop_count += 1

    def recover(self) -> None:
        self._require()

    def home_gripper(self) -> float:
        self._require()
        self.gripper_width = self.max_width
        return self.max_width

    def move_gripper(self, width: float, speed: float, force: float | None = None) -> bool:
        self._require()
        self.gripper_width = float(width)
        return True

    def stop_gripper(self) -> None:
        self.stop_count += 1

    def grasp(self, width: float, speed: float, force: float) -> bool:
        self._require()
        self.gripper_width = float(width)
        return True
