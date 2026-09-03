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
    robot_time_s: float | None = None
    gripper_time_s: float | None = None


class RobotBackend(Protocol):
    def connect(self) -> None: ...
    def disconnect(self) -> None: ...
    def state(self) -> BackendState: ...
    def diagnostics(self) -> dict[str, object]: ...
    def command_joints(self, position: Sequence[float], dynamics_factor: float) -> None: ...
    def poll_motion(self) -> bool: ...
    def stop(self) -> None: ...
    def recover(self) -> bool: ...
    def home_gripper(self) -> float: ...
    def move_gripper(self, width: float, speed: float, force: float | None = None) -> bool: ...
    def grasp(self, width: float, speed: float, force: float) -> bool: ...
    def stop_gripper(self) -> None: ...


def _tuple(value: Any, size: int) -> tuple[float, ...]:
    values = tuple(float(x) for x in value)
    if len(values) != size:
        raise RuntimeError(f"Franka 状态向量长度错误：期望 {size}，实际 {len(values)}")
    return values


def _active_error_names(errors: Any) -> tuple[str, ...]:
    """Return active libfranka errors across Franky binding versions.

    Franky 2.0 exposes ``RobotState.current_errors`` as an ``Errors`` object
    whose public boolean attributes are the individual error flags.  Older
    bindings may expose an iterable instead, so retain support for that form.
    """
    if errors is None:
        return ()

    try:
        return tuple(str(error) for error in errors if error)
    except TypeError:
        pass

    active: list[str] = []
    for name in dir(errors):
        if name.startswith("_"):
            continue
        value = getattr(errors, name, None)
        if isinstance(value, bool) and value:
            active.append(name)
    return tuple(active)


def _duration_seconds(value: Any) -> float | None:
    if value is None:
        return None
    to_sec = getattr(value, "to_sec", None)
    return float(to_sec()) if to_sec is not None else None


class FrankyBackend:
    """对 Franky 2.x Python API 的最小、可测试封装。"""

    def __init__(self, ip: str, *, dynamics_factor: float = 0.05,
                 expected_gripper_server_version: int | None = None,
                 check_realtime: bool = True) -> None:
        self.ip = ip
        self.dynamics_factor = dynamics_factor
        self.robot: Any | None = None
        self.gripper: Any | None = None
        self.expected_gripper_server_version = expected_gripper_server_version
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
        if self.expected_gripper_server_version is not None:
            actual = int(self.gripper.server_version)
            if actual != self.expected_gripper_server_version:
                self.robot = None
                self.gripper = None
                raise RuntimeError(
                    "Gripper Server 版本不匹配："
                    f"期望 {self.expected_gripper_server_version}，实际 {actual}"
                )
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
        error_names = _active_error_names(errors)
        return BackendState(
            q,
            dq,
            tau,
            width,
            max_width,
            time.monotonic(),
            error_names,
            _duration_seconds(getattr(rs, "time", None)),
            _duration_seconds(getattr(gs, "time", None)) if gs is not None else None,
        )

    def diagnostics(self) -> dict[str, object]:
        robot, gripper = self._require()
        rs = robot.state
        gs = gripper.state
        return {
            "robot_mode": str(getattr(rs, "robot_mode", "unknown")),
            "has_errors": bool(getattr(robot, "has_errors", False)),
            "is_in_control": bool(getattr(robot, "is_in_control", False)),
            "current_errors": list(_active_error_names(getattr(rs, "current_errors", ()))),
            "last_motion_errors": list(_active_error_names(getattr(rs, "last_motion_errors", ()))),
            "gripper_server_version": int(gripper.server_version),
            "gripper_is_grasped": bool(getattr(gs, "is_grasped", False)),
            "gripper_temperature_c": int(getattr(gs, "temperature", 0)),
        }

    def command_joints(self, position: Sequence[float], dynamics_factor: float) -> None:
        robot, _ = self._require()
        import franky  # type: ignore

        robot.relative_dynamics_factor = dynamics_factor
        # Franky 2.x 的 JointMotion 是 JointPositionMotion 的 Python 暴露名称。
        target = franky.JointState(list(float(x) for x in position))
        robot.move(franky.JointMotion(target), asynchronous=True)

    def poll_motion(self) -> bool:
        robot, _ = self._require()
        # poll_motion also re-raises errors produced by asynchronous motions.
        return bool(robot.poll_motion())

    def stop(self) -> None:
        if self.robot is not None:
            self.robot.stop()

    def recover(self) -> bool:
        robot, _ = self._require()
        recover = getattr(robot, "recover_from_errors", None)
        if recover is None:
            raise RuntimeError("当前 Franky 版本不提供 recover_from_errors")
        return bool(recover())

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
    gripper_command_history: list[tuple[str, float, float, float | None]] = field(default_factory=list)
    stop_count: int = 0
    recover_count: int = 0
    timestamp_offset: float = 0.0
    robot_mode: str = "RobotMode.Idle"
    errors: tuple[str, ...] = ()
    last_motion_errors: tuple[str, ...] = ()
    grasp_result: bool = True
    gripper_temperature_c: int = 25
    gripper_is_grasped: bool = False

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
        source_time = time.monotonic()
        return BackendState(tuple(self.position), (0.0,) * 7, (0.0,) * 7,
                            self.gripper_width, self.max_width,
                            source_time - self.timestamp_offset, self.errors,
                            source_time, source_time)

    def diagnostics(self) -> dict[str, object]:
        self._require()
        return {
            "robot_mode": self.robot_mode,
            "has_errors": bool(self.errors),
            "is_in_control": False,
            "current_errors": list(self.errors),
            "last_motion_errors": list(self.last_motion_errors),
            "gripper_server_version": 3,
            "gripper_is_grasped": self.gripper_is_grasped,
            "gripper_temperature_c": self.gripper_temperature_c,
        }

    def command_joints(self, position: Sequence[float], dynamics_factor: float) -> None:
        self._require()
        target = tuple(float(x) for x in position)
        if len(target) != 7:
            raise ValueError("fake 关节目标必须为 7 轴")
        self.position[:] = target
        self.command_history.append(target)

    def poll_motion(self) -> bool:
        self._require()
        return False

    def stop(self) -> None:
        self.stop_count += 1

    def recover(self) -> bool:
        self._require()
        self.recover_count += 1
        self.errors = ()
        self.robot_mode = "RobotMode.Idle"
        return True

    def home_gripper(self) -> float:
        self._require()
        self.gripper_width = self.max_width
        self.gripper_command_history.append(("homing", self.max_width, 0.0, None))
        return self.max_width

    def move_gripper(self, width: float, speed: float, force: float | None = None) -> bool:
        self._require()
        self.gripper_width = float(width)
        self.gripper_is_grasped = False
        self.gripper_command_history.append(("move", float(width), float(speed), force))
        return True

    def stop_gripper(self) -> None:
        self.stop_count += 1

    def grasp(self, width: float, speed: float, force: float) -> bool:
        self._require()
        self.gripper_width = float(width)
        self.gripper_is_grasped = self.grasp_result
        self.gripper_command_history.append(("grasp", float(width), float(speed), float(force)))
        return self.grasp_result
