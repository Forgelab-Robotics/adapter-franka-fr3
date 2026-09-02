"""FR3v2 仿真 action/proprio 契约与安全 sweep 目标。"""

from __future__ import annotations

import math
from collections.abc import Mapping, Sequence
from dataclasses import dataclass

from robots_franka_fr3.contract import (
    ACTUATOR_ORDER,
    ARM_JOINT_ORDER,
    GRIPPER_MAX_WIDTH_M,
    HOME_POSITION,
)


FINGER_MAX_WIDTH_M = GRIPPER_MAX_WIDTH_M / 2.0
SWEEP_MARGIN_FRACTION = 0.10
SWEEP_MIN_MARGIN_RAD = 0.05


class SimulationContractError(ValueError):
    """Action 或 MuJoCo 状态违反 FR3v2 仿真契约。"""


@dataclass(frozen=True)
class SweepWaypoint:
    """一个完整、合法且顺序固定的仿真目标。"""

    name: str
    position: tuple[float, ...]
    active_actuator: str


def _position_bounds(
    joint_limits: Mapping[str, Mapping[str, object]],
) -> dict[str, tuple[float, float]]:
    bounds: dict[str, tuple[float, float]] = {}
    for name in ARM_JOINT_ORDER:
        raw_limit = joint_limits[name]["limit"]
        if not isinstance(raw_limit, Mapping):
            raise SimulationContractError(f"{name} 的 limit 格式错误")
        bounds[name] = (float(raw_limit["lower"]), float(raw_limit["upper"]))
    bounds["gripper"] = (0.0, GRIPPER_MAX_WIDTH_M)
    return bounds


def validate_action(
    names: Sequence[str],
    positions: Sequence[float],
    joint_limits: Mapping[str, Mapping[str, object]],
) -> tuple[float, ...]:
    """校验完整 position action，并返回不可变的 canonical position。"""

    if tuple(names) != ACTUATOR_ORDER:
        raise SimulationContractError(
            "action 名称或顺序错误；期望 " + ", ".join(ACTUATOR_ORDER)
        )
    if len(positions) != len(ACTUATOR_ORDER):
        raise SimulationContractError(
            f"action position 长度应为 {len(ACTUATOR_ORDER)}，实际 {len(positions)}"
        )

    bounds = _position_bounds(joint_limits)
    result = tuple(float(value) for value in positions)
    for name, value in zip(ACTUATOR_ORDER, result, strict=True):
        if not math.isfinite(value):
            raise SimulationContractError(f"{name} action 不是有限数：{value!r}")
        lower, upper = bounds[name]
        if value < lower or value > upper:
            raise SimulationContractError(
                f"{name} action {value} 超出 [{lower}, {upper}]"
            )
    return result


def proprio_from_qpos(
    qpos: Sequence[float],
    *,
    mimic_tolerance: float = 1e-4,
) -> tuple[float, ...]:
    """把 7 arm + 2 finger qpos 转为 7 arm + 总开口 proprio。"""

    if len(qpos) != 9:
        raise SimulationContractError(f"MuJoCo qpos 长度应为 9，实际 {len(qpos)}")
    values = tuple(float(value) for value in qpos)
    if not all(math.isfinite(value) for value in values):
        raise SimulationContractError("MuJoCo qpos 包含 NaN 或 Inf")
    left, right = values[7], values[8]
    if not (-mimic_tolerance <= left <= FINGER_MAX_WIDTH_M + mimic_tolerance):
        raise SimulationContractError(f"左指位置越界：{left}")
    if not (-mimic_tolerance <= right <= FINGER_MAX_WIDTH_M + mimic_tolerance):
        raise SimulationContractError(f"右指位置越界：{right}")
    if abs(left - right) > mimic_tolerance:
        raise SimulationContractError(
            f"Franka Hand mimic 失配：left={left}, right={right}"
        )
    total_opening = min(
        GRIPPER_MAX_WIDTH_M,
        max(0.0, left + right),
    )
    return (*values[:7], total_opening)


def build_sweep_waypoints(
    joint_limits: Mapping[str, Mapping[str, object]],
) -> tuple[SweepWaypoint, ...]:
    """构造 near-min → home → near-max → home 及夹爪开合序列。"""

    bounds = _position_bounds(joint_limits)
    home = tuple(HOME_POSITION)
    waypoints: list[SweepWaypoint] = []
    for index, name in enumerate(ARM_JOINT_ORDER):
        lower, upper = bounds[name]
        margin = max(
            SWEEP_MIN_MARGIN_RAD,
            SWEEP_MARGIN_FRACTION * (upper - lower),
        )
        for label, value in (
            ("near_min", lower + margin),
            ("home_after_min", home[index]),
            ("near_max", upper - margin),
            ("home_after_max", home[index]),
        ):
            target = list(home)
            target[index] = value
            waypoints.append(
                SweepWaypoint(
                    name=f"{name}_{label}",
                    position=validate_action(ACTUATOR_ORDER, target, joint_limits),
                    active_actuator=name,
                )
            )
    for label, width in (
        ("open", GRIPPER_MAX_WIDTH_M),
        ("half", GRIPPER_MAX_WIDTH_M / 2.0),
        ("closed", 0.0),
        ("reopen", GRIPPER_MAX_WIDTH_M),
    ):
        target = list(home)
        target[-1] = width
        waypoints.append(
            SweepWaypoint(
                name=f"gripper_{label}",
                position=validate_action(ACTUATOR_ORDER, target, joint_limits),
                active_actuator="gripper",
            )
        )
    return tuple(waypoints)
