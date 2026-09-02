"""构型契约入口；数值关节限位始终从归档官方 YAML 读取。"""

from __future__ import annotations

from pathlib import Path

import yaml


ARM_JOINT_ORDER = tuple(f"fr3v2_joint{index}" for index in range(1, 8))
ACTUATOR_ORDER = ARM_JOINT_ORDER + ("gripper",)
GRIPPER_MAX_WIDTH_M = 0.08

# 项目安全基准姿态，尚待现场小速度确认；不是官方运动学零位。
HOME_POSITION = (
    0.0,
    -0.7853981633974483,
    0.0,
    -2.356194490192345,
    0.0,
    1.5707963267948966,
    0.7853981633974483,
    GRIPPER_MAX_WIDTH_M,
)
RESET_POSITION = HOME_POSITION


def repository_root() -> Path:
    """返回源码/editable 安装布局中的机器人仓库根目录。"""
    return Path(__file__).resolve().parents[2]


def joint_limits_path(root: Path | None = None) -> Path:
    base = root or repository_root()
    archived = (
        base
        / "assets/source/franka_description/robots/fr3v2/joint_limits.yaml"
    )
    if archived.is_file():
        return archived
    packaged = Path(__file__).resolve().parent / "data/fr3v2_joint_limits.yaml"
    if packaged.is_file():
        return packaged
    raise FileNotFoundError("找不到归档或包内生成的 FR3v2 joint_limits.yaml")


def load_joint_limits(root: Path | None = None) -> dict[str, dict[str, object]]:
    """读取上游 SSOT，并把 joint1..7 映射为 canonical 逻辑名称。"""
    raw = yaml.safe_load(joint_limits_path(root).read_text())
    return {
        f"fr3v2_joint{index}": raw[f"joint{index}"]
        for index in range(1, 8)
    }
