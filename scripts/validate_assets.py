#!/usr/bin/env python3
"""验证来源 manifest、URDF 自包含性、限位 SSOT 与 MJCF 契约。"""

from __future__ import annotations

import hashlib
import json
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import yaml


ARM_JOINTS = tuple(f"fr3v2_joint{i}" for i in range(1, 8))
ACTUATORS = ARM_JOINTS + ("gripper",)
FINGER_JOINTS = ("fr3v2_finger_joint1", "fr3v2_finger_joint2")


def _assert_unique(elements: list[ET.Element], label: str) -> None:
    names = [element.get("name", "") for element in elements]
    assert all(names), f"{label} 存在空 name"
    assert len(names) == len(set(names)), f"{label} name 不唯一：{names}"


def _float_pair(raw: str) -> tuple[float, float]:
    values = tuple(float(value) for value in raw.split())
    assert len(values) == 2, f"期望两个数，实际：{raw}"
    return values  # type: ignore[return-value]


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def validate_manifest(root: Path) -> None:
    manifest_path = root / "assets/SOURCE_MANIFEST.json"
    manifest = json.loads(manifest_path.read_text())
    expected_source = manifest["source"]
    assert expected_source["url"] == "https://github.com/frankarobotics/franka_description.git"
    assert expected_source["tag"] == "2.9.0"
    assert expected_source["commit"] == "7aeeddc449edf8d62b594f9e36a81da53e7796f9"
    for relative, metadata in manifest["files"].items():
        path = root / relative
        assert path.is_file(), f"manifest 文件缺失：{relative}"
        assert path.stat().st_size == metadata["size"], f"文件大小变化：{relative}"
        assert sha256(path) == metadata["sha256"], f"文件哈希变化：{relative}"


def validate_urdf(root: Path) -> None:
    runtime_path = root / "assets/urdf/fr3v2_franka_hand.urdf"
    original_path = (
        root
        / "assets/source/franka_description/urdfs/fr3v2_franka_hand.urdf"
    )
    runtime_text = runtime_path.read_text()
    original_text = original_path.read_text()
    assert "package://" not in runtime_text
    reconstructed = runtime_text.replace(
        "../meshes/", "package://franka_description/meshes/"
    )
    assert reconstructed == original_text, "runtime URDF 存在未记录的额外修改"
    assert (
        root / "assets/urdf/fr3v2_franka_hand.srdf"
    ).read_bytes() == (
        root
        / "assets/source/franka_description/urdfs/fr3v2_franka_hand.srdf"
    ).read_bytes()

    tree = ET.parse(runtime_path).getroot()
    for mesh in tree.findall(".//mesh"):
        filename = mesh.get("filename", "")
        assert not filename.startswith("/"), f"禁止绝对 mesh 路径：{filename}"
        path = (runtime_path.parent / filename).resolve()
        assert path.is_file(), f"URDF mesh 缺失：{filename}"

    official = yaml.safe_load(
        (
            root
            / "assets/source/franka_description/robots/fr3v2/joint_limits.yaml"
        ).read_text()
    )
    joints = {joint.get("name"): joint for joint in tree.findall("joint")}
    for index, name in enumerate(ARM_JOINTS, 1):
        joint = joints[name]
        assert joint.find("axis").get("xyz") == "0 0 1"  # type: ignore[union-attr]
        limit = joint.find("limit")
        assert limit is not None
        source_limit = official[f"joint{index}"]["limit"]
        for attribute in ("lower", "upper", "velocity", "effort"):
            assert float(limit.get(attribute, "nan")) == float(source_limit[attribute])

    left = joints["fr3v2_finger_joint1"]
    right = joints["fr3v2_finger_joint2"]
    for joint in (left, right):
        limit = joint.find("limit")
        assert limit is not None
        assert float(limit.get("lower", "nan")) == 0.0
        assert float(limit.get("upper", "nan")) == 0.04
    assert right.find("mimic").get("joint") == "fr3v2_finger_joint1"  # type: ignore[union-attr]


def validate_mjcf(root: Path) -> None:
    model_path = root / "assets/mjcf/fr3v2_franka_hand.xml"
    scene_path = root / "assets/mjcf/scene.xml"
    provenance = json.loads((root / "assets/mjcf/PROVENANCE.json").read_text())
    for relative, digest in provenance["inputs"].items():
        assert sha256(root / relative) == digest, f"MJCF 输入已变化：{relative}"
    for relative, digest in provenance["outputs"].items():
        assert sha256(root / relative) == digest, f"MJCF 输出已变化：{relative}"

    scene_tree = ET.parse(scene_path).getroot()
    includes = list(scene_tree.findall("include"))
    assert len(includes) == 1, "scene.xml 必须且只能 include 一个主体 MJCF"
    include_file = includes[0].get("file", "")
    assert include_file == model_path.name, f"scene include 错误：{include_file}"
    assert not Path(include_file).is_absolute(), "scene include 禁止绝对路径"
    assert (scene_path.parent / include_file).is_file(), "scene include 文件不存在"
    assert not scene_tree.findall(".//camera"), "本阶段 scene 不应包含相机"
    floor = scene_tree.find("./worldbody/geom[@name='floor']")
    assert floor is not None
    assert floor.get("contype") == "0" and floor.get("conaffinity") == "0", (
        "资产 sweep 场景地面必须仅作视觉参照，不能阻塞合法关节范围"
    )

    source = ET.parse(model_path).getroot()
    worldbody = source.find("worldbody")
    actuators = source.find("actuator")
    equalities = source.find("equality")
    sensors = source.find("sensor")
    keyframe = source.find("keyframe")
    contact = source.find("contact")
    assert worldbody is not None
    assert actuators is not None
    assert equalities is not None
    assert sensors is not None
    assert keyframe is not None
    assert contact is not None

    physical_joints = list(worldbody.findall(".//joint"))
    actuator_items = list(actuators)
    equality_items = list(equalities)
    sensor_items = list(sensors)
    key_items = list(keyframe)
    for elements, label in (
        (list(worldbody.findall(".//body")), "body"),
        (physical_joints, "joint"),
        (list(worldbody.findall(".//geom")), "geom"),
        (list(worldbody.findall(".//site")), "site"),
        (list(source.findall("./asset/mesh")), "mesh"),
        (actuator_items, "actuator"),
        (equality_items, "equality"),
        (sensor_items, "sensor"),
        (key_items, "keyframe"),
    ):
        _assert_unique(elements, label)

    mesh_files = [mesh.get("file", "") for mesh in source.findall("./asset/mesh")]
    for mesh_file in mesh_files:
        assert mesh_file, "MJCF mesh file 为空"
        assert not Path(mesh_file).is_absolute(), f"MJCF 禁止绝对 mesh：{mesh_file}"
        assert (model_path.parent / mesh_file).is_file(), f"MJCF mesh 缺失：{mesh_file}"

    physical_joint_names = tuple(joint.get("name", "") for joint in physical_joints)
    assert physical_joint_names == ARM_JOINTS + FINGER_JOINTS
    actuator_names = tuple(item.get("name", "") for item in actuator_items)
    assert actuator_names == ACTUATORS
    for item in actuator_items:
        target_joint = item.get("joint", "")
        assert target_joint in physical_joint_names, (
            f"actuator {item.get('name')} 绑定未知 joint：{target_joint}"
        )

    official = yaml.safe_load(
        (
            root
            / "assets/source/franka_description/robots/fr3v2/joint_limits.yaml"
        ).read_text()
    )
    for index, item in enumerate(actuator_items[:7], 1):
        expected = official[f"joint{index}"]["limit"]
        assert _float_pair(item.get("ctrlrange", "")) == (
            float(expected["lower"]),
            float(expected["upper"]),
        )
        assert float(item.get("kp", "nan")) > 0.0
        assert float(item.get("kv", "nan")) > 0.0
    # J4 全负、J6 全正；防止实现把所有关节错误地假设为跨零范围。
    assert _float_pair(actuator_items[3].get("ctrlrange", ""))[1] < 0.0
    assert _float_pair(actuator_items[5].get("ctrlrange", ""))[0] > 0.0

    gripper = actuator_items[-1]
    assert gripper.get("joint") == FINGER_JOINTS[0]
    assert float(gripper.get("gear", "nan")) == 2.0
    assert _float_pair(gripper.get("ctrlrange", "")) == (0.0, 0.08)
    assert float(gripper.get("kv", "nan")) > 0.0

    assert len(equality_items) == 1
    mimic = equality_items[0]
    assert mimic.get("joint1") == FINGER_JOINTS[1]
    assert mimic.get("joint2") == FINGER_JOINTS[0]
    assert tuple(float(value) for value in mimic.get("polycoef", "").split()) == (
        0.0,
        1.0,
        0.0,
        0.0,
        0.0,
    )

    robot_bodies = [
        body
        for body in worldbody.findall(".//body")
        if body.get("name", "").startswith("fr3v2_")
    ]
    assert robot_bodies
    assert all(body.get("gravcomp") == "1" for body in robot_bodies)

    archived_srdf = ET.parse(
        root / "assets/urdf/fr3v2_franka_hand.srdf"
    ).getroot()
    expected_excludes = {
        tuple(sorted((item.get("link1", ""), item.get("link2", ""))))
        for item in archived_srdf.findall("disable_collisions")
    }
    actual_excludes = {
        tuple(sorted((item.get("body1", ""), item.get("body2", ""))))
        for item in contact.findall("exclude")
    }
    assert actual_excludes == expected_excludes, "MJCF contact exclude 与官方 SRDF 漂移"

    key_names = {item.get("name", "") for item in key_items}
    expected_keys = {
        "home",
        "gripper_open",
        "gripper_half",
        "gripper_closed",
        *(f"joint{index}_{bound}" for index in range(1, 8) for bound in ("near_min", "near_max")),
    }
    assert key_names == expected_keys
    for item in key_items:
        assert len(item.get("qpos", "").split()) == 9
        assert len(item.get("ctrl", "").split()) == 8

    try:
        import mujoco
    except ImportError as error:
        raise RuntimeError("缺少 mujoco；请先执行 uv sync --all-groups") from error

    # Runtime URDF 必须能在没有 ROS package index、没有相邻源码仓库时直接解析。
    urdf_model = mujoco.MjModel.from_xml_path(
        str(root / "assets/urdf/fr3v2_franka_hand.urdf")
    )
    urdf_joint_names = tuple(
        mujoco.mj_id2name(urdf_model, mujoco.mjtObj.mjOBJ_JOINT, index)
        for index in range(urdf_model.njnt)
    )
    assert urdf_joint_names == ARM_JOINTS + (
        "fr3v2_finger_joint1",
        "fr3v2_finger_joint2",
    )

    for compiled_path in (model_path, scene_path):
        model = mujoco.MjModel.from_xml_path(str(compiled_path))
        assert model.nq == 9, f"{compiled_path.name}: 期望 nq=9，实际 {model.nq}"
        assert model.nu == 8, f"{compiled_path.name}: 期望 nu=8，实际 {model.nu}"
        assert model.njnt == 9, (
            f"{compiled_path.name}: 期望 njnt=9，实际 {model.njnt}"
        )
        joint_names = tuple(
            mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, index)
            for index in range(model.njnt)
        )
        assert joint_names == ARM_JOINTS + FINGER_JOINTS
        compiled_actuator_names = tuple(
            mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, index)
            for index in range(model.nu)
        )
        assert compiled_actuator_names == ACTUATORS
        gripper_id = mujoco.mj_name2id(
            model, mujoco.mjtObj.mjOBJ_ACTUATOR, "gripper"
        )
        assert tuple(model.actuator_ctrlrange[gripper_id]) == (0.0, 0.08)


def main() -> int:
    root = Path(__file__).resolve().parents[1]
    checks = (
        ("source manifest", validate_manifest),
        ("URDF/SRDF/limits", validate_urdf),
        ("MJCF", validate_mjcf),
    )
    try:
        for label, check in checks:
            check(root)
            print(f"[x] {label}")
    except (AssertionError, RuntimeError, OSError, ValueError) as error:
        print(f"[ ] 校验失败：{error}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
