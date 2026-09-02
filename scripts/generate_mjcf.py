#!/usr/bin/env python3
"""从归档的官方生成 URDF 整理可独立加载的 FR3v2 + Franka Hand MJCF。"""

from __future__ import annotations

import hashlib
import json
import xml.etree.ElementTree as ET
from datetime import datetime
from pathlib import Path
from zoneinfo import ZoneInfo


ARM_JOINTS = tuple(f"fr3v2_joint{i}" for i in range(1, 8))
CHAIN_JOINTS = ARM_JOINTS + (
    "fr3v2_joint8",
    "fr3v2_hand_joint",
)
FINGER_JOINTS = ("fr3v2_finger_joint1", "fr3v2_finger_joint2")
HOME_ARM = (0.0, -0.7853981633974483, 0.0, -2.356194490192345,
            0.0, 1.5707963267948966, 0.7853981633974483)
SWEEP_MARGIN_FRACTION = 0.10
SWEEP_MIN_MARGIN_RAD = 0.05


def sha256(path: Path) -> str:
    digest = hashlib.sha256(path.read_bytes()).hexdigest()
    return digest


def vector(element: ET.Element | None, attribute: str, default: str) -> str:
    return default if element is None else element.get(attribute, default)


def add_pose(target: ET.Element, origin: ET.Element | None) -> None:
    if origin is None:
        return
    target.set("pos", vector(origin, "xyz", "0 0 0"))
    target.set("euler", vector(origin, "rpy", "0 0 0"))


def add_inertial(body: ET.Element, link: ET.Element) -> None:
    source = link.find("inertial")
    if source is None:
        return
    mass = source.find("mass")
    inertia = source.find("inertia")
    if mass is None or inertia is None:
        raise RuntimeError(f"link {link.get('name')} 的 inertial 不完整")
    output = ET.SubElement(body, "inertial")
    origin = source.find("origin")
    output.set("pos", vector(origin, "xyz", "0 0 0"))
    rpy = tuple(float(value) for value in vector(origin, "rpy", "0 0 0").split())
    if any(abs(value) > 1e-12 for value in rpy):
        raise RuntimeError(
            f"link {link.get('name')} 的非零 inertial rpy 尚未实现矩阵旋转"
        )
    output.set("mass", mass.get("value", ""))
    output.set(
        "fullinertia",
        " ".join(
            inertia.get(name, "")
            for name in ("ixx", "iyy", "izz", "ixy", "ixz", "iyz")
        ),
    )


def geometry_attributes(geometry: ET.Element) -> dict[str, str]:
    mesh = geometry.find("mesh")
    if mesh is not None:
        filename = mesh.get("filename", "")
        if not filename.endswith(".stl"):
            raise RuntimeError(f"MJCF 只归档 STL collision mesh，收到：{filename}")
        return {"type": "mesh", "file": filename}
    box = geometry.find("box")
    if box is not None:
        half = [float(value) / 2.0 for value in box.get("size", "").split()]
        return {"type": "box", "size": " ".join(f"{value:.12g}" for value in half)}
    sphere = geometry.find("sphere")
    if sphere is not None:
        return {"type": "sphere", "size": sphere.get("radius", "")}
    cylinder = geometry.find("cylinder")
    if cylinder is not None:
        radius = cylinder.get("radius", "")
        half_length = float(cylinder.get("length", "")) / 2.0
        return {"type": "cylinder", "size": f"{radius} {half_length:.12g}"}
    raise RuntimeError("遇到不支持的 URDF geometry")


def add_geometries(
    body: ET.Element,
    link: ET.Element,
    asset: ET.Element,
    mesh_names: dict[str, str],
) -> None:
    link_name = link.get("name", "")
    collisions = list(link.findall("collision"))
    for index, collision in enumerate(collisions):
        geometry = collision.find("geometry")
        if geometry is None:
            continue
        attributes = geometry_attributes(geometry)
        mesh_file = attributes.pop("file", None)
        if mesh_file is not None:
            mesh_name = mesh_names.get(mesh_file)
            if mesh_name is None:
                mesh_name = f"{link_name}_collision_mesh_{len(mesh_names)}"
                ET.SubElement(asset, "mesh", name=mesh_name, file=mesh_file)
                mesh_names[mesh_file] = mesh_name
            attributes["mesh"] = mesh_name

        visual = ET.SubElement(
            body,
            "geom",
            name=f"{link_name}_visual_{index}",
            group="2",
            contype="0",
            conaffinity="0",
            material="franka_white",
            **attributes,
        )
        add_pose(visual, collision.find("origin"))

        physical = ET.SubElement(
            body,
            "geom",
            name=f"{link_name}_collision_{index}",
            group="3",
            contype="1",
            conaffinity="1",
            rgba="0.2 0.2 0.2 0.25",
            **attributes,
        )
        add_pose(physical, collision.find("origin"))


def add_joint(body: ET.Element, source: ET.Element) -> None:
    joint_type = source.get("type")
    if joint_type == "fixed":
        return
    output_type = {"revolute": "hinge", "prismatic": "slide"}.get(joint_type or "")
    if output_type is None:
        raise RuntimeError(f"不支持的 joint type：{joint_type}")
    limit = source.find("limit")
    axis = source.find("axis")
    dynamics = source.find("dynamics")
    if limit is None:
        raise RuntimeError(f"joint {source.get('name')} 缺少 limit")
    attributes = {
        "name": source.get("name", ""),
        "type": output_type,
        "axis": vector(axis, "xyz", "1 0 0"),
        "range": f"{limit.get('lower')} {limit.get('upper')}",
        "limited": "true",
        "damping": vector(dynamics, "damping", "0.0"),
    }
    ET.SubElement(body, "joint", **attributes)


def build_model(urdf_path: Path, srdf_path: Path) -> ET.ElementTree:
    urdf = ET.parse(urdf_path).getroot()
    srdf = ET.parse(srdf_path).getroot()
    links = {item.get("name", ""): item for item in urdf.findall("link")}
    joints = {item.get("name", ""): item for item in urdf.findall("joint")}

    model = ET.Element("mujoco", model="fr3v2_franka_hand")
    model.append(ET.Comment(
        " GENERATED by scripts/generate_mjcf.py from the archived official URDF; do not hand edit. "
    ))
    ET.SubElement(
        model,
        "compiler",
        angle="radian",
        autolimits="true",
        discardvisual="false",
        balanceinertia="false",
    )
    ET.SubElement(model, "option", timestep="0.002", integrator="implicitfast")
    default = ET.SubElement(model, "default")
    ET.SubElement(default, "joint", armature="0.1", frictionloss="0.01")
    ET.SubElement(default, "position", ctrllimited="true")

    asset = ET.SubElement(model, "asset")
    ET.SubElement(asset, "material", name="franka_white", rgba="0.9 0.9 0.9 1")
    mesh_names: dict[str, str] = {}

    worldbody = ET.SubElement(model, "worldbody")
    current_body = ET.SubElement(
        worldbody, "body", name="fr3v2_link0", gravcomp="1"
    )
    add_inertial(current_body, links["fr3v2_link0"])
    add_geometries(current_body, links["fr3v2_link0"], asset, mesh_names)
    ET.SubElement(current_body, "site", name="fr3v2_base", size="0.005", rgba="1 0 0 1")

    for joint_name in CHAIN_JOINTS:
        source_joint = joints[joint_name]
        child_name = source_joint.find("child").get("link")  # type: ignore[union-attr]
        child_body = ET.SubElement(
            current_body, "body", name=child_name, gravcomp="1"
        )
        add_pose(child_body, source_joint.find("origin"))
        add_joint(child_body, source_joint)
        add_inertial(child_body, links[child_name])
        add_geometries(child_body, links[child_name], asset, mesh_names)
        current_body = child_body
        if child_name == "fr3v2_link8":
            ET.SubElement(
                current_body,
                "site",
                name="fr3v2_flange",
                size="0.005",
                rgba="0 1 0 1",
            )

    hand_body = current_body
    tcp_joint = joints["fr3v2_hand_tcp_joint"]
    tcp = ET.SubElement(
        hand_body,
        "site",
        name="fr3v2_hand_tcp",
        size="0.006",
        rgba="0 0 1 1",
    )
    add_pose(tcp, tcp_joint.find("origin"))

    for joint_name in FINGER_JOINTS:
        source_joint = joints[joint_name]
        child_name = source_joint.find("child").get("link")  # type: ignore[union-attr]
        finger_body = ET.SubElement(
            hand_body, "body", name=child_name, gravcomp="1"
        )
        add_pose(finger_body, source_joint.find("origin"))
        add_joint(finger_body, source_joint)
        add_inertial(finger_body, links[child_name])
        add_geometries(finger_body, links[child_name], asset, mesh_names)

    equality = ET.SubElement(model, "equality")
    ET.SubElement(
        equality,
        "joint",
        name="franka_hand_mimic",
        joint1="fr3v2_finger_joint2",
        joint2="fr3v2_finger_joint1",
        polycoef="0 1 0 0 0",
        solref="0.002 1",
    )

    # SRDF 是官方规划碰撞矩阵来源；这些相邻/永不碰撞 body pair 若不排除，
    # collision STL 会在合法姿态产生假接触并阻塞单轴 sweep。
    contact = ET.SubElement(model, "contact")
    seen_pairs: set[tuple[str, str]] = set()
    for disabled in srdf.findall("disable_collisions"):
        body1 = disabled.get("link1", "")
        body2 = disabled.get("link2", "")
        pair = tuple(sorted((body1, body2)))
        if not body1 or not body2 or pair in seen_pairs:
            continue
        seen_pairs.add(pair)
        ET.SubElement(contact, "exclude", body1=body1, body2=body2)

    actuators = ET.SubElement(model, "actuator")
    for joint_name in ARM_JOINTS:
        source = joints[joint_name]
        limit = source.find("limit")
        assert limit is not None
        ET.SubElement(
            actuators,
            "position",
            name=joint_name,
            joint=joint_name,
            kp="450",
            kv="40",
            ctrlrange=f"{limit.get('lower')} {limit.get('upper')}",
            forcerange=f"-{limit.get('effort')} {limit.get('effort')}",
            forcelimited="true",
        )
    ET.SubElement(
        actuators,
        "position",
        name="gripper",
        joint="fr3v2_finger_joint1",
        gear="2",
        kp="100",
        kv="10",
        ctrlrange="0 0.08",
        forcerange="-100 100",
        forcelimited="true",
    )

    sensors = ET.SubElement(model, "sensor")
    for joint_name in ARM_JOINTS + FINGER_JOINTS:
        ET.SubElement(sensors, "jointpos", name=f"{joint_name}_pos", joint=joint_name)
        ET.SubElement(sensors, "jointvel", name=f"{joint_name}_vel", joint=joint_name)

    keyframe = ET.SubElement(model, "keyframe")
    def add_key(name: str, arm: tuple[float, ...], gripper: float = 0.08) -> None:
        finger = gripper / 2.0
        qpos = (*arm, finger, finger)
        ctrl = (*arm, gripper)
        ET.SubElement(
            keyframe,
            "key",
            name=name,
            qpos=" ".join(f"{value:.16g}" for value in qpos),
            ctrl=" ".join(f"{value:.16g}" for value in ctrl),
        )

    add_key("home", HOME_ARM)
    for index, joint_name in enumerate(ARM_JOINTS):
        limit = joints[joint_name].find("limit")
        assert limit is not None
        lower = float(limit.get("lower", "nan"))
        upper = float(limit.get("upper", "nan"))
        margin = max(
            SWEEP_MIN_MARGIN_RAD,
            SWEEP_MARGIN_FRACTION * (upper - lower),
        )
        near_min = list(HOME_ARM)
        near_min[index] = lower + margin
        add_key(f"joint{index + 1}_near_min", tuple(near_min))
        near_max = list(HOME_ARM)
        near_max[index] = upper - margin
        add_key(f"joint{index + 1}_near_max", tuple(near_max))
    add_key("gripper_open", HOME_ARM, 0.08)
    add_key("gripper_half", HOME_ARM, 0.04)
    add_key("gripper_closed", HOME_ARM, 0.0)
    ET.indent(model, space="  ")
    return ET.ElementTree(model)


def write_scene(path: Path) -> None:
    scene = ET.Element("mujoco", model="fr3v2_franka_hand_scene")
    ET.SubElement(scene, "include", file="fr3v2_franka_hand.xml")
    ET.SubElement(scene, "statistic", center="0 0 0.45", extent="1.2")
    visual = ET.SubElement(scene, "visual")
    ET.SubElement(visual, "headlight", ambient="0.35 0.35 0.35", diffuse="0.8 0.8 0.8")
    asset = ET.SubElement(scene, "asset")
    ET.SubElement(
        asset,
        "texture",
        name="groundplane",
        type="2d",
        builtin="checker",
        rgb1="0.2 0.3 0.4",
        rgb2="0.1 0.2 0.3",
        width="512",
        height="512",
    )
    ET.SubElement(
        asset,
        "material",
        name="groundplane",
        texture="groundplane",
        texrepeat="5 5",
        reflectance="0.2",
    )
    worldbody = ET.SubElement(scene, "worldbody")
    ET.SubElement(worldbody, "light", pos="0 0 1.5", dir="0 0 -1", directional="true")
    ET.SubElement(
        worldbody,
        "geom",
        name="floor",
        type="plane",
        size="2 2 0.1",
        material="groundplane",
        # 资产验收场景的地面仅用于坐标/尺度参照。FR3 合法单轴范围会进入
        # 安装平面下方；若把它当工作台碰撞体将错误阻塞 near-limit sweep。
        contype="0",
        conaffinity="0",
    )
    ET.indent(scene, space="  ")
    ET.ElementTree(scene).write(path, encoding="utf-8", xml_declaration=True)


def main() -> int:
    root = Path(__file__).resolve().parents[1]
    urdf = root / "assets/urdf/fr3v2_franka_hand.urdf"
    srdf = root / "assets/urdf/fr3v2_franka_hand.srdf"
    limits = root / "assets/source/franka_description/robots/fr3v2/joint_limits.yaml"
    output_dir = root / "assets/mjcf"
    output_dir.mkdir(parents=True, exist_ok=True)
    model_path = output_dir / "fr3v2_franka_hand.xml"
    scene_path = output_dir / "scene.xml"
    build_model(urdf, srdf).write(
        model_path, encoding="utf-8", xml_declaration=True
    )
    write_scene(scene_path)

    provenance = {
        "schema_version": 1,
        "generated_at": datetime.now(ZoneInfo("Asia/Shanghai")).isoformat(timespec="seconds"),
        "generator": "scripts/generate_mjcf.py",
        "inputs": {
            "assets/urdf/fr3v2_franka_hand.urdf": sha256(urdf),
            "assets/urdf/fr3v2_franka_hand.srdf": sha256(srdf),
            "assets/source/franka_description/robots/fr3v2/joint_limits.yaml": sha256(limits),
            "scripts/generate_mjcf.py": sha256(Path(__file__)),
        },
        "outputs": {
            "assets/mjcf/fr3v2_franka_hand.xml": sha256(model_path),
            "assets/mjcf/scene.xml": sha256(scene_path),
        },
        "notes": [
            "运动学、惯量、collision 与限位来自归档官方生成 URDF。",
            "contact exclude 来自同版本官方 SRDF disable_collisions。",
            "机器人 body 启用重力补偿，position actuator 使用速度反馈阻尼。",
            "MuJoCo 不直接使用 DAE visual；MJCF 以同源 STL collision 和官方 finger primitive 兼作可视化。",
            "gripper actuator 的控制量是两指总开口，范围 0..0.08 m。",
            "scene 地面仅作坐标和尺度参照，不参与碰撞。",
        ],
    }
    (output_dir / "PROVENANCE.json").write_text(
        json.dumps(provenance, ensure_ascii=False, indent=2) + "\n"
    )
    print(f"generated: {model_path.relative_to(root)}, {scene_path.relative_to(root)}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
