from __future__ import annotations

import os
from pathlib import Path
import unittest
import xml.etree.ElementTree as ET

import yaml


ROOT = Path(__file__).resolve().parents[1]
SKILL = ROOT / "examples/03_workflows/paos_move_arm_by_ee/skill"
ARM_JOINTS = [f"fr3v2_joint{index}" for index in range(1, 8)]


class PaosMoveArmByEeTest(unittest.TestCase):
    def test_manifest_profiles_and_locked_runtime(self) -> None:
        manifest = yaml.safe_load((SKILL / "skill.yaml").read_text())
        self.assertEqual(manifest["manifest_version"], 2)
        self.assertEqual(manifest["name"], "move-arm-by-ee-franka-fr3")
        self.assertEqual(
            manifest["required_tools"],
            [
                "motion.resolve_relative_pose",
                "motion.move_pose",
                "gripper.set_opening",
            ],
        )
        self.assertEqual(set(manifest["profiles"]), {"franka_fake", "franka_real"})
        real_environment = manifest["profiles"]["franka_real"]["required_environment"]
        self.assertIn("FRANKA_FR3_PROJECT", real_environment)
        self.assertIn("PAOS_FRANKA_REAL_ACK", real_environment)
        for lock in manifest["artifacts"]["nodes"].values():
            self.assertEqual(lock["artifact_type"], "executable_tar_gz")
            self.assertEqual(len(lock["sha256"]), 64)

    def test_urdf_chain_and_profile_contract_match(self) -> None:
        urdf = ET.parse(SKILL / "assets/urdf/fr3v2_franka_hand.urdf").getroot()
        joint_names = {joint.attrib["name"] for joint in urdf.findall("joint")}
        link_names = {link.attrib["name"] for link in urdf.findall("link")}
        self.assertTrue(set(ARM_JOINTS).issubset(joint_names))
        self.assertIn("fr3v2_link0", link_names)
        self.assertIn("fr3v2_hand_tcp", link_names)
        for profile in ("franka_fake", "franka_real"):
            directory = SKILL / "profiles" / profile
            relative = yaml.safe_load((directory / "relative_pose_policy.yaml").read_text())
            motion = yaml.safe_load((directory / "motion_server.yaml").read_text())
            controller = yaml.safe_load((directory / "controller.yaml").read_text())
            self.assertEqual(relative["joint_names"], ARM_JOINTS)
            self.assertEqual(controller["joint_names"], ARM_JOINTS)
            self.assertEqual(relative["base_frame"], "fr3v2_link0")
            self.assertEqual(relative["tip_frame"], "fr3v2_hand_tcp")
            self.assertEqual(motion["groups"]["fr3v2_arm"]["joint_names"], ARM_JOINTS)

    def test_gateway_is_state_gated_and_commissioning_bounded(self) -> None:
        for profile in ("franka_fake", "franka_real"):
            path = SKILL / "profiles" / profile / "gateway.yaml"
            gateway = yaml.safe_load(path.read_text())
            self.assertTrue(gateway["readiness"]["require_proprio_state"])
            specs = {item["tool_id"]: item for item in gateway["tools"]["specs"]}
            self.assertEqual(set(specs), {
                "motion.resolve_relative_pose", "motion.move_pose", "gripper.set_opening"
            })
            for spec in specs.values():
                self.assertEqual(spec["readiness"], ["proprio_state"])
                frame = spec["robot_frame_profile"]
                self.assertEqual(frame["base_frame"], "fr3v2_link0")
                self.assertEqual(frame["tool_frame"], "fr3v2_hand_tcp")
            resolve = specs["motion.resolve_relative_pose"]["input_schema"]["properties"]
            for axis in "xyz":
                bounds = resolve["translation_m"]["properties"][axis]
                self.assertEqual((bounds["minimum"], bounds["maximum"]), (-0.05, 0.05))
            move = specs["motion.move_pose"]["input_schema"]["properties"]
            self.assertEqual(move["velocity_scale"]["maximum"], 0.1)
            self.assertEqual(move["acceleration_scale"]["maximum"], 0.1)
            gripper = specs["gripper.set_opening"]["input_schema"]["properties"]
            self.assertEqual(gripper["opening_m"]["maximum"], 0.08)

    def test_real_profile_is_absolute_and_cancel_reaches_driver(self) -> None:
        directory = SKILL / "profiles/franka_real"
        robot = yaml.safe_load((directory / "robot.yaml").read_text())
        self.assertEqual(robot["robot"]["backend"], "franky")
        self.assertEqual(robot["control"]["position_command_semantics"], "absolute")
        self.assertFalse(robot["control"]["require_homing"])
        dataflow = yaml.safe_load((directory / "dataflow.yaml").read_text())
        nodes = {node["id"]: node for node in dataflow["nodes"]}
        self.assertEqual(
            nodes["franka"]["env"]["PAOS_FRANKA_REAL_ACK"],
            "${PAOS_FRANKA_REAL_ACK}",
        )
        self.assertEqual(
            nodes["franka"]["inputs"]["stop/arm"],
            "motion_server/trajectory_cancel",
        )
        self.assertEqual(
            nodes["franka"]["inputs"]["stop/gripper"],
            "gripper_action_policy/gripper_cancel",
        )
        gate = SKILL / "scripts/run_franka_real_driver.sh"
        self.assertTrue(os.access(gate, os.X_OK))
        self.assertIn("I_UNDERSTAND_THIS_MOVES_FR3", gate.read_text())


if __name__ == "__main__":
    unittest.main()
