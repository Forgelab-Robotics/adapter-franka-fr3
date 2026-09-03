from __future__ import annotations

import importlib.util
import unittest
from pathlib import Path

import yaml

from robots_franka_fr3.contract import ACTUATOR_ORDER


ROOT = Path(__file__).resolve().parents[1]
WORKFLOW = ROOT / "examples/03_workflows/sim_basic_motion"


class SimulationWorkflowTest(unittest.TestCase):
    def setUp(self) -> None:
        self.simulator = yaml.safe_load((WORKFLOW / "simulator.yaml").read_text())
        self.task_robot = yaml.safe_load((WORKFLOW / "task_robot.yaml").read_text())
        self.dataflow = yaml.safe_load((WORKFLOW / "dataflow.yaml").read_text())

    def test_configs_share_one_action_proprio_contract(self) -> None:
        for config in (self.simulator, self.task_robot):
            self.assertEqual(
                tuple(item["name"] for item in config["joints"]),
                ACTUATOR_ORDER,
            )
            robot = config["robots"][0]
            self.assertEqual(tuple(robot["actuator_order"]), ACTUATOR_ORDER)
            self.assertEqual(tuple(robot["joint_to_actuator"]), ACTUATOR_ORDER)
            self.assertEqual(
                tuple(robot["joint_to_actuator"].values()), ACTUATOR_ORDER
            )

    def test_gripper_is_total_opening_in_meters(self) -> None:
        gripper = self.simulator["joints"][-1]
        self.assertEqual(gripper["mode"], "prismatic")
        self.assertEqual(gripper["unit"], "meters")
        self.assertEqual(gripper["state_expr"], "sum")
        self.assertEqual(
            gripper["state_from_joints"],
            ["fr3v2_finger_joint1", "fr3v2_finger_joint2"],
        )

    def test_workflow_has_no_camera_or_image_dependency(self) -> None:
        self.assertNotIn("image_forward", self.simulator)
        self.assertNotIn("image_forward", self.task_robot)
        serialized = (WORKFLOW / "dataflow.yaml").read_text().lower()
        self.assertNotIn("camera", serialized)
        self.assertNotIn("image", serialized)

    def test_dataflow_uses_runtime_environment_variable(self) -> None:
        nodes = {node["id"]: node for node in self.dataflow["nodes"]}
        self.assertEqual(set(nodes), {"mujoco", "task_robot", "test_action_source"})
        self.assertIn("${FORGE_RUNTIME_ROOT}", nodes["mujoco"]["path"])
        self.assertIn("${FORGE_RUNTIME_ROOT}", nodes["task_robot"]["path"])
        self.assertEqual(nodes["test_action_source"]["path"], "test_action_source.py")
        self.assertEqual(nodes["mujoco"]["inputs"]["action"], "task_robot/action")
        self.assertEqual(
            nodes["task_robot"]["inputs"]["proprio_state"],
            "mujoco/proprio_state",
        )

    def test_action_source_anchors_offsets_to_first_proprio(self) -> None:
        source = WORKFLOW / "test_action_source.py"
        spec = importlib.util.spec_from_file_location("fr3_sim_action_source", source)
        assert spec is not None and spec.loader is not None
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)

        reference = (0.3, -0.7, 0.1, -2.3, 0.0, 1.5, 0.7, 0.04)
        positive = module.command_for_elapsed(1.1, 1.0, reference)
        repeated = module.command_for_elapsed(1.9, 1.0, reference)
        returned = module.command_for_elapsed(2.1, 1.0, reference)
        self.assertAlmostEqual(positive.position[0], 0.35)
        self.assertEqual(positive.position, repeated.position)
        self.assertEqual(tuple(returned.position), reference)

    def test_model_path_resolves_inside_repository(self) -> None:
        path = (WORKFLOW / self.simulator["model_path"]).resolve()
        self.assertEqual(path, (ROOT / "assets/mjcf/scene.xml").resolve())
        self.assertTrue(path.is_file())


if __name__ == "__main__":
    unittest.main()
