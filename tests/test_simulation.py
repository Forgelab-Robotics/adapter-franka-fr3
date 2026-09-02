from __future__ import annotations

import importlib.util
import math
import unittest
from pathlib import Path

from robots_franka_fr3.contract import (
    ACTUATOR_ORDER,
    GRIPPER_MAX_WIDTH_M,
    HOME_POSITION,
    load_joint_limits,
)
from robots_franka_fr3.simulation import (
    SimulationContractError,
    build_sweep_waypoints,
    proprio_from_qpos,
    validate_action,
)


ROOT = Path(__file__).resolve().parents[1]


class SimulationContractTest(unittest.TestCase):
    def setUp(self) -> None:
        self.limits = load_joint_limits(ROOT)

    def test_j4_and_j6_do_not_cross_zero(self) -> None:
        self.assertLess(self.limits["fr3v2_joint4"]["limit"]["upper"], 0.0)
        self.assertGreater(self.limits["fr3v2_joint6"]["limit"]["lower"], 0.0)

    def test_all_sweep_waypoints_are_complete_and_legal(self) -> None:
        waypoints = build_sweep_waypoints(self.limits)
        self.assertEqual(len(waypoints), 32)
        for waypoint in waypoints:
            self.assertEqual(len(waypoint.position), len(ACTUATOR_ORDER))
            self.assertEqual(
                validate_action(ACTUATOR_ORDER, waypoint.position, self.limits),
                waypoint.position,
            )
        self.assertTrue(
            any(item.name == "fr3v2_joint4_home_after_max" for item in waypoints)
        )
        self.assertTrue(
            any(item.name == "fr3v2_joint6_home_after_max" for item in waypoints)
        )

    def test_gripper_proprio_is_total_opening(self) -> None:
        proprio = proprio_from_qpos((*HOME_POSITION[:7], 0.02, 0.02))
        self.assertAlmostEqual(proprio[-1], 0.04)
        open_proprio = proprio_from_qpos((*HOME_POSITION[:7], 0.04, 0.04))
        self.assertEqual(open_proprio[-1], GRIPPER_MAX_WIDTH_M)

    def test_mimic_mismatch_is_rejected(self) -> None:
        with self.assertRaises(SimulationContractError):
            proprio_from_qpos((*HOME_POSITION[:7], 0.01, 0.02))

    def test_invalid_actions_are_rejected(self) -> None:
        cases = (
            ((*ACTUATOR_ORDER[:-1], "unknown"), HOME_POSITION),
            (tuple(reversed(ACTUATOR_ORDER)), HOME_POSITION),
            (ACTUATOR_ORDER, (*HOME_POSITION[:-1], math.nan)),
            (ACTUATOR_ORDER, (*HOME_POSITION[:-1], math.inf)),
            (ACTUATOR_ORDER, (*HOME_POSITION[:-1], 0.081)),
        )
        for names, positions in cases:
            with self.subTest(names=names, positions=positions):
                with self.assertRaises(SimulationContractError):
                    validate_action(names, positions, self.limits)


class MujocoSweepTest(unittest.TestCase):
    def test_native_sweep_and_smoke(self) -> None:
        spec = importlib.util.spec_from_file_location(
            "run_mujoco_sweep", ROOT / "scripts/run_mujoco_sweep.py"
        )
        assert spec and spec.loader
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        lines = module.run_validation(
            ROOT / "assets/mjcf/scene.xml",
            smoke_seconds=0.2,
        )
        self.assertEqual(lines[-1], "RESULT: PASS")


if __name__ == "__main__":
    unittest.main()
