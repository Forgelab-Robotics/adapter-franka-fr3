from __future__ import annotations

import unittest
from pathlib import Path

import yaml

from robots_franka_fr3.contract import ACTUATOR_ORDER, HOME_POSITION
from robots_franka_fr3.backend import DynamicsFactors, FakeBackend
from robots_franka_fr3.config import build_driver_from_config, load_config


ROOT = Path(__file__).resolve().parents[1]


class ConfigTest(unittest.TestCase):
    def test_load_config_reads_robot_yaml(self) -> None:
        config = load_config(ROOT / "config/robot.example.yaml")
        self.assertEqual(config["robot"]["backend"], "fake")
        self.assertFalse(config["control"]["allow_real_motion"])

    def test_example_matches_contract(self) -> None:
        config = yaml.safe_load((ROOT / "config/robot.example.yaml").read_text())
        self.assertEqual(
            tuple(config["contract"]["actuator_order"]), ACTUATOR_ORDER
        )
        home = config["positions"]["home"]
        self.assertEqual(
            tuple(home[name] for name in ACTUATOR_ORDER), HOME_POSITION
        )
        self.assertEqual(config["positions"]["reset"], home)
        self.assertEqual(config["robot"]["verification_status"], "pending_on_hardware")
        self.assertEqual(
            DynamicsFactors.coerce(config["safety"]["relative_dynamics_factors"]),
            DynamicsFactors(0.05, 0.05, 0.05),
        )
        self.assertEqual(config["safety"]["position_margin_rad"], 0.05)
        self.assertEqual(config["control"]["worker_period_s"], 0.02)
        self.assertEqual(config["control"]["min_command_interval_s"], 0.05)
        self.assertEqual(config["control"]["position_command_semantics"], "relative")

    def test_limit_reference_resolves(self) -> None:
        config_path = ROOT / "config/robot.example.yaml"
        config = yaml.safe_load(config_path.read_text())
        referenced = (
            config_path.parent / config["contract"]["joint_limits_file"]
        ).resolve()
        expected = (
            ROOT
            / "assets/source/franka_description/robots/fr3v2/joint_limits.yaml"
        ).resolve()
        self.assertEqual(referenced, expected)
        self.assertTrue(referenced.is_file())

    def test_legacy_scalar_config_remains_supported(self) -> None:
        driver = build_driver_from_config({"backend": "fake", "dynamics_factor": 0.1})
        self.assertIsInstance(driver.backend, FakeBackend)
        self.assertEqual(driver.dynamics_factors, DynamicsFactors(0.1, 0.1, 0.1))
        self.assertEqual(driver.position_command_semantics, "relative")

    def test_invalid_position_command_semantics_is_rejected(self) -> None:
        with self.assertRaisesRegex(ValueError, "position_command_semantics"):
            build_driver_from_config(
                {"backend": "fake", "position_command_semantics": "incremental"}
            )

    def test_real_backend_requires_explicit_permission(self) -> None:
        with self.assertRaises(PermissionError):
            build_driver_from_config({"robot": {"backend": "franky"}})


if __name__ == "__main__":
    unittest.main()
