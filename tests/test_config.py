from __future__ import annotations

import unittest
from pathlib import Path

import yaml

from robots_franka_fr3.contract import ACTUATOR_ORDER, HOME_POSITION


ROOT = Path(__file__).resolve().parents[1]


class ConfigTest(unittest.TestCase):
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


if __name__ == "__main__":
    unittest.main()
