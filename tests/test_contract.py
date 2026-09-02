from __future__ import annotations

import unittest
from pathlib import Path

from robots_franka_fr3.contract import (
    ACTUATOR_ORDER,
    GRIPPER_MAX_WIDTH_M,
    HOME_POSITION,
    load_joint_limits,
)


ROOT = Path(__file__).resolve().parents[1]


class ContractTest(unittest.TestCase):
    def test_order_and_gripper_contract(self) -> None:
        self.assertEqual(len(ACTUATOR_ORDER), 8)
        self.assertEqual(ACTUATOR_ORDER[-1], "gripper")
        self.assertEqual(GRIPPER_MAX_WIDTH_M, 0.08)

    def test_home_is_within_official_limits(self) -> None:
        limits = load_joint_limits(ROOT)
        for name, value in zip(ACTUATOR_ORDER[:7], HOME_POSITION[:7], strict=True):
            limit = limits[name]["limit"]
            self.assertLessEqual(limit["lower"], value)
            self.assertGreaterEqual(limit["upper"], value)


if __name__ == "__main__":
    unittest.main()
