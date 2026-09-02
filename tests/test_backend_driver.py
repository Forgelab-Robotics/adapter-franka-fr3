from __future__ import annotations

import math
import time
import unittest

from forge_msgs import JointCommand

from robots_franka_fr3.backend import FakeBackend
from robots_franka_fr3.contract import ACTUATOR_ORDER, HOME_POSITION
from robots_franka_fr3.driver import FrankaFR3Driver


class DriverTest(unittest.TestCase):
    def setUp(self) -> None:
        self.backend = FakeBackend()
        self.driver = FrankaFR3Driver(backend=self.backend, require_homing=True, max_step_rad=0.05)
        self.driver.connect()

    def tearDown(self) -> None:
        self.driver.disconnect()

    def test_state_order_and_units(self) -> None:
        state = self.driver.get_state()
        self.assertEqual(state.name, list(ACTUATOR_ORDER))
        self.assertEqual(state.position, list(HOME_POSITION))
        self.assertEqual(len(state.velocity), 8)

    def test_sparse_command_preserves_omitted_joints(self) -> None:
        self.driver.set_command(JointCommand(name=["fr3v2_joint1"], position=[0.02]))
        time.sleep(0.03)
        self.assertAlmostEqual(self.backend.position[0], 0.02)
        self.assertEqual(tuple(self.driver._target[1:7]), tuple(HOME_POSITION[1:7]))  # noqa: SLF001

    def test_gripper_total_opening(self) -> None:
        self.driver.set_command(JointCommand(name=["gripper"], position=[0.04]))
        time.sleep(0.03)
        self.assertAlmostEqual(self.backend.gripper_width, 0.04)

    def test_invalid_action_rejected(self) -> None:
        with self.assertRaises(ValueError):
            self.driver.set_command(JointCommand(name=["unknown"], position=[0.0]))
        with self.assertRaises(ValueError):
            self.driver.set_command(JointCommand(name=["gripper"], position=[math.inf]))
        with self.assertRaises(ValueError):
            self.driver.set_command(JointCommand(name=["fr3v2_joint1"], position=[0.2]))

    def test_disconnect_invalidates_state(self) -> None:
        self.driver.disconnect()
        with self.assertRaises(RuntimeError):
            self.driver.get_state()

    def test_stale_state_stops_and_is_rejected(self) -> None:
        self.backend.timestamp_offset = 1.0
        with self.assertRaises(TimeoutError):
            self.driver.get_state()
        self.assertGreaterEqual(self.backend.stop_count, 1)

    def test_worker_failure_is_reported(self) -> None:
        self.backend.fail_next = "simulated transport failure"
        with self.assertRaises(RuntimeError):
            self.driver.set_command(JointCommand(name=["fr3v2_joint1"], position=[0.01]))


if __name__ == "__main__":
    unittest.main()
