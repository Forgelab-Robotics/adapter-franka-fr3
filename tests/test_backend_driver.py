from __future__ import annotations

import math
import time
import unittest

from forge_msgs import JointCommand

from robots_franka_fr3.backend import FakeBackend, _active_error_names
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

    def test_connect_does_not_command_measured_pose(self) -> None:
        time.sleep(0.08)
        self.assertEqual(self.backend.command_history, [])

    def test_sparse_command_preserves_omitted_joints(self) -> None:
        self.driver.set_command(JointCommand(name=["fr3v2_joint1"], position=[0.02]))
        time.sleep(0.03)
        self.assertAlmostEqual(self.backend.position[0], 0.02)
        self.assertEqual(tuple(self.driver._target[1:7]), tuple(HOME_POSITION[1:7]))  # noqa: SLF001

    def test_unchanged_target_is_sent_only_once(self) -> None:
        command = JointCommand(name=["fr3v2_joint1"], position=[0.02])
        self.driver.set_command(command)
        time.sleep(0.08)
        self.driver.set_command(command)
        time.sleep(0.08)
        self.assertEqual(len(self.backend.command_history), 1)

    def test_gripper_total_opening(self) -> None:
        self.driver.set_command(JointCommand(name=["gripper"], position=[0.04]))
        time.sleep(0.03)
        self.assertAlmostEqual(self.backend.gripper_width, 0.04)
        self.assertEqual(self.backend.command_history, [])
        self.assertEqual(self.backend.gripper_command_history[-1][0], "move")

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

    def test_asynchronous_motion_failure_is_reported(self) -> None:
        class PollFailureBackend(FakeBackend):
            motion_started = False

            def command_joints(self, position: object, dynamics_factor: float) -> None:
                self._require()
                self.motion_started = True

            def poll_motion(self) -> bool:
                self._require()
                if self.motion_started:
                    raise RuntimeError("simulated asynchronous motion failure")
                return False

        backend = PollFailureBackend()
        driver = FrankaFR3Driver(backend=backend, require_homing=False)
        driver.connect()
        try:
            driver.set_command(JointCommand(name=["fr3v2_joint1"], position=[0.01]))
            deadline = time.monotonic() + 0.5
            while time.monotonic() < deadline:
                try:
                    driver.get_state()
                except RuntimeError as exc:
                    self.assertIn("motion worker", str(exc))
                    break
                time.sleep(0.01)
            else:
                self.fail("asynchronous motion failure was not reported")
        finally:
            driver.disconnect()


class FrankyErrorConversionTest(unittest.TestCase):
    def test_attribute_based_errors(self) -> None:
        class Errors:
            communication_constraints_violation = False
            joint_reflex = True
            helper = "not an error flag"

        self.assertEqual(_active_error_names(Errors()), ("joint_reflex",))

    def test_iterable_errors_remain_supported(self) -> None:
        self.assertEqual(
            _active_error_names(["joint_reflex", "", "cartesian_reflex"]),
            ("joint_reflex", "cartesian_reflex"),
        )

    def test_missing_errors_are_empty(self) -> None:
        self.assertEqual(_active_error_names(None), ())


if __name__ == "__main__":
    unittest.main()
