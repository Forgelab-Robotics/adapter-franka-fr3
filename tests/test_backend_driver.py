from __future__ import annotations

import math
import time
import unittest

from forge_msgs import JointCommand

from robots_franka_fr3.backend import DynamicsFactors, FakeBackend, _active_error_names
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
        self.driver.set_command(JointCommand(name=["gripper"], position=[-0.04]))
        time.sleep(0.03)
        self.assertAlmostEqual(self.backend.gripper_width, 0.04)
        self.assertEqual(self.backend.command_history, [])
        self.assertEqual(self.backend.gripper_command_history[-1][0], "move")

    def test_invalid_action_rejected(self) -> None:
        with self.assertLogs("robots_franka_fr3.driver", level="ERROR") as captured:
            with self.assertRaises(ValueError):
                self.driver.set_command(JointCommand(name=["unknown"], position=[0.0]))
        rejected = next(
            record for record in captured.records
            if record.event == "fr3_action_rejected"
        )
        self.assertEqual(rejected.exception_chain[0]["type"], "FrankaCommandError")
        self.assertGreater(self.backend.robot_stop_count, 0)
        self.assertGreater(self.backend.gripper_stop_count, 0)
        with self.assertRaises(RuntimeError):
            self.driver.set_command(JointCommand(name=["fr3v2_joint1"], position=[0.01]))

    def test_nonfinite_extra_fields_and_margin_are_rejected(self) -> None:
        cases = (
            JointCommand(name=["gripper"], position=[math.inf]),
            JointCommand(name=["fr3v2_joint1"], position=[0.01], velocity=[0.0]),
            JointCommand(name=["fr3v2_joint1"], position=[2.87]),
        )
        for command in cases:
            with self.subTest(command=command):
                backend = FakeBackend()
                driver = FrankaFR3Driver(backend=backend, require_homing=False)
                driver.connect()
                try:
                    with self.assertRaises(ValueError):
                        driver.set_command(command)
                    self.assertEqual(backend.command_history, [])
                    self.assertGreater(backend.robot_stop_count, 0)
                    self.assertGreater(backend.gripper_stop_count, 0)
                finally:
                    driver.disconnect()

    def test_independent_dynamics_factors_reach_backend(self) -> None:
        backend = FakeBackend()
        factors = DynamicsFactors(0.1, 0.2, 0.3)
        driver = FrankaFR3Driver(
            backend=backend, dynamics_factor=factors, require_homing=False,
            min_command_interval=0.005,
        )
        driver.connect()
        try:
            driver.set_command(JointCommand(name=["fr3v2_joint1"], position=[0.01]))
            time.sleep(0.04)
            self.assertEqual(backend.dynamics_history, [factors])
        finally:
            driver.disconnect()

    def test_relative_action_uses_fresh_connect_state_as_reference(self) -> None:
        backend = FakeBackend()
        backend.position[0] = -0.4
        backend.gripper_width = 0.06
        driver = FrankaFR3Driver(
            backend=backend, require_homing=False,
            worker_period=0.005, min_command_interval=0.005,
        )
        driver.connect()
        try:
            command = JointCommand(
                name=["fr3v2_joint1", "gripper"],
                position=[0.02, -0.01],
            )
            driver.set_command(command)
            time.sleep(0.04)
            self.assertAlmostEqual(backend.position[0], -0.38)
            self.assertAlmostEqual(backend.gripper_width, 0.05)

            # Dora 可周期性重发同一偏移；目标锚定在 connect state，不会累加。
            driver.set_command(command)
            time.sleep(0.04)
            self.assertAlmostEqual(backend.position[0], -0.38)
            self.assertEqual(len(backend.command_history), 1)
            self.assertEqual(len(backend.gripper_command_history), 1)
        finally:
            driver.disconnect()

    def test_absolute_semantics_remain_explicitly_available(self) -> None:
        backend = FakeBackend()
        backend.position[0] = -0.4
        driver = FrankaFR3Driver(
            backend=backend,
            require_homing=False,
            position_command_semantics="absolute",
            worker_period=0.005,
            min_command_interval=0.005,
        )
        driver.connect()
        try:
            driver.set_command(
                JointCommand(name=["fr3v2_joint1"], position=[-0.39])
            )
            time.sleep(0.04)
            self.assertAlmostEqual(backend.position[0], -0.39)
        finally:
            driver.disconnect()

    def test_latest_only_coalesces_while_motion_is_active(self) -> None:
        backend = FakeBackend(motion_polls_remaining=8)
        driver = FrankaFR3Driver(
            backend=backend, require_homing=False, worker_period=0.005,
            min_command_interval=0.005, action_timeout=1.0,
        )
        driver.connect()
        try:
            driver.set_command(JointCommand(name=["fr3v2_joint1"], position=[0.01]))
            time.sleep(0.012)
            for value in (0.02, 0.03, 0.04):
                driver.set_command(JointCommand(name=["fr3v2_joint1"], position=[value]))
            time.sleep(0.09)
            self.assertEqual([round(item[0], 2) for item in backend.command_history], [0.01, 0.04])
        finally:
            driver.disconnect()

    def test_frozen_source_clock_becomes_stale(self) -> None:
        backend = FakeBackend(robot_time_override=1.0)
        driver = FrankaFR3Driver(
            backend=backend, require_homing=False, state_timeout=0.02,
        )
        driver.connect()
        try:
            time.sleep(0.03)
            with self.assertRaises(TimeoutError):
                driver.get_state()
            self.assertGreater(backend.stop_count, 0)
        finally:
            driver.disconnect()

    def test_robot_source_clock_cannot_go_backwards(self) -> None:
        backend = FakeBackend(robot_time_override=2.0)
        driver = FrankaFR3Driver(backend=backend, require_homing=False)
        driver.connect()
        try:
            backend.robot_time_override = 1.0
            with self.assertRaisesRegex(RuntimeError, "倒退"):
                driver.get_state()
        finally:
            driver.disconnect()

    def test_recovery_rebuilds_cache_and_worker(self) -> None:
        backend = FakeBackend(errors=("joint_reflex",), robot_mode="RobotMode.Reflex")
        driver = FrankaFR3Driver(backend=backend, require_homing=False, min_command_interval=0.005)
        driver.connect()
        try:
            with self.assertRaises(RuntimeError):
                driver.get_state()
            self.assertTrue(driver.recover())
            driver.set_command(JointCommand(name=["fr3v2_joint1"], position=[0.01]))
            time.sleep(0.03)
            self.assertAlmostEqual(backend.position[0], 0.01)
        finally:
            driver.disconnect()

    def test_connect_failure_is_transactional(self) -> None:
        class InvalidLimitsBackend(FakeBackend):
            def dynamics_limits(self):  # type: ignore[no-untyped-def]
                result = super().dynamics_limits()
                return type(result)((0.0,) * 7, result.acceleration, result.jerk)

        backend = InvalidLimitsBackend()
        driver = FrankaFR3Driver(backend=backend, require_homing=False)
        with self.assertRaises(RuntimeError):
            driver.connect()
        self.assertFalse(driver.connected)
        self.assertFalse(backend.connected)

    def test_slow_initial_state_read_cleans_up_connection(self) -> None:
        class SlowStateBackend(FakeBackend):
            def state(self):  # type: ignore[no-untyped-def]
                time.sleep(0.02)
                return super().state()

        backend = SlowStateBackend()
        driver = FrankaFR3Driver(
            backend=backend, require_homing=False, state_timeout=0.01,
        )
        with self.assertRaises(TimeoutError):
            driver.connect()
        self.assertFalse(backend.connected)

    def test_action_watchdog_stops_active_motion(self) -> None:
        backend = FakeBackend(motion_polls_remaining=100)
        driver = FrankaFR3Driver(
            backend=backend, require_homing=False, worker_period=0.005,
            min_command_interval=0.005, action_timeout=0.03,
        )
        driver.connect()
        try:
            driver.set_command(JointCommand(name=["fr3v2_joint1"], position=[0.01]))
            time.sleep(0.07)
            with self.assertRaises(RuntimeError):
                driver.get_state()
            self.assertGreater(backend.robot_stop_count, 0)
            self.assertGreater(backend.gripper_stop_count, 0)
        finally:
            driver.disconnect()

    def test_hard_motion_timeout_cannot_be_refreshed_forever(self) -> None:
        backend = FakeBackend(motion_polls_remaining=100)
        driver = FrankaFR3Driver(
            backend=backend, require_homing=False, worker_period=0.005,
            min_command_interval=0.005, action_timeout=1.0, motion_timeout=0.03,
        )
        driver.connect()
        try:
            command = JointCommand(name=["fr3v2_joint1"], position=[0.01])
            driver.set_command(command)
            time.sleep(0.02)
            driver.set_command(command)
            time.sleep(0.05)
            with self.assertRaises(RuntimeError):
                driver.get_state()
        finally:
            driver.disconnect()

    def test_gripper_timeout_is_fail_closed(self) -> None:
        backend = FakeBackend(gripper_polls_remaining=100)
        driver = FrankaFR3Driver(
            backend=backend, require_homing=False, worker_period=0.005,
            min_command_interval=0.005, action_timeout=1.0, gripper_timeout=0.03,
        )
        driver.connect()
        try:
            driver.set_command(JointCommand(name=["gripper"], position=[-0.04]))
            time.sleep(0.07)
            with self.assertRaises(RuntimeError):
                driver.get_state()
            self.assertGreater(backend.robot_stop_count, 0)
            self.assertGreater(backend.gripper_stop_count, 0)
        finally:
            driver.disconnect()

    def test_structured_lifecycle_log_has_event_field(self) -> None:
        backend = FakeBackend()
        driver = FrankaFR3Driver(backend=backend, require_homing=False)
        with self.assertLogs("robots_franka_fr3.driver", level="INFO") as captured:
            driver.connect()
            driver.disconnect()
        self.assertIn("fr3_connect_ok", [record.event for record in captured.records])
        self.assertIn("fr3_disconnect_ok", [record.event for record in captured.records])

    def test_disconnect_requires_explicit_reconnect(self) -> None:
        self.driver.disconnect()
        with self.assertRaises(RuntimeError):
            self.driver.set_command(JointCommand(name=["fr3v2_joint1"], position=[0.01]))
        self.assertFalse(self.backend.connected)
        self.driver.connect()
        self.assertTrue(self.driver.connected)

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
