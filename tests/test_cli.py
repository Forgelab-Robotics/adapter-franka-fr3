from __future__ import annotations

import argparse
import contextlib
from dataclasses import replace
import io
import json
import unittest
from unittest.mock import patch

from robots_franka_fr3.backend import FakeBackend
from robots_franka_fr3.cli import (
    gripper_open_close,
    joint_min_max_home,
    move_single_joint,
    observe_safety,
    parser,
    read_state,
    recover,
    safety_stop,
)
from robots_franka_fr3.contract import ACTUATOR_ORDER


class ReadStateTest(unittest.TestCase):
    def test_read_state_does_not_start_motion(self) -> None:
        class ReadOnlyBackend(FakeBackend):
            def command_joints(self, position: object, dynamics_factor: float) -> None:
                raise AssertionError("read-state must not command joints")

            def home_gripper(self) -> float:
                raise AssertionError("read-state must not home the gripper")

            def move_gripper(self, width: float, speed: float, force: float | None = None) -> bool:
                raise AssertionError("read-state must not move the gripper")

        backend = ReadOnlyBackend()
        args = argparse.Namespace(backend="fake")
        output = io.StringIO()

        with patch("robots_franka_fr3.cli._backend", return_value=backend):
            with contextlib.redirect_stdout(output):
                self.assertEqual(read_state(args), 0)

        payload = json.loads(output.getvalue())
        self.assertEqual(payload["actuator_order"], list(ACTUATOR_ORDER))
        self.assertEqual(payload["event"], "read_state")
        self.assertEqual(backend.command_history, [])
        self.assertFalse(backend.connected)

    def test_static_robot_source_timestamp_is_rejected(self) -> None:
        class StaticClockBackend(FakeBackend):
            def state(self):  # type: ignore[no-untyped-def]
                return replace(super().state(), robot_time_s=1.0)

        backend = StaticClockBackend()
        args = argparse.Namespace(backend="fake", samples=2, period=0.0,
                                  max_age=0.5, max_read_ms=500.0)
        with patch("robots_franka_fr3.cli._backend", return_value=backend):
            with contextlib.redirect_stdout(io.StringIO()):
                with self.assertRaisesRegex(TimeoutError, "RobotState.time 未前进"):
                    read_state(args)


class MoveSingleJointTest(unittest.TestCase):
    @staticmethod
    def args(**overrides: object) -> argparse.Namespace:
        values: dict[str, object] = {
            "backend": "fake",
            "ip": "172.16.0.2",
            "dynamics_factor": 0.05,
            "max_step_rad": 0.05,
            "no_homing": False,
            "gripper_speed": 0.03,
            "gripper_force": 50.0,
            "joint": "fr3v2_joint1",
            "offset": 0.01,
            "period": 0.005,
            "timeout": 0.5,
            "tolerance": 0.001,
        }
        values.update(overrides)
        return argparse.Namespace(**values)

    def test_moves_out_and_returns_to_initial_without_gripper_homing(self) -> None:
        class ArmOnlyBackend(FakeBackend):
            def home_gripper(self) -> float:
                raise AssertionError("arm-only test must not home the gripper")

        backend = ArmOnlyBackend()
        backend.position[0] = -0.4
        output = io.StringIO()

        with patch("robots_franka_fr3.cli._backend", return_value=backend):
            with contextlib.redirect_stdout(output):
                self.assertEqual(move_single_joint(self.args()), 0)

        self.assertEqual(len(backend.command_history), 2)
        self.assertAlmostEqual(backend.command_history[0][0], -0.39)
        self.assertAlmostEqual(backend.position[0], -0.4)
        events = [json.loads(line)["event"] for line in output.getvalue().splitlines()]
        self.assertEqual(events[-1], "move_single_joint_complete")

    def test_motion_timeout_stops_instead_of_looping_forever(self) -> None:
        class StuckBackend(FakeBackend):
            def command_joints(self, position: object, dynamics_factor: float) -> None:
                self._require()

        backend = StuckBackend()
        with patch("robots_franka_fr3.cli._backend", return_value=backend):
            with self.assertRaises(TimeoutError):
                move_single_joint(self.args(timeout=0.03))

        self.assertGreaterEqual(backend.stop_count, 1)

    def test_out_of_range_target_is_rejected_before_motion(self) -> None:
        backend = FakeBackend()
        backend.position[0] = 2.8
        with patch("robots_franka_fr3.cli._backend", return_value=backend):
            with self.assertRaisesRegex(ValueError, "超出官方限位"):
                move_single_joint(self.args(offset=0.2, max_offset=0.3))

        self.assertEqual(backend.command_history, [])

    def test_real_motion_requires_explicit_execute(self) -> None:
        with self.assertRaisesRegex(SystemExit, "--execute"):
            move_single_joint(self.args(backend="franky"))

    def test_real_motion_uses_current_pose_without_configured_home_check(self) -> None:
        backend = FakeBackend()
        backend.position[0] = -0.4
        backend.position[2] = 0.35
        args = self.args(backend="franky", execute=True)
        with patch("robots_franka_fr3.cli._backend", return_value=backend):
            with contextlib.redirect_stdout(io.StringIO()):
                self.assertEqual(move_single_joint(args), 0)

        self.assertAlmostEqual(backend.position[0], -0.4)
        self.assertAlmostEqual(backend.position[2], 0.35)


class AcceptanceCommandTest(unittest.TestCase):
    @staticmethod
    def parse(*arguments: str) -> argparse.Namespace:
        return parser().parse_args(list(arguments))

    def test_joint_range_dry_run_has_no_connection(self) -> None:
        args = self.parse("joint-min-max-home", "fr3v2_joint1",
                          "--acceptance-min", "-0.05", "--acceptance-max", "0.05",
                          "--dry-run")
        output = io.StringIO()
        with contextlib.redirect_stdout(output):
            self.assertEqual(joint_min_max_home(args), 0)
        self.assertEqual(json.loads(output.getvalue())["event"], "joint_range_dry_run")

    def test_joint_range_visits_each_target_and_returns_home(self) -> None:
        backend = FakeBackend()
        args = self.parse("joint-min-max-home", "fr3v2_joint1",
                          "--acceptance-min", "-0.05", "--acceptance-max", "0.05",
                          "--period", "0.005", "--timeout", "0.5")
        with patch("robots_franka_fr3.cli._backend", return_value=backend):
            with contextlib.redirect_stdout(io.StringIO()):
                self.assertEqual(joint_min_max_home(args), 0)
        self.assertEqual([round(command[0], 3) for command in backend.command_history],
                         [-0.04, -0.05, -0.01, 0.0, 0.04, 0.05, 0.01, 0.0])
        self.assertAlmostEqual(backend.position[0], 0.0)

    def test_joint_range_recovers_retries_and_continues_same_phase(self) -> None:
        class RecoveringFlakyBackend(FakeBackend):
            failures_remaining = 1

            def command_joints(self, position: object, dynamics_factor: float) -> None:
                if self.failures_remaining:
                    self.failures_remaining -= 1
                    self.errors = ("communication_constraints_violation",)
                    self.robot_mode = "RobotMode.Reflex"
                    raise RuntimeError("transient motion failure")
                super().command_joints(position, dynamics_factor)  # type: ignore[arg-type]

        backend = RecoveringFlakyBackend()
        args = self.parse("joint-min-max-home", "fr3v2_joint1",
                          "--acceptance-min", "-0.05", "--acceptance-max", "0.05",
                          "--period", "0.005", "--timeout", "0.5",
                          "--retries", "1", "--retry-delay", "0",
                          "--recovery-delay", "0")
        output = io.StringIO()
        with patch("robots_franka_fr3.cli._backend", return_value=backend):
            with contextlib.redirect_stdout(output):
                self.assertEqual(joint_min_max_home(args), 0)

        events = [json.loads(line)["event"] for line in output.getvalue().splitlines()]
        self.assertIn("joint_range_attempt_failed", events)
        self.assertIn("joint_range_retry", events)
        self.assertIn("joint_range_recover", events)
        self.assertEqual(backend.recover_count, 1)
        self.assertAlmostEqual(backend.position[0], 0.0)

    def test_joint_range_ctrl_c_exits_without_retry(self) -> None:
        backend = FakeBackend()
        args = self.parse("joint-min-max-home", "fr3v2_joint1",
                          "--acceptance-min", "-0.05", "--acceptance-max", "0.05",
                          "--retries", "2", "--retry-delay", "0")
        output = io.StringIO()
        with patch("robots_franka_fr3.cli._backend", return_value=backend):
            with patch("robots_franka_fr3.cli._move_joint_to",
                       side_effect=KeyboardInterrupt):
                with contextlib.redirect_stdout(output):
                    with self.assertRaises(KeyboardInterrupt):
                        joint_min_max_home(args)

        events = [json.loads(line)["event"] for line in output.getvalue().splitlines()]
        self.assertEqual(events, ["joint_range_attempt"])
        self.assertGreaterEqual(backend.stop_count, 1)
        self.assertFalse(backend.connected)

    def test_gripper_cycle_never_commands_arm(self) -> None:
        backend = FakeBackend()
        args = self.parse("gripper-open-close")
        with patch("robots_franka_fr3.cli._backend", return_value=backend):
            with contextlib.redirect_stdout(io.StringIO()):
                self.assertEqual(gripper_open_close(args), 0)
        self.assertEqual(backend.command_history, [])
        self.assertEqual([entry[0] for entry in backend.gripper_command_history],
                         ["homing", "move", "move", "move", "move"])

    def test_gripper_expected_failed_grasp_is_recorded(self) -> None:
        backend = FakeBackend(grasp_result=False)
        args = self.parse("gripper-open-close", "--grasp-width", "0.03",
                          "--expect-grasp", "failure", "--hold-seconds", "0")
        output = io.StringIO()
        with patch("robots_franka_fr3.cli._backend", return_value=backend):
            with contextlib.redirect_stdout(output):
                self.assertEqual(gripper_open_close(args), 0)
        records = [json.loads(line) for line in output.getvalue().splitlines()]
        grasp = next(record for record in records if record["event"] == "gripper_grasp")
        self.assertFalse(grasp["sdk_result"])

    def test_software_stop_and_recover(self) -> None:
        backend = FakeBackend(errors=("joint_reflex",), robot_mode="RobotMode.Reflex")
        stop_args = self.parse("safety-stop", "--stop-after", "0")
        with patch("robots_franka_fr3.cli._backend", return_value=backend):
            with contextlib.redirect_stdout(io.StringIO()):
                self.assertEqual(safety_stop(stop_args), 0)
        self.assertGreaterEqual(backend.stop_count, 1)

        recover_args = self.parse("recover")
        with patch("robots_franka_fr3.cli._backend", return_value=backend):
            with contextlib.redirect_stdout(io.StringIO()):
                self.assertEqual(recover(recover_args), 0)
        self.assertEqual(backend.recover_count, 1)
        self.assertEqual(backend.errors, ())

    def test_observe_operator_user_stop(self) -> None:
        backend = FakeBackend(robot_mode="RobotMode.UserStopped")
        args = self.parse("observe-safety", "--event", "user-stop", "--duration", "0.1")
        with patch("robots_franka_fr3.cli._backend", return_value=backend):
            with contextlib.redirect_stdout(io.StringIO()):
                self.assertEqual(observe_safety(args), 0)


if __name__ == "__main__":
    unittest.main()
