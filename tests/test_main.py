from __future__ import annotations

import contextlib
import io
import json
import unittest
from unittest.mock import patch

from typer.testing import CliRunner

from robots_franka_fr3.backend import FakeBackend
from robots_franka_fr3.contract import ACTUATOR_ORDER
from robots_franka_fr3.main import (
    Args,
    app,
    gripper_open_close,
    joint_min_max_home,
    move_single_joint,
    observe_safety,
    read_state,
    recover,
    run_node,
    safety_stop,
)

runner = CliRunner()


class TyperWiringTest(unittest.TestCase):
    def test_help_lists_all_commands(self) -> None:
        result = runner.invoke(app, ["--help"])
        self.assertEqual(result.exit_code, 0)
        for name in ("read-state", "move-single-joint", "joint-min-max-home",
                     "gripper-open-close", "safety-stop", "observe-safety",
                     "recover", "run"):
            self.assertIn(name, result.output)

    def test_version_flag(self) -> None:
        result = runner.invoke(app, ["--version"])
        self.assertEqual(result.exit_code, 0)
        self.assertIn("franka-fr3 0.1.0", result.output)

    def test_read_state_end_to_end_via_typer(self) -> None:
        result = runner.invoke(app, ["read-state", "--samples", "1", "--period", "0"])
        self.assertEqual(result.exit_code, 0, result.output)
        self.assertEqual(json.loads(result.output)["event"], "read_state")

    def test_invalid_backend_choice_is_rejected(self) -> None:
        result = runner.invoke(app, ["read-state", "--backend", "bogus"])
        self.assertNotEqual(result.exit_code, 0)
        self.assertIn("backend", result.output)

    def test_move_single_joint_typer_parses_positional(self) -> None:
        result = runner.invoke(
            app, ["move-single-joint", "fr3v2_joint2", "--backend", "fake",
                  "--offset", "0.01", "--timeout", "0.5", "--period", "0.005"],
        )
        self.assertEqual(result.exit_code, 0, result.output)
        records = [json.loads(line) for line in result.output.splitlines()]
        self.assertEqual(records[0]["event"], "move_single_joint_start")
        self.assertEqual(records[-1]["event"], "move_single_joint_complete")


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
        args = Args(backend="fake", samples=1)
        output = io.StringIO()

        with patch("robots_franka_fr3.main._backend", return_value=backend):
            with contextlib.redirect_stdout(output):
                self.assertEqual(read_state(args), 0)

        payload = json.loads(output.getvalue())
        self.assertEqual(payload["actuator_order"], list(ACTUATOR_ORDER))
        self.assertEqual(payload["event"], "read_state")
        self.assertEqual(backend.command_history, [])
        self.assertFalse(backend.connected)

    def test_static_robot_source_timestamp_is_rejected(self) -> None:
        from dataclasses import replace

        class StaticClockBackend(FakeBackend):
            def state(self):  # type: ignore[no-untyped-def]
                return replace(super().state(), robot_time_s=1.0)

        backend = StaticClockBackend()
        args = Args(backend="fake", samples=2, period=0.0,
                    max_age=0.5, max_read_ms=500.0)
        with patch("robots_franka_fr3.main._backend", return_value=backend):
            with contextlib.redirect_stdout(io.StringIO()):
                with self.assertRaisesRegex(TimeoutError, "RobotState.time 未前进"):
                    read_state(args)


class MoveSingleJointTest(unittest.TestCase):
    @staticmethod
    def args(**overrides: object) -> Args:
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
        return Args(**values)

    def test_moves_out_and_returns_to_initial_without_gripper_homing(self) -> None:
        class ArmOnlyBackend(FakeBackend):
            def home_gripper(self) -> float:
                raise AssertionError("arm-only test must not home the gripper")

        backend = ArmOnlyBackend()
        backend.position[0] = -0.4
        output = io.StringIO()

        with patch("robots_franka_fr3.main._backend", return_value=backend):
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
        with patch("robots_franka_fr3.main._backend", return_value=backend):
            with self.assertRaises(TimeoutError):
                move_single_joint(self.args(timeout=0.03))

        self.assertGreaterEqual(backend.stop_count, 1)

    def test_out_of_range_target_is_rejected_before_motion(self) -> None:
        backend = FakeBackend()
        backend.position[0] = 2.8
        with patch("robots_franka_fr3.main._backend", return_value=backend):
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
        with patch("robots_franka_fr3.main._backend", return_value=backend):
            with contextlib.redirect_stdout(io.StringIO()):
                self.assertEqual(move_single_joint(args), 0)

        self.assertAlmostEqual(backend.position[0], -0.4)
        self.assertAlmostEqual(backend.position[2], 0.35)


class AcceptanceCommandTest(unittest.TestCase):
    @staticmethod
    def args(**overrides: object) -> Args:
        values: dict[str, object] = {
            "backend": "fake",
            "ip": "172.16.0.2",
            "dynamics_factor": 0.05,
            "max_step_rad": 0.05,
            "gripper_speed": 0.03,
            "gripper_force": 50.0,
            "joint": "fr3v2_joint1",
            "acceptance_min": -0.05,
            "acceptance_max": 0.05,
            "margin": 0.1,
            "period": 0.005,
            "timeout": 0.5,
            "tolerance": 0.003,
            "retries": 2,
            "retry_delay": 1.0,
            "recover_on_retry": True,
            "recovery_delay": 1.0,
            "dry_run": False,
            "execute": False,
        }
        values.update(overrides)
        return Args(**values)

    def test_joint_range_dry_run_has_no_connection(self) -> None:
        args = self.args(dry_run=True)
        output = io.StringIO()
        with contextlib.redirect_stdout(output):
            self.assertEqual(joint_min_max_home(args), 0)
        self.assertEqual(json.loads(output.getvalue())["event"], "joint_range_dry_run")

    def test_joint_range_visits_each_target_and_returns_home(self) -> None:
        backend = FakeBackend()
        args = self.args()
        with patch("robots_franka_fr3.main._backend", return_value=backend):
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
        args = self.args(retries=1, retry_delay=0.0, recovery_delay=0.0)
        output = io.StringIO()
        with patch("robots_franka_fr3.main._backend", return_value=backend):
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
        args = self.args(retries=2, retry_delay=0.0)
        output = io.StringIO()
        with patch("robots_franka_fr3.main._backend", return_value=backend):
            with patch("robots_franka_fr3.main._move_joint_to",
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
        args = self.args()
        with patch("robots_franka_fr3.main._backend", return_value=backend):
            with contextlib.redirect_stdout(io.StringIO()):
                self.assertEqual(gripper_open_close(args), 0)
        self.assertEqual(backend.command_history, [])
        self.assertEqual([entry[0] for entry in backend.gripper_command_history],
                         ["homing", "move", "move", "move", "move"])

    def test_gripper_expected_failed_grasp_is_recorded(self) -> None:
        backend = FakeBackend(grasp_result=False)
        args = self.args(grasp_width=0.03, expect_grasp="failure", hold_seconds=0.0)
        output = io.StringIO()
        with patch("robots_franka_fr3.main._backend", return_value=backend):
            with contextlib.redirect_stdout(output):
                self.assertEqual(gripper_open_close(args), 0)
        records = [json.loads(line) for line in output.getvalue().splitlines()]
        grasp = next(record for record in records if record["event"] == "gripper_grasp")
        self.assertFalse(grasp["sdk_result"])

    def test_software_stop_and_recover(self) -> None:
        backend = FakeBackend(errors=("joint_reflex",), robot_mode="RobotMode.Reflex")
        stop_args = self.args(stop_after=0.0, offset=0.03)
        with patch("robots_franka_fr3.main._backend", return_value=backend):
            with contextlib.redirect_stdout(io.StringIO()):
                self.assertEqual(safety_stop(stop_args), 0)
        self.assertGreaterEqual(backend.stop_count, 1)

        recover_args = self.args()
        with patch("robots_franka_fr3.main._backend", return_value=backend):
            with contextlib.redirect_stdout(io.StringIO()):
                self.assertEqual(recover(recover_args), 0)
        self.assertEqual(backend.recover_count, 1)
        self.assertEqual(backend.errors, ())

    def test_observe_operator_user_stop(self) -> None:
        backend = FakeBackend(robot_mode="RobotMode.UserStopped")
        args = self.args(event="user-stop", duration=0.1)
        with patch("robots_franka_fr3.main._backend", return_value=backend):
            with contextlib.redirect_stdout(io.StringIO()):
                self.assertEqual(observe_safety(args), 0)

    def test_run_real_node_requires_execute_gate(self) -> None:
        args = self.args(backend="franky")
        with self.assertRaisesRegex(SystemExit, "--execute"):
            run_node(args)


if __name__ == "__main__":
    unittest.main()
