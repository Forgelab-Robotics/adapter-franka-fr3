from __future__ import annotations

from types import SimpleNamespace
import sys
import unittest
from unittest.mock import patch

from robots_franka_fr3.backend import DynamicsFactors, FrankyBackend


class _Future:
    def __init__(self, result: bool = True, *, ready: bool = True) -> None:
        self.result = result
        self.ready = ready

    def wait(self, timeout: float | None = None) -> bool:
        del timeout
        return self.ready

    def get(self) -> bool:
        return self.result


class _Limit:
    def __init__(self, values: list[float]) -> None:
        self.max = values


class _Robot:
    instances: list[_Robot] = []

    def __init__(self, ip: str) -> None:
        self.ip = ip
        self.relative_dynamics_factor = None
        self.current_joint_positions = [0.0] * 7
        self.current_joint_velocities = [0.0] * 7
        self.joint_velocity_limit = _Limit([2.0] * 7)
        self.joint_acceleration_limit = _Limit([3.0] * 7)
        self.joint_jerk_limit = _Limit([4.0] * 7)
        self.has_errors = False
        self.is_in_control = False
        self.state = SimpleNamespace(
            q=[0.0] * 7,
            dq=[0.0] * 7,
            tau_J=[0.0] * 7,
            current_errors=(),
            last_motion_errors=(),
            robot_mode="RobotMode.Idle",
            time=SimpleNamespace(to_sec=lambda: 1.0),
        )
        self.moves: list[tuple[object, bool]] = []
        self.stop_count = 0
        self.__class__.instances.append(self)

    def move(self, motion: object, *, asynchronous: bool) -> None:
        self.moves.append((motion, asynchronous))

    def poll_motion(self) -> bool:
        return False

    def stop(self) -> None:
        self.stop_count += 1

    def recover_from_errors(self) -> bool:
        return True


class _Gripper:
    instances: list[_Gripper] = []

    def __init__(self, ip: str) -> None:
        self.ip = ip
        self.server_version = 3
        self.max_width = 0.08
        self.width = 0.08
        self.state = SimpleNamespace(
            width=0.08,
            max_width=0.08,
            is_grasped=False,
            temperature=25,
            time=SimpleNamespace(to_sec=lambda: 2.0),
        )
        self.move_args: tuple[float, float] | None = None
        self.stop_count = 0
        self.__class__.instances.append(self)

    def homing_async(self) -> _Future:
        return _Future()

    def move_async(self, width: float, speed: float) -> _Future:
        self.move_args = (width, speed)
        return _Future()

    def move(self, width: float, speed: float) -> bool:
        self.move_args = (width, speed)
        return True

    def grasp(self, width: float, speed: float, force: float) -> bool:
        self.move_args = (width, speed)
        return force > 0.0

    def stop(self) -> bool:
        self.stop_count += 1
        return True


class _JointState:
    def __init__(self, values: list[float]) -> None:
        self.values = values


class _JointMotion:
    def __init__(self, state: _JointState) -> None:
        self.state = state


class _RelativeDynamicsFactor:
    def __init__(self, velocity: float, acceleration: float, jerk: float) -> None:
        self.values = (velocity, acceleration, jerk)


FRANKY = SimpleNamespace(
    Robot=_Robot,
    Gripper=_Gripper,
    JointState=_JointState,
    JointMotion=_JointMotion,
    RelativeDynamicsFactor=_RelativeDynamicsFactor,
)


class FrankyBackendContractTest(unittest.TestCase):
    def setUp(self) -> None:
        _Robot.instances.clear()
        _Gripper.instances.clear()

    def test_locked_api_is_async_and_preserves_total_gripper_width(self) -> None:
        factors = DynamicsFactors(0.1, 0.2, 0.3)
        with patch.dict(sys.modules, {"franky": FRANKY}):
            backend = FrankyBackend(
                "192.0.2.1", dynamics_factor=factors,
                expected_gripper_server_version=3, check_realtime=False,
            )
            backend.connect()
            limits = backend.dynamics_limits()
            self.assertEqual(limits.acceleration, (3.0,) * 7)
            backend.command_joints([0.01] * 7, factors)
            backend.command_gripper(0.04, 0.03)
            self.assertFalse(backend.poll_gripper())
            self.assertEqual(backend.home_gripper(1.0), 0.08)

        robot = _Robot.instances[-1]
        gripper = _Gripper.instances[-1]
        self.assertTrue(robot.moves[-1][1])
        self.assertEqual(robot.relative_dynamics_factor.values, (0.1, 0.2, 0.3))
        self.assertEqual(robot.moves[-1][0].state.values, [0.01] * 7)
        self.assertEqual(gripper.move_args, (0.04, 0.03))

    def test_gripper_future_timeout_is_clear(self) -> None:
        class SlowGripper(_Gripper):
            def homing_async(self) -> _Future:
                return _Future(ready=False)

        module = SimpleNamespace(**FRANKY.__dict__)
        module.Gripper = SlowGripper
        with patch.dict(sys.modules, {"franky": module}):
            backend = FrankyBackend("192.0.2.1", check_realtime=False)
            backend.connect()
            with self.assertRaises(TimeoutError):
                backend.home_gripper(0.01)


if __name__ == "__main__":
    unittest.main()
