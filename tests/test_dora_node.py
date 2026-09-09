from __future__ import annotations

import re
import time
import unittest
from unittest.mock import patch

from typer.testing import CliRunner

from forge_msgs import JointCommand, JointState
from robots_franka_fr3.backend import FakeBackend
from robots_franka_fr3.driver import FrankaFR3Driver
from robots_franka_fr3.node import app as node_app
from robots_franka_fr3.node import run_franka_dora_node

runner = CliRunner()

_ANSI_RE = re.compile(r"\x1b\[[0-9;]*m")


def _plain(text: str) -> str:
    """去除 rich/typer 在 FORCE_COLOR（CI）环境下注入的 ANSI 样式码。"""
    return _ANSI_RE.sub("", text)


class NodeCliTest(unittest.TestCase):
    def test_help_documents_config_option(self) -> None:
        result = runner.invoke(node_app, ["--help"])
        self.assertEqual(result.exit_code, 0, result.output)
        self.assertIn("--config", _plain(result.output))

    def test_missing_config_is_usage_error(self) -> None:
        result = runner.invoke(node_app, [])
        self.assertEqual(result.exit_code, 2, result.output)


class _Node:
    instance: _Node | None = None

    def __init__(self) -> None:
        self.outputs: list[tuple[str, object]] = []
        self.events = [
            {"type": "INPUT", "id": "tick", "value": None},
            {"type": "INPUT", "id": "action", "value": JointCommand(
                name=["fr3v2_joint1"], position=[0.02]
            ).to_arrow()},
            {"type": "INPUT", "id": "action/end_effector", "value": JointCommand(
                name=["gripper"], position=[-0.04]
            ).to_arrow()},
            {"type": "INPUT", "id": "tick", "value": None},
            {"type": "INPUT", "id": "stop/arm", "value": None},
            {"type": "STOP"},
        ]
        self.__class__.instance = self

    def __iter__(self):  # type: ignore[no-untyped-def]
        for index, event in enumerate(self.events):
            if index == 3:
                time.sleep(0.05)
            yield event

    def send_output(self, output_id: str, value: object) -> None:
        self.outputs.append((output_id, value))

    def merge_external_events(self, subscription: object) -> None:
        del subscription


class DoraSingleNodeTest(unittest.TestCase):
    def test_state_action_and_end_effector_contract(self) -> None:
        backend = FakeBackend()
        driver = FrankaFR3Driver(
            backend=backend, require_homing=False,
            worker_period=0.005, min_command_interval=0.005,
        )
        driver.connect()
        with patch("robots_franka_fr3.node.Node", _Node):
            self.assertEqual(run_franka_dora_node(driver), 0)

        assert _Node.instance is not None
        self.assertEqual([name for name, _ in _Node.instance.outputs], ["state", "state"])
        final = JointState.from_arrow(_Node.instance.outputs[-1][1])
        self.assertEqual(final.name[-1], "gripper")
        self.assertAlmostEqual(final.position[0], 0.02)
        self.assertAlmostEqual(final.position[-1], 0.04)
        self.assertFalse(driver.connected)
        # stop/arm 与最终 disconnect 都必须传播到两个后端通道。
        self.assertGreaterEqual(backend.robot_stop_count, 2)
        self.assertGreaterEqual(backend.gripper_stop_count, 2)

    def test_invalid_action_exits_and_disconnects(self) -> None:
        class InvalidActionNode(_Node):
            def __init__(self) -> None:
                super().__init__()
                self.events = [
                    {"type": "INPUT", "id": "action", "value": JointCommand(
                        name=["unknown"], position=[0.0]
                    ).to_arrow()},
                ]

        backend = FakeBackend()
        driver = FrankaFR3Driver(backend=backend, require_homing=False)
        driver.connect()
        with patch("robots_franka_fr3.node.Node", InvalidActionNode):
            with self.assertRaises(ValueError):
                run_franka_dora_node(driver)
        self.assertFalse(driver.connected)
        self.assertFalse(backend.connected)


if __name__ == "__main__":
    unittest.main()
