from __future__ import annotations

import unittest
from pathlib import Path

import yaml


ROOT = Path(__file__).resolve().parents[1]


class RealWorkflowTest(unittest.TestCase):
    def test_has_driver_and_safe_action_source(self) -> None:
        path = ROOT / "examples/03_workflows/real_basic_motion/dataflow.yaml"
        data = yaml.safe_load(path.read_text())
        nodes = {n["id"]: n for n in data["nodes"]}
        self.assertEqual(set(nodes), {"franka_fr3", "test_action_source"})
        self.assertIn("src/robots_franka_fr3/node.py", nodes["franka_fr3"]["path"])
        self.assertEqual(nodes["franka_fr3"]["inputs"]["action"], "test_action_source/action")
        self.assertIn("--mode target", nodes["test_action_source"]["args"])
        self.assertNotIn("camera", path.read_text().lower())


if __name__ == "__main__":
    unittest.main()
