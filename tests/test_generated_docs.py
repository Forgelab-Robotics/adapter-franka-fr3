from __future__ import annotations

import importlib.util
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location(
    "generate_contract_docs", ROOT / "scripts/generate_contract_docs.py"
)
assert SPEC and SPEC.loader
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


class GeneratedDocsTest(unittest.TestCase):
    def test_joint_limits_markdown_is_current(self) -> None:
        source = (
            ROOT
            / "assets/source/franka_description/robots/fr3v2/joint_limits.yaml"
        )
        output = ROOT / "config/generated/joint_limits.md"
        self.assertEqual(output.read_text(), MODULE.render(source))
        package_copy = (
            ROOT / "src/robots_franka_fr3/data/fr3v2_joint_limits.yaml"
        )
        self.assertEqual(package_copy.read_bytes(), source.read_bytes())


if __name__ == "__main__":
    unittest.main()
