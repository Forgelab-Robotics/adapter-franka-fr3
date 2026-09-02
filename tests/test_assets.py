from __future__ import annotations

import importlib.util
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location(
    "validate_assets", ROOT / "scripts/validate_assets.py"
)
assert SPEC and SPEC.loader
VALIDATE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(VALIDATE)


class AssetTest(unittest.TestCase):
    def test_source_manifest(self) -> None:
        VALIDATE.validate_manifest(ROOT)

    def test_urdf_and_limits(self) -> None:
        VALIDATE.validate_urdf(ROOT)

    def test_mjcf(self) -> None:
        VALIDATE.validate_mjcf(ROOT)


if __name__ == "__main__":
    unittest.main()
