from __future__ import annotations

import os
import subprocess
import tempfile
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
SCRIPT = ROOT / "scripts/build_glb_robot_asset_pipeline.sh"


class GlbDeliveryScriptTest(unittest.TestCase):
    def test_script_has_no_personal_absolute_path(self) -> None:
        text = SCRIPT.read_text()
        # Split so the public-tree machine-path grep stays clean.
        self.assertNotIn("/ho" "me/", text)
        self.assertIn("ROBOT_ASSET_PIPELINE_ROOT", text)
        self.assertIn('f"{name}.provenance.json"', text)

    def test_missing_pipeline_fails_clearly(self) -> None:
        with tempfile.TemporaryDirectory() as temp_dir:
            missing = Path(temp_dir) / "missing-pipeline"
            env = os.environ.copy()
            env["ROBOT_ASSET_PIPELINE_ROOT"] = str(missing)
            result = subprocess.run(
                ["bash", str(SCRIPT)],
                cwd=ROOT,
                env=env,
                capture_output=True,
                text=True,
                check=False,
            )
        self.assertEqual(result.returncode, 2)
        self.assertIn("未找到 robot_asset_pipeline", result.stderr)
        self.assertIn("ROBOT_ASSET_PIPELINE_ROOT", result.stderr)


if __name__ == "__main__":
    unittest.main()
