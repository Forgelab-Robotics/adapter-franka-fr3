#!/usr/bin/env bash
# 由本仓 MJCF 生成 GLB。外部 pipeline 仅是转换工具，输入资产仍来自锁定的官方来源。
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PIPELINE_ROOT="${ROBOT_ASSET_PIPELINE_ROOT:-$(cd "$ROOT/../.." && pwd)/robot_asset_pipeline}"
PYTHON_BIN="${ROBOT_ASSET_PIPELINE_PYTHON:-python3}"
INPUT="$ROOT/assets/mjcf/fr3v2_franka_hand.xml"
OUTPUT_DIR="${FRANKA_FR3_GLB_OUT:-$ROOT/assets/glb}"
NAME="${FRANKA_FR3_GLB_NAME:-franka_fr3}"

if [[ ! -f "$INPUT" ]]; then
  echo "缺少 MJCF：$INPUT；先运行 uv run python scripts/generate_mjcf.py" >&2
  exit 2
fi
if [[ ! -f "$PIPELINE_ROOT/main.py" ]]; then
  echo "未找到 robot_asset_pipeline：$PIPELINE_ROOT" >&2
  echo "请设置 ROBOT_ASSET_PIPELINE_ROOT=/absolute/path/to/robot_asset_pipeline" >&2
  exit 2
fi
if ! command -v "$PYTHON_BIN" >/dev/null 2>&1; then
  echo "找不到 Python：$PYTHON_BIN" >&2
  exit 2
fi

mkdir -p "$OUTPUT_DIR"
(
  cd "$PIPELINE_ROOT"
  "$PYTHON_BIN" main.py "$INPUT" -o "$OUTPUT_DIR" -n "$NAME" --skip-preview
)

if [[ ! -f "$OUTPUT_DIR/$NAME.glb" ]]; then
  echo "转换程序正常返回但未生成：$OUTPUT_DIR/$NAME.glb" >&2
  exit 1
fi

PIPELINE_COMMIT="$(git -C "$PIPELINE_ROOT" rev-parse HEAD 2>/dev/null || echo unknown)"
PIPELINE_ORIGIN="$(git -C "$PIPELINE_ROOT" remote get-url origin 2>/dev/null || echo unknown)"
PIPELINE_DIRTY="$(git -C "$PIPELINE_ROOT" status --porcelain 2>/dev/null || true)"
export ROOT PIPELINE_ROOT PIPELINE_COMMIT PIPELINE_ORIGIN PIPELINE_DIRTY
export INPUT OUTPUT_DIR NAME
"$PYTHON_BIN" - <<'PY'
import hashlib
import json
import os
from datetime import datetime
from pathlib import Path
from zoneinfo import ZoneInfo


def digest(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


root = Path(os.environ["ROOT"])
output = Path(os.environ["OUTPUT_DIR"])
name = os.environ["NAME"]
input_path = Path(os.environ["INPUT"])
output_files = [output / f"{name}.glb"]
normalized_xml = output / f"{name}.xml"
if normalized_xml.is_file():
    output_files.append(normalized_xml)

record = {
    "schema_version": 1,
    "generated_at": datetime.now(ZoneInfo("Asia/Shanghai")).isoformat(timespec="seconds"),
    "input": {
        "path": str(input_path.relative_to(root)),
        "sha256": digest(input_path),
    },
    "tool": {
        "name": "robot_asset_pipeline",
        "origin": os.environ["PIPELINE_ORIGIN"],
        "commit": os.environ["PIPELINE_COMMIT"],
        "dirty": bool(os.environ["PIPELINE_DIRTY"]),
    },
    "outputs": {
        path.name: {"sha256": digest(path), "size": path.stat().st_size}
        for path in output_files
    },
}
(output / f"{name}.provenance.json").write_text(
    json.dumps(record, ensure_ascii=False, indent=2) + "\n"
)
PY

echo "已生成 $OUTPUT_DIR/$NAME.glb 和 $OUTPUT_DIR/$NAME.provenance.json"
