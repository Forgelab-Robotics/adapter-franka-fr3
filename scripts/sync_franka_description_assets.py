#!/usr/bin/env python3
"""从锁定的官方 franka_description 版本同步 FR3v2 + white hand 资产。"""

from __future__ import annotations

import argparse
import hashlib
import json
import shutil
import subprocess
from datetime import datetime
from pathlib import Path
from zoneinfo import ZoneInfo


UPSTREAM_URL = "https://github.com/frankarobotics/franka_description.git"
UPSTREAM_TAG = "2.9.0"
UPSTREAM_COMMIT = "7aeeddc449edf8d62b594f9e36a81da53e7796f9"
LICENSE_EXPRESSION = (
    "Apache-2.0（上游完整 LICENSE 同时保留 BSD-3-Clause 文本）"
)

ARCHIVED_FILES = (
    "LICENSE",
    "NOTICE",
    "package.xml",
    "urdfs/fr3v2_franka_hand.urdf",
    "urdfs/fr3v2_franka_hand.srdf",
    "robots/fr3v2/joint_limits.yaml",
    "robots/fr3v2/kinematics.yaml",
    "robots/fr3v2/dynamics.yaml",
    "robots/fr3v2/inertials.yaml",
    "end_effectors/franka_hand/inertials.yaml",
)

MESH_DIRECTORIES = (
    "meshes/robots/fr3v2/visual",
    "meshes/robots/fr3v2/collision",
    "meshes/robot_ee/franka_hand_white/visual",
    "meshes/robot_ee/franka_hand_white/collision",
)


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def git(source: Path, *args: str) -> str:
    return subprocess.check_output(
        ["git", "-C", str(source), *args], text=True
    ).strip()


def check_source(source: Path, allow_dirty: bool) -> None:
    if not (source / ".git").exists():
        raise RuntimeError(f"不是 Git 仓库：{source}")
    head = git(source, "rev-parse", "HEAD")
    if head != UPSTREAM_COMMIT:
        raise RuntimeError(
            f"franka_description HEAD 不匹配：期望 {UPSTREAM_COMMIT}，实际 {head}"
        )
    remote = git(source, "remote", "get-url", "origin")
    accepted = {UPSTREAM_URL, UPSTREAM_URL.removesuffix(".git")}
    if remote not in accepted:
        raise RuntimeError(f"origin 不匹配官方仓库：{remote}")
    dirty = git(source, "status", "--short")
    if dirty and not allow_dirty:
        raise RuntimeError("franka_description 工作区有未提交修改；拒绝同步")


def copy_assets(source: Path, root: Path) -> None:
    archive = root / "assets/source/franka_description"
    meshes = root / "assets/meshes"
    for relative in ARCHIVED_FILES:
        source_file = source / relative
        if not source_file.is_file():
            raise FileNotFoundError(source_file)
        destination = archive / relative
        destination.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(source_file, destination)

    for relative in MESH_DIRECTORIES:
        source_dir = source / relative
        destination = meshes / Path(relative).relative_to("meshes")
        if not source_dir.is_dir():
            raise FileNotFoundError(source_dir)
        shutil.copytree(source_dir, destination, dirs_exist_ok=True)

    runtime_urdf = root / "assets/urdf/fr3v2_franka_hand.urdf"
    runtime_srdf = root / "assets/urdf/fr3v2_franka_hand.srdf"
    runtime_urdf.parent.mkdir(parents=True, exist_ok=True)
    urdf_text = (archive / "urdfs/fr3v2_franka_hand.urdf").read_text()
    package_prefix = "package://franka_description/meshes/"
    if package_prefix not in urdf_text:
        raise RuntimeError("上游生成 URDF 中未找到预期的 package mesh URI")
    runtime_urdf.write_text(urdf_text.replace(package_prefix, "../meshes/"))
    shutil.copy2(archive / "urdfs/fr3v2_franka_hand.srdf", runtime_srdf)


def write_manifest(source: Path, root: Path) -> None:
    asset_root = root / "assets"
    tracked_roots = (
        asset_root / "source/franka_description",
        asset_root / "meshes",
        asset_root / "urdf",
    )
    files: dict[str, dict[str, int | str]] = {}
    for tracked_root in tracked_roots:
        for path in sorted(
            item
            for item in tracked_root.rglob("*")
            if item.is_file() and item.name != "README.md"
        ):
            relative = path.relative_to(root).as_posix()
            files[relative] = {"sha256": sha256(path), "size": path.stat().st_size}

    manifest = {
        "schema_version": 1,
        "source": {
            "name": "franka_description",
            "url": UPSTREAM_URL,
            "tag": UPSTREAM_TAG,
            "commit": UPSTREAM_COMMIT,
            "commit_date": git(source, "show", "-s", "--format=%cI", "HEAD"),
            "license": LICENSE_EXPRESSION,
        },
        "synced_at": datetime.now(ZoneInfo("Asia/Shanghai")).isoformat(timespec="seconds"),
        "runtime_modifications": [
            {
                "path": "assets/urdf/fr3v2_franka_hand.urdf",
                "change": (
                    "仅将 package://franka_description/meshes/ 替换为 "
                    "../meshes/；结构、参数和数值不变"
                ),
            },
            {
                "path": "assets/urdf/fr3v2_franka_hand.srdf",
                "change": "无修改，逐字节复制",
            },
        ],
        "files": files,
    }
    (asset_root / "SOURCE_MANIFEST.json").write_text(
        json.dumps(manifest, ensure_ascii=False, indent=2) + "\n"
    )


def main() -> int:
    root = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--source",
        type=Path,
        default=root.parent / "franka_description",
        help="锁定版本的 franka_description 工作区",
    )
    parser.add_argument(
        "--allow-dirty",
        action="store_true",
        help="仅在明确审计过本地修改时允许脏工作区（不会绕过 commit 检查）",
    )
    args = parser.parse_args()
    source = args.source.resolve()
    check_source(source, args.allow_dirty)
    copy_assets(source, root)
    write_manifest(source, root)
    print(f"已从 {UPSTREAM_TAG}/{UPSTREAM_COMMIT[:7]} 同步资产并更新 manifest")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
