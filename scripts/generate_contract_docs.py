#!/usr/bin/env python3
"""由官方 joint_limits.yaml 生成只读 Markdown 契约表。"""

from __future__ import annotations

import argparse
from pathlib import Path

import yaml


def render(source: Path) -> str:
    values = yaml.safe_load(source.read_text())
    lines = [
        "# FR3v2 关节限位（自动生成）",
        "",
        "> 请勿手工编辑。本文件由 `scripts/generate_contract_docs.py` 从",
        "> `assets/source/franka_description/robots/fr3v2/joint_limits.yaml` 生成。",
        "",
        "| 逻辑名称 | 上游键 | 位置下限 rad | 位置上限 rad | 最大速度 rad/s | 最大力矩 N·m |",
        "|---|---|---:|---:|---:|---:|",
    ]
    for index in range(1, 8):
        key = f"joint{index}"
        limit = values[key]["limit"]
        lines.append(
            "| fr3v2_joint{index} | {key} | {lower:.10f} | {upper:.10f} | "
            "{velocity:.4f} | {effort:.1f} |".format(index=index, key=key, **limit)
        )

    lines.extend(
        [
            "",
            "## 位置相关速度限位参数",
            "",
            "Franky/libfranka 真机侧仍应使用底层库的动态限位；下表用于资产核对，",
            "不能用固定最大速度替代位置相关限速。",
            "",
            "| 逻辑名称 | velocity_offset | deceleration_limit |",
            "|---|---:|---:|",
        ]
    )
    for index in range(1, 8):
        data = values[f"joint{index}"]["position_based_velocity_limits"]
        lines.append(
            f"| fr3v2_joint{index} | {data['velocity_offset']:.12f} | "
            f"{data['deceleration_limit']:.4f} |"
        )
    return "\n".join(lines) + "\n"


def main() -> int:
    root = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--check", action="store_true", help="只检查已生成文件是否最新"
    )
    args = parser.parse_args()
    source = (
        root
        / "assets/source/franka_description/robots/fr3v2/joint_limits.yaml"
    )
    output = root / "config/generated/joint_limits.md"
    package_copy = root / "src/robots_franka_fr3/data/fr3v2_joint_limits.yaml"
    expected = render(source)
    if args.check:
        if not output.is_file() or output.read_text() != expected:
            raise SystemExit(f"生成文件已过期：{output}")
        if not package_copy.is_file() or package_copy.read_bytes() != source.read_bytes():
            raise SystemExit(f"包内限位镜像已过期：{package_copy}")
        print(f"ok: {output.relative_to(root)}")
        return 0
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(expected)
    package_copy.parent.mkdir(parents=True, exist_ok=True)
    package_copy.write_bytes(source.read_bytes())
    print(f"generated: {output.relative_to(root)}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
