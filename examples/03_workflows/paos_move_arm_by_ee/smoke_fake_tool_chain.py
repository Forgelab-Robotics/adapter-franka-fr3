#!/usr/bin/env python3
"""只对 managed franka_fake runtime 执行 resolve→move→gripper smoke。"""

from __future__ import annotations

import argparse
import asyncio
import json
from pathlib import Path
import time

from PhyAgentOS.forge.tool_client import ForgeToolClient


SKILL_NAME = "move-arm-by-ee-franka-fr3"
TERMINAL_PHASES = {"completed", "failed", "cancelled", "stopped", "unknown"}


def _assert_fake_runtime() -> None:
    state_path = Path.home() / ".PhyAgentOS/run/skills" / f"{SKILL_NAME}.json"
    state = json.loads(state_path.read_text(encoding="utf-8"))
    if state.get("status") != "running" or state.get("profile") != "franka_fake":
        raise RuntimeError("拒绝执行：当前 managed runtime 不是 running/franka_fake")


async def _wait_result(client: ForgeToolClient, invocation_id: str, timeout_s: float) -> dict:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        status = await client.invocation_status(invocation_id)
        if status["data"]["phase"] in TERMINAL_PHASES:
            return await client.invocation_result(invocation_id)
        await asyncio.sleep(0.1)
    raise TimeoutError(f"invocation {invocation_id} 未在 {timeout_s} s 内终止")


async def _run(base_url: str) -> None:
    _assert_fake_runtime()
    client = ForgeToolClient(base_url, timeout_s=10.0)
    try:
        for tool_id in (
            "motion.resolve_relative_pose",
            "motion.move_pose",
            "gripper.set_opening",
        ):
            context = await client.get_tool_context(tool_id)
            if not context["data"]["ready"]:
                raise RuntimeError(f"Tool 未就绪：{tool_id}")

        query = await client.invoke_query_tool(
            "motion.resolve_relative_pose",
            {
                "group_name": "fr3v2_arm",
                "target_frame": "fr3v2_hand_tcp",
                "reference": "current",
                "translation_frame": "base",
                "translation_m": {"x": 0.0, "y": 0.0, "z": 0.01},
                "orientation_mode": "preserve",
                "axis_angle_rad": None,
                "max_state_age_ms": 200,
            },
            timeout_ms=5_000,
        )
        outputs = query["data"]["response"]["result"]["outputs"]
        action = await client.invoke_action(
            "motion.move_pose",
            {
                "group_name": "fr3v2_arm",
                "reference_frame": outputs["frames"]["reference_frame"],
                "target_frame": "fr3v2_hand_tcp",
                "target_pose": outputs["target_pose"],
                "velocity_scale": 0.05,
                "acceleration_scale": 0.05,
                "position_tolerance_m": 0.01,
                "orientation_tolerance_rad": 0.05,
            },
            caller_id="fr3-fake-smoke",
            timeout_ms=30_000,
        )
        motion_result = await _wait_result(client, action["data"]["invocation_id"], 40.0)
        if motion_result["data"]["result"]["status"] != "succeeded":
            raise RuntimeError(f"move_pose 失败：{motion_result}")

        gripper = await client.invoke_action(
            "gripper.set_opening",
            {"opening_m": 0.04},
            caller_id="fr3-fake-smoke",
            timeout_ms=15_000,
        )
        gripper_result = await _wait_result(
            client, gripper["data"]["invocation_id"], 20.0
        )
        result = gripper_result["data"]["result"]
        if (
            result["status"] != "succeeded"
            or not result["outputs"]["gripper_result"]["reached_goal"]
        ):
            raise RuntimeError(f"gripper.set_opening 失败：{gripper_result}")
        print(json.dumps({
            "status": "PASS",
            "motion": motion_result["data"]["result"],
            "gripper": result,
        }, ensure_ascii=False))
    finally:
        await client.close()


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--gateway", default="http://127.0.0.1:19012")
    parser.add_argument("--execute", action="store_true")
    args = parser.parse_args()
    if not args.execute:
        parser.error("该 smoke 会改变 FakeBackend 状态；请显式添加 --execute")
    asyncio.run(_run(args.gateway))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
