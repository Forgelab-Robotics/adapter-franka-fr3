#!/usr/bin/env python3
"""向 FR3v2 仿真发送完整、合法、以安全 home 为基准的 sweep action。"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path


ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / "src"))

from dora import Node  # noqa: E402
from forge_msgs import JointCommand  # noqa: E402
from robots_franka_fr3.contract import ACTUATOR_ORDER, load_joint_limits  # noqa: E402
from robots_franka_fr3.simulation import build_sweep_waypoints  # noqa: E402


def command_for_elapsed(elapsed: float, target_seconds: float) -> JointCommand:
    """按时间选择一个完整 waypoint；所有非活动轴保持安全 home。"""

    waypoints = build_sweep_waypoints(load_joint_limits(ROOT))
    index = int(elapsed / target_seconds) % len(waypoints)
    waypoint = waypoints[index]
    return JointCommand(
        name=list(ACTUATOR_ORDER),
        mode="position",
        position=list(waypoint.position),
    )


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--target-seconds",
        type=float,
        default=2.0,
        help="每个 sweep waypoint 的保持时间",
    )
    args = parser.parse_args()
    if args.target_seconds <= 0:
        parser.error("--target-seconds 必须大于 0")
    return args


def main() -> int:
    args = _parse_args()
    node = Node()
    start = time.monotonic()
    for event in node:
        match event["type"]:
            case "INPUT" if event["id"] == "tick":
                command = command_for_elapsed(
                    time.monotonic() - start,
                    args.target_seconds,
                )
                node.send_output("action", command.to_arrow())
            case "STOP":
                break
            case "ERROR":
                print(
                    f"[fr3_sim_action_source] {event.get('error', 'unknown error')}",
                    file=sys.stderr,
                )
                return 1
            case _:
                pass
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
