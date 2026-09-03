#!/usr/bin/env python3
"""以首个 fresh proprio 为参考，向 FR3v2 仿真发送相对 J1 偏移。"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path


ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / "src"))

from dora import Node  # noqa: E402
from forge_msgs import JointCommand, JointState  # noqa: E402
from robots_franka_fr3.contract import ACTUATOR_ORDER, load_joint_limits  # noqa: E402
from robots_franka_fr3.simulation import validate_action  # noqa: E402


J1_OFFSETS_RAD = (0.0, 0.05, 0.0, -0.05)


def command_for_elapsed(
    elapsed: float,
    target_seconds: float,
    reference: tuple[float, ...],
) -> JointCommand:
    """把相对启动姿态的 J1 偏移转换为仿真所需的绝对 position。"""

    index = int(elapsed / target_seconds) % len(J1_OFFSETS_RAD)
    target = list(reference)
    target[0] += J1_OFFSETS_RAD[index]
    return JointCommand(
        name=list(ACTUATOR_ORDER),
        mode="position",
        position=list(validate_action(ACTUATOR_ORDER, target, load_joint_limits(ROOT))),
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
    reference: tuple[float, ...] | None = None
    for event in node:
        match event["type"]:
            case "INPUT" if event["id"] == "proprio_state" and reference is None:
                state = JointState.from_arrow(event["value"])
                ordered = state.to_np(list(ACTUATOR_ORDER), field="position")
                reference = tuple(float(value) for value in ordered)
            case "INPUT" if event["id"] == "tick" and reference is not None:
                command = command_for_elapsed(
                    time.monotonic() - start,
                    args.target_seconds,
                    reference,
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
