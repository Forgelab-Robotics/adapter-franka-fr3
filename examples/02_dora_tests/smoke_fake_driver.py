#!/usr/bin/env python3
"""不启动 Dora 的 fake 驱动 smoke test；用于先验收 topic payload 契约。"""

from __future__ import annotations

import time

from forge_msgs import JointCommand

from robots_franka_fr3.backend import FakeBackend
from robots_franka_fr3.driver import FrankaFR3Driver


def main() -> int:
    backend = FakeBackend()
    driver = FrankaFR3Driver(backend=backend, auto_connect=True)
    try:
        driver.set_command(JointCommand(name=["fr3v2_joint1"], position=[0.02]))
        driver.set_command(JointCommand(name=["gripper"], position=[-0.04]))
        time.sleep(0.08)
        state = driver.get_state()
        assert state.name[-1] == "gripper"
        assert abs(state.position[0] - 0.02) < 1e-9
        assert abs(state.position[-1] - 0.04) < 1e-9
        print("RESULT: PASS", state.name)
        return 0
    finally:
        driver.disconnect()


if __name__ == "__main__":
    raise SystemExit(main())
