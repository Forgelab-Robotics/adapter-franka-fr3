"""Dora 节点入口；配置文件只包含连接和安全参数，不保存现场凭据。"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

if __package__ in (None, ""):
    sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

import yaml

from forge_robot.node_runner import run_dora_robot_node

from robots_franka_fr3.backend import FakeBackend, FrankyBackend
from robots_franka_fr3.driver import FrankaFR3Driver


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", required=True)
    args = parser.parse_args()
    config = yaml.safe_load(Path(args.config).read_text()) or {}
    robot_config = config.get("robot", {})
    control = config.get("control", {})
    safety = config.get("safety", {})
    backend_name = config.get("backend", robot_config.get("backend", "fake"))
    allow_real = config.get("allow_real_motion", control.get("allow_real_motion", False))
    if backend_name not in ("fake", "franky"):
        raise SystemExit(f"不支持的 backend：{backend_name}")
    if backend_name == "franky" and not allow_real:
        raise SystemExit("backend=franky 必须显式设置 allow_real_motion=true")
    ip = config.get("ip", robot_config.get("ip", "172.16.0.2"))
    dynamics_factor = float(config.get("dynamics_factor", safety.get("relative_dynamics_factor_initial", 0.05)))
    backend = FakeBackend() if backend_name == "fake" else FrankyBackend(
        ip,
        dynamics_factor=dynamics_factor,
        expected_server_version=robot_config.get("robot_server_version"),
        )
    driver = FrankaFR3Driver(
        ip=ip,
        backend=backend,
        dynamics_factor=dynamics_factor,
        action_timeout=float(config.get("action_timeout_s", control.get("action_timeout_s", 0.5))),
        state_timeout=float(config.get("state_timeout_s", control.get("state_timeout_s", 0.5))),
        max_step_rad=float(config.get("max_step_rad", control.get("max_step_rad", 0.05))),
        gripper_speed=float(config.get("gripper_speed_mps", control.get("gripper_speed_mps", 0.03))),
        gripper_force=float(config.get("gripper_force_n", control.get("gripper_force_n", 50.0))),
        expected_server_version=robot_config.get("robot_server_version"),
        auto_connect=True,
    )
    try:
        return run_dora_robot_node(driver, joint_order=driver.joint_order)
    finally:
        driver.disconnect()


if __name__ == "__main__":
    raise SystemExit(main())
