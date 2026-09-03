"""Dora 节点入口；配置文件只包含连接和安全参数，不保存现场凭据。"""

from __future__ import annotations

import argparse
import logging
import sys
from pathlib import Path
from typing import Any, Mapping

if __package__ in (None, ""):
    sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

import yaml
from dora import Node
from forge_msgs import JointCommand
from forge_robot.arrow_validation import RobotArrowSchemaError, validate_robot_control_arrow

from robots_franka_fr3.backend import DynamicsFactors, FakeBackend, FrankyBackend
from robots_franka_fr3.driver import FrankaFR3Driver


def load_robot_config(path: str | Path) -> dict[str, Any]:
    config = yaml.safe_load(Path(path).read_text()) or {}
    if not isinstance(config, dict):
        raise ValueError("robot config 顶层必须是 mapping")
    return config


def _value(config: Mapping[str, Any], section: Mapping[str, Any], key: str, default: Any) -> Any:
    return config[key] if key in config else section.get(key, default)


def build_driver_from_config(config: Mapping[str, Any]) -> FrankaFR3Driver:
    """兼容旧顶层字段，并以 robot/control/safety 分区为标准。"""
    robot = config.get("robot", {}) or {}
    control = config.get("control", {}) or {}
    safety = config.get("safety", {}) or {}
    if not all(isinstance(item, Mapping) for item in (robot, control, safety)):
        raise ValueError("robot/control/safety 配置必须是 mapping")
    backend_name = _value(config, robot, "backend", "fake")
    allow_real = _value(config, control, "allow_real_motion", False)
    if backend_name not in ("fake", "franky"):
        raise ValueError(f"不支持的 backend：{backend_name}")
    if backend_name == "franky" and not allow_real:
        raise PermissionError("backend=franky 必须显式设置 allow_real_motion=true")
    ip = str(_value(config, robot, "ip", "172.16.0.2"))
    raw_factors = config.get(
        "dynamics_factors",
        safety.get(
            "relative_dynamics_factors",
            config.get("dynamics_factor", safety.get("relative_dynamics_factor_initial", 0.05)),
        ),
    )
    factors = DynamicsFactors.coerce(raw_factors)
    backend = FakeBackend() if backend_name == "fake" else FrankyBackend(
        ip,
        dynamics_factor=factors,
        expected_gripper_server_version=robot.get("gripper_server_version"),
    )
    return FrankaFR3Driver(
        ip=ip,
        backend=backend,
        dynamics_factor=factors,
        action_timeout=float(_value(config, control, "action_timeout_s", 0.5)),
        state_timeout=float(_value(config, control, "state_timeout_s", 0.5)),
        motion_timeout=float(_value(config, control, "motion_timeout_s", 10.0)),
        gripper_timeout=float(_value(config, control, "gripper_timeout_s", 5.0)),
        homing_timeout=float(_value(config, control, "homing_timeout_s", 15.0)),
        worker_join_timeout=float(_value(config, control, "worker_join_timeout_s", 2.0)),
        worker_period=float(_value(config, control, "worker_period_s", 0.02)),
        min_command_interval=float(_value(config, control, "min_command_interval_s", 0.05)),
        position_margin_rad=float(_value(config, safety, "position_margin_rad", 0.05)),
        max_step_rad=float(_value(config, control, "max_step_rad", 0.05)),
        gripper_speed=float(_value(config, control, "gripper_speed_mps", 0.03)),
        gripper_force=float(_value(config, control, "gripper_force_n", 50.0)),
        require_homing=bool(_value(config, control, "require_homing", True)),
        position_command_semantics=str(
            _value(config, control, "position_command_semantics", "relative")
        ),
    )


def run_franka_dora_node(driver: FrankaFR3Driver, *, debug: bool = False) -> int:
    """运行 FR3 Dora 节点，并把上游 Action cancel 明确传播到真机 stop。

    通用 ``run_dora_robot_node`` 只处理 ``action``，无法观察轨迹/夹爪
    controller 的 cancel。末端 Skill 若只停止发送 waypoint，Franky 已接收的异步
    ``JointMotion`` 仍可能继续运行。因此本节点额外接受任意 ``stop/<source>`` 输入，
    对 arm/Hand 同时调用软件 stop。软件 stop 仍不替代 Desk/外部急停。
    """
    node = Node()
    joint_order = driver.joint_order
    try:
        for event in node:
            kind = event.get("kind")
            if kind not in (None, "dora"):
                continue
            match event.get("type"):
                case "INPUT":
                    input_id = str(event["id"])
                    if input_id == "tick":
                        state = driver.get_state()
                        node.send_output("state", state.to_arrow())
                        continue
                    if input_id.startswith("stop/") and len(input_id) > len("stop/"):
                        logging.warning("收到 %s，向 FR3 arm/Hand 传播软件 stop", input_id)
                        driver.stop()
                        continue
                    if input_id == "action" or (
                        input_id.startswith("action/")
                        and len(input_id) > len("action/")
                    ):
                        value = event.get("value")
                        try:
                            validate_robot_control_arrow(value, joint_order)
                        except RobotArrowSchemaError as exc:
                            logging.error("拒绝无效 %s（Arrow schema）：%s", input_id, exc)
                            driver.stop()
                            raise
                        command = JointCommand.from_arrow(value)
                        if debug:
                            logging.debug(
                                "收到 %s：joints=%s mode=%s",
                                input_id,
                                command.name,
                                command.mode,
                            )
                        driver.set_command(command)
                case "STOP":
                    break
                case "ERROR":
                    logging.error("节点收到 Dora ERROR：%s", event.get("error", "unknown"))
                    driver.stop()
                    break
                case _:
                    pass
    finally:
        driver.disconnect()
    return 0


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", required=True)
    parser.add_argument("--debug", action="store_true")
    args = parser.parse_args()
    logging.basicConfig(level=logging.DEBUG if args.debug else logging.INFO)
    try:
        driver = build_driver_from_config(load_robot_config(args.config))
    except (OSError, ValueError, PermissionError) as exc:
        parser.error(str(exc))
    driver.connect()
    try:
        return run_franka_dora_node(driver, debug=args.debug)
    finally:
        driver.disconnect()


if __name__ == "__main__":
    raise SystemExit(main())
