"""Dora 节点入口；配置文件只包含连接和安全参数，不保存现场凭据。"""

from __future__ import annotations

import logging

import typer
from dora import Node
from forge_common import get_logger
from forge_msgs import JointCommand
from forge_robot.arrow_validation import RobotArrowSchemaError, validate_robot_control_arrow

from robots_franka_fr3.config import build_driver_from_config, load_config
from robots_franka_fr3.driver import FrankaFR3Driver

logger = get_logger(__name__)

app = typer.Typer(
    name="robots-franka-fr3-node",
    help="FR3 Dora 节点（--config robot.yaml）",
    no_args_is_help=True,
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
                        logger.warning("收到 %s，向 FR3 arm/Hand 传播软件 stop", input_id)
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
                            logger.error("拒绝无效 %s（Arrow schema）：%s", input_id, exc)
                            driver.stop()
                            raise
                        command = JointCommand.from_arrow(value)
                        if debug:
                            logger.debug(
                                "收到 %s：joints=%s mode=%s",
                                input_id,
                                command.name,
                                command.mode,
                            )
                        driver.set_command(command)
                case "STOP":
                    break
                case "ERROR":
                    logger.error("节点收到 Dora ERROR：%s", event.get("error", "unknown"))
                    driver.stop()
                    break
                case _:
                    pass
    finally:
        driver.disconnect()
    return 0


@app.command()
def main(
    config: str = typer.Option(..., "--config", help="robot YAML 配置文件路径"),
    debug: bool = typer.Option(False, "--debug", help="输出 debug 级别日志"),
) -> None:
    """运行 FR3 Dora 节点。"""
    level = logging.DEBUG if debug else logging.INFO
    logger.setLevel(level)
    for handler in logger.handlers:
        handler.setLevel(level)
    try:
        driver = build_driver_from_config(load_config(config))
    except (OSError, ValueError, PermissionError) as exc:
        typer.echo(f"配置错误：{exc}", err=True)
        raise typer.Exit(2) from exc
    driver.connect()
    try:
        return run_franka_dora_node(driver, debug=debug)
    finally:
        driver.disconnect()


if __name__ == "__main__":
    app()
