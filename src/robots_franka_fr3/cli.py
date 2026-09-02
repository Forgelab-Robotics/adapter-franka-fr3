"""FR3 Franky SDK 最小测试与 Dora 节点 CLI。"""

from __future__ import annotations

import argparse
import json
import time
from pathlib import Path

import yaml
from forge_msgs import JointCommand

from .backend import FakeBackend, FrankyBackend
from .contract import ACTUATOR_ORDER, ARM_JOINT_ORDER, HOME_POSITION, GRIPPER_MAX_WIDTH_M, load_joint_limits
from .driver import FrankaFR3Driver


def _driver(args: argparse.Namespace) -> FrankaFR3Driver:
    backend = FakeBackend() if args.backend == "fake" else FrankyBackend(
        args.ip, dynamics_factor=args.dynamics_factor,
        expected_server_version=getattr(args, "server_version", None),
        check_realtime=True,
    )
    return FrankaFR3Driver(ip=args.ip, backend=backend, dynamics_factor=args.dynamics_factor,
                           max_step_rad=args.max_step_rad, require_homing=not args.no_homing,
                           gripper_speed=args.gripper_speed, gripper_force=args.gripper_force)


def read_state(args: argparse.Namespace) -> int:
    driver = _driver(args)
    driver.connect()
    try:
        state = driver.get_state()
        raw = driver.backend.state()
        print(json.dumps({"name": state.name, "position_rad_or_m": state.position,
                          "velocity_rad_per_s": state.velocity, "effort_nm": state.effort,
                          "gripper_width_m": raw.gripper_width,
                          "gripper_max_width_m": raw.gripper_max_width,
                          "errors": raw.errors,
                          "backend": args.backend}, ensure_ascii=False))
    finally:
        driver.disconnect()
    return 0


def move_single_joint(args: argparse.Namespace) -> int:
    if args.joint not in ARM_JOINT_ORDER:
        raise SystemExit(f"未知关节：{args.joint}")
    driver = _driver(args)
    driver.connect()
    try:
        current = driver.get_state().position
        index = ARM_JOINT_ORDER.index(args.joint)
        target = current[index] + args.offset
        # 低速小步执行，避免单条命令绕过每周期变化限制。
        step = max(1e-6, driver.max_step_rad)
        while abs(target - current[index]) > 1e-9:
            value = current[index] + max(-step, min(step, target - current[index]))
            driver.set_command(JointCommand(name=[args.joint], position=[value]))
            time.sleep(args.period)
            current = driver.get_state().position
        driver.set_command(JointCommand(name=[args.joint], position=[HOME_POSITION[index]]))
        print(f"完成 {args.joint}: offset={args.offset} rad")
    finally:
        driver.disconnect()
    return 0


def joint_min_max_home(args: argparse.Namespace) -> int:
    limits = load_joint_limits()
    joints = [args.joint] if args.joint else list(ARM_JOINT_ORDER)
    driver = _driver(args)
    if args.dry_run:
        for name in joints:
            limit = limits[name]["limit"]
            margin = args.margin
            print(name, float(limit["lower"]) + margin, float(limit["upper"]) - margin, HOME_POSITION[ARM_JOINT_ORDER.index(name)])
        return 0
    driver.connect()
    try:
        for name in joints:
            limit = limits[name]["limit"]
            index = ARM_JOINT_ORDER.index(name)
            for value in (float(limit["lower"]) + args.margin, HOME_POSITION[index], float(limit["upper"]) - args.margin, HOME_POSITION[index]):
                if args.only_read:
                    print(name, value)
                else:
                    current = driver.get_state().position[index]
                    while abs(value - current) > driver.max_step_rad:
                        current += max(-driver.max_step_rad, min(driver.max_step_rad, value - current))
                        driver.set_command(JointCommand(name=[name], position=[current]))
                        time.sleep(args.period)
                    driver.set_command(JointCommand(name=[name], position=[value]))
                    time.sleep(args.period)
    finally:
        driver.disconnect()
    return 0


def gripper_open_close(args: argparse.Namespace) -> int:
    driver = _driver(args)
    driver.connect()
    try:
        max_width = driver.backend.state().gripper_max_width
        for label, width in (("open", max_width), ("half", max_width / 2), ("closed", 0.0), ("reopen", max_width)):
            driver.set_command(JointCommand(name=["gripper"], position=[width]))
            time.sleep(args.period)
            print(label, width, driver.backend.state().gripper_width)
    finally:
        driver.disconnect()
    return 0


def safety_stop(args: argparse.Namespace) -> int:
    driver = _driver(args)
    driver.connect()
    try:
        driver.stop()
        print("软件 stop 已调用；外部急停仍需现场单独验证")
    finally:
        driver.disconnect()
    return 0


def run_node(args: argparse.Namespace) -> int:
    driver = _driver(args)
    driver.connect()
    try:
        from forge_robot.node_runner import run_dora_robot_node
        return run_dora_robot_node(driver, joint_order=driver.joint_order, debug=args.debug)
    finally:
        driver.disconnect()


def parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(prog="robots-franka-fr3")
    sub = p.add_subparsers(dest="command", required=True)
    for name, func in (("read-state", read_state), ("move-single-joint", move_single_joint),
                       ("joint-min-max-home", joint_min_max_home), ("gripper-open-close", gripper_open_close),
                       ("safety-stop", safety_stop), ("run", run_node)):
        q = sub.add_parser(name)
        q.set_defaults(func=func, backend="fake", ip="172.16.0.2", dynamics_factor=0.05,
                       max_step_rad=0.05, period=0.05, no_homing=False, margin=0.05,
                       only_read=False, dry_run=False, debug=False, server_version=None)
        q.add_argument("--backend", choices=("fake", "franky"))
        q.add_argument("--ip")
        q.add_argument("--dynamics-factor", type=float)
        q.add_argument("--max-step-rad", type=float)
        q.add_argument("--period", type=float)
        q.add_argument("--no-homing", action="store_true")
        q.add_argument("--debug", action="store_true")
        q.add_argument("--server-version", type=int)
        q.add_argument("--gripper-speed", type=float, default=0.03)
        q.add_argument("--gripper-force", type=float, default=50.0)
    sub.choices["move-single-joint"].add_argument("joint", choices=ARM_JOINT_ORDER)
    sub.choices["move-single-joint"].add_argument("--offset", type=float, default=0.02)
    sub.choices["joint-min-max-home"].add_argument("--joint", choices=ARM_JOINT_ORDER)
    sub.choices["joint-min-max-home"].add_argument("--margin", type=float, default=0.05)
    sub.choices["joint-min-max-home"].add_argument("--only-read", action="store_true")
    sub.choices["joint-min-max-home"].add_argument("--dry-run", action="store_true")
    return p


def main(argv: list[str] | None = None) -> int:
    args = parser().parse_args(argv)
    return int(args.func(args))
