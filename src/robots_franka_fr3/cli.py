"""FR3 Franky SDK 最小测试与 Dora 节点 CLI。"""

from __future__ import annotations

import argparse
from datetime import UTC, datetime
import json
import math
import time
from pathlib import Path

from forge_msgs import JointCommand

from .backend import FakeBackend, FrankyBackend, RobotBackend
from .contract import ACTUATOR_ORDER, ARM_JOINT_ORDER, HOME_POSITION, GRIPPER_MAX_WIDTH_M, load_joint_limits
from .driver import FrankaFR3Driver


def _backend(args: argparse.Namespace) -> RobotBackend:
    return FakeBackend() if args.backend == "fake" else FrankyBackend(
        args.ip, dynamics_factor=args.dynamics_factor,
        expected_gripper_server_version=getattr(args, "gripper_server_version", None),
        check_realtime=True,
    )


def _driver(args: argparse.Namespace, *, require_homing: bool | None = None) -> FrankaFR3Driver:
    backend = _backend(args)
    if require_homing is None:
        require_homing = not args.no_homing
    return FrankaFR3Driver(ip=args.ip, backend=backend, dynamics_factor=args.dynamics_factor,
                           max_step_rad=args.max_step_rad, require_homing=require_homing,
                           gripper_speed=args.gripper_speed, gripper_force=args.gripper_force)


class _Recorder:
    def __init__(self, path: str | None) -> None:
        self.path = Path(path).expanduser() if path else None

    def emit(self, event: str, **fields: object) -> dict[str, object]:
        record: dict[str, object] = {
            "timestamp_utc": datetime.now(UTC).isoformat(),
            "event": event,
            **fields,
        }
        line = json.dumps(record, ensure_ascii=False)
        print(line, flush=True)
        if self.path is not None:
            self.path.parent.mkdir(parents=True, exist_ok=True)
            with self.path.open("a", encoding="utf-8") as stream:
                stream.write(line + "\n")
        return record


def _recorder(args: argparse.Namespace) -> _Recorder:
    return _Recorder(getattr(args, "log", None))


def _require_real_execute(args: argparse.Namespace, action: str) -> None:
    if args.backend == "franky" and not getattr(args, "execute", False):
        raise SystemExit(
            f"拒绝执行真机{action}：确认工作区、Desk、外部急停、监护人和批准姿态后，"
            "显式添加 --execute"
        )


def _approved_home(args: argparse.Namespace) -> tuple[float, ...]:
    configured = getattr(args, "home_rad", None) or HOME_POSITION[:7]
    values = tuple(float(value) for value in configured)
    if len(values) != 7 or not all(math.isfinite(value) for value in values):
        raise ValueError("home-rad 必须包含 7 个有限的 rad 值")
    return values


def _validate_joint_target(joint: str, target: float) -> None:
    if not math.isfinite(target):
        raise ValueError("关节目标必须是有限数")
    limit = load_joint_limits()[joint]["limit"]
    lower, upper = float(limit["lower"]), float(limit["upper"])  # type: ignore[index]
    if not lower <= target <= upper:
        raise ValueError(f"{joint} 目标 {target} 超出官方限位 [{lower}, {upper}] rad")


def read_state(args: argparse.Namespace) -> int:
    # State inspection must not start the motion worker or home the gripper.
    backend = _backend(args)
    recorder = _recorder(args)
    samples = int(getattr(args, "samples", 1))
    period = float(getattr(args, "period", 0.1))
    max_age = float(getattr(args, "max_age", 0.5))
    max_read_ms = float(getattr(args, "max_read_ms", 500.0))
    if samples <= 0 or period < 0.0 or max_age <= 0.0 or max_read_ms <= 0.0:
        raise ValueError("samples 必须为正；period、max-age、max-read-ms 必须为非负有效值")
    backend.connect()
    previous_robot_time: float | None = None
    try:
        for sample_index in range(samples):
            started = time.monotonic()
            raw = backend.state()
            read_ms = (time.monotonic() - started) * 1000.0
            age_ms = (time.monotonic() - raw.timestamp) * 1000.0
            vectors = raw.position + raw.velocity + raw.effort + (
                raw.gripper_width,
                raw.gripper_max_width,
            )
            if not all(math.isfinite(value) for value in vectors):
                raise RuntimeError("SDK 状态包含 NaN/Inf")
            if age_ms > max_age * 1000.0 or read_ms > max_read_ms:
                raise TimeoutError(
                    f"状态不新鲜：age={age_ms:.3f} ms, read={read_ms:.3f} ms"
                )
            source_delta_ms = None
            if raw.robot_time_s is not None and previous_robot_time is not None:
                source_delta_ms = (raw.robot_time_s - previous_robot_time) * 1000.0
                if source_delta_ms <= 0.0:
                    raise TimeoutError(
                        f"RobotState.time 未前进：delta={source_delta_ms:.3f} ms"
                    )
            recorder.emit(
                "read_state",
                sample_index=sample_index,
                backend=args.backend,
                actuator_order=list(ACTUATOR_ORDER),
                units={"position": "rad (arm), m total opening (gripper)",
                       "velocity": "rad/s", "effort": "N*m"},
                position=list(raw.position) + [raw.gripper_width],
                velocity=list(raw.velocity) + [0.0],
                effort=list(raw.effort) + [0.0],
                gripper={"width_m": raw.gripper_width,
                         "max_width_m": raw.gripper_max_width},
                errors=list(raw.errors),
                diagnostics=backend.diagnostics(),
                sample_age_ms=age_ms,
                read_duration_ms=read_ms,
                robot_source_time_s=raw.robot_time_s,
                robot_source_delta_ms=source_delta_ms,
                gripper_source_time_s=raw.gripper_time_s,
            )
            previous_robot_time = raw.robot_time_s
            if sample_index + 1 < samples:
                time.sleep(period)
    finally:
        backend.disconnect()
    return 0


def _move_joint_to(driver: FrankaFR3Driver, joint: str, target: float, *,
                   period: float, timeout: float, tolerance: float) -> None:
    if period <= 0.0 or timeout <= 0.0 or tolerance <= 0.0:
        raise ValueError("period、timeout 和 tolerance 必须大于 0")

    index = ARM_JOINT_ORDER.index(joint)
    deadline = time.monotonic() + timeout
    current = driver.get_state().position[index]
    # Leave numerical headroom below the driver's maximum single-step limit.
    step = driver.max_step_rad * 0.8
    if step <= 0.0:
        raise ValueError("max-step-rad 必须大于 0")
    while abs(target - current) > tolerance:
        waypoint = current + max(-step, min(step, target - current))
        while abs(waypoint - current) > tolerance:
            if time.monotonic() >= deadline:
                raw = driver.backend.state()
                driver.stop()
                raise TimeoutError(
                    f"{joint} 运动超时：当前 {current:.6f} rad，目标 {target:.6f} rad，"
                    f"errors={list(raw.errors)}"
                )
            # Repeating the same target refreshes the command watchdog; the
            # worker itself sends it to Franky only once.
            driver.set_command(JointCommand(name=[joint], position=[waypoint]))
            time.sleep(period)
            current = driver.get_state().position[index]


def _exception_chain(exc: BaseException) -> list[dict[str, str]]:
    chain: list[dict[str, str]] = []
    current: BaseException | None = exc
    seen: set[int] = set()
    while current is not None and id(current) not in seen:
        seen.add(id(current))
        chain.append({"type": type(current).__name__, "message": str(current)})
        current = current.__cause__ or current.__context__
    return chain


def _run_joint_range_phase(args: argparse.Namespace, recorder: _Recorder, *,
                           joint: str, phase: str, target: float) -> None:
    index = ARM_JOINT_ORDER.index(joint)
    retries = int(args.retries)
    if retries < 0 or args.retry_delay < 0.0 or args.recovery_delay < 0.0:
        raise ValueError("retries、retry-delay 和 recovery-delay 不能为负")

    phase_started = time.monotonic()
    last_error: Exception | None = None
    for attempt in range(1, retries + 2):
        driver = _driver(args, require_homing=False)
        attempt_started = time.monotonic()
        try:
            driver.connect()
            diagnostics = driver.backend.diagnostics()
            if bool(diagnostics.get("has_errors")):
                if not args.recover_on_retry:
                    raise RuntimeError(
                        f"检测到当前错误 {diagnostics.get('current_errors')}；"
                        "如已排除故障源，可添加 --recover-on-retry"
                    )
                recovered = driver.backend.recover()
                recorder.emit("joint_range_recover", joint=joint, phase=phase,
                              attempt=attempt, sdk_result=recovered,
                              before=diagnostics,
                              after=driver.backend.diagnostics())
                if not recovered:
                    raise RuntimeError("Franky automatic error recovery 返回 false")
                if args.recovery_delay:
                    time.sleep(args.recovery_delay)

            recorder.emit("joint_range_attempt", joint=joint, phase=phase,
                          target_rad=target, attempt=attempt,
                          max_attempts=retries + 1)
            _move_joint_to(driver, joint, target, period=args.period,
                           timeout=args.timeout, tolerance=args.tolerance)
            raw = driver.backend.state()
            recorder.emit("joint_range_waypoint", joint=joint, phase=phase,
                          target_rad=target, actual_rad=raw.position[index],
                          attempt=attempt,
                          attempt_elapsed_s=time.monotonic() - attempt_started,
                          phase_elapsed_s=time.monotonic() - phase_started,
                          errors=list(raw.errors))
            return
        except KeyboardInterrupt:
            # Ctrl+C is an operator cancellation, not a transient SDK error:
            # stop once, let finally disconnect, and never enter retry logic.
            try:
                driver.stop()
            finally:
                raise
        except Exception as exc:
            last_error = exc
            actual_rad: float | None = None
            diagnostics: dict[str, object] | None = None
            try:
                actual_rad = driver.backend.state().position[index]
                diagnostics = driver.backend.diagnostics()
            except Exception:
                pass
            recorder.emit("joint_range_attempt_failed", joint=joint, phase=phase,
                          target_rad=target, actual_rad=actual_rad,
                          attempt=attempt, max_attempts=retries + 1,
                          retrying=attempt <= retries,
                          exception_chain=_exception_chain(exc),
                          diagnostics=diagnostics)
            try:
                driver.stop()
            except Exception as stop_exc:
                recorder.emit("joint_range_stop_failed", joint=joint, phase=phase,
                              attempt=attempt,
                              exception_chain=_exception_chain(stop_exc))
        finally:
            try:
                driver.disconnect()
            except Exception as disconnect_exc:
                recorder.emit("joint_range_disconnect_failed", joint=joint, phase=phase,
                              attempt=attempt,
                              exception_chain=_exception_chain(disconnect_exc))

        if attempt <= retries:
            recorder.emit("joint_range_retry", joint=joint, phase=phase,
                          failed_attempt=attempt, next_attempt=attempt + 1,
                          delay_s=args.retry_delay)
            if args.retry_delay:
                time.sleep(args.retry_delay)

    assert last_error is not None
    raise RuntimeError(
        f"{joint} phase={phase} 在 {retries + 1} 次尝试后仍失败"
    ) from last_error


def move_single_joint(args: argparse.Namespace) -> int:
    if args.joint not in ARM_JOINT_ORDER:
        raise SystemExit(f"未知关节：{args.joint}")
    _require_real_execute(args, "单关节运动")
    recorder = _recorder(args)
    max_offset = float(getattr(args, "max_offset", 0.05))
    if max_offset <= 0.0 or not math.isfinite(args.offset) or abs(args.offset) > max_offset:
        raise ValueError(f"offset 必须为有限数且绝对值不超过 {max_offset} rad")
    # An arm-only SDK test must never home or move the gripper as a side effect.
    driver = _driver(args, require_homing=False)
    driver.connect()
    try:
        current = driver.get_state().position
        index = ARM_JOINT_ORDER.index(args.joint)
        initial = current[index]
        target = current[index] + args.offset
        _validate_joint_target(args.joint, target)
        recorder.emit("move_single_joint_start", joint=args.joint, initial_rad=initial,
                      target_rad=target, offset_rad=args.offset,
                      start_position_rad=list(current[:7]),
                      dynamics_factor=args.dynamics_factor)
        started = time.monotonic()
        _move_joint_to(driver, args.joint, target, period=args.period,
                       timeout=args.timeout, tolerance=args.tolerance)
        reached = driver.backend.state()
        recorder.emit("move_single_joint_reached", joint=args.joint,
                      target_rad=target, actual_rad=reached.position[index],
                      elapsed_s=time.monotonic() - started, errors=list(reached.errors))
        returned_at = time.monotonic()
        _move_joint_to(driver, args.joint, initial, period=args.period,
                       timeout=args.timeout, tolerance=args.tolerance)
        returned = driver.backend.state()
        recorder.emit("move_single_joint_complete", joint=args.joint,
                      home_rad=initial, actual_rad=returned.position[index],
                      return_elapsed_s=time.monotonic() - returned_at,
                      errors=list(returned.errors))
    finally:
        driver.disconnect()
    return 0


def joint_min_max_home(args: argparse.Namespace) -> int:
    limits = load_joint_limits()
    name = args.joint
    index = ARM_JOINT_ORDER.index(name)
    approved_home = _approved_home(args)
    home = approved_home[index]
    acceptance_min = float(args.acceptance_min)
    acceptance_max = float(args.acceptance_max)
    margin = float(args.margin)
    if not all(math.isfinite(value) for value in (acceptance_min, acceptance_max, margin)):
        raise ValueError("验收范围和 margin 必须是有限数")
    if margin < 0.0:
        raise ValueError("margin 不能为负数")
    official = limits[name]["limit"]
    official_min = float(official["lower"])
    official_max = float(official["upper"])
    if not official_min + margin <= acceptance_min < home < acceptance_max <= official_max - margin:
        raise ValueError(
            f"验收范围必须满足 {official_min + margin:.6f} <= min < home({home:.6f}) "
            f"< max <= {official_max - margin:.6f} rad"
        )
    recorder = _recorder(args)
    if args.dry_run:
        recorder.emit("joint_range_dry_run", joint=name, acceptance_min_rad=acceptance_min,
                      acceptance_max_rad=acceptance_max, home_rad=home,
                      official_min_rad=official_min, official_max_rad=official_max,
                      mechanical_margin_rad=margin)
        return 0
    _require_real_execute(args, "单轴验收范围运动")
    for phase, target in (("min", acceptance_min), ("home_after_min", home),
                          ("max", acceptance_max), ("home_after_max", home)):
        _run_joint_range_phase(args, recorder, joint=name, phase=phase, target=target)
    return 0


def gripper_open_close(args: argparse.Namespace) -> int:
    _require_real_execute(args, "夹爪运动")
    if args.gripper_speed <= 0.0 or args.gripper_force <= 0.0:
        raise ValueError("gripper-speed 和 gripper-force 必须大于 0")
    if args.width_tolerance <= 0.0 or args.hold_seconds < 0.0 or args.drop_threshold <= 0.0:
        raise ValueError("width-tolerance/drop-threshold 必须大于 0，hold-seconds 不能为负")
    if args.grasp_width is not None and not 0.0 <= args.grasp_width <= GRIPPER_MAX_WIDTH_M:
        raise ValueError(f"grasp-width 必须位于 [0, {GRIPPER_MAX_WIDTH_M}] m")
    if args.expect_grasp != "none" and args.grasp_width is None:
        raise ValueError("设置 expect-grasp 时必须同时提供 grasp-width")
    backend = _backend(args)
    recorder = _recorder(args)
    backend.connect()
    try:
        started = time.monotonic()
        max_width = backend.home_gripper()
        homed = backend.state()
        recorder.emit("gripper_homing", success=True, max_width_m=max_width,
                      actual_width_m=homed.gripper_width,
                      elapsed_s=time.monotonic() - started, errors=list(homed.errors))
        if not 0.0 < max_width <= GRIPPER_MAX_WIDTH_M + args.width_tolerance:
            raise RuntimeError(f"异常 gripper max_width={max_width} m")
        for label, width in (("open", max_width), ("half", max_width / 2.0),
                             ("closed", 0.0), ("reopen", max_width)):
            started = time.monotonic()
            result = backend.move_gripper(width, args.gripper_speed)
            raw = backend.state()
            reached = abs(raw.gripper_width - width) <= args.width_tolerance
            recorder.emit("gripper_move", phase=label, target_width_m=width,
                          actual_width_m=raw.gripper_width,
                          speed_m_per_s=args.gripper_speed, sdk_result=result,
                          reached=reached, elapsed_s=time.monotonic() - started,
                          errors=list(raw.errors))
            if not result or not reached:
                raise RuntimeError(f"夹爪 {label} 失败：result={result}, width={raw.gripper_width}")

        if args.grasp_width is not None:
            started = time.monotonic()
            result = backend.grasp(args.grasp_width, args.gripper_speed, args.gripper_force)
            before_hold = backend.state().gripper_width
            time.sleep(args.hold_seconds)
            after_hold = backend.state().gripper_width
            dropped = abs(after_hold - before_hold) > args.drop_threshold
            recorder.emit("gripper_grasp", target_width_m=args.grasp_width,
                          speed_m_per_s=args.gripper_speed, force_n=args.gripper_force,
                          sdk_result=result, expected=args.expect_grasp,
                          diagnostics=backend.diagnostics(),
                          width_before_hold_m=before_hold, width_after_hold_m=after_hold,
                          width_drift_m=after_hold - before_hold,
                          possible_drop=dropped, elapsed_s=time.monotonic() - started)
            expected = None if args.expect_grasp == "none" else args.expect_grasp == "success"
            if expected is not None and result is not expected:
                raise RuntimeError(f"夹持返回值 {result} 与期望 {args.expect_grasp} 不一致")
            backend.move_gripper(max_width, args.gripper_speed)
    finally:
        backend.stop_gripper()
        backend.disconnect()
    return 0


def safety_stop(args: argparse.Namespace) -> int:
    _require_real_execute(args, "软件 stop 测试")
    if (args.max_offset <= 0.0 or not math.isfinite(args.offset)
            or abs(args.offset) > args.max_offset):
        raise ValueError(f"offset 绝对值不能超过 {args.max_offset} rad")
    if args.stop_after < 0.0 or args.settle_seconds < 0.0:
        raise ValueError("stop-after 和 settle-seconds 不能为负")
    backend = _backend(args)
    recorder = _recorder(args)
    backend.connect()
    try:
        initial = backend.state()
        index = ARM_JOINT_ORDER.index(args.joint)
        target = list(initial.position)
        target[index] += args.offset
        _validate_joint_target(args.joint, target[index])
        recorder.emit("software_stop_start", joint=args.joint,
                      initial_rad=initial.position[index], target_rad=target[index],
                      stop_after_s=args.stop_after,
                      warning="software stop is not a safety emergency stop")
        backend.command_joints(target, args.dynamics_factor)
        time.sleep(args.stop_after)
        was_active = backend.poll_motion()
        backend.stop()
        time.sleep(args.settle_seconds)
        stopped = backend.state()
        diagnostics = backend.diagnostics()
        recorder.emit("software_stop_complete", joint=args.joint,
                      actual_rad=stopped.position[index], was_active_before_stop=was_active,
                      diagnostics=diagnostics, errors=list(stopped.errors),
                      warning="external emergency stop must be verified separately")
        if args.backend == "franky" and not was_active:
            raise RuntimeError("调用 stop 前运动已经结束；本次不能证明软件 stop 能中断运动")
        if bool(diagnostics.get("is_in_control")):
            raise RuntimeError("stop 后机器人仍处于控制状态")
    finally:
        try:
            backend.stop()
        finally:
            backend.disconnect()
    return 0


def observe_safety(args: argparse.Namespace) -> int:
    """Observe an operator-triggered stop/error/disconnect without commanding motion."""
    if args.duration <= 0.0 or args.period <= 0.0:
        raise ValueError("duration 和 period 必须大于 0")
    backend = _backend(args)
    recorder = _recorder(args)
    backend.connect()
    deadline = time.monotonic() + args.duration
    detected = False
    last: dict[str, object] | None = None
    try:
        while time.monotonic() < deadline:
            try:
                raw = backend.state()
                diagnostics = backend.diagnostics()
            except Exception as exc:
                recorder.emit("safety_observation_connection_error", expected_event=args.event,
                              error_type=type(exc).__name__, error=str(exc))
                detected = args.event == "disconnect"
                break
            if diagnostics != last:
                recorder.emit("safety_observation", expected_event=args.event,
                              diagnostics=diagnostics, errors=list(raw.errors))
                last = diagnostics
            mode = str(diagnostics.get("robot_mode", ""))
            has_errors = bool(diagnostics.get("has_errors")) or bool(raw.errors)
            if args.event in ("user-stop", "external-estop"):
                detected = "UserStopped" in mode
            elif args.event == "fci-error":
                detected = has_errors or "Reflex" in mode
            if detected:
                break
            time.sleep(args.period)
    finally:
        backend.disconnect()
    recorder.emit("safety_observation_complete", expected_event=args.event,
                  detected=detected,
                  note="external-estop 与 user-stop 必须结合现场按钮、Desk 和视频记录判定")
    if not detected:
        raise TimeoutError(f"{args.duration} s 内未观察到 {args.event}")
    return 0


def recover(args: argparse.Namespace) -> int:
    _require_real_execute(args, "异常恢复")
    backend = _backend(args)
    recorder = _recorder(args)
    backend.connect()
    try:
        before = backend.diagnostics()
        result = backend.recover()
        after = backend.diagnostics()
        recorder.emit("recover", sdk_result=result, before=before, after=after)
        if not result or bool(after.get("has_errors")):
            raise RuntimeError("Franka 自动恢复失败或恢复后仍存在错误")
    finally:
        backend.disconnect()
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
                       ("safety-stop", safety_stop), ("observe-safety", observe_safety),
                       ("recover", recover), ("run", run_node)):
        q = sub.add_parser(name)
        q.set_defaults(func=func, backend="fake", ip="172.16.0.2", dynamics_factor=0.05,
                       max_step_rad=0.05, period=0.05, no_homing=False, margin=0.05,
                       only_read=False, dry_run=False, debug=False,
                       gripper_server_version=None)
        q.add_argument("--backend", choices=("fake", "franky"), help="默认 fake；真机显式选 franky")
        q.add_argument("--ip", help="FR3 FCI IP")
        q.add_argument("--dynamics-factor", type=float, help="Franky 全局 dynamics factor")
        q.add_argument("--max-step-rad", type=float, help="单个软件 waypoint 最大步长")
        q.add_argument("--period", type=float, help="状态/命令检查周期（秒）")
        q.add_argument("--debug", action="store_true")
        q.add_argument("--gripper-server-version", type=int,
                       help="可选 Franka Hand server version 检查；与 Robot Server wheel 版本不同")
        q.add_argument("--gripper-speed", type=float, default=0.03)
        q.add_argument("--gripper-force", type=float, default=50.0)
        q.add_argument("--log", help="可选 JSONL 验收日志路径")

    read = sub.choices["read-state"]
    read.add_argument("--samples", type=int, default=10)
    read.add_argument("--max-age", type=float, default=0.5, help="最大状态年龄（秒）")
    read.add_argument("--max-read-ms", type=float, default=500.0, help="单次 SDK 读取最大耗时")

    move = sub.choices["move-single-joint"]
    move.add_argument("joint", choices=ARM_JOINT_ORDER)
    move.add_argument("--offset", type=float, default=0.01)
    move.add_argument("--max-offset", type=float, default=0.05,
                      help="现场单关节测试允许的最大绝对偏移")
    move.add_argument("--timeout", type=float, default=10.0)
    move.add_argument("--tolerance", type=float, default=0.002)
    move.add_argument("--execute", action="store_true", help="确认现场安全条件并允许真机运动")

    sweep = sub.choices["joint-min-max-home"]
    sweep.add_argument("joint", choices=ARM_JOINT_ORDER,
                       help="强制每次只验收一个关节")
    sweep.add_argument("--acceptance-min", type=float, required=True,
                       help="现场批准的验收下界（rad），不是机械限位")
    sweep.add_argument("--acceptance-max", type=float, required=True,
                       help="现场批准的验收上界（rad），不是机械限位")
    sweep.add_argument("--margin", type=float, default=0.1,
                       help="验收目标与官方机械限位的最小间隔（rad）")
    sweep.add_argument("--home-rad", type=float, nargs=7, metavar=("Q1", "Q2", "Q3", "Q4", "Q5", "Q6", "Q7"),
                       help="现场批准的 7 轴 home；默认使用 contract.HOME_POSITION")
    sweep.add_argument("--timeout", type=float, default=30.0, help="每个 waypoint 超时")
    sweep.add_argument("--tolerance", type=float, default=0.003)
    sweep.add_argument("--retries", type=int, default=2,
                       help="每个 phase 失败后的额外尝试次数")
    sweep.add_argument("--retry-delay", type=float, default=1.0,
                       help="重建连接并重试前等待秒数")
    sweep.add_argument("--recover-on-retry", action=argparse.BooleanOptionalAction, default=True,
                       help="检测到当前 Franka 错误时，在重试前调用 automatic recovery（默认开启）")
    sweep.add_argument("--recovery-delay", type=float, default=1.0,
                       help="automatic recovery 后等待秒数")
    sweep.add_argument("--dry-run", action="store_true")
    sweep.add_argument("--execute", action="store_true", help="确认现场安全条件并允许真机运动")

    gripper = sub.choices["gripper-open-close"]
    gripper.add_argument("--width-tolerance", type=float, default=0.003)
    gripper.add_argument("--grasp-width", type=float,
                         help="可选夹持目标总开口（m）；提供后才执行 grasp")
    gripper.add_argument("--expect-grasp", choices=("none", "success", "failure"), default="none")
    gripper.add_argument("--hold-seconds", type=float, default=2.0)
    gripper.add_argument("--drop-threshold", type=float, default=0.002,
                         help="保持期间宽度漂移告警阈值（m）")
    gripper.add_argument("--execute", action="store_true", help="确认现场安全条件并允许真机夹爪运动")

    stop = sub.choices["safety-stop"]
    stop.add_argument("--joint", choices=ARM_JOINT_ORDER, default="fr3v2_joint1")
    stop.add_argument("--offset", type=float, default=0.03)
    stop.add_argument("--max-offset", type=float, default=0.05)
    stop.add_argument("--stop-after", type=float, default=0.2)
    stop.add_argument("--settle-seconds", type=float, default=0.5)
    stop.add_argument("--execute", action="store_true", help="确认现场安全条件并允许真机 stop 测试")

    observe = sub.choices["observe-safety"]
    observe.add_argument("--event", choices=("user-stop", "external-estop", "disconnect", "fci-error"),
                         required=True)
    observe.add_argument("--duration", type=float, default=30.0)

    recovery = sub.choices["recover"]
    recovery.add_argument("--execute", action="store_true",
                          help="确认故障源已移除并允许调用自动恢复")

    sub.choices["run"].add_argument("--no-homing", action="store_true",
                                    help="启动 Dora 节点时跳过 Hand homing")
    return p


def main(argv: list[str] | None = None) -> int:
    args = parser().parse_args(argv)
    return int(args.func(args))
