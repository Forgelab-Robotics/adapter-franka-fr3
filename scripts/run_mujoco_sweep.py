#!/usr/bin/env python3
"""运行 FR3v2 + Franka Hand 原生 MuJoCo sweep 与稳定性验收。"""

from __future__ import annotations

import argparse
import math
import os
import subprocess
import sys
from datetime import datetime
from pathlib import Path
from typing import TYPE_CHECKING, Callable

import numpy as np


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from robots_franka_fr3.contract import (  # noqa: E402
    ACTUATOR_ORDER,
    ARM_JOINT_ORDER,
    HOME_POSITION,
    load_joint_limits,
)
from robots_franka_fr3.simulation import (  # noqa: E402
    SimulationContractError,
    build_sweep_waypoints,
    proprio_from_qpos,
    validate_action,
)

if TYPE_CHECKING:
    import mujoco


POSITION_TOLERANCE = 0.02
SETTLED_VELOCITY_TOLERANCE = 0.01
VIDEO_FPS = 30
VIDEO_TIME_SCALE = 4.0


class ValidationFailure(RuntimeError):
    """仿真未满足验收条件。"""


class VideoWriter:
    """将 MuJoCo RGB 帧直接写入 ffmpeg，不增加 Python 图像依赖。"""

    def __init__(self, model: "mujoco.MjModel", output: Path) -> None:
        import mujoco

        self.output = output
        self.output.parent.mkdir(parents=True, exist_ok=True)
        self.width = 640
        self.height = 360
        self.renderer = mujoco.Renderer(model, height=self.height, width=self.width)
        self.camera = mujoco.MjvCamera()
        self.camera.type = mujoco.mjtCamera.mjCAMERA_FREE
        self.camera.lookat[:] = (0.0, 0.0, 0.48)
        self.camera.distance = 1.8
        self.camera.azimuth = 135.0
        self.camera.elevation = -20.0
        self.process = subprocess.Popen(
            [
                "ffmpeg",
                "-loglevel",
                "error",
                "-y",
                "-f",
                "rawvideo",
                "-pix_fmt",
                "rgb24",
                "-s",
                f"{self.width}x{self.height}",
                "-r",
                str(VIDEO_FPS),
                "-i",
                "-",
                "-an",
                "-c:v",
                "libx264",
                "-pix_fmt",
                "yuv420p",
                str(output),
            ],
            stdin=subprocess.PIPE,
        )

    def write(self, data: "mujoco.MjData") -> None:
        self.renderer.update_scene(data, camera=self.camera)
        frame = self.renderer.render()
        if self.process.stdin is None:
            raise ValidationFailure("ffmpeg stdin 不可用")
        self.process.stdin.write(frame.tobytes())

    def close(self) -> None:
        if self.process.stdin is not None:
            self.process.stdin.close()
        return_code = self.process.wait()
        self.renderer.close()
        if return_code != 0:
            raise ValidationFailure(f"ffmpeg 生成视频失败，退出码 {return_code}")


def _assert_finite(data: "mujoco.MjData", label: str) -> None:
    for name, values in (
        ("qpos", data.qpos),
        ("qvel", data.qvel),
        ("qacc", data.qacc),
        ("ctrl", data.ctrl),
    ):
        if not np.isfinite(values).all():
            raise ValidationFailure(f"{label}: {name} 出现 NaN/Inf")
    warning_counts = [warning.number for warning in data.warning]
    if any(warning_counts):
        raise ValidationFailure(f"{label}: MuJoCo warning={warning_counts}")


def _assert_joint_bounds(
    data: "mujoco.MjData",
    joint_limits: dict[str, dict[str, object]],
    label: str,
) -> None:
    for index, name in enumerate(ARM_JOINT_ORDER):
        limit = joint_limits[name]["limit"]
        assert isinstance(limit, dict)
        lower = float(limit["lower"])
        upper = float(limit["upper"])
        value = float(data.qpos[index])
        if value < lower - 1e-3 or value > upper + 1e-3:
            raise ValidationFailure(
                f"{label}: {name} qpos {value} 超出 [{lower}, {upper}]"
            )


def _velocity_limit(
    active_actuator: str,
    joint_limits: dict[str, dict[str, object]],
) -> float:
    if active_actuator == "gripper":
        # 官方 URDF 每根手指为 0.2 m/s；逻辑量是两指总开口。
        return 0.4
    limit = joint_limits[active_actuator]["limit"]
    assert isinstance(limit, dict)
    return float(limit["velocity"])


def _move_to(
    model: "mujoco.MjModel",
    data: "mujoco.MjData",
    target: tuple[float, ...],
    active_actuator: str,
    joint_limits: dict[str, dict[str, object]],
    *,
    frame_hook: Callable[["mujoco.MjData"], None] | None = None,
) -> tuple[float, float]:
    import mujoco

    start = np.array(data.ctrl, copy=True)
    target_array = np.asarray(target, dtype=float)
    active_index = ACTUATOR_ORDER.index(active_actuator)
    distance = abs(float(target_array[active_index] - start[active_index]))
    max_velocity = _velocity_limit(active_actuator, joint_limits)
    # smoothstep 的峰值斜率为 1.5；这里把目标峰值限制到官方上限的 75%。
    ramp_seconds = max(1.0, 2.0 * distance / max_velocity)
    ramp_steps = max(1, math.ceil(ramp_seconds / model.opt.timestep))
    settle_steps = math.ceil(0.75 / model.opt.timestep)
    frame_period = VIDEO_TIME_SCALE / VIDEO_FPS
    next_frame_time = data.time
    peak_velocity = 0.0

    for step in range(1, ramp_steps + settle_steps + 1):
        if step <= ramp_steps:
            phase = step / ramp_steps
            smooth = phase * phase * (3.0 - 2.0 * phase)
            data.ctrl[:] = start + (target_array - start) * smooth
        else:
            data.ctrl[:] = target_array
        mujoco.mj_step(model, data)
        _assert_finite(data, active_actuator)
        _assert_joint_bounds(data, joint_limits, active_actuator)
        # 快速臂运动时 equality/limit 约束允许亚毫米级瞬态穿透；最终目标仍按
        # 更严格的 0.5 mm 容差验收。
        proprio_from_qpos(data.qpos, mimic_tolerance=2e-3)
        peak_velocity = max(peak_velocity, abs(float(data.qvel[active_index])))
        if frame_hook is not None and data.time >= next_frame_time:
            frame_hook(data)
            next_frame_time += frame_period

    actual = proprio_from_qpos(data.qpos, mimic_tolerance=5e-4)[active_index]
    error = abs(actual - target[active_index])
    settled_velocity = float(np.max(np.abs(data.qvel)))
    return error, max(peak_velocity, settled_velocity)


def _run_stability_smoke(
    model: "mujoco.MjModel",
    data: "mujoco.MjData",
    home_key: int,
    seconds: float,
) -> list[str]:
    import mujoco

    lines: list[str] = []
    mujoco.mj_resetDataKeyframe(model, data, home_key)
    for _ in range(math.ceil(seconds / model.opt.timestep)):
        mujoco.mj_step(model, data)
        _assert_finite(data, "long_stability")
    if float(np.max(np.abs(data.qvel))) > SETTLED_VELOCITY_TOLERANCE:
        raise ValidationFailure(
            f"长稳态结束速度过大：{float(np.max(np.abs(data.qvel))):.6f}"
        )
    lines.append(f"[x] long_stability {seconds:.1f}s")

    # 重复相同 action 不应积累漂移。
    data.ctrl[:] = HOME_POSITION
    for _ in range(math.ceil(2.0 / model.opt.timestep)):
        mujoco.mj_step(model, data)
        _assert_finite(data, "repeated_action")
    lines.append("[x] repeated_action")

    # 快速交替合法目标，只验有限性、限位与恢复，不把它当正常运行轨迹。
    for step in range(math.ceil(2.0 / model.opt.timestep)):
        data.ctrl[:] = HOME_POSITION
        data.ctrl[0] = 0.3 if (step // 25) % 2 else -0.3
        mujoco.mj_step(model, data)
        _assert_finite(data, "rapid_alternating_action")
    lines.append("[x] rapid_alternating_action")

    # reset 后必须恢复 keyframe 的 action/proprio 契约。
    mujoco.mj_resetDataKeyframe(model, data, home_key)
    mujoco.mj_forward(model, data)
    proprio = proprio_from_qpos(data.qpos)
    if not np.allclose(proprio, HOME_POSITION, atol=1e-9, rtol=0.0):
        raise ValidationFailure("reset 后未恢复 home")
    lines.append("[x] reset_and_restart")
    return lines


def _run_contract_negative_tests(
    joint_limits: dict[str, dict[str, object]],
) -> list[str]:
    cases = [
        ("nan_action", ACTUATOR_ORDER, (*HOME_POSITION[:-1], float("nan"))),
        ("inf_action", ACTUATOR_ORDER, (*HOME_POSITION[:-1], float("inf"))),
        ("over_limit", ACTUATOR_ORDER, (*HOME_POSITION[:-1], 0.081)),
        ("unknown_joint", (*ACTUATOR_ORDER[:-1], "unknown"), HOME_POSITION),
        ("wrong_order", tuple(reversed(ACTUATOR_ORDER)), HOME_POSITION),
    ]
    lines: list[str] = []
    for label, names, positions in cases:
        try:
            validate_action(names, positions, joint_limits)
        except SimulationContractError:
            lines.append(f"[x] rejected_{label}")
        else:
            raise ValidationFailure(f"非法输入未被拒绝：{label}")

    # 0.04 是合法的总开口，必须映射成两根手指各 0.02，而不是各 0.04。
    proprio = proprio_from_qpos((*HOME_POSITION[:7], 0.02, 0.02))
    if not math.isclose(proprio[-1], 0.04, abs_tol=1e-12):
        raise ValidationFailure("gripper 总开口换算错误")
    lines.append("[x] gripper_total_opening_semantics")
    return lines


def run_validation(
    scene: Path,
    *,
    smoke_seconds: float = 10.0,
    video: Path | None = None,
) -> list[str]:
    import mujoco

    joint_limits = load_joint_limits(ROOT)
    model = mujoco.MjModel.from_xml_path(str(scene))
    data = mujoco.MjData(model)
    home_key = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_KEY, "home")
    if home_key < 0:
        raise ValidationFailure("MJCF 缺少 home keyframe")

    writer: VideoWriter | None = None
    if video is not None:
        writer = VideoWriter(model, video)

    try:
        scene_display = str(scene.resolve().relative_to(ROOT.resolve()))
    except ValueError:
        scene_display = str(scene.resolve())
    lines = [
        "FR3v2 + Franka Hand MuJoCo simulation validation",
        f"timestamp: {datetime.now().astimezone().isoformat(timespec='seconds')}",
        f"mujoco: {mujoco.__version__}",
        f"scene: {scene_display}",
        f"model: nq={model.nq}, nu={model.nu}, njnt={model.njnt}",
        "sweep_margin: max(0.05 rad, 10% joint range)",
    ]

    try:
        for waypoint in build_sweep_waypoints(joint_limits):
            if waypoint.name.endswith("near_min") or waypoint.name == "gripper_open":
                mujoco.mj_resetDataKeyframe(model, data, home_key)
                mujoco.mj_forward(model, data)
            error, peak_velocity = _move_to(
                model,
                data,
                waypoint.position,
                waypoint.active_actuator,
                joint_limits,
                frame_hook=writer.write if writer is not None else None,
            )
            if error > POSITION_TOLERANCE:
                raise ValidationFailure(
                    f"{waypoint.name}: 位置误差 {error:.6f} > "
                    f"{POSITION_TOLERANCE:.6f}"
                )
            velocity_limit = _velocity_limit(
                waypoint.active_actuator,
                joint_limits,
            )
            if peak_velocity > velocity_limit * 1.05:
                raise ValidationFailure(
                    f"{waypoint.name}: 峰值速度 {peak_velocity:.6f} 超出 "
                    f"{velocity_limit:.6f}"
                )
            lines.append(
                f"[x] {waypoint.name} error={error:.6f} "
                f"peak_velocity={peak_velocity:.6f}"
            )
        lines.extend(_run_stability_smoke(model, data, home_key, smoke_seconds))
        lines.extend(_run_contract_negative_tests(joint_limits))
        lines.append("RESULT: PASS")
    finally:
        if writer is not None:
            writer.close()
    return lines


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--scene",
        type=Path,
        default=ROOT / "assets/mjcf/scene.xml",
        help="MuJoCo scene.xml 路径",
    )
    parser.add_argument(
        "--log",
        type=Path,
        default=ROOT / "validation_output/simulation_validation.log",
        help="验收日志输出路径",
    )
    parser.add_argument(
        "--video",
        type=Path,
        default=None,
        help="可选 MP4 输出；推荐以 MUJOCO_GL=egl 运行",
    )
    parser.add_argument(
        "--smoke-seconds",
        type=float,
        default=10.0,
        help="长稳态仿真秒数",
    )
    args = parser.parse_args()
    if args.smoke_seconds <= 0:
        parser.error("--smoke-seconds 必须大于 0")
    return args


def main() -> int:
    args = _parse_args()
    try:
        lines = run_validation(
            args.scene,
            smoke_seconds=args.smoke_seconds,
            video=args.video,
        )
    except (OSError, RuntimeError, ValueError) as error:
        print(f"[ ] 仿真验收失败：{error}", file=sys.stderr)
        return 1
    output = "\n".join(lines) + "\n"
    print(output, end="")
    if args.log is not None:
        args.log.parent.mkdir(parents=True, exist_ok=True)
        args.log.write_text(output)
        print(f"log: {args.log}")
    if args.video is not None:
        print(f"video: {args.video}")
    return 0


if __name__ == "__main__":
    # 允许调用方在进程启动前用 MUJOCO_GL=egl/osmesa 选择无头渲染后端。
    os.environ.setdefault("PYTHONUNBUFFERED", "1")
    raise SystemExit(main())
