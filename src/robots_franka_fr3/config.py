"""Franka FR3 配置加载：robot YAML（连接/安全参数）→ driver。"""

from __future__ import annotations

import os
from pathlib import Path
from typing import Any, Mapping

import yaml

from robots_franka_fr3.backend import DynamicsFactors, FakeBackend, FrankyBackend
from robots_franka_fr3.driver import FrankaFR3Driver


def _expand_env(value: Any) -> Any:
    """递归展开字符串值中的 ${VAR}（未知变量保持原样）。"""
    if isinstance(value, str):
        return os.path.expandvars(value)
    if isinstance(value, list):
        return [_expand_env(item) for item in value]
    if isinstance(value, dict):
        return {key: _expand_env(item) for key, item in value.items()}
    return value


def load_config(path: str | Path) -> dict[str, Any]:
    """读取 robot YAML；顶层必须是 mapping。字符串值支持 ${ENV_VAR} 展开。"""
    config = yaml.safe_load(Path(path).read_text()) or {}
    if not isinstance(config, dict):
        raise ValueError("robot config 顶层必须是 mapping")
    return _expand_env(config)


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
