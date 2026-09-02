"""Franka FR3v2 + Franka Hand 适配包。"""

from robots_franka_fr3.contract import (
    ACTUATOR_ORDER,
    GRIPPER_MAX_WIDTH_M,
    HOME_POSITION,
    RESET_POSITION,
    load_joint_limits,
)
from robots_franka_fr3.driver import FrankaFR3Driver
from robots_franka_fr3.backend import BackendState, FakeBackend, FrankyBackend

__all__ = [
    "ACTUATOR_ORDER",
    "GRIPPER_MAX_WIDTH_M",
    "HOME_POSITION",
    "RESET_POSITION",
    "load_joint_limits",
    "FrankaFR3Driver",
    "BackendState",
    "FakeBackend",
    "FrankyBackend",
]

__version__ = "0.1.0"
