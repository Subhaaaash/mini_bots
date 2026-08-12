"""Loads config/robot.yaml and builds the camera/motor/behavior/kinematics/
governor objects it describes, via the small registries in each package's
__init__.py. This is what makes hardware/behavior selection a YAML edit
instead of a code change.
"""

import inspect
from pathlib import Path
from typing import Any

import yaml

from robot_core.behaviors import BEHAVIOR_REGISTRY, Behavior
from robot_core.camera import CAMERA_REGISTRY, CameraSource
from robot_core.drive_command import DifferentialDriveKinematics
from robot_core.motors import MOTOR_DRIVER_REGISTRY, MotorDriver
from robot_core.safety.governor import SafetyGovernor


def load_config(path: str) -> dict:
    with open(path) as f:
        return yaml.safe_load(f)


def _build_from_registry(registry: dict, cfg: dict):
    """Instantiate registry[cfg['type']], passing through only the config
    keys the class's __init__ actually accepts (so one shared robot.yaml
    section, e.g. camera:, can carry keys for several possible `type`s
    without each concrete class choking on the extras).
    """
    cfg = dict(cfg)
    type_name = cfg.pop("type")
    if type_name not in registry:
        raise ValueError(f"Unknown type {type_name!r}. Available: {sorted(registry)}")
    cls = registry[type_name]
    valid_params = {
        name for name, p in inspect.signature(cls.__init__).parameters.items()
        if name != "self" and p.kind != inspect.Parameter.VAR_KEYWORD
    }
    kwargs = {k: v for k, v in cfg.items() if k in valid_params}
    return cls(**kwargs)


def build_camera(cfg: dict) -> CameraSource:
    return _build_from_registry(CAMERA_REGISTRY, cfg)


def build_motor_driver(cfg: dict) -> MotorDriver:
    return _build_from_registry(MOTOR_DRIVER_REGISTRY, cfg)


def build_behavior(cfg: dict) -> Behavior:
    cfg = dict(cfg)
    type_name = cfg.pop("type")
    if type_name not in BEHAVIOR_REGISTRY:
        raise ValueError(f"Unknown behavior type {type_name!r}. Available: {sorted(BEHAVIOR_REGISTRY)}")
    cls = BEHAVIOR_REGISTRY[type_name]
    extra_params = cfg.pop("params", {})
    return cls(**cfg, **extra_params)


def build_kinematics(chassis_cfg: dict) -> DifferentialDriveKinematics:
    return DifferentialDriveKinematics(min_effective_speed=chassis_cfg.get("min_effective_speed", 0.0))


def build_governor(safety_cfg: dict) -> SafetyGovernor:
    return SafetyGovernor(
        max_lost_frames=safety_cfg.get("max_lost_frames", 10),
        manual_timeout=safety_cfg.get("manual_timeout", 0.4),
    )
