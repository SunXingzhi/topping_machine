"""YAML 配置加载。"""

from __future__ import annotations

import math
from pathlib import Path

import yaml

from .geometry import DeltaGeometry


def load_geometry(path: str | Path) -> DeltaGeometry:
    with open(path, "r", encoding="utf-8") as stream:
        cfg = yaml.safe_load(stream) or {}

    phi_deg = cfg.get("phi_deg", [0.0, 120.0, 240.0])
    return DeltaGeometry(
        R=float(cfg.get("R", 0.139)),
        r=float(cfg.get("r", 0.039)),
        L=float(cfg.get("L", 0.140)),
        l=float(cfg.get("l", 0.200)),
        phi=tuple(math.radians(float(x)) for x in phi_deg),
        theta_min=math.radians(float(cfg.get("theta_min_deg", -180.0))),
        theta_max=math.radians(float(cfg.get("theta_max_deg", 180.0))),
        platform_offset=float(cfg.get("platform_offset", 0.0)),
        ball_tilt=math.radians(float(cfg.get("ball_tilt_deg", 0.0))),
    )


def load_constraints(path: str | Path) -> tuple[str, bool]:
    with open(path, "r", encoding="utf-8") as stream:
        cfg = yaml.safe_load(stream) or {}

    mode = str(cfg.get("mode", "shadow"))
    c0_enabled = bool(cfg.get("c0_reachability", {}).get("enabled", True))
    return mode, c0_enabled
