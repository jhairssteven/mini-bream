"""Fixed lemniscate reference geometry for fair BO and re-validation."""

from __future__ import annotations

import copy
from pathlib import Path
from typing import Any, Dict

import yaml

# Validated clean figure-8 (H9_slow_tight): 77 samples, no spurious Dubins loops.
DEFAULT_REFERENCE: Dict[str, Any] = {
    "waypoints": {
        "relative_to_start": True,
        "trajectory": {
            "type": "lemniscate",
            "scale_m": 7.0,
            "num_points": 32,
        },
    },
    "path": {
        "resample_step_m": 0.3958010892129231,
        "closed": True,
        "smooth_dubins": True,
        "monotonic_progress": False,
    },
    "dubins": {
        "turning_radius_m": 5.705838163404552,
        "step_size_m": 1.25,
    },
}


def reference_path_block(cfg_path: Path | None = None) -> Dict[str, Any]:
    if cfg_path is not None and cfg_path.is_file():
        cfg = yaml.safe_load(cfg_path.read_text(encoding="utf-8"))
        return {
            "waypoints": {
                "relative_to_start": True,
                "trajectory": copy.deepcopy(cfg["waypoints"]["trajectory"]),
            },
            "path": {
                k: copy.deepcopy(v)
                for k, v in cfg["path"].items()
                if k != "cruise_speed_mps"
            },
            "dubins": copy.deepcopy(cfg["dubins"]),
        }
    return copy.deepcopy(DEFAULT_REFERENCE)


def apply_reference_path(cfg: dict, ref: Dict[str, Any] | None = None) -> dict:
    """Return cfg with shared lemniscate geometry; cruise speed is preserved."""
    ref = ref or DEFAULT_REFERENCE
    out = copy.deepcopy(cfg)
    out.setdefault("waypoints", {}).update(copy.deepcopy(ref["waypoints"]))
    out.setdefault("path", {}).update(copy.deepcopy(ref["path"]))
    out["dubins"] = copy.deepcopy(ref["dubins"])
    return out
