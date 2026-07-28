"""Config builder for the path-follower interface (pluggable controllers)."""

from __future__ import annotations

import copy
from pathlib import Path
from typing import Dict, List, Tuple

import yaml

PKG_DIR = Path(__file__).resolve().parent
MOLO_DIR = PKG_DIR.parent
CONFIG_DIR = PKG_DIR / "config"

PATH_TOPIC_OVERLAY = CONFIG_DIR / "path_topic_overlay.yaml"

CONTROLLERS: Dict[str, str] = {
    "ilos": "ilos_boat",
    "h0": "h0_boat",
    "mpc": "h0_boat",
}


def load_yaml(path: Path | str) -> dict:
    with open(path, "r", encoding="utf-8") as f:
        return yaml.safe_load(f) or {}


def deep_merge(base: dict, patch: dict) -> dict:
    out = copy.deepcopy(base)
    for key, value in patch.items():
        if isinstance(value, dict) and isinstance(out.get(key), dict):
            out[key] = deep_merge(out[key], value)
        else:
            out[key] = copy.deepcopy(value)
    return out


def build_path_follower_config(
    controller: str = "ilos",
    platform: str = "sim",
    *,
    use_external_path: bool = True,
    overlay_path: Path | str | None = None,
    origin_latlon: Tuple[float, float] | None = None,
) -> dict:
    """Build a merged controller config with optional external-path overlay."""
    key = controller.lower().strip()
    if key not in CONTROLLERS:
        valid = ", ".join(sorted(CONTROLLERS))
        raise ValueError(f"unknown controller {controller!r}; expected one of: {valid}")

    if key == "ilos":
        from ilos_boat.config import build_ilos_boat_config

        cfg = build_ilos_boat_config(platform, origin_latlon=origin_latlon)
    else:
        from h0_boat.config import build_h0_boat_config

        cfg = build_h0_boat_config(platform, origin_latlon=origin_latlon)

    if use_external_path and PATH_TOPIC_OVERLAY.is_file():
        cfg = deep_merge(cfg, load_yaml(PATH_TOPIC_OVERLAY))

    if overlay_path is not None:
        extra = Path(overlay_path)
        if extra.is_file():
            cfg = deep_merge(cfg, load_yaml(extra))

    return cfg


def controller_stack_module(controller: str) -> str:
    return CONTROLLERS[controller.lower().strip()]
