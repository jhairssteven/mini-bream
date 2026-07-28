"""Config helpers for the H0 lemniscate experiment across platform profiles."""

from __future__ import annotations

import copy
from pathlib import Path
from typing import Any, Dict, List, Tuple

import yaml

PKG_DIR = Path(__file__).resolve().parent
MOLO_DIR = PKG_DIR.parent
MPC_DIR = MOLO_DIR / "mpc"
CONFIG_DIR = PKG_DIR / "config"

H0_SIM_CONFIG = (
    MPC_DIR / "experiments" / "results" / "lemniscate_validation" / "H0_baseline" / "config.yaml"
)
BOAT_OVERLAY = CONFIG_DIR / "h0_boat_overlay.yaml"
SIM_OVERLAY = CONFIG_DIR / "h0_sim_hal_overlay.yaml"
MOCK_OVERLAY = CONFIG_DIR / "h0_mock_overlay.yaml"
BENCH_OVERLAY = CONFIG_DIR / "h0_bench_overlay.yaml"
BLUEBOAT_BOAT_OVERLAY = CONFIG_DIR / "h0_blueboat_boat.yaml"
TUNED_OVERLAY = CONFIG_DIR / "h0_tuned_overlay.yaml"

# Platform profile → ordered overlay list (algorithm config is always H0_SIM_CONFIG).
PLATFORM_PROFILES: Dict[str, List[Path]] = {
    "real": [BOAT_OVERLAY, BLUEBOAT_BOAT_OVERLAY],
    "sim": [SIM_OVERLAY, BLUEBOAT_BOAT_OVERLAY],
    "mock": [BOAT_OVERLAY, BLUEBOAT_BOAT_OVERLAY, MOCK_OVERLAY],
    "bench": [BOAT_OVERLAY, BLUEBOAT_BOAT_OVERLAY, BENCH_OVERLAY],
}

FIELD_TESTS_RESULTS = Path("/workspace/field_tests/h0_boat")
DEFAULT_RESULTS = (
    FIELD_TESTS_RESULTS if FIELD_TESTS_RESULTS.parent.is_dir() else PKG_DIR / "results"
)


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


def platform_overlays(platform: str) -> List[Path]:
    """Return overlay files for a named platform profile."""
    key = platform.lower().strip()
    if key not in PLATFORM_PROFILES:
        valid = ", ".join(sorted(PLATFORM_PROFILES))
        raise ValueError(f"unknown platform {platform!r}; expected one of: {valid}")
    return PLATFORM_PROFILES[key]


def build_h0_boat_config(
    platform: str = "real",
    overlay_path: Path | str | None = None,
    origin_latlon: Tuple[float, float] | None = None,
) -> dict:
    """Merge tuned H0 algorithm config with a platform profile and optional extra overlay."""
    if not H0_SIM_CONFIG.is_file():
        raise FileNotFoundError(f"H0 sim config not found: {H0_SIM_CONFIG}")

    cfg = load_yaml(H0_SIM_CONFIG)
    applied: List[Path] = []
    for overlay in platform_overlays(platform):
        if overlay.is_file():
            cfg = deep_merge(cfg, load_yaml(overlay))
            applied.append(overlay)

    if TUNED_OVERLAY.is_file():
        cfg = deep_merge(cfg, load_yaml(TUNED_OVERLAY))
        applied.append(TUNED_OVERLAY)

    if overlay_path is not None:
        extra = Path(overlay_path)
        if extra.is_file() and all(extra.resolve() != p.resolve() for p in applied):
            cfg = deep_merge(cfg, load_yaml(extra))

    if origin_latlon is not None:
        cfg.setdefault("origin", {})
        cfg["origin"]["lat"] = float(origin_latlon[0])
        cfg["origin"]["lon"] = float(origin_latlon[1])

    return cfg


def platform_name(cfg: dict, fallback: str = "unknown") -> str:
    """Human-readable platform label stored in experiment metadata."""
    plat = cfg.get("platform", {})
    if isinstance(plat, dict) and plat.get("name"):
        return str(plat["name"])
    return fallback


def prepare_run_config(
    cfg: dict,
    run_dir: Path,
    log_csv_name: str = "log.csv",
) -> dict:
    """Write experiment artifacts config for a single boat run."""
    out = copy.deepcopy(cfg)
    out.setdefault("experiment", {})
    out["experiment"]["log_csv"] = str(run_dir / log_csv_name)
    run_dir.mkdir(parents=True, exist_ok=True)
    with open(run_dir / "config.yaml", "w", encoding="utf-8") as f:
        yaml.safe_dump(out, f, sort_keys=False)
    return out


def bridge_topics(cfg: dict) -> Dict[str, Any]:
    return cfg.get("boat_bridge", {})


def stack_components(cfg: dict) -> Dict[str, bool]:
    """Which helper nodes stack_runner should start for this platform."""
    bridge = bridge_topics(cfg)
    sim = bool(cfg.get("sim_enable", False))
    return {
        "velocity_odom": bool(bridge.get("velocity_odom_enabled", not sim)),
    }
