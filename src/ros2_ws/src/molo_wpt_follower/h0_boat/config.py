"""Config helpers for running H0 baseline on the real boat."""

from __future__ import annotations

import copy
from pathlib import Path
from typing import Any, Dict, Tuple

import yaml

PKG_DIR = Path(__file__).resolve().parent
MOLO_DIR = PKG_DIR.parent
MPC_DIR = MOLO_DIR / "mpc"

H0_SIM_CONFIG = (
    MPC_DIR / "experiments" / "results" / "lemniscate_validation" / "H0_baseline" / "config.yaml"
)
BOAT_OVERLAY = PKG_DIR / "config" / "h0_boat_overlay.yaml"
BENCH_OVERLAY = PKG_DIR / "config" / "h0_bench_overlay.yaml"
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


def build_h0_boat_config(
    overlay_path: Path | str | None = None,
    origin_latlon: Tuple[float, float] | None = None,
) -> dict:
    """Merge tuned H0 sim config with hardware overlay(s)."""
    if not H0_SIM_CONFIG.is_file():
        raise FileNotFoundError(f"H0 sim config not found: {H0_SIM_CONFIG}")

    cfg = load_yaml(H0_SIM_CONFIG)
    if BOAT_OVERLAY.is_file():
        cfg = deep_merge(cfg, load_yaml(BOAT_OVERLAY))

    if overlay_path is not None:
        extra = Path(overlay_path)
        if extra.is_file() and extra.resolve() != BOAT_OVERLAY.resolve():
            cfg = deep_merge(cfg, load_yaml(extra))

    if origin_latlon is not None:
        cfg.setdefault("origin", {})
        cfg["origin"]["lat"] = float(origin_latlon[0])
        cfg["origin"]["lon"] = float(origin_latlon[1])

    return cfg


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
