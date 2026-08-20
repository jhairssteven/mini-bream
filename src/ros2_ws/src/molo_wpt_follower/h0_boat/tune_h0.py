#!/usr/bin/env python3
"""
Bayesian optimization of H0 baseline_ilos_velocity on BlueBoat (sim or field).

Tunes MPC / ILOS / PID / guidance parameters for the lemniscate mission while
keeping control.approach fixed. Uses scikit-optimize (gp_minimize, EI acquisition),
matching the methodology in mpc/experiments/tune_lemniscate.py.

Examples:
  python3 tune_h0.py --platform sim
  python3 tune_h0.py --platform real --n-calls 20
  python3 tune_h0.py --platform sim --quick
"""

from __future__ import annotations

import argparse
import copy
import json
import math
import subprocess
import sys
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Dict, List

import yaml

PKG_DIR = Path(__file__).resolve().parent
MOLO_DIR = PKG_DIR.parent
MPC_DIR = MOLO_DIR / "mpc"
EXP_DIR = MPC_DIR / "experiments"

for path in (str(PKG_DIR), str(MPC_DIR), str(EXP_DIR)):
    if path not in sys.path:
        sys.path.insert(0, path)

from config import (  # noqa: E402
    H0_SIM_CONFIG,
    TUNED_OVERLAY,
    build_h0_boat_config,
    deep_merge,
    platform_name,
)
from reference_path import apply_reference_path, reference_path_block  # noqa: E402
from trial_runner import bash_cmd, kill_stack, run_boat_trial  # noqa: E402

OPEN_WATER_LAUNCH = (
    "/workspace/ros2_ws/install/blueboat_sim/share/blueboat_sim/launch/open_water.launch.py"
)
TARGET_RMSE_M = 0.1
REF_CONFIG = (
    MPC_DIR / "experiments" / "results" / "tune_lemniscate_full" / "H9_slow_tight" / "best_config.yaml"
)

# Timing presets per platform
TIMING = {
    "sim": {"warmup_s": 5.0, "evaluate_s": 45.0, "skip_initial_s": 5.0},
    "real": {"warmup_s": 20.0, "evaluate_s": 55.0, "skip_initial_s": 20.0},
    "bench": {"warmup_s": 8.0, "evaluate_s": 30.0, "skip_initial_s": 8.0},
}

VALIDATION_TIMING = {
    "sim": {"warmup_s": 5.0, "evaluate_s": 60.0, "skip_initial_s": 5.0},
    "real": {"warmup_s": 20.0, "evaluate_s": 55.0, "skip_initial_s": 20.0},
    "bench": {"warmup_s": 8.0, "evaluate_s": 30.0, "skip_initial_s": 8.0},
}


def build_h0_refine_space(center: Dict[str, Any], fraction: float = 0.35):
    """Local search space around a known good parameter set."""
    from skopt.space import Integer, Real

    def band(val, lo_scale, hi_scale, int_val=False):
        if int_val:
            v = int(val)
            return Integer(max(1, int(v * lo_scale)), max(v + 1, int(v * hi_scale)), name="")
        return Real(val * lo_scale, val * hi_scale, name="")

    names = [
        "cruise", "ilos_min", "ilos_max", "ilos_conv", "pid_kp", "pid_kd",
        "max_yaw_r", "lateral_v", "horizon", "Q_u", "Q_v", "Q_r", "R_in", "Q_term",
    ]
    space = []
    for name in names:
        val = center[name]
        if name == "horizon":
            space.append(Integer(max(8, int(val) - 4), int(val) + 4, name=name))
        elif name in ("ilos_min", "ilos_max"):
            lo = max(0.2, float(val) * (1.0 - fraction))
            hi = float(val) * (1.0 + fraction) + 0.1
            space.append(Real(lo, hi, name=name))
        elif name == "R_in":
            space.append(Real(max(0.003, float(val) * 0.5), float(val) * 2.0, name=name))
        else:
            lo = max(1e-3, float(val) * (1.0 - fraction))
            hi = float(val) * (1.0 + fraction)
            space.append(Real(lo, hi, name=name))
    return space


def build_h0_search_space():
    """Focused search space for baseline_ilos_velocity (14 parameters)."""
    from skopt.space import Integer, Real

    return [
        Real(0.10, 0.40, name="cruise"),
        Real(0.35, 1.2, name="ilos_min"),
        Real(0.8, 2.8, name="ilos_max"),
        Real(5.0, 14.0, name="ilos_conv"),
        Real(0.8, 2.8, name="pid_kp"),
        Real(0.10, 1.0, name="pid_kd"),
        Real(0.35, 1.0, name="max_yaw_r"),
        Real(0.0, 1.2, name="lateral_v"),
        Integer(10, 22, name="horizon"),
        Real(8.0, 60.0, name="Q_u"),
        Real(0.3, 20.0, name="Q_v"),
        Real(40.0, 320.0, name="Q_r"),
        Real(0.005, 0.06, name="R_in"),
        Real(3.0, 14.0, name="Q_term"),
    ]


def vector_to_dict(space, x) -> Dict[str, Any]:
    return {d.name: (int(v) if d.name == "horizon" else float(v)) for d, v in zip(space, x)}


def apply_h0_params(cfg: dict, p: Dict[str, Any]) -> dict:
    """Apply BO parameters to H0 baseline_ilos_velocity config."""
    c = copy.deepcopy(cfg)
    c.setdefault("path", {})["cruise_speed_mps"] = p["cruise"]
    c.setdefault("ilos", {}).update(
        {
            "lookahead_min_m": p["ilos_min"],
            "lookahead_max_m": max(p["ilos_max"], p["ilos_min"] + 0.15),
            "conv_rate": p["ilos_conv"],
        }
    )
    c.setdefault("pid", {}).update({"kp": p["pid_kp"], "kd": p["pid_kd"]})
    c.setdefault("guidance", {})["max_yaw_rate_rad_s"] = p["max_yaw_r"]
    c.setdefault("control", {}).update(
        {
            "approach": "baseline_ilos_velocity",
            "lateral_v_gain": p["lateral_v"],
        }
    )
    c.setdefault("mpc", {}).update(
        {
            "mode": "velocity",
            "horizon": int(p["horizon"]),
            "Q_vel_diag": [p["Q_u"], p["Q_v"], p["Q_r"]],
            "R_diag": [p["R_in"], p["R_in"]],
            "Q_terminal_scale": p["Q_term"],
        }
    )
    return c


def extract_tuned_overlay(cfg: dict) -> dict:
    """Algorithm-only patch suitable for h0_tuned_overlay.yaml."""
    return {
        "path": {"cruise_speed_mps": cfg.get("path", {}).get("cruise_speed_mps")},
        "ilos": {
            k: cfg.get("ilos", {}).get(k)
            for k in ("lookahead_min_m", "lookahead_max_m", "conv_rate")
        },
        "pid": {k: cfg.get("pid", {}).get(k) for k in ("kp", "kd")},
        "guidance": {"max_yaw_rate_rad_s": cfg.get("guidance", {}).get("max_yaw_rate_rad_s")},
        "control": {
            "approach": "baseline_ilos_velocity",
            "lateral_v_gain": cfg.get("control", {}).get("lateral_v_gain"),
        },
        "mpc": {
            k: cfg.get("mpc", {}).get(k)
            for k in ("mode", "horizon", "Q_vel_diag", "R_diag", "Q_terminal_scale")
        },
        "tuning": {
            "platform": cfg.get("platform", {}).get("name"),
            "target_rmse_m": TARGET_RMSE_M,
        },
    }


def gazebo_ready() -> bool:
    return (
        subprocess.run(
            bash_cmd(
                "timeout 4 ros2 topic echo /blueboat/sensors/gps/gps/fix --once 2>/dev/null | grep -q latitude"
            ),
            capture_output=True,
        ).returncode
        == 0
    )


def kill_gazebo() -> None:
    subprocess.run(
        bash_cmd(
            "pkill -f 'h0_boat/stack_runner.py' 2>/dev/null; "
            "pkill -f 'gz sim' 2>/dev/null; "
            "pkill -f 'ros2 launch blueboat_sim' 2>/dev/null; sleep 2; true"
        ),
        capture_output=True,
    )


def ensure_gazebo(launch: str = OPEN_WATER_LAUNCH) -> bool:
    if gazebo_ready():
        return True
    kill_gazebo()
    subprocess.Popen(
        bash_cmd(f"ros2 launch {launch} headless:=True"),
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
    )
    for _ in range(45):
        time.sleep(2)
        if gazebo_ready():
            time.sleep(5)
            return True
    return False


def base_config(platform: str, overlay_path: Path | str | None = None) -> dict:
    cfg = build_h0_boat_config(platform=platform, overlay_path=overlay_path)
    ref_block = reference_path_block(REF_CONFIG if REF_CONFIG.is_file() else None)
    cfg = apply_reference_path(cfg, ref_block)
    cfg.setdefault("control", {})["approach"] = "baseline_ilos_velocity"
    return cfg


def run_tuning(
    platform: str,
    out_dir: Path,
    n_calls: int,
    launch: str,
    restart_sim: bool,
    install_overlay: bool,
    refine_from: Path | None = None,
    overlay_path: Path | str | None = None,
) -> Dict[str, Any]:
    try:
        from skopt import gp_minimize
    except ImportError as exc:
        raise SystemExit("Install scikit-optimize: pip install scikit-optimize") from exc

    timing = TIMING.get(platform, TIMING["sim"])
    val_timing = VALIDATION_TIMING.get(platform, VALIDATION_TIMING["sim"])

    cfg_base = base_config(platform, overlay_path=overlay_path)
    if refine_from is not None and refine_from.is_file():
        with open(refine_from, encoding="utf-8") as f:
            prev = json.load(f)
        center = prev.get("best_params", {})
        space = build_h0_refine_space(center)
        print(
            f"Refining around prior best (rmse={float(prev.get('validation_rmse', float('nan'))):.4f} m)",
            flush=True,
        )
    else:
        space = build_h0_search_space()
    history: List[dict] = []
    trials_dir = out_dir / "trials"
    trials_dir.mkdir(parents=True, exist_ok=True)

    ref_block = reference_path_block(REF_CONFIG if REF_CONFIG.is_file() else None)
    with open(out_dir / "reference_path.yaml", "w", encoding="utf-8") as f:
        yaml.safe_dump(ref_block, f, sort_keys=False)

    if platform == "sim":
        kill_gazebo()
        if not ensure_gazebo(launch):
            raise RuntimeError("Gazebo / blueboat_sim failed to start")

    def objective(x):
        p = vector_to_dict(space, x)
        cfg = apply_reference_path(apply_h0_params(cfg_base, p), ref_block)
        trial_dir = trials_dir / f"trial_{len(history):04d}"
        trial_dir.mkdir(parents=True, exist_ok=True)
        with open(trial_dir / "params.json", "w", encoding="utf-8") as f:
            json.dump(p, f, indent=2)

        if restart_sim and platform == "sim":
            kill_gazebo()
            time.sleep(2)
            if not ensure_gazebo(launch):
                return 1e3

        score = run_boat_trial(
            cfg,
            trial_dir,
            warmup_s=timing["warmup_s"],
            evaluate_s=timing["evaluate_s"],
            skip_initial_s=timing["skip_initial_s"],
            wait_gps_timeout_s=30.0 if platform == "sim" else 60.0,
            wait_xte_timeout_s=30.0 if platform == "sim" else 45.0,
        )
        history.append({"params": p, "rmse": score, "trial_dir": str(trial_dir)})
        print(f"  trial {len(history):3d}  rmse={score:.4f} m", flush=True)

        if platform == "sim" and not gazebo_ready():
            ensure_gazebo(launch)
        return score if math.isfinite(score) else 1e3

    print(
        f"\nH0 Bayesian tuning  platform={platform} ({platform_name(cfg_base)})\n"
        f"  n_calls={n_calls}  target_rmse={TARGET_RMSE_M} m\n"
        f"  output={out_dir}\n",
        flush=True,
    )

    res = gp_minimize(
        objective,
        space,
        n_calls=n_calls,
        n_initial_points=min(8, max(3, n_calls // 4)),
        random_state=42,
        acq_func="EI",
    )

    best_p = vector_to_dict(space, res.x)
    best_cfg = apply_reference_path(apply_h0_params(cfg_base, best_p), ref_block)

    val_dir = out_dir / "best_validation"
    val_dir.mkdir(parents=True, exist_ok=True)
    if platform == "sim" and restart_sim:
        kill_gazebo()
        time.sleep(2)
        ensure_gazebo(launch)
    val_rmse = run_boat_trial(
        best_cfg,
        val_dir,
        warmup_s=val_timing["warmup_s"],
        evaluate_s=val_timing["evaluate_s"],
        skip_initial_s=val_timing["skip_initial_s"],
        wait_gps_timeout_s=30.0 if platform == "sim" else 60.0,
        wait_xte_timeout_s=30.0 if platform == "sim" else 45.0,
    )

    with open(out_dir / "best_config.yaml", "w", encoding="utf-8") as f:
        yaml.safe_dump(best_cfg, f, sort_keys=False)

    tuned_patch = extract_tuned_overlay(best_cfg)
    tuned_patch["tuning"].update(
        {
            "bo_best_rmse": float(res.fun),
            "validation_rmse": float(val_rmse),
            "n_calls": n_calls,
            "timestamp": datetime.now(timezone.utc).isoformat(),
        }
    )
    with open(out_dir / "h0_tuned_overlay.yaml", "w", encoding="utf-8") as f:
        yaml.safe_dump(tuned_patch, f, sort_keys=False)

    if install_overlay:
        with open(TUNED_OVERLAY, "w", encoding="utf-8") as f:
            yaml.safe_dump(tuned_patch, f, sort_keys=False)
        print(f"Installed tuned overlay → {TUNED_OVERLAY}", flush=True)

    result = {
        "hypothesis_id": "H0_baseline",
        "platform": platform_name(cfg_base),
        "platform_profile": platform,
        "target_rmse_m": TARGET_RMSE_M,
        "bo_best_rmse": float(res.fun),
        "validation_rmse": float(val_rmse),
        "pass": math.isfinite(val_rmse) and val_rmse < TARGET_RMSE_M,
        "n_calls": n_calls,
        "best_params": best_p,
        "source_config": str(H0_SIM_CONFIG),
    }
    with open(out_dir / "tune_result.json", "w", encoding="utf-8") as f:
        json.dump(result, f, indent=2)
    with open(out_dir / "history.jsonl", "w", encoding="utf-8") as f:
        for h in history:
            f.write(json.dumps({"rmse": h["rmse"], "params": h["params"]}) + "\n")

    if platform == "sim":
        kill_stack()

    status = "PASS" if result["pass"] else "FAIL"
    print(
        f"\nH0 tuning complete  BO={res.fun:.4f} m  validation={val_rmse:.4f} m  {status}",
        flush=True,
    )
    print(f"Results: {out_dir}", flush=True)
    return result


def main() -> None:
    try:
        import rclpy  # noqa: F401
    except ImportError as exc:
        raise SystemExit(
            "ROS Python not available. Run via:\n"
            "  source /opt/ros/humble/setup.bash && source install/setup.bash && python3 tune_h0.py ..."
        ) from exc

    parser = argparse.ArgumentParser(description="Bayesian tune H0 baseline for BlueBoat")
    parser.add_argument("--platform", choices=("sim", "real", "bench"), default="sim")
    parser.add_argument("--n-calls", type=int, default=28, help="BO iterations")
    parser.add_argument("--quick", action="store_true", help="8 BO calls (smoke test)")
    parser.add_argument("--out", default=None, help="Output directory")
    parser.add_argument("--launch", default=OPEN_WATER_LAUNCH)
    parser.add_argument("--no-restart", action="store_true", help="Do not restart Gazebo each trial")
    parser.add_argument(
        "--install",
        action="store_true",
        help="Write best params to config/h0_tuned_overlay.yaml",
    )
    parser.add_argument(
        "--refine",
        default=None,
        help="Path to tune_result.json for local refinement around prior best",
    )
    parser.add_argument("--overlay", default=None, help="Optional extra YAML overlay")
    args = parser.parse_args()

    n_calls = 8 if args.quick else args.n_calls
    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    out_dir = (
        Path(args.out)
        if args.out
        else PKG_DIR / "results" / f"tune_{args.platform}_{stamp}"
    )
    out_dir.mkdir(parents=True, exist_ok=True)

    run_tuning(
        platform=args.platform,
        out_dir=out_dir,
        n_calls=n_calls,
        launch=args.launch,
        restart_sim=not args.no_restart,
        install_overlay=args.install,
        refine_from=Path(args.refine) if args.refine else None,
        overlay_path=args.overlay,
    )


if __name__ == "__main__":
    main()
