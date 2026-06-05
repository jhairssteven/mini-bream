#!/usr/bin/env python3
"""
Bayesian optimization of all control hypotheses on the lemniscate trajectory (open-water Gazebo).

Each hypothesis keeps its control.approach fixed; MPC/ILOS/guidance parameters are tuned.
Lemniscate geometry (trajectory, resample, Dubins) is fixed via reference_path.py so all
approaches share the same clean figure-8 reference.
"""

from __future__ import annotations

import argparse
import copy
import json
import math
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Dict, List, Tuple

import numpy as np
import yaml

import sys

ROOT = Path(__file__).resolve().parents[1]
EXP = Path(__file__).resolve().parent
for p in (str(ROOT), str(EXP)):
    if p not in sys.path:
        sys.path.insert(0, p)

from reference_path import apply_reference_path, reference_path_block  # noqa: E402
from run_campaign import (  # noqa: E402
    OPEN_WATER_LAUNCH,
    deep_merge,
    ensure_gazebo,
    gazebo_ready,
    kill_gazebo,
    merge_traj_cfg,
    run_trial,
)

REF_CONFIG = ROOT / "experiments" / "results" / "tune_lemniscate_full" / "H9_slow_tight" / "best_config.yaml"

PARAMS = ROOT / "params.yaml"
HYPOTHESES = ROOT / "experiments" / "hypotheses.yaml"
TRAJECTORIES = ROOT / "trajectories.yaml"

WARMUP_S = 20.0
EVALUATE_S = 55.0
SKIP_INITIAL_S = 20.0


def _is_spatial(hyp: dict) -> bool:
    patch = hyp.get("config_patch", {})
    return str(patch.get("mpc", {}).get("mode", "")).lower() == "spatial"


def build_velocity_space():
    from skopt.space import Integer, Real

    return [
        Real(0.14, 0.34, name="cruise"),
        Real(0.45, 1.3, name="ilos_min"),
        Real(1.0, 2.6, name="ilos_max"),
        Real(6.0, 14.0, name="ilos_conv"),
        Real(0.9, 2.6, name="pid_kp"),
        Real(0.15, 1.0, name="pid_kd"),
        Real(0.4, 0.95, name="max_yaw_r"),
        Real(0.85, 1.45, name="kappa_ff"),
        Real(1.5, 5.5, name="kappa_scale"),
        Real(0.0, 1.3, name="lateral_v"),
        Real(0.5, 2.2, name="stanley_k"),
        Real(0.3, 1.8, name="smc_k_r"),
        Real(0.2, 1.2, name="smc_k_v"),
        Real(0.05, 0.25, name="smc_phi"),
        Real(0.5, 1.6, name="frenet_k_y"),
        Real(0.6, 2.0, name="frenet_k_psi"),
        Real(0.6, 2.0, name="backstep_k1"),
        Real(1.0, 3.5, name="backstep_k2"),
        Real(0.5, 2.2, name="contour_k_d"),
        Real(0.1, 1.2, name="contour_k_dd"),
        Integer(2, 8, name="pp_steps"),
        Real(1.0, 3.5, name="los_delta"),
        Integer(12, 20, name="horizon"),
        Real(18.0, 50.0, name="Q_u"),
        Real(0.5, 18.0, name="Q_v"),
        Real(90.0, 280.0, name="Q_r"),
        Real(0.008, 0.045, name="R_in"),
        Real(4.0, 12.0, name="Q_term"),
    ]


def build_spatial_space():
    from skopt.space import Integer, Real

    return [
        Real(0.14, 0.32, name="cruise"),
        Real(0.45, 1.3, name="ilos_min"),
        Real(1.0, 2.6, name="ilos_max"),
        Real(6.0, 14.0, name="ilos_conv"),
        Integer(12, 22, name="horizon"),
        Real(400.0, 2000.0, name="Q_xy"),
        Real(80.0, 350.0, name="Q_psi"),
        Real(2.0, 12.0, name="Q_u"),
        Real(2.0, 12.0, name="Q_v"),
        Real(30.0, 120.0, name="Q_r"),
        Real(0.02, 0.08, name="R_in"),
        Real(4.0, 10.0, name="Q_term"),
    ]


def vector_to_dict(space, x) -> Dict[str, Any]:
    return {d.name: (int(v) if d.name in ("horizon", "pp_steps") else float(v)) for d, v in zip(space, x)}


def apply_velocity_params(cfg: dict, p: Dict[str, Any]) -> dict:
    c = copy.deepcopy(cfg)
    c.setdefault("path", {})
    c.setdefault("dubins", {})
    c.setdefault("ilos", {})
    c.setdefault("pid", {})
    c.setdefault("guidance", {})
    c.setdefault("mpc", {})
    c.setdefault("control", {})

    c["path"]["cruise_speed_mps"] = p["cruise"]
    c["ilos"]["lookahead_min_m"] = p["ilos_min"]
    c["ilos"]["lookahead_max_m"] = max(p["ilos_max"], p["ilos_min"] + 0.2)
    c["ilos"]["conv_rate"] = p["ilos_conv"]
    c["pid"]["kp"] = p["pid_kp"]
    c["pid"]["kd"] = p["pid_kd"]
    c["guidance"].update(
        {
            "max_yaw_rate_rad_s": p["max_yaw_r"],
            "kappa_ff_gain": p["kappa_ff"],
            "kappa_speed_scale": p["kappa_scale"],
            "lateral_v_gain": p["lateral_v"],
            "stanley_k": p["stanley_k"],
            "smc_k_r": p["smc_k_r"],
            "smc_k_v": p["smc_k_v"],
            "smc_phi": p["smc_phi"],
            "frenet_k_y": p["frenet_k_y"],
            "frenet_k_psi": p["frenet_k_psi"],
            "backstep_k1": p["backstep_k1"],
            "backstep_k2": p["backstep_k2"],
            "contour_k_d": p["contour_k_d"],
            "contour_k_dd": p["contour_k_dd"],
            "pure_pursuit_steps": int(p["pp_steps"]),
            "los_delta_m": p["los_delta"],
        }
    )
    c["control"]["lateral_v_gain"] = p["lateral_v"]
    c["mpc"]["mode"] = "velocity"
    c["mpc"]["horizon"] = int(p["horizon"])
    c["mpc"]["Q_vel_diag"] = [p["Q_u"], p["Q_v"], p["Q_r"]]
    c["mpc"]["R_diag"] = [p["R_in"], p["R_in"]]
    c["mpc"]["Q_terminal_scale"] = p["Q_term"]
    return c


def apply_spatial_params(cfg: dict, p: Dict[str, Any]) -> dict:
    c = copy.deepcopy(cfg)
    c.setdefault("path", {})
    c.setdefault("dubins", {})
    c.setdefault("ilos", {})
    c.setdefault("mpc", {})
    c["path"]["cruise_speed_mps"] = p["cruise"]
    c["ilos"]["lookahead_min_m"] = p["ilos_min"]
    c["ilos"]["lookahead_max_m"] = max(p["ilos_max"], p["ilos_min"] + 0.2)
    c["ilos"]["conv_rate"] = p["ilos_conv"]
    c["mpc"]["mode"] = "spatial"
    c["mpc"]["horizon"] = int(p["horizon"])
    c["mpc"]["Q_spatial_diag"] = [
        p["Q_xy"],
        p["Q_xy"],
        p["Q_psi"],
        p["Q_u"],
        p["Q_v"],
        p["Q_r"],
    ]
    c["mpc"]["R_diag"] = [p["R_in"], p["R_in"]]
    c["mpc"]["Q_terminal_scale"] = p["Q_term"]
    return c


def tune_one_hypothesis(
    hyp: dict,
    base_cfg: dict,
    lemniscate_traj: dict,
    out_dir: Path,
    n_calls: int,
    launch: str,
    restart_every: bool,
    ref_block: Dict[str, Any],
) -> Dict[str, Any]:
    hid = hyp["id"]
    hyp_dir = out_dir / hid
    hyp_dir.mkdir(parents=True, exist_ok=True)

    cfg_base = apply_reference_path(
        merge_traj_cfg(deep_merge(base_cfg, hyp.get("config_patch", {})), lemniscate_traj),
        ref_block,
    )
    spatial = _is_spatial(hyp)

    print(f"\n{'='*60}\nBO: {hid}  approach={cfg_base.get('control', {}).get('approach', 'spatial')}\n{'='*60}", flush=True)

    try:
        from skopt import gp_minimize
    except ImportError:
        raise SystemExit("Install scikit-optimize: pip install scikit-optimize")

    space = build_spatial_space() if spatial else build_velocity_space()
    history: List[dict] = []

    def objective(x):
        p = vector_to_dict(space, x)
        cfg = apply_reference_path(
            apply_spatial_params(cfg_base, p) if spatial else apply_velocity_params(cfg_base, p),
            ref_block,
        )
        trial_dir = hyp_dir / f"trial_{len(history):04d}"
        trial_dir.mkdir(parents=True, exist_ok=True)
        with open(trial_dir / "params.json", "w", encoding="utf-8") as f:
            json.dump(p, f, indent=2)
        score = run_trial(
            cfg,
            trial_dir,
            WARMUP_S,
            EVALUATE_S,
            SKIP_INITIAL_S,
            launch,
            restart_sim=restart_every,
        )
        history.append({"params": p, "rmse": score, "trial_dir": str(trial_dir)})
        print(f"  [{hid}] trial {len(history)} rmse={score:.4f}", flush=True)
        if not gazebo_ready():
            ensure_gazebo(launch)
        return score if math.isfinite(score) else 1e3

    res = gp_minimize(
        objective,
        space,
        n_calls=n_calls,
        n_initial_points=min(8, max(3, n_calls // 4)),
        random_state=42,
        acq_func="EI",
    )

    best_p = vector_to_dict(space, res.x)
    best_cfg = apply_reference_path(
        apply_spatial_params(cfg_base, best_p) if spatial else apply_velocity_params(cfg_base, best_p),
        ref_block,
    )
    val_dir = hyp_dir / "best_validation"
    val_dir.mkdir(parents=True, exist_ok=True)
    with open(val_dir / "config.yaml", "w", encoding="utf-8") as f:
        yaml.safe_dump(best_cfg, f, sort_keys=False)
    val_rmse = run_trial(
        best_cfg,
        val_dir,
        WARMUP_S,
        EVALUATE_S,
        SKIP_INITIAL_S,
        launch,
        restart_sim=True,
    )

    result = {
        "hypothesis_id": hid,
        "approach": cfg_base.get("control", {}).get("approach", "spatial_mpc"),
        "spatial_mpc": spatial,
        "bo_best_rmse": float(res.fun),
        "validation_rmse": float(val_rmse),
        "n_calls": n_calls,
        "best_params": best_p,
        "history": history,
    }
    with open(hyp_dir / "tune_result.json", "w", encoding="utf-8") as f:
        json.dump({k: v for k, v in result.items() if k != "history"}, f, indent=2)
    with open(hyp_dir / "best_config.yaml", "w", encoding="utf-8") as f:
        yaml.safe_dump(best_cfg, f, sort_keys=False)
    with open(hyp_dir / "history.jsonl", "w", encoding="utf-8") as f:
        for h in history:
            f.write(json.dumps({"rmse": h["rmse"], "params": h["params"]}) + "\n")

    print(f"  [{hid}] BO best={res.fun:.4f}  validation={val_rmse:.4f}", flush=True)
    return result


def main() -> None:
    parser = argparse.ArgumentParser(description="Bayesian tune all hypotheses on lemniscate")
    parser.add_argument("--params", default=str(PARAMS))
    parser.add_argument("--hypotheses", default=str(HYPOTHESES))
    parser.add_argument("--trajectories", default=str(TRAJECTORIES))
    parser.add_argument("--n-calls", type=int, default=28, help="BO iterations per hypothesis")
    parser.add_argument("--hypothesis", nargs="*", help="Subset of hypothesis ids (default: all)")
    parser.add_argument("--out", default=None)
    parser.add_argument("--launch", default=OPEN_WATER_LAUNCH)
    parser.add_argument("--no-restart", action="store_true", help="Do not restart Gazebo each trial")
    parser.add_argument("--quick", action="store_true", help="8 BO calls per hypothesis (smoke test)")
    args = parser.parse_args()

    n_calls = 8 if args.quick else args.n_calls

    with open(args.hypotheses, encoding="utf-8") as f:
        campaign = yaml.safe_load(f)
    with open(args.params, encoding="utf-8") as f:
        base_params = yaml.safe_load(f)
    with open(args.trajectories, encoding="utf-8") as f:
        traj_suite = yaml.safe_load(f)

    lemniscate = next(t for t in traj_suite["trajectories"] if t["name"] == "lemniscate")
    hypotheses = campaign.get("hypotheses", [])
    if args.hypothesis:
        allow = set(args.hypothesis)
        hypotheses = [h for h in hypotheses if h["id"] in allow]

    stamp = datetime.now(timezone.utc).strftime("%Y%m%d_%H%M%S")
    out_dir = Path(args.out) if args.out else ROOT / "experiments" / "results" / f"tune_lemniscate_{stamp}"
    out_dir.mkdir(parents=True, exist_ok=True)

    ref_cfg = REF_CONFIG if REF_CONFIG.is_file() else None
    ref_block = reference_path_block(ref_cfg)
    with open(out_dir / "reference_path.yaml", "w", encoding="utf-8") as f:
        yaml.safe_dump(ref_block, f, sort_keys=False)

    meta = {
        "trajectory": "lemniscate",
        "reference_path": ref_block,
        "n_calls_per_hypothesis": n_calls,
        "warmup_s": WARMUP_S,
        "evaluate_s": EVALUATE_S,
        "skip_initial_s": SKIP_INITIAL_S,
        "hypothesis_ids": [h["id"] for h in hypotheses],
    }
    with open(out_dir / "tune_meta.json", "w", encoding="utf-8") as f:
        json.dump(meta, f, indent=2)

    kill_gazebo()
    if not ensure_gazebo(args.launch):
        raise SystemExit("Gazebo failed to start")

    results: List[dict] = []
    restart = not args.no_restart
    for hyp in hypotheses:
        try:
            r = tune_one_hypothesis(
                hyp, base_params, lemniscate, out_dir, n_calls, args.launch, restart, ref_block
            )
            results.append(r)
        except Exception as exc:
            print(f"ERROR {hyp['id']}: {exc}", flush=True)
            results.append(
                {
                    "hypothesis_id": hyp["id"],
                    "error": str(exc),
                    "validation_rmse": float("inf"),
                }
            )

    kill_gazebo()

    ranked = sorted(
        [r for r in results if math.isfinite(r.get("validation_rmse", float("inf")))],
        key=lambda r: r["validation_rmse"],
    )
    summary = {
        "trajectory": "lemniscate",
        "target_rmse_m": 0.1,
        "n_calls": n_calls,
        "ranked": [
            {
                "hypothesis_id": r["hypothesis_id"],
                "approach": r.get("approach"),
                "validation_rmse": r.get("validation_rmse"),
                "bo_best_rmse": r.get("bo_best_rmse"),
            }
            for r in ranked
        ],
        "best": ranked[0]["hypothesis_id"] if ranked else None,
    }
    with open(out_dir / "tune_summary.json", "w", encoding="utf-8") as f:
        json.dump(summary, f, indent=2)

    print("\n" + "=" * 60, flush=True)
    print("LEMNISCATE BO SUMMARY (validation RMSE)", flush=True)
    for r in ranked:
        print(
            f"  {r['hypothesis_id']:<22} {r.get('approach', ''):<22} "
            f"val={r['validation_rmse']:.4f}  bo={r.get('bo_best_rmse', float('nan')):.4f}",
            flush=True,
        )
    print(f"\nResults: {out_dir}", flush=True)


if __name__ == "__main__":
    main()
