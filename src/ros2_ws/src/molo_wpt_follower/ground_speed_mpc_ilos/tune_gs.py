#!/usr/bin/env python3
"""Staged constrained gain tuning for ground-speed MPC → ILOS → PI/PD.

Strategy (per phase):
  1. Coarse Latin-hypercube / random search in *tight* bounds
  2. Local trust-region refinement around the best *valid* seed
  3. Final validation re-run

Validity constraint: lemniscate completion ≥ 80%. Invalid trials are quarantined
(rejected/) and never used as candidates or overlay sources.

Phases: inner → ilos → mpc → global (small joint polish).
Target RMSE = 0.1 m.
"""

from __future__ import annotations

import argparse
import copy
import json
import math
import shutil
import sys
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Dict, List, Optional, Sequence, Tuple

import numpy as np
import yaml

PKG_DIR = Path(__file__).resolve().parent
MOLO_DIR = PKG_DIR.parent
MPC_DIR = MOLO_DIR / "mpc"
EXP_DIR = MPC_DIR / "experiments"

for path in (str(PKG_DIR), str(MPC_DIR), str(EXP_DIR)):
    if path not in sys.path:
        sys.path.insert(0, path)

from config import (  # noqa: E402
    BASE_CONFIG,
    TUNED_OVERLAY,
    build_gs_boat_config,
    platform_name,
)
from plot_gs_results import plot_run  # noqa: E402
from trial_runner import kill_stack, run_boat_trial  # noqa: E402

TARGET_RMSE_M = 0.1
MIN_COMPLETION = 0.80
INVALID_OBJ = 1.0e3

# Lemniscate ≈36.7 m; ≥80% needs ≈29.3 m of forward progress.
# At honest cruise 0.28–0.40 m/s (no k_ff overdrive), that requires ~100–120 s.
TIMING = {
    "sim": {"warmup_s": 5.0, "evaluate_s": 100.0, "skip_initial_s": 5.0},
    "real": {"warmup_s": 20.0, "evaluate_s": 55.0, "skip_initial_s": 20.0},
    "bench": {"warmup_s": 8.0, "evaluate_s": 30.0, "skip_initial_s": 8.0},
}
VALIDATION_TIMING = {
    "sim": {"warmup_s": 5.0, "evaluate_s": 120.0, "skip_initial_s": 5.0},
    "real": {"warmup_s": 20.0, "evaluate_s": 55.0, "skip_initial_s": 20.0},
    "bench": {"warmup_s": 8.0, "evaluate_s": 30.0, "skip_initial_s": 8.0},
}

PHASES = ("inner", "ilos", "mpc", "global")

# (name, lo, hi, is_int) — tight boxes around known-stable defaults (avoid chatter).
BOUND_INNER: List[Tuple[str, float, float, bool]] = [
    # Floor high enough that 100 s evaluate can still hit 80% completion.
    ("cruise", 0.26, 0.42, False),
    ("surge_kp", 6.0, 16.0, False),
    ("surge_ki", 0.5, 4.0, False),
    # Keep k_ff near plant gain; old 30–55 range made U_g ≫ U_d.
    ("surge_kff", 2.0, 12.0, False),
    ("surge_dx_max", 25.0, 70.0, False),
    ("surge_lpf", 0.15, 0.40, False),
    ("surge_xmax", 15.0, 28.0, False),
    ("yaw_kp", 1.2, 3.5, False),
    ("yaw_kd", 0.6, 2.0, False),
    ("yaw_nmax", 4.0, 9.0, False),
    ("u_d_floor", 0.65, 0.90, False),
]
BOUND_ILOS: List[Tuple[str, float, float, bool]] = [
    ("ilos_delta", 1.2, 2.4, False),
    ("ilos_sigma", 0.0, 0.18, False),
    ("ilos_ei_lim", 1.5, 4.0, False),
    ("angle_fast", 0.10, 0.25, False),
    ("angle_slow", 0.45, 0.80, False),
    ("min_speed_scale", 0.35, 0.60, False),
]
BOUND_MPC: List[Tuple[str, float, float, bool]] = [
    ("horizon", 10, 16, True),
    ("w_path", 30.0, 80.0, False),
    ("w_u", 6.0, 20.0, False),
    ("w_r", 0.8, 3.0, False),
    ("w_du", 1.0, 4.0, False),
    ("w_dr", 0.5, 2.5, False),
    ("u_max", 0.32, 0.45, False),
    ("r_max", 0.35, 0.55, False),
]

# Nominal seed = base yaml defaults (inside tight bounds).
SEED_INNER = {
    "cruise": 0.32,
    "surge_kp": 10.0,
    "surge_ki": 2.0,
    "surge_kff": 6.0,
    "surge_dx_max": 50.0,
    "surge_lpf": 0.25,
    "surge_xmax": 25.0,
    "yaw_kp": 2.5,
    "yaw_kd": 1.2,
    "yaw_nmax": 6.0,
    "u_d_floor": 0.75,
}
SEED_ILOS = {
    "ilos_delta": 1.8,
    "ilos_sigma": 0.06,
    "ilos_ei_lim": 2.5,
    "angle_fast": 0.15,
    "angle_slow": 0.60,
    "min_speed_scale": 0.40,
}
SEED_MPC = {
    "horizon": 12,
    "w_path": 50.0,
    "w_u": 12.0,
    "w_r": 1.5,
    "w_du": 2.0,
    "w_dr": 1.0,
    "u_max": 0.40,
    "r_max": 0.45,
}


def bounds_inner():
    return list(BOUND_INNER)


def bounds_ilos():
    return list(BOUND_ILOS)


def bounds_mpc():
    return list(BOUND_MPC)


def bounds_global():
    return bounds_inner() + bounds_ilos() + bounds_mpc()


def seed_for(phase: str) -> Dict[str, Any]:
    if phase == "inner":
        return dict(SEED_INNER)
    if phase == "ilos":
        return dict(SEED_ILOS)
    if phase == "mpc":
        return dict(SEED_MPC)
    out = {}
    out.update(SEED_INNER)
    out.update(SEED_ILOS)
    out.update(SEED_MPC)
    return out


def clip_params(bounds, p: Dict[str, Any]) -> Dict[str, Any]:
    out = {}
    for name, lo, hi, is_int in bounds:
        v = p.get(name, 0.5 * (lo + hi))
        v = float(np.clip(float(v), lo, hi))
        out[name] = int(round(v)) if is_int else float(v)
    return out


def params_to_x(bounds, p: Dict[str, Any]) -> np.ndarray:
    p = clip_params(bounds, p)
    return np.array([float(p[name]) for name, *_ in bounds], dtype=float)


def x_to_params(bounds, x: Sequence[float]) -> Dict[str, Any]:
    out = {}
    for (name, lo, hi, is_int), v in zip(bounds, x):
        v = float(np.clip(float(v), lo, hi))
        out[name] = int(round(v)) if is_int else float(v)
    return out


def lhs_unit(n: int, dim: int, rng: np.random.Generator) -> np.ndarray:
    """Simple Latin hypercube in [0,1]^dim."""
    if n <= 0:
        return np.zeros((0, dim))
    cut = np.linspace(0.0, 1.0, n + 1)
    u = rng.uniform(cut[:-1], cut[1:], size=(dim, n)).T
    for j in range(dim):
        rng.shuffle(u[:, j])
    return u


def sample_coarse(bounds, n: int, seed_params: Dict[str, Any], rng: np.random.Generator) -> List[Dict[str, Any]]:
    """Seed + LHS samples mapped into bounds."""
    samples = [clip_params(bounds, seed_params)]
    dim = len(bounds)
    need = max(0, n - 1)
    U = lhs_unit(need, dim, rng)
    for row in U:
        p = {}
        for (name, lo, hi, is_int), u in zip(bounds, row):
            v = lo + float(u) * (hi - lo)
            p[name] = int(round(v)) if is_int else float(v)
        # keep angle_fast < angle_slow when both present
        if "angle_fast" in p and "angle_slow" in p:
            p["angle_fast"] = min(p["angle_fast"], p["angle_slow"] - 0.05)
        samples.append(p)
    return samples


def apply_params(cfg: dict, p: Dict[str, Any]) -> dict:
    """Apply any subset of tuned params present in p."""
    c = copy.deepcopy(cfg)
    if "cruise" in p:
        c.setdefault("path", {})["cruise_speed_mps"] = p["cruise"]
        # Keep GS-MPC speed limit from clipping the chosen cruise.
        mpc0 = c.setdefault("gs_mpc", {})
        mpc0["u_max"] = max(float(mpc0.get("u_max", 0.4)), float(p["cruise"]))

    surge = c.setdefault("surge_pi", {})
    if "surge_kp" in p:
        surge["kp"] = p["surge_kp"]
    if "surge_ki" in p:
        surge["ki"] = p["surge_ki"]
    if "surge_kff" in p:
        surge["k_ff"] = p["surge_kff"]
    if "surge_dx_max" in p:
        surge["dx_max"] = p["surge_dx_max"]
    if "surge_lpf" in p:
        surge["u_lpf_tau_s"] = p["surge_lpf"]
    if "surge_xmax" in p:
        surge["x_max"] = p["surge_xmax"]
        surge["x_min"] = -abs(p["surge_xmax"])
    # Preserve post-fix FF fade even if an old overlay omitted it.
    surge.setdefault("ff_overspeed_fade_mps", 0.12)
    surge.setdefault("speed_feedback", "body_u")

    yaw = c.setdefault("yaw_pd", {})
    if "yaw_kp" in p:
        yaw["kp"] = p["yaw_kp"]
    if "yaw_kd" in p:
        yaw["kd"] = p["yaw_kd"]
    if "yaw_nmax" in p:
        yaw["n_max"] = p["yaw_nmax"]

    speed = c.setdefault("speed", {})
    if "u_d_floor" in p:
        speed["u_d_floor_frac_of_cruise"] = p["u_d_floor"]
    if "angle_fast" in p and "angle_slow" in p:
        speed["angle_threshold_fast_rad"] = min(p["angle_fast"], p["angle_slow"] - 0.05)
        speed["angle_threshold_slow_rad"] = p["angle_slow"]
    if "min_speed_scale" in p:
        speed["min_speed_scale"] = p["min_speed_scale"]

    ilos = c.setdefault("ilos", {})
    if "ilos_delta" in p:
        ilos["delta_m"] = p["ilos_delta"]
    if "ilos_sigma" in p:
        ilos["sigma"] = p["ilos_sigma"]
    if "ilos_ei_lim" in p:
        ilos["e_i_limit"] = p["ilos_ei_lim"]

    mpc = c.setdefault("gs_mpc", {})
    if "horizon" in p:
        mpc["horizon"] = int(p["horizon"])
    if "w_path" in p:
        mpc["w_path"] = p["w_path"]
    if "w_u" in p:
        mpc["w_u"] = p["w_u"]
    if "w_r" in p:
        mpc["w_r"] = p["w_r"]
    if "w_du" in p:
        mpc["w_du"] = p["w_du"]
    if "w_dr" in p:
        mpc["w_dr"] = p["w_dr"]
    if "u_max" in p:
        mpc["u_max"] = p["u_max"]
        mpc["u_min"] = min(float(mpc.get("u_min", 0.15)), 0.6 * p["u_max"])
    if "r_max" in p:
        mpc["r_max"] = p["r_max"]

    c.setdefault("control", {})["approach"] = "gs_mpc_ilos"
    return c


def extract_overlay(cfg: dict, keys: Sequence[str], phase: str, meta: dict) -> dict:
    out: dict = {}
    for k in keys:
        if k in cfg:
            out[k] = copy.deepcopy(cfg[k])
    out["tuning"] = {
        "phase": phase,
        "method": "lhs_then_local",
        "target_rmse_m": TARGET_RMSE_M,
        "min_completion": MIN_COMPLETION,
        **meta,
    }
    return out


INNER_KEYS = ("path", "surge_pi", "yaw_pd", "speed")
ILOS_KEYS = ("ilos", "speed")
MPC_KEYS = ("gs_mpc",)
ALL_KEYS = ("path", "surge_pi", "yaw_pd", "speed", "ilos", "gs_mpc")


def plot_phase_summary(phase_dir: Path, history: List[dict], val_metrics: dict, target: float) -> None:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    if not history:
        return
    objs = [h["objective"] for h in history]
    rmses = [h["rmse"] for h in history]
    comps = [h["completion"] for h in history]
    stages = [h.get("stage", "?") for h in history]

    fig, axes = plt.subplots(2, 2, figsize=(11, 8))
    fig.suptitle(f"GS tune phase: {phase_dir.name} (LHS→local, constrained)")

    ax = axes[0, 0]
    colors = ["#1f77b4" if s == "coarse" else "#ff7f0e" for s in stages]
    ax.scatter(range(len(objs)), objs, c=colors, s=28)
    ax.plot(objs, "-", color="0.6", lw=0.8, alpha=0.7)
    ax.axhline(target, color="k", ls="--", label=f"target {target} m")
    ax.set_xlabel("valid trial")
    ax.set_ylabel("objective (RMSE)")
    ax.set_title("blue=coarse  orange=local")
    ax.grid(True, alpha=0.3)
    ax.legend(fontsize=8)

    ax = axes[0, 1]
    ax.scatter(range(len(rmses)), rmses, c=colors, s=28)
    ax.axhline(target, color="k", ls="--")
    ax.set_xlabel("valid trial")
    ax.set_ylabel("XTE RMSE [m]")
    ax.set_title("RMSE (valid only)")
    ax.grid(True, alpha=0.3)

    ax = axes[1, 0]
    ax.bar(range(len(comps)), comps, color=colors, alpha=0.85)
    ax.axhline(MIN_COMPLETION, color="k", ls="--", label="80% gate")
    ax.set_xlabel("valid trial")
    ax.set_ylabel("path completion")
    ax.set_ylim(0, max(1.05, max(comps) * 1.05 if comps else 1.05))
    ax.legend(fontsize=8)
    ax.set_title("Lemniscate completion")
    ax.grid(True, axis="y", alpha=0.3)

    ax = axes[1, 1]
    ax.axis("off")
    txt = (
        f"best objective: {min(objs):.4f}\n"
        f"validation RMSE: {val_metrics.get('rmse', float('nan')):.4f} m\n"
        f"validation completion: {val_metrics.get('completion', 0):.1%}\n"
        f"validation valid: {val_metrics.get('valid')}\n"
        f"pass (<{target} m): {val_metrics.get('pass_target')}\n"
        f"valid trials: {len(history)}\n"
        f"method: LHS coarse → local trust region"
    )
    ax.text(0.05, 0.5, txt, transform=ax.transAxes, va="center", fontsize=11, family="monospace")

    fig.tight_layout()
    fig.savefig(phase_dir / "phase_summary.png", dpi=130)
    plt.close(fig)


class TrialEvaluator:
    """Run constrained trials; keep only completion≥80% as candidates."""

    def __init__(
        self,
        phase: str,
        platform: str,
        cfg_base: dict,
        phase_dir: Path,
        timing: dict,
    ):
        self.phase = phase
        self.platform = platform
        self.cfg_base = cfg_base
        self.phase_dir = phase_dir
        self.timing = timing
        self.trials_dir = phase_dir / "trials"
        self.rejected_dir = phase_dir / "rejected"
        self.trials_dir.mkdir(parents=True, exist_ok=True)
        self.rejected_dir.mkdir(parents=True, exist_ok=True)
        self.history: List[dict] = []
        self.n_rejected = 0
        self.n_attempts = 0

    def evaluate(self, p: Dict[str, Any], stage: str) -> Dict[str, Any]:
        self.n_attempts += 1
        trial_dir = self.trials_dir / f"trial_{self.n_attempts:04d}"
        trial_dir.mkdir(parents=True, exist_ok=True)
        with open(trial_dir / "params.json", "w", encoding="utf-8") as f:
            json.dump(p, f, indent=2)

        cfg = apply_params(self.cfg_base, p)
        metrics = run_boat_trial(
            cfg,
            trial_dir,
            warmup_s=self.timing["warmup_s"],
            evaluate_s=self.timing["evaluate_s"],
            skip_initial_s=self.timing["skip_initial_s"],
            wait_gps_timeout_s=30.0 if self.platform == "sim" else 60.0,
            wait_xte_timeout_s=30.0 if self.platform == "sim" else 45.0,
            min_completion=MIN_COMPLETION,
            target_rmse_m=TARGET_RMSE_M,
            return_metrics=True,
        )
        assert isinstance(metrics, dict)

        if not metrics["valid"]:
            self.n_rejected += 1
            dest = self.rejected_dir / f"reject_{self.n_rejected:04d}"
            if dest.exists():
                shutil.rmtree(dest)
            shutil.move(str(trial_dir), str(dest))
            with open(dest / "reject.json", "w", encoding="utf-8") as f:
                json.dump(
                    {
                        "reason": "completion_gate",
                        "stage": stage,
                        "rmse": metrics["rmse"],
                        "completion": metrics["completion"],
                        "min_completion": MIN_COMPLETION,
                        "params": p,
                    },
                    f,
                    indent=2,
                )
            print(
                f"  [{self.phase}/{stage}] REJECT #{self.n_rejected}  "
                f"rmse={metrics['rmse']:.4f}  comp={metrics['completion']:.1%}  "
                f"(need ≥{MIN_COMPLETION:.0%})",
                flush=True,
            )
            return {
                "valid": False,
                "objective": float(INVALID_OBJ),
                "rmse": float(metrics["rmse"]),
                "completion": float(metrics["completion"]),
                "params": p,
            }

        entry = {
            "params": p,
            "rmse": float(metrics["rmse"]),
            "completion": float(metrics["completion"]),
            "valid": True,
            "objective": float(metrics["objective"]),
            "stage": stage,
            "trial_dir": str(trial_dir),
        }
        self.history.append(entry)
        print(
            f"  [{self.phase}/{stage}] valid {len(self.history):3d}  "
            f"obj={entry['objective']:.4f}  rmse={entry['rmse']:.4f}  "
            f"comp={entry['completion']:.1%}",
            flush=True,
        )
        try:
            plot_run(trial_dir, entry["rmse"], TARGET_RMSE_M)
        except Exception as exc:
            print(f"  plot skipped: {exc}", flush=True)
        return entry


def run_coarse(ev: TrialEvaluator, bounds, seed_p: Dict[str, Any], n: int, rng: np.random.Generator) -> None:
    samples = sample_coarse(bounds, n, seed_p, rng)
    print(f"\n--- {ev.phase}: COARSE LHS  n={len(samples)} ---", flush=True)
    for p in samples:
        ev.evaluate(p, stage="coarse")


def run_local(
    ev: TrialEvaluator,
    bounds,
    n: int,
    rng: np.random.Generator,
    trust0: float = 0.28,
) -> None:
    """Trust-region local search around best valid (constrained: invalid → ignore move)."""
    if n <= 0:
        return
    if not ev.history:
        print(f"\n--- {ev.phase}: LOCAL skipped (no valid coarse seed) ---", flush=True)
        return

    best = min(ev.history, key=lambda h: h["objective"])
    x = params_to_x(bounds, best["params"])
    best_obj = float(best["objective"])
    print(
        f"\n--- {ev.phase}: LOCAL trust-region  n={n}  "
        f"seed_obj={best_obj:.4f} ---",
        flush=True,
    )

    spans = np.array([hi - lo for _, lo, hi, _ in bounds], dtype=float)
    trust = float(trust0)
    no_improve = 0

    for k in range(n):
        # Gaussian step inside shrinking trust box
        step = rng.normal(0.0, 1.0, size=len(bounds)) * (trust * spans)
        x_prop = x + step
        # project into global bounds
        for i, (_, lo, hi, _) in enumerate(bounds):
            x_prop[i] = float(np.clip(x_prop[i], lo, hi))
        p = x_to_params(bounds, x_prop)
        if "angle_fast" in p and "angle_slow" in p:
            p["angle_fast"] = min(p["angle_fast"], p["angle_slow"] - 0.05)

        res = ev.evaluate(p, stage="local")
        if res["valid"] and res["objective"] < best_obj - 1e-4:
            x = params_to_x(bounds, res["params"])
            best_obj = float(res["objective"])
            no_improve = 0
            trust = min(0.45, trust * 1.15)
            print(f"    local accept → obj={best_obj:.4f}  trust={trust:.3f}", flush=True)
        else:
            no_improve += 1
            if no_improve >= 3:
                trust *= 0.65
                no_improve = 0
                print(f"    shrink trust → {trust:.3f}", flush=True)
            if trust < 0.03:
                print("    trust floor reached; stopping local early", flush=True)
                break


def run_phase(
    phase: str,
    platform: str,
    out_dir: Path,
    n_coarse: int,
    n_local: int,
    frozen_overlays: List[Path],
    bounds_builder,
    overlay_keys: Sequence[str],
    seed_params: Optional[Dict[str, Any]] = None,
) -> Dict[str, Any]:
    timing = TIMING.get(platform, TIMING["sim"])
    val_timing = VALIDATION_TIMING.get(platform, VALIDATION_TIMING["sim"])
    bounds = bounds_builder()
    phase_dir = out_dir / f"phase_{phase}"
    phase_dir.mkdir(parents=True, exist_ok=True)

    cfg_base = build_gs_boat_config(
        platform=platform,
        include_tuned=False,
        extra_overlays=frozen_overlays,
    )
    seed_p = clip_params(bounds, seed_params or seed_for(phase))
    rng = np.random.default_rng(42 + abs(hash(phase)) % 1000)

    print(
        f"\n{'='*60}\n"
        f"PHASE {phase.upper()}  coarse={n_coarse}  local={n_local}  platform={platform}\n"
        f"  frozen overlays: {[str(p.name) for p in frozen_overlays]}\n"
        f"  output: {phase_dir}\n"
        f"  constraint: completion≥{MIN_COMPLETION:.0%}  target RMSE={TARGET_RMSE_M} m\n"
        f"  method: LHS coarse → local trust-region (tight bounds)\n"
        f"{'='*60}",
        flush=True,
    )

    ev = TrialEvaluator(phase, platform, cfg_base, phase_dir, timing)
    run_coarse(ev, bounds, seed_p, n_coarse, rng)
    run_local(ev, bounds, n_local, rng)

    if ev.history:
        best_entry = min(ev.history, key=lambda h: h["objective"])
        best_p = best_entry["params"]
        bo_best = float(best_entry["objective"])
    else:
        best_p = seed_p
        bo_best = float(INVALID_OBJ)
        print(
            f"  [{phase}] WARNING: no valid trials; validating seed defaults only",
            flush=True,
        )

    best_cfg = apply_params(cfg_base, best_p)
    val_dir = phase_dir / "best_validation"
    val_dir.mkdir(parents=True, exist_ok=True)
    val_metrics = run_boat_trial(
        best_cfg,
        val_dir,
        warmup_s=val_timing["warmup_s"],
        evaluate_s=val_timing["evaluate_s"],
        skip_initial_s=val_timing["skip_initial_s"],
        wait_gps_timeout_s=30.0 if platform == "sim" else 60.0,
        wait_xte_timeout_s=30.0 if platform == "sim" else 45.0,
        min_completion=MIN_COMPLETION,
        target_rmse_m=TARGET_RMSE_M,
        return_metrics=True,
    )
    assert isinstance(val_metrics, dict)
    try:
        plot_run(val_dir, val_metrics["rmse"], TARGET_RMSE_M)
    except Exception as exc:
        print(f"validation plot skipped: {exc}", flush=True)

    overlay_path = phase_dir / f"{phase}_overlay.yaml"
    if val_metrics["valid"]:
        with open(phase_dir / "best_config.yaml", "w", encoding="utf-8") as f:
            yaml.safe_dump(best_cfg, f, sort_keys=False)
        overlay = extract_overlay(
            best_cfg,
            overlay_keys,
            phase,
            {
                "best_objective": bo_best,
                "validation_rmse": float(val_metrics["rmse"]),
                "validation_completion": float(val_metrics["completion"]),
                "validation_valid": True,
                "n_coarse": n_coarse,
                "n_local": n_local,
                "n_valid": len(ev.history),
                "n_rejected": ev.n_rejected,
                "timestamp": datetime.now(timezone.utc).isoformat(),
                "platform": platform_name(cfg_base),
            },
        )
        with open(overlay_path, "w", encoding="utf-8") as f:
            yaml.safe_dump(overlay, f, sort_keys=False)
    else:
        print(
            f"  [{phase}] validation INVALID (comp={val_metrics['completion']:.1%}) — "
            f"overlay NOT written",
            flush=True,
        )

    result = {
        "phase": phase,
        "platform": platform_name(cfg_base),
        "platform_profile": platform,
        "method": "lhs_then_local",
        "target_rmse_m": TARGET_RMSE_M,
        "min_completion": MIN_COMPLETION,
        "best_objective": bo_best,
        "validation_rmse": float(val_metrics["rmse"]),
        "validation_completion": float(val_metrics["completion"]),
        "validation_valid": bool(val_metrics["valid"]),
        "pass": bool(val_metrics.get("pass_target")),
        "n_coarse": n_coarse,
        "n_local": n_local,
        "n_valid": len(ev.history),
        "n_rejected": ev.n_rejected,
        "best_params": best_p if val_metrics["valid"] else {},
        "overlay": str(overlay_path) if val_metrics["valid"] else "",
        "source_config": str(BASE_CONFIG),
    }
    # aliases for staged bar chart compatibility
    result["bo_best_objective"] = bo_best

    with open(phase_dir / "tune_result.json", "w", encoding="utf-8") as f:
        json.dump(result, f, indent=2)
    with open(phase_dir / "history.jsonl", "w", encoding="utf-8") as f:
        for h in ev.history:
            f.write(
                json.dumps(
                    {
                        "stage": h.get("stage"),
                        "objective": h["objective"],
                        "rmse": h["rmse"],
                        "completion": h["completion"],
                        "valid": h["valid"],
                        "params": h["params"],
                    }
                )
                + "\n"
            )

    plot_phase_summary(phase_dir, ev.history, val_metrics, TARGET_RMSE_M)

    status = "PASS" if result["pass"] else "FAIL"
    print(
        f"\nPHASE {phase} done  best={bo_best:.4f}  "
        f"val_rmse={val_metrics['rmse']:.4f} m  "
        f"comp={val_metrics['completion']:.1%}  "
        f"valid={len(ev.history)} rejected={ev.n_rejected}  {status}",
        flush=True,
    )
    return result


def run_staged(
    platform: str,
    out_dir: Path,
    n_coarse_stage: int,
    n_local_stage: int,
    n_coarse_global: int,
    n_local_global: int,
    install_overlay: bool,
) -> Dict[str, Any]:
    out_dir.mkdir(parents=True, exist_ok=True)
    summary: Dict[str, Any] = {
        "timestamp": datetime.now(timezone.utc).isoformat(),
        "platform": platform,
        "method": "lhs_then_local",
        "target_rmse_m": TARGET_RMSE_M,
        "min_completion": MIN_COMPLETION,
        "sim_world": "open_water_harner",
        "phases": {},
    }

    r1 = run_phase(
        "inner",
        platform,
        out_dir,
        n_coarse_stage,
        n_local_stage,
        frozen_overlays=[],
        bounds_builder=bounds_inner,
        overlay_keys=INNER_KEYS,
    )
    summary["phases"]["inner"] = r1
    frozen: List[Path] = []
    if r1.get("overlay"):
        frozen.append(Path(r1["overlay"]))

    r2 = run_phase(
        "ilos",
        platform,
        out_dir,
        n_coarse_stage,
        n_local_stage,
        frozen_overlays=list(frozen),
        bounds_builder=bounds_ilos,
        overlay_keys=ILOS_KEYS,
    )
    summary["phases"]["ilos"] = r2
    if r2.get("overlay"):
        frozen.append(Path(r2["overlay"]))

    r3 = run_phase(
        "mpc",
        platform,
        out_dir,
        n_coarse_stage,
        n_local_stage,
        frozen_overlays=list(frozen),
        bounds_builder=bounds_mpc,
        overlay_keys=MPC_KEYS,
    )
    summary["phases"]["mpc"] = r3

    seed = {}
    seed.update(r1.get("best_params") or {})
    seed.update(r2.get("best_params") or {})
    seed.update(r3.get("best_params") or {})
    # Global: small trust polish around staged seed (still constrained).
    r4 = run_phase(
        "global",
        platform,
        out_dir,
        n_coarse_global,
        n_local_global,
        frozen_overlays=[],
        bounds_builder=bounds_global,
        overlay_keys=ALL_KEYS,
        seed_params=seed if seed else None,
    )
    summary["phases"]["global"] = r4

    if install_overlay and r4.get("overlay") and r4.get("validation_valid"):
        final_overlay = Path(r4["overlay"])
        if final_overlay.is_file():
            TUNED_OVERLAY.write_text(final_overlay.read_text(encoding="utf-8"), encoding="utf-8")
            print(f"Installed tuned overlay → {TUNED_OVERLAY}", flush=True)
    elif install_overlay:
        print("Skipping --install: global validation was INVALID", flush=True)

    try:
        import matplotlib

        matplotlib.use("Agg")
        import matplotlib.pyplot as plt

        names = list(summary["phases"].keys())
        rmses = [summary["phases"][n]["validation_rmse"] for n in names]
        comps = [summary["phases"][n]["validation_completion"] for n in names]
        fig, ax = plt.subplots(figsize=(8, 4))
        x = range(len(names))
        colors = [
            "#2ca02c" if summary["phases"][n].get("pass") else "#d62728" for n in names
        ]
        ax.bar(x, rmses, color=colors, alpha=0.85)
        ax.axhline(TARGET_RMSE_M, color="k", ls="--", label=f"target {TARGET_RMSE_M} m")
        for i, (r, c) in enumerate(zip(rmses, comps)):
            ax.text(i, r + 0.02, f"{r:.3f}\n({c:.0%})", ha="center", fontsize=8)
        ax.set_xticks(list(x))
        ax.set_xticklabels(names)
        ax.set_ylabel("validation XTE RMSE [m]")
        ax.set_title("Staged GS tune (LHS→local, open_water) — validation per phase")
        ax.legend()
        ax.grid(True, axis="y", alpha=0.3)
        fig.tight_layout()
        fig.savefig(out_dir / "staged_validation_bar.png", dpi=130)
        plt.close(fig)
    except Exception as exc:
        print(f"campaign plot skipped: {exc}", flush=True)

    with open(out_dir / "staged_summary.json", "w", encoding="utf-8") as f:
        json.dump(summary, f, indent=2)

    kill_stack()
    print(f"\nStaged tuning complete → {out_dir}", flush=True)
    return summary


def main() -> None:
    try:
        import rclpy  # noqa: F401
    except ImportError as exc:
        raise SystemExit(
            "ROS Python not available. Source Humble + workspace first."
        ) from exc

    parser = argparse.ArgumentParser(
        description="Staged LHS→local constrained tuning for GS-MPC+ILOS"
    )
    parser.add_argument("--platform", choices=("sim", "real", "bench"), default="sim")
    parser.add_argument("--n-coarse-stage", type=int, default=8)
    parser.add_argument("--n-local-stage", type=int, default=12)
    parser.add_argument("--n-coarse-global", type=int, default=6)
    parser.add_argument("--n-local-global", type=int, default=12)
    parser.add_argument(
        "--quick",
        action="store_true",
        help="4+6 per stage, 3+6 global",
    )
    parser.add_argument("--out", default=None)
    parser.add_argument("--install", action="store_true")
    parser.add_argument(
        "--phase",
        choices=PHASES + ("staged",),
        default="staged",
    )
    args = parser.parse_args()

    if args.quick:
        n_cs, n_ls, n_cg, n_lg = 4, 6, 3, 6
    else:
        n_cs = args.n_coarse_stage
        n_ls = args.n_local_stage
        n_cg = args.n_coarse_global
        n_lg = args.n_local_global

    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    out_dir = (
        Path(args.out)
        if args.out
        else PKG_DIR / "results" / f"tune_lhs_local_{args.platform}_{stamp}"
    )

    if args.phase == "staged":
        run_staged(
            platform=args.platform,
            out_dir=out_dir,
            n_coarse_stage=n_cs,
            n_local_stage=n_ls,
            n_coarse_global=n_cg,
            n_local_global=n_lg,
            install_overlay=args.install,
        )
        return

    builders = {
        "inner": (bounds_inner, INNER_KEYS),
        "ilos": (bounds_ilos, ILOS_KEYS),
        "mpc": (bounds_mpc, MPC_KEYS),
        "global": (bounds_global, ALL_KEYS),
    }
    b_builder, keys = builders[args.phase]
    frozen_list: List[Path] = []
    if args.phase == "ilos":
        cand = out_dir / "phase_inner" / "inner_overlay.yaml"
        if cand.is_file():
            frozen_list = [cand]
    elif args.phase == "mpc":
        for name in ("phase_inner/inner_overlay.yaml", "phase_ilos/ilos_overlay.yaml"):
            cand = out_dir / name
            if cand.is_file():
                frozen_list.append(cand)

    n_c = n_cg if args.phase == "global" else n_cs
    n_l = n_lg if args.phase == "global" else n_ls
    run_phase(
        args.phase,
        args.platform,
        out_dir,
        n_c,
        n_l,
        frozen_overlays=frozen_list,
        bounds_builder=b_builder,
        overlay_keys=keys,
    )


if __name__ == "__main__":
    main()
