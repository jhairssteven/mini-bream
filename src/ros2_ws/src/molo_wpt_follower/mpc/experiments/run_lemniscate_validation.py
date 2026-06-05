#!/usr/bin/env python3
"""
Re-validate all BO-tuned approaches on a clean lemniscate reference path.

Uses each hypothesis ``best_config.yaml`` (control / MPC / ILOS / guidance) but forces
shared lemniscate geometry from H9_slow_tight (produces a proper figure-8 without
spurious Dubins loops).  Earlier ``best_validation`` runs could use a mismatched
``dubins.step_size_m`` (e.g. 0.25 vs 1.25 in best_config), which inflated the path
with extra circles.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Dict, List

import yaml

ROOT = Path(__file__).resolve().parents[1]
EXP = Path(__file__).resolve().parent
for p in (str(ROOT), str(EXP)):
    if p not in sys.path:
        sys.path.insert(0, p)

from reference_path import apply_reference_path, reference_path_block  # noqa: E402
from run_campaign import (  # noqa: E402
    OPEN_WATER_LAUNCH,
    ensure_gazebo,
    kill_gazebo,
    run_trial,
    save_ref_path,
)

TUNE_ROOT = ROOT / "experiments" / "results" / "tune_lemniscate_full"
REF_HYP = "H9_slow_tight"
REF_CONFIG = TUNE_ROOT / REF_HYP / "best_config.yaml"
OUT_DIR = ROOT / "experiments" / "results" / "lemniscate_validation"

WARMUP_S = 20.0
EVALUATE_S = 55.0
SKIP_INITIAL_S = 20.0
TARGET_RMSE_M = 0.1


def path_point_count(cfg: dict) -> int:
    from mpc import build_mission_path

    samples, _ = build_mission_path(cfg, (0.0, 0.0))
    return len(samples)


def discover_hypotheses(tune_root: Path) -> List[str]:
    ids = []
    for d in sorted(tune_root.iterdir()):
        if (d / "best_config.yaml").is_file():
            ids.append(d.name)
    return ids


def main() -> None:
    parser = argparse.ArgumentParser(description="Lemniscate re-validation with fixed reference path")
    parser.add_argument("--tune-root", default=str(TUNE_ROOT))
    parser.add_argument("--ref-hypothesis", default=REF_HYP)
    parser.add_argument("--out", default=str(OUT_DIR))
    parser.add_argument("--launch", default=OPEN_WATER_LAUNCH)
    parser.add_argument("--hypothesis", nargs="*", help="Subset of hypothesis ids")
    parser.add_argument("--no-restart", action="store_true")
    parser.add_argument("--dry-run", action="store_true", help="Only write merged configs and path stats")
    args = parser.parse_args()

    tune_root = Path(args.tune_root)
    out_dir = Path(args.out)
    out_dir.mkdir(parents=True, exist_ok=True)

    ref_cfg_path = tune_root / args.ref_hypothesis / "best_config.yaml"
    if not ref_cfg_path.is_file() and not REF_CONFIG.is_file():
        raise SystemExit(f"Reference config not found: {ref_cfg_path}")
    ref_block = reference_path_block(ref_cfg_path if ref_cfg_path.is_file() else REF_CONFIG)
    with open(out_dir / "reference_path.yaml", "w", encoding="utf-8") as f:
        yaml.safe_dump(ref_block, f, sort_keys=False)

    hypos = discover_hypotheses(tune_root)
    if args.hypothesis:
        allow = set(args.hypothesis)
        hypos = [h for h in hypos if h in allow]

    print(f"Reference path from: {args.ref_hypothesis}", flush=True)
    print(
        f"  trajectory={ref_block['waypoints']['trajectory']}\n"
        f"  resample={ref_block['path']['resample_step_m']:.4f} m\n"
        f"  dubins R={ref_block['dubins']['turning_radius_m']:.3f} m, "
        f"step={ref_block['dubins']['step_size_m']:.3f} m",
        flush=True,
    )

    runs: List[dict] = []
    for hid in hypos:
        src = tune_root / hid / "best_config.yaml"
        cfg = yaml.safe_load(src.read_text(encoding="utf-8"))
        merged = apply_reference_path(cfg, ref_block)
        run_dir = out_dir / hid
        run_dir.mkdir(parents=True, exist_ok=True)

        n_pts = path_point_count(merged)
        with open(run_dir / "config.yaml", "w", encoding="utf-8") as f:
            yaml.safe_dump(merged, f, sort_keys=False)
        save_ref_path(merged, run_dir / "ref_path.csv", (0.0, 0.0))

        meta = {
            "hypothesis_id": hid,
            "approach": merged.get("control", {}).get("approach", ""),
            "mpc_mode": merged.get("mpc", {}).get("mode", "velocity"),
            "path_points": n_pts,
            "cruise_speed_mps": merged.get("path", {}).get("cruise_speed_mps"),
            "source_config": str(src),
        }
        print(f"  [{hid}] path_points={n_pts}  approach={meta['approach']}", flush=True)

        if args.dry_run:
            runs.append({**meta, "rmse": None, "dry_run": True})
            continue

        print(f"=== Running {hid} ===", flush=True)
        rmse = run_trial(
            merged,
            run_dir,
            WARMUP_S,
            EVALUATE_S,
            SKIP_INITIAL_S,
            args.launch,
            restart_sim=not args.no_restart,
        )
        meta["rmse"] = float(rmse)
        meta["pass"] = math.isfinite(rmse) and rmse < TARGET_RMSE_M
        meta["target_rmse_m"] = TARGET_RMSE_M
        with open(run_dir / "result.json", "w", encoding="utf-8") as f:
            json.dump(meta, f, indent=2)
        runs.append(meta)
        status = "PASS" if meta["pass"] else "FAIL"
        print(f"  [{hid}] rmse={rmse:.4f} m  {status}", flush=True)

    ranked = sorted(
        [r for r in runs if r.get("rmse") is not None and math.isfinite(r["rmse"])],
        key=lambda r: r["rmse"],
    )
    summary = {
        "timestamp": datetime.now(timezone.utc).isoformat(),
        "reference_hypothesis": args.ref_hypothesis,
        "reference_path": ref_block,
        "target_rmse_m": TARGET_RMSE_M,
        "runs": runs,
        "ranked": [
            {
                "hypothesis_id": r["hypothesis_id"],
                "approach": r["approach"],
                "validation_rmse": r["rmse"],
                "path_points": r["path_points"],
            }
            for r in ranked
        ],
        "best": ranked[0]["hypothesis_id"] if ranked else None,
    }
    with open(out_dir / "validation_summary.json", "w", encoding="utf-8") as f:
        json.dump(summary, f, indent=2)

    if not args.dry_run:
        kill_gazebo()
        try:
            from plot_results import plot_validation_all

            plot_validation_all(out_dir)
        except Exception as exc:
            print(f"plot_validation_all skipped: {exc}", flush=True)

    print(f"\nDone: {out_dir}", flush=True)
    if ranked:
        print("Top 3:", flush=True)
        for r in ranked[:3]:
            print(f"  {r['hypothesis_id']}: {r['rmse']:.4f} m", flush=True)


if __name__ == "__main__":
    main()
