#!/usr/bin/env python3
"""
Run the ILOS+PID lemniscate experiment on any platform profile.

Pure ILOS heading guidance + PID yaw rate + differential thrust mixing.
No MPC and no boat model. Same artifact layout as h0_boat for analysis/plotting.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from datetime import datetime, timezone
from pathlib import Path
from typing import Optional, Tuple

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
    DEFAULT_RESULTS,
    PLATFORM_PROFILES,
    build_ilos_boat_config,
    platform_name,
    prepare_run_config,
)
from trial_runner import run_boat_trial, save_ref_path  # noqa: E402


def _mpc_imports():
    from mpc import build_mission_path

    return build_mission_path


def main() -> None:
    parser = argparse.ArgumentParser(description="ILOS+PID lemniscate experiment (no MPC)")
    parser.add_argument(
        "--platform",
        choices=sorted(PLATFORM_PROFILES),
        default="real",
        help="Platform profile: real (frontseat), sim (blueboat_sim), bench",
    )
    parser.add_argument("--overlay", default=None, help="Optional extra YAML overlay")
    parser.add_argument("--out", default=None, help="Output directory")
    parser.add_argument("--origin-lat", type=float, default=None)
    parser.add_argument("--origin-lon", type=float, default=None)
    parser.add_argument("--warmup", type=float, default=None)
    parser.add_argument("--duration", type=float, default=None)
    parser.add_argument("--skip-initial", type=float, default=None)
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument("--no-plot", action="store_true")
    args = parser.parse_args()

    origin: Optional[Tuple[float, float]] = None
    if args.origin_lat is not None and args.origin_lon is not None:
        origin = (args.origin_lat, args.origin_lon)

    cfg = build_ilos_boat_config(
        platform=args.platform,
        overlay_path=args.overlay,
        origin_latlon=origin,
    )
    exp = cfg.get("experiment", {})
    plat_label = platform_name(cfg, fallback=args.platform)

    warmup_s = float(args.warmup if args.warmup is not None else exp.get("warmup_s", 20.0))
    evaluate_s = float(args.duration if args.duration is not None else exp.get("evaluate_s", 55.0))
    skip_initial_s = float(
        args.skip_initial if args.skip_initial is not None else exp.get("skip_initial_s", 20.0)
    )
    target_rmse_m = float(exp.get("target_rmse_m", 0.1))
    wait_gps_timeout_s = float(exp.get("wait_for_gps_timeout_s", 60.0))
    wait_xte_timeout_s = float(exp.get("wait_for_xte_timeout_s", 45.0))

    ts = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    out_dir = Path(args.out) if args.out else DEFAULT_RESULTS / ts
    run_dir = out_dir / "ILOS_PID"
    run_dir.mkdir(parents=True, exist_ok=True)

    ref_block = {
        "waypoints": cfg.get("waypoints", {}),
        "path": {k: v for k, v in cfg.get("path", {}).items() if k != "cruise_speed_mps"},
        "dubins": cfg.get("dubins", {}),
    }
    with open(out_dir / "reference_path.yaml", "w", encoding="utf-8") as f:
        yaml.safe_dump(ref_block, f, sort_keys=False)

    try:
        samples, _ = _mpc_imports()(cfg, (0.0, 0.0))
        path_points = len(samples)
    except Exception as exc:
        print(f"WARNING: could not build mission path ({exc})", flush=True)
        path_points = 77

    meta = {
        "hypothesis_id": "ILOS_PID",
        "approach": cfg.get("control", {}).get("approach", "ilos_pid"),
        "controller": "ILOS+PID differential thrust (no MPC)",
        "path_points": path_points,
        "cruise_speed_mps": cfg.get("path", {}).get("cruise_speed_mps"),
        "source_config": str(BASE_CONFIG),
        "platform": plat_label,
        "platform_profile": args.platform,
        "extra_overlay": str(args.overlay) if args.overlay else None,
    }

    print(
        f"ILOS boat experiment\n"
        f"  platform:   {plat_label} ({args.platform})\n"
        f"  base config:{BASE_CONFIG}\n"
        f"  output:     {run_dir}\n"
        f"  path_pts:   {meta['path_points']}\n"
        f"  cruise:     {meta['cruise_speed_mps']} m/s\n"
        f"  timing:     warmup={warmup_s:.0f}s  evaluate={evaluate_s:.0f}s  "
        f"skip_initial={skip_initial_s:.0f}s",
        flush=True,
    )

    if args.dry_run:
        prepare_run_config(cfg, run_dir)
        try:
            save_ref_path(cfg, run_dir / "ref_path.csv", (0.0, 0.0))
        except Exception as exc:
            print(f"WARNING: ref_path.csv not generated ({exc})", flush=True)
        meta["dry_run"] = True
        with open(run_dir / "result.json", "w", encoding="utf-8") as f:
            json.dump(meta, f, indent=2)
        print("Dry run complete.", flush=True)
        return

    rmse = run_boat_trial(
        cfg,
        run_dir,
        warmup_s,
        evaluate_s,
        skip_initial_s,
        wait_gps_timeout_s,
        wait_xte_timeout_s,
    )
    meta["rmse"] = float(rmse)
    meta["pass"] = math.isfinite(rmse) and rmse < target_rmse_m
    meta["target_rmse_m"] = target_rmse_m

    with open(run_dir / "result.json", "w", encoding="utf-8") as f:
        json.dump(meta, f, indent=2)

    summary = {
        "timestamp": datetime.now(timezone.utc).isoformat(),
        "reference_hypothesis": "ILOS_PID",
        "reference_path": ref_block,
        "target_rmse_m": target_rmse_m,
        "platform": plat_label,
        "platform_profile": args.platform,
        "runs": [meta],
        "ranked": [
            {
                "hypothesis_id": "ILOS_PID",
                "approach": meta["approach"],
                "validation_rmse": meta["rmse"],
                "path_points": meta["path_points"],
            }
        ]
        if math.isfinite(rmse)
        else [],
        "best": "ILOS_PID" if math.isfinite(rmse) else None,
    }
    with open(out_dir / "validation_summary.json", "w", encoding="utf-8") as f:
        json.dump(summary, f, indent=2)

    status = "PASS" if meta["pass"] else "FAIL"
    print(f"\nILOS_PID rmse={rmse:.4f} m  {status}", flush=True)
    print(f"Results: {out_dir}", flush=True)

    if not args.no_plot:
        try:
            from plot_results import plot_validation_all

            plot_validation_all(out_dir)
        except Exception as exc:
            print(f"plot_validation_all skipped: {exc}", flush=True)


if __name__ == "__main__":
    main()
