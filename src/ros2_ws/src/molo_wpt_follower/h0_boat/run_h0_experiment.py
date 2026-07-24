#!/usr/bin/env python3
"""
Run the H0 baseline lemniscate experiment on the real boat (frontseat).

Uses the tuned sim config from mpc/experiments/results/lemniscate_validation/H0_baseline
with a hardware overlay. Produces the same artifacts as the sim validation run:
  config.yaml, log.csv, ref_path.csv, origin.json, result.json, plots/

Prerequisites:
  - frontseat running (RTK GPS, IMU, motor_controller → pwm_daemon)
  - pwm_daemon + radio_rx running on the Pi (see src/teleop/README.md)
  - Radio deadman released so pwm_daemon accepts ROS thrust (arm=false on radio)
"""

from __future__ import annotations

import argparse
import json
import math
import signal
import subprocess
import sys
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

import yaml

PKG_DIR = Path(__file__).resolve().parent
MOLO_DIR = PKG_DIR.parent
MPC_DIR = MOLO_DIR / "mpc"
EXP_DIR = MPC_DIR / "experiments"

for path in (str(PKG_DIR), str(MPC_DIR), str(EXP_DIR)):
    if path not in sys.path:
        sys.path.insert(0, path)

from config import (  # noqa: E402
    BOAT_OVERLAY,
    DEFAULT_RESULTS,
    H0_SIM_CONFIG,
    bridge_topics,
    build_h0_boat_config,
    prepare_run_config,
)


def _mpc_imports():
    from mpc import build_mission_path

    return build_mission_path


def save_ref_path(cfg: dict, path: Path, origin_xy: tuple[float, float] = (0.0, 0.0)) -> None:
    build_mission_path = _mpc_imports()
    path.parent.mkdir(parents=True, exist_ok=True)
    samples, _ = build_mission_path(cfg, origin_xy)
    with open(path, "w", encoding="utf-8") as f:
        f.write("x,y,psi,kappa\n")
        for s in samples:
            f.write(f"{s.x:.6f},{s.y:.6f},{s.psi:.6f},{s.kappa:.6f}\n")


def wait_for_xte(timeout_s: float) -> bool:
    cmd = (
        f"timeout {timeout_s} bash -c "
        "'until ros2 topic echo /molo_mpc/cross_track_error --once 2>/dev/null | grep -q data; do sleep 0.5; done'"
    )
    return subprocess.run(bash_cmd(cmd), capture_output=True).returncode == 0


def ros_env() -> str:
    parts = ["source /opt/ros/humble/setup.bash"]
    ws = Path("/workspace/ros2_ws/install/setup.bash")
    if ws.is_file():
        parts.append(f"source {ws}")
    local_ws = MOLO_DIR.parents[2] / "install" / "setup.bash"
    if local_ws.is_file():
        parts.append(f"source {local_ws}")
    return " && ".join(parts)


def bash_cmd(cmd: str) -> List[str]:
    return ["bash", "--noprofile", "--norc", "-lc", f"{ros_env()} && {cmd}"]


def wait_for_gps(timeout_s: float, gps_topic: str = "/wamv/sensors/gps/gps/fix") -> Optional[Tuple[float, float]]:
    cmd = (
        f"timeout {timeout_s} bash -c "
        f"'until ros2 topic echo {gps_topic} --once 2>/dev/null "
        "| grep -q latitude; do sleep 0.5; done'"
    )
    proc = subprocess.run(bash_cmd(cmd), capture_output=True, text=True)
    if proc.returncode != 0:
        return None

    read_cmd = f"ros2 topic echo {gps_topic} --once"
    proc = subprocess.run(bash_cmd(read_cmd), capture_output=True, text=True, timeout=timeout_s + 5)
    lat = lon = None
    for line in proc.stdout.splitlines():
        line = line.strip()
        if line.startswith("latitude:"):
            lat = float(line.split(":", 1)[1].strip())
        elif line.startswith("longitude:"):
            lon = float(line.split(":", 1)[1].strip())
    if lat is None or lon is None or (lat == 0.0 and lon == 0.0):
        return None
    return lat, lon


def run_evaluate(duration: float, skip_initial: float) -> float:
    cmd = f"cd {MPC_DIR} && python3 evaluate_mpc.py --duration {duration} --skip-initial {skip_initial}"
    proc = subprocess.run(
        bash_cmd(cmd),
        capture_output=True,
        text=True,
        timeout=duration + 90,
    )
    for line in proc.stdout.splitlines():
        if line.startswith("SCORE "):
            return float(line.split()[1])
    return float("inf")


def rmse_from_log(log_path: Path, skip_initial_s: float) -> Optional[float]:
    if not log_path.is_file() or log_path.stat().st_size < 80:
        return None
    samples: List[float] = []
    with open(log_path, encoding="utf-8") as f:
        header = f.readline().strip().split(",")
        try:
            xte_idx = header.index("xte")
            t_idx = header.index("t")
        except ValueError:
            return None
        for line in f:
            parts = line.strip().split(",")
            if len(parts) <= max(xte_idx, t_idx):
                continue
            t = float(parts[t_idx])
            if t < skip_initial_s:
                continue
            xte = abs(float(parts[xte_idx]))
            if math.isfinite(xte):
                samples.append(xte)
    if not samples:
        return None
    return math.sqrt(sum(s * s for s in samples) / len(samples))


def start_process(cmd: str, stderr_path: Optional[Path] = None) -> subprocess.Popen:
    err = open(stderr_path, "w", encoding="utf-8") if stderr_path else subprocess.DEVNULL
    return subprocess.Popen(bash_cmd(cmd), stdout=subprocess.DEVNULL, stderr=err)


def stop_process(proc: subprocess.Popen, grace_s: float = 4.0) -> None:
    if proc.poll() is not None:
        return
    proc.send_signal(signal.SIGINT)
    try:
        proc.wait(timeout=grace_s)
    except subprocess.TimeoutExpired:
        proc.kill()


def run_boat_trial(
    cfg: dict,
    run_dir: Path,
    warmup_s: float,
    evaluate_s: float,
    skip_initial_s: float,
    wait_gps_timeout_s: float,
    wait_xte_timeout_s: float,
) -> float:
    build_mission_path = _mpc_imports()
    bridge = bridge_topics(cfg)
    origin = cfg.get("origin", {})
    origin_lat = origin.get("lat")
    origin_lon = origin.get("lon")

    if origin_lat is None or origin_lon is None:
        gps_topic = cfg.get("topics", {}).get("gps", "/wamv/sensors/gps/gps/fix")
        print(f"Waiting for GPS fix on {gps_topic}...", flush=True)
        fix = wait_for_gps(wait_gps_timeout_s, gps_topic)
        if fix is None:
            print("ERROR: no GPS fix within timeout", flush=True)
            return float("inf")
        origin_lat, origin_lon = fix
        cfg.setdefault("origin", {})
        cfg["origin"]["lat"] = origin_lat
        cfg["origin"]["lon"] = origin_lon
        print(f"Origin set from GPS: lat={origin_lat:.7f}, lon={origin_lon:.7f}", flush=True)

    cfg = prepare_run_config(cfg, run_dir)
    bridge = bridge_topics(cfg)

    procs: List[subprocess.Popen] = []
    stderr_files: List[Any] = []

    stack_cmd = (
        f"cd {PKG_DIR} && exec python3 stack_runner.py --config {run_dir / 'config.yaml'}"
    )
    stack_err = open(run_dir / "stack_stderr.log", "w", encoding="utf-8")
    stderr_files.append(stack_err)
    procs.append(subprocess.Popen(bash_cmd(stack_cmd), stdout=subprocess.DEVNULL, stderr=stack_err))

    try:
        print(f"Warmup {warmup_s:.0f}s...", flush=True)
        time.sleep(warmup_s)

        origin_file = run_dir / "origin.json"
        origin_xy = (0.0, 0.0)
        if origin_file.exists():
            with open(origin_file, encoding="utf-8") as f:
                o = json.load(f)
                origin_xy = (float(o["x"]), float(o["y"]))
        save_ref_path(cfg, run_dir / "ref_path.csv", origin_xy)

        if not wait_for_xte(wait_xte_timeout_s):
            print(
                "WARNING: cross-track error topic not seen; scoring from log.csv if available",
                flush=True,
            )

        rmse = run_evaluate(evaluate_s, skip_initial_s)
        if not math.isfinite(rmse):
            log_rmse = rmse_from_log(run_dir / "log.csv", skip_initial_s)
            if log_rmse is not None:
                rmse = log_rmse

        return rmse
    finally:
        for proc in procs:
            stop_process(proc)
        for fh in stderr_files:
            fh.close()
        time.sleep(1)


def main() -> None:
    parser = argparse.ArgumentParser(description="H0 baseline lemniscate experiment on real boat")
    parser.add_argument("--overlay", default=str(BOAT_OVERLAY), help="Hardware overlay YAML")
    parser.add_argument("--out", default=None, help="Output directory (default: h0_boat/results/<timestamp>)")
    parser.add_argument("--origin-lat", type=float, default=None, help="Override origin latitude")
    parser.add_argument("--origin-lon", type=float, default=None, help="Override origin longitude")
    parser.add_argument("--warmup", type=float, default=None)
    parser.add_argument("--duration", type=float, default=None)
    parser.add_argument("--skip-initial", type=float, default=None)
    parser.add_argument("--dry-run", action="store_true", help="Write config and ref_path only")
    parser.add_argument("--no-plot", action="store_true")
    args = parser.parse_args()

    origin = None
    if args.origin_lat is not None and args.origin_lon is not None:
        origin = (args.origin_lat, args.origin_lon)

    cfg = build_h0_boat_config(args.overlay, origin_latlon=origin)
    exp = cfg.get("experiment", {})

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
    run_dir = out_dir / "H0_baseline"
    run_dir.mkdir(parents=True, exist_ok=True)

    with open(out_dir / "reference_path.yaml", "w", encoding="utf-8") as f:
        ref_block = {
            "waypoints": cfg.get("waypoints", {}),
            "path": {k: v for k, v in cfg.get("path", {}).items() if k != "cruise_speed_mps"},
            "dubins": cfg.get("dubins", {}),
        }
        yaml.safe_dump(ref_block, f, sort_keys=False)

    try:
        samples, _ = _mpc_imports()(cfg, (0.0, 0.0))
        path_points = len(samples)
    except Exception as exc:
        print(f"WARNING: could not build mission path ({exc}); using sim reference count", flush=True)
        path_points = 77
    meta = {
        "hypothesis_id": "H0_baseline",
        "approach": cfg.get("control", {}).get("approach", "baseline_ilos_velocity"),
        "mpc_mode": cfg.get("mpc", {}).get("mode", "velocity"),
        "path_points": path_points,
        "cruise_speed_mps": cfg.get("path", {}).get("cruise_speed_mps"),
        "source_config": str(H0_SIM_CONFIG),
        "boat_overlay": str(args.overlay),
        "platform": "real_boat",
    }

    print(
        f"H0 boat experiment\n"
        f"  sim config: {H0_SIM_CONFIG}\n"
        f"  overlay:    {args.overlay}\n"
        f"  output:     {run_dir}\n"
        f"  path_pts:   {meta['path_points']}\n"
        f"  cruise:     {meta['cruise_speed_mps']} m/s",
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
        "reference_hypothesis": "H0_baseline",
        "reference_path": ref_block,
        "target_rmse_m": target_rmse_m,
        "platform": "real_boat",
        "runs": [meta],
        "ranked": [
            {
                "hypothesis_id": "H0_baseline",
                "approach": meta["approach"],
                "validation_rmse": meta["rmse"],
                "path_points": meta["path_points"],
            }
        ]
        if math.isfinite(rmse)
        else [],
        "best": "H0_baseline" if math.isfinite(rmse) else None,
    }
    with open(out_dir / "validation_summary.json", "w", encoding="utf-8") as f:
        json.dump(summary, f, indent=2)

    status = "PASS" if meta["pass"] else "FAIL"
    print(f"\nH0_baseline rmse={rmse:.4f} m  {status}", flush=True)
    print(f"Results: {out_dir}", flush=True)

    if not args.no_plot:
        try:
            from plot_results import plot_validation_all

            plot_validation_all(out_dir)
        except Exception as exc:
            print(f"plot_validation_all skipped: {exc}", flush=True)


if __name__ == "__main__":
    main()
