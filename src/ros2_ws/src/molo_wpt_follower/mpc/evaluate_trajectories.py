#!/usr/bin/env python3
"""Run MPC across a diverse trajectory suite in open-water Gazebo."""

from __future__ import annotations

import argparse
import copy
import json
import math
import signal
import subprocess
import time
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parent
PARAMS = ROOT / "params.yaml"
TRAJ_SUITE = ROOT / "trajectories.yaml"
RESULTS = ROOT / "trajectory_eval_results.json"

try:
    from tune_mpc import OPEN_WATER_LAUNCH, kill_gazebo, ros_env, ensure_gazebo, gazebo_ready, BASH
except ImportError:
    OPEN_WATER_LAUNCH = (
        "/workspace/codebase/mini-bream/src/ros2_ws/install/linc_gz/share/linc_gz/launch/open_water.launch.py"
    )
    BASH = "bash --noprofile --norc -lc"

    def ros_env() -> str:
        return (
            "source /opt/ros/humble/setup.bash && "
            "source /workspace/codebase/vrx_ws/install/setup.bash && "
            "source /workspace/codebase/mini-bream/src/ros2_ws/install/setup.bash"
        )

    def kill_gazebo() -> None:
        subprocess.run(
            [*BASH.split(), "pkill -9 -f 'mpc/mpc.py'; pkill -9 -f 'gz sim'; sleep 2; true"],
            check=False,
        )

    def gazebo_ready() -> bool:
        r = subprocess.run(
            [*BASH.split(), f"{ros_env()} && timeout 4 ros2 topic echo /wamv/sensors/gps/gps/fix --once 2>/dev/null | grep -q latitude"],
            capture_output=True,
        )
        return r.returncode == 0

    def ensure_gazebo(launch: str = OPEN_WATER_LAUNCH) -> bool:
        if gazebo_ready():
            return True
        kill_gazebo()
        subprocess.Popen(
            [*BASH.split(), f"{ros_env()} && ros2 launch {launch} headless:=True"],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )
        for _ in range(45):
            time.sleep(2)
            if gazebo_ready():
                time.sleep(5)
                return True
        return False


def merge_cfg(base: dict, traj_entry: dict) -> dict:
    cfg = copy.deepcopy(base)
    wp = cfg.setdefault("waypoints", {})
    wp["relative_to_start"] = True
    wp.pop("lemniscate", None)
    wp.pop("points", None)
    wp["trajectory"] = copy.deepcopy(traj_entry["trajectory"])
    for k, v in traj_entry.get("path", {}).items():
        cfg.setdefault("path", {})[k] = v
    for k, v in traj_entry.get("ilos", {}).items():
        cfg.setdefault("ilos", {})[k] = v
    return cfg


def run_evaluate(duration: float, skip_initial: float, warmup: float) -> tuple[float, int]:
    """Return (rmse, sample_count) from evaluate_mpc subprocess."""
    cmd = (
        f"{ros_env()} && cd {ROOT} && "
        f"python3 evaluate_mpc.py --duration {duration} --skip-initial {skip_initial}"
    )
    proc = subprocess.run([*BASH.split(), cmd], capture_output=True, text=True, timeout=duration + 60)
    rmse = float("inf")
    n = 0
    for line in proc.stdout.splitlines():
        if line.startswith("SCORE "):
            rmse = float(line.split()[1])
        if "n=" in line and "rmse=" in line:
            try:
                n = int(line.split("n=")[-1].strip())
            except ValueError:
                pass
    return rmse, n


def run_trajectory(
    cfg: dict,
    name: str,
    *,
    warmup: float,
    duration: float,
    skip_initial: float,
    restart_sim: bool,
    launch: str,
) -> dict:
    subprocess.run(
        [*BASH.split(), f"{ros_env()} && pkill -9 -f 'mpc/mpc.py' 2>/dev/null; true"],
        check=False,
    )
    time.sleep(1)

    if restart_sim:
        kill_gazebo()
        time.sleep(3)
        if not ensure_gazebo(launch):
            return {"name": name, "rmse": float("inf"), "pass": False, "error": "gazebo_start_failed"}

    tmp = ROOT / f"_eval_{name}.yaml"
    with open(tmp, "w", encoding="utf-8") as f:
        yaml.safe_dump(cfg, f, sort_keys=False)

    mpc = subprocess.Popen(
        [*BASH.split(), f"{ros_env()} && cd {ROOT} && exec python3 mpc.py --config {tmp}"],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
    )
    time.sleep(warmup)
    if not gazebo_ready():
        mpc.kill()
        tmp.unlink(missing_ok=True)
        return {"name": name, "rmse": float("inf"), "pass": False, "error": "gazebo_lost"}
    rmse, n = run_evaluate(duration, skip_initial, warmup)
    mpc.send_signal(signal.SIGINT)
    try:
        mpc.wait(timeout=5)
    except subprocess.TimeoutExpired:
        mpc.kill()
    tmp.unlink(missing_ok=True)
    return {"name": name, "rmse": rmse, "n": n, "pass": math.isfinite(rmse)}


def main() -> None:
    parser = argparse.ArgumentParser(description="MPC multi-trajectory benchmark")
    parser.add_argument("--suite", default=str(TRAJ_SUITE))
    parser.add_argument("--params", default=str(PARAMS))
    parser.add_argument("--target-rmse", type=float, default=None)
    parser.add_argument("--restart-each", action="store_true", default=True)
    parser.add_argument("--launch", default=OPEN_WATER_LAUNCH)
    parser.add_argument("--names", nargs="*", help="Subset of trajectory names")
    args = parser.parse_args()

    with open(args.suite, encoding="utf-8") as f:
        suite = yaml.safe_load(f)
    with open(args.params, encoding="utf-8") as f:
        base = yaml.safe_load(f)

    target = args.target_rmse if args.target_rmse is not None else float(suite.get("target_rmse_m", 0.1))
    warmup = float(suite.get("warmup_s", 18.0))
    duration = float(suite.get("evaluate_s", 55.0))
    skip_initial = float(suite.get("skip_initial_s", 12.0))

    trajs = suite.get("trajectories", [])
    if args.names:
        names = set(args.names)
        trajs = [t for t in trajs if t["name"] in names]

    kill_gazebo()
    time.sleep(2)
    if not ensure_gazebo(args.launch):
        raise SystemExit("Gazebo failed to start")

    results = []
    for entry in trajs:
        name = entry["name"]
        cfg = merge_cfg(base, entry)
        print(f"[eval] {name} ...", flush=True)
        row = run_trajectory(
            cfg,
            name,
            warmup=warmup,
            duration=duration,
            skip_initial=skip_initial,
            restart_sim=args.restart_each,
            launch=args.launch,
        )
        row["pass"] = row.get("rmse", float("inf")) < target
        row["target_rmse_m"] = target
        results.append(row)
        print(f"  {name}: rmse={row['rmse']:.4f} m  {'PASS' if row['pass'] else 'FAIL'}", flush=True)

    kill_gazebo()
    worst = max((r["rmse"] for r in results if math.isfinite(r["rmse"])), default=float("inf"))
    passed = sum(1 for r in results if r.get("pass"))
    summary = {
        "target_rmse_m": target,
        "passed": passed,
        "total": len(results),
        "worst_rmse_m": worst,
        "all_pass": passed == len(results) and len(results) > 0,
        "trajectories": results,
    }
    with open(RESULTS, "w", encoding="utf-8") as f:
        json.dump(summary, f, indent=2)
    print(json.dumps(summary, indent=2), flush=True)
    raise SystemExit(0 if summary["all_pass"] else 1)


if __name__ == "__main__":
    main()
