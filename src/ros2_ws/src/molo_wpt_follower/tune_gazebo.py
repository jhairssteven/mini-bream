#!/usr/bin/env python3
"""Parameter search for ILOS and polar modes using Gazebo lemniscate tracking tests."""

from __future__ import annotations

import argparse
import copy
import json
import math
import os
import signal
import subprocess
import sys
import time
from pathlib import Path
from typing import Any, Dict, List, Optional

import yaml

ROOT = Path(__file__).resolve().parent
PARAMS = ROOT / "params.yaml"
WS_SETUP = "/opt/ros/humble/setup.bash"
INSTALL_SETUP = "/workspace/codebase/mini-bream/src/ros2_ws/install/setup.bash"


def lemniscate_points(a: float = 10.0, n: int = 16) -> List[dict]:
    pts = []
    for i in range(n):
        t = 2.0 * math.pi * i / n
        denom = 1.0 + math.sin(t) ** 2
        x = a * math.cos(t) / denom
        y = a * math.sin(t) * math.cos(t) / denom
        pts.append({"x": round(x, 3), "y": round(y, 3), "heading": None})
    return pts


def shell(cmd: str, timeout: Optional[float] = None) -> subprocess.CompletedProcess:
    return subprocess.run(
        ["bash", "-lc", cmd],
        capture_output=True,
        text=True,
        timeout=timeout,
    )


def kill_gazebo() -> None:
    shell("pkill -f 'gz sim' 2>/dev/null; pkill -f 'ruby.*gz sim' 2>/dev/null; "
          "pkill -f 'ros2 launch linc_gz' 2>/dev/null; sleep 2; true")


def gazebo_rtf() -> Optional[float]:
    r = shell(
        f"source {WS_SETUP} && timeout 3 gz topic -e -t /world/lake_harner/stats -n 1 2>/dev/null | head -5"
    )
    for line in (r.stdout + r.stderr).splitlines():
        if "real_time_factor" in line.lower() or "rtf" in line.lower():
            for tok in line.replace(":", " ").split():
                try:
                    val = float(tok)
                    if 0.0 < val <= 2.0:
                        return val
                except ValueError:
                    continue
    return None


def gazebo_ready() -> bool:
    r = shell(
        f"source {WS_SETUP} && "
        "timeout 4 ros2 topic echo /wamv/sensors/gps/gps/fix --once 2>/dev/null | grep -q latitude"
    )
    return r.returncode == 0


def ensure_gazebo(min_rtf: float = 0.70, max_attempts: int = 3) -> bool:
    if gazebo_ready():
        rtf = gazebo_rtf()
        if rtf is None or rtf >= min_rtf:
            return True

    kill_gazebo()
    for attempt in range(max_attempts):
        launch_cmd = (
            f"source {WS_SETUP} && source {INSTALL_SETUP} && "
            "ros2 launch linc_gz docking_harner.launch.py headless:=True"
        )
        subprocess.Popen(
            ["bash", "-lc", launch_cmd],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )
        for _ in range(40):
            time.sleep(2)
            if gazebo_ready():
                break
        else:
            kill_gazebo()
            continue

        time.sleep(5)
        rtf = gazebo_rtf()
        if rtf is None or rtf >= min_rtf:
            return True
        print(f"RTF {rtf} below {min_rtf}, restarting gazebo (attempt {attempt + 1})", flush=True)
        kill_gazebo()
    return False


def write_params(cfg: dict, path: Path) -> None:
    with open(path, "w", encoding="utf-8") as f:
        yaml.safe_dump(cfg, f, default_flow_style=False, sort_keys=False)


def run_trial(cfg: dict, duration: float, trial_id: str) -> float:
    tmp = ROOT / f"_tune_{trial_id}.yaml"
    write_params(cfg, tmp)

    follower = subprocess.Popen(
        ["bash", "-lc", f"source {WS_SETUP} && cd {ROOT} && python3 wpt_follower.py --config {tmp}"],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
    )
    time.sleep(12)

    eval_proc = subprocess.run(
        ["bash", "-lc", f"source {WS_SETUP} && cd {ROOT} && python3 evaluate_tracking.py --duration {duration}"],
        capture_output=True,
        text=True,
        timeout=duration + 30,
    )

    follower.send_signal(signal.SIGINT)
    try:
        follower.wait(timeout=5)
    except subprocess.TimeoutExpired:
        follower.kill()

    tmp.unlink(missing_ok=True)

    for line in eval_proc.stdout.splitlines():
        if line.startswith("SCORE "):
            return float(line.split()[1])
    print(eval_proc.stdout, eval_proc.stderr, file=sys.stderr)
    return float("inf")


def base_config(mode: str) -> dict:
    with open(PARAMS, encoding="utf-8") as f:
        cfg = yaml.safe_load(f)
    cfg = copy.deepcopy(cfg)
    cfg["control_mode"] = mode
    cfg["waypoints"] = {"relative_to_start": True, "points": lemniscate_points(a=10.0, n=16)}
    cfg["trajectory"] = {"min_distance_m": 0.3}
    return cfg


def grid_ilos() -> List[dict]:
    return [
        {"dubins.turning_radius_m": 1.0, "ilos.lookahead_min_m": 1.0, "ilos.lookahead_max_m": 3.0,
         "pid.kp": 1.0, "pid.kd": 0.2, "speed.max_linear_pct": 0.03},
        {"dubins.turning_radius_m": 1.2, "ilos.lookahead_min_m": 0.8, "ilos.lookahead_max_m": 3.5,
         "pid.kp": 1.2, "pid.kd": 0.25, "speed.max_linear_pct": 0.025},
        {"dubins.turning_radius_m": 0.8, "ilos.lookahead_min_m": 0.7, "ilos.lookahead_max_m": 2.5,
         "pid.kp": 1.4, "pid.kd": 0.3, "speed.max_linear_pct": 0.02},
        {"dubins.turning_radius_m": 1.5, "ilos.lookahead_min_m": 1.2, "ilos.lookahead_max_m": 4.0,
         "pid.kp": 1.0, "pid.kd": 0.35, "speed.max_linear_pct": 0.03},
        {"dubins.turning_radius_m": 1.0, "ilos.lookahead_min_m": 0.6, "ilos.lookahead_max_m": 2.0,
         "pid.kp": 1.5, "pid.kd": 0.2, "speed.max_linear_pct": 0.02},
        {"dubins.turning_radius_m": 1.2, "ilos.lookahead_min_m": 1.0, "ilos.lookahead_max_m": 2.5,
         "pid.kp": 1.3, "pid.kd": 0.28, "speed.max_linear_pct": 0.022},
        {"dubins.turning_radius_m": 1.8, "ilos.lookahead_min_m": 1.0, "ilos.lookahead_max_m": 3.0,
         "pid.kp": 0.9, "pid.kd": 0.22, "speed.max_linear_pct": 0.028},
        {"dubins.turning_radius_m": 1.0, "ilos.lookahead_min_m": 0.9, "ilos.lookahead_max_m": 3.2,
         "pid.kp": 1.1, "pid.kd": 0.32, "speed.max_linear_pct": 0.024},
    ]


def grid_polar() -> List[dict]:
    return [
        {"polar.kr": 0.07, "polar.ka": 0.8, "polar.kb": 0.1,
         "polar.max_linear": 0.5, "polar.max_angular": 0.2, "polar.arrival_radius_m": 2.0},
        {"polar.kr": 0.08, "polar.ka": 1.1, "polar.kb": 0.12,
         "polar.max_linear": 0.45, "polar.max_angular": 0.25, "polar.arrival_radius_m": 1.7},
        {"polar.kr": 0.075, "polar.ka": 0.95, "polar.kb": 0.08,
         "polar.max_linear": 0.48, "polar.max_angular": 0.22, "polar.arrival_radius_m": 1.8},
        {"polar.kr": 0.065, "polar.ka": 0.85, "polar.kb": 0.06,
         "polar.max_linear": 0.42, "polar.max_angular": 0.18, "polar.arrival_radius_m": 1.5},
        {"polar.kr": 0.09, "polar.ka": 1.0, "polar.kb": 0.1,
         "polar.max_linear": 0.4, "polar.max_angular": 0.2, "polar.arrival_radius_m": 1.6},
        {"polar.kr": 0.07, "polar.ka": 0.9, "polar.kb": 0.14,
         "polar.max_linear": 0.44, "polar.max_angular": 0.23, "polar.arrival_radius_m": 1.9},
    ]


def apply_patch(cfg: dict, patch: dict) -> dict:
    out = copy.deepcopy(cfg)
    for key, val in patch.items():
        parts = key.split(".")
        node = out
        for p in parts[:-1]:
            node = node.setdefault(p, {})
        node[parts[-1]] = val
    return out


def tune_mode(mode: str, duration: float) -> dict:
    grid = grid_ilos() if mode == "ilos" else grid_polar()
    best_score = float("inf")
    best_patch: dict = {}
    base = base_config(mode)

    if not ensure_gazebo():
        print("Failed to start Gazebo with acceptable RTF", file=sys.stderr)
        sys.exit(1)

    for i, patch in enumerate(grid):
        rtf = gazebo_rtf()
        if rtf is not None and rtf < 0.70:
            print(f"RTF dropped to {rtf:.2f}, restarting gazebo", flush=True)
            if not ensure_gazebo():
                break

        cfg = apply_patch(base, patch)
        score = run_trial(cfg, duration, f"{mode}_{i}")
        print(f"[{mode} {i+1}/{len(grid)}] patch={patch} score={score:.4f}", flush=True)
        if score < best_score:
            best_score = score
            best_patch = patch

    return {"mode": mode, "best_score": best_score, "best_patch": best_patch}


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--mode", choices=["ilos", "polar", "both"], default="both")
    parser.add_argument("--duration", type=float, default=75.0)
    args = parser.parse_args()

    results = []
    try:
        if args.mode in ("ilos", "both"):
            results.append(tune_mode("ilos", args.duration))
        if args.mode in ("polar", "both"):
            if not ensure_gazebo():
                sys.exit(1)
            results.append(tune_mode("polar", args.duration))
    finally:
        kill_gazebo()

    out_path = ROOT / "tune_results.json"
    with open(out_path, "w", encoding="utf-8") as f:
        json.dump(results, f, indent=2)

    with open(PARAMS, encoding="utf-8") as f:
        final = yaml.safe_load(f)

    final["waypoints"] = {"relative_to_start": True, "points": lemniscate_points(a=10.0, n=16)}
    for res in results:
        if res["best_patch"]:
            final = apply_patch(final, res["best_patch"])

    write_params(final, PARAMS)
    print(json.dumps(results, indent=2), flush=True)
    print(f"Updated {PARAMS}", flush=True)


if __name__ == "__main__":
    main()
