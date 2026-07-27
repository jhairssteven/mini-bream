#!/usr/bin/env python3
"""Bayesian optimization of MPC weights in Gazebo (target XTE RMSE < 0.1 m)."""

from __future__ import annotations

import argparse
import copy
import json
import signal
import subprocess
import time
from pathlib import Path

import numpy as np
import yaml

ROOT = Path(__file__).resolve().parent
PARAMS = ROOT / "params.yaml"
WS = "/opt/ros/humble/setup.bash"
VRX_INSTALL = "/workspace/codebase/vrx_ws/install/setup.bash"
INSTALL = "/workspace/codebase/mini-bream/src/ros2_ws/install/setup.bash"
OPEN_WATER_LAUNCH = (
    "/workspace/ros2_ws/install/blueboat_sim/share/blueboat_sim/launch/open_water.launch.py"
)
BASH = "bash --noprofile --norc -lc"


def ros_env() -> str:
    return f"source {WS} && source {VRX_INSTALL} && source {INSTALL}"


def shell(cmd: str, timeout: float | None = None) -> subprocess.CompletedProcess:
    return subprocess.run(
        [*BASH.split(), cmd],
        capture_output=True,
        text=True,
        timeout=timeout,
    )


def kill_gazebo() -> None:
    shell(
        "pkill -9 -f 'mpc/mpc.py' 2>/dev/null; pkill -f 'gz sim' 2>/dev/null; "
        "pkill -f 'ruby.*gz sim' 2>/dev/null; pkill -f 'ros2 launch blueboat_sim' 2>/dev/null; "
        "pkill -f 'open_water.launch' 2>/dev/null; sleep 2; true"
    )


def gazebo_ready() -> bool:
    return (
        shell(
            f"{ros_env()} && timeout 4 ros2 topic echo /blueboat/sensors/gps/gps/fix --once 2>/dev/null | grep -q latitude"
        ).returncode
        == 0
    )


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


def write_cfg(cfg: dict, path: Path) -> None:
    with open(path, "w", encoding="utf-8") as f:
        yaml.safe_dump(cfg, f, sort_keys=False)


def run_trial(cfg: dict, duration: float, tag: str) -> float:
    tmp = ROOT / f"_tune_{tag}.yaml"
    write_cfg(cfg, tmp)
    proc = subprocess.Popen(
        [*BASH.split(), f"{ros_env()} && cd {ROOT} && python3 mpc.py --config {tmp}"],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
    )
    time.sleep(14)
    ev = subprocess.run(
        [*BASH.split(), f"{ros_env()} && cd {ROOT} && python3 evaluate_mpc.py --duration {duration}"],
        capture_output=True,
        text=True,
        timeout=duration + 40,
    )
    proc.send_signal(signal.SIGINT)
    try:
        proc.wait(timeout=5)
    except subprocess.TimeoutExpired:
        proc.kill()
    tmp.unlink(missing_ok=True)
    for line in ev.stdout.splitlines():
        if line.startswith("SCORE "):
            return float(line.split()[1])
    return float("inf")


def apply_trial(base: dict, q_xy: float, q_psi: float, r_in: float, horizon: int, cruise: float) -> dict:
    c = copy.deepcopy(base)
    c["mpc"]["horizon"] = int(horizon)
    c["mpc"]["Q_diag"] = [q_xy, q_xy, q_psi, 1.0, 1.0, 0.5 * q_psi]
    c["mpc"]["R_diag"] = [r_in, r_in]
    c["mpc"]["Q_terminal_scale"] = 10.0
    c["path"]["cruise_speed_mps"] = float(cruise)
    return c


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=70.0)
    parser.add_argument("--n-calls", type=int, default=20)
    parser.add_argument(
        "--launch",
        default=OPEN_WATER_LAUNCH,
        help="Full path to launch file (default: mini-bream open_water.launch.py)",
    )
    args = parser.parse_args()

    with open(PARAMS, encoding="utf-8") as f:
        base = yaml.safe_load(f)

    if not ensure_gazebo(args.launch):
        raise SystemExit("Gazebo failed to start")

    try:
        from skopt import gp_minimize
        from skopt.space import Integer, Real
    except ImportError:
        print("scikit-optimize not installed; using random search", flush=True)
        best_score, best_params = float("inf"), {}
        rng = np.random.default_rng(0)
        for i in range(args.n_calls):
            cfg = apply_trial(
                base,
                q_xy=float(rng.uniform(300, 2000)),
                q_psi=float(rng.uniform(80, 300)),
                r_in=float(rng.uniform(0.005, 0.1)),
                horizon=int(rng.integers(15, 35)),
                cruise=float(rng.uniform(0.5, 1.2)),
            )
            score = run_trial(cfg, args.duration, f"r{i}")
            print(f"[random {i+1}] score={score:.4f}", flush=True)
            if score < best_score:
                best_score, best_params = score, cfg
        kill_gazebo()
        if best_params:
            write_cfg(best_params, PARAMS)
        print(json.dumps({"best_score": best_score}, indent=2))
        return

    space = [
        Real(400.0, 2500.0, name="q_xy"),
        Real(80.0, 350.0, name="q_psi"),
        Real(0.005, 0.08, name="r_in"),
        Integer(18, 32, name="horizon"),
        Real(0.55, 1.1, name="cruise"),
    ]

    def objective(x):
        q_xy, q_psi, r_in, horizon, cruise = x
        cfg = apply_trial(base, q_xy, q_psi, r_in, horizon, cruise)
        score = run_trial(cfg, args.duration, f"bo{objective.calls}")
        objective.calls += 1
        print(f"[BO {objective.calls}] score={score:.4f} x={x}", flush=True)
        if not gazebo_ready():
            ensure_gazebo(args.launch)
        return score

    objective.calls = 0
    res = gp_minimize(objective, space, n_calls=args.n_calls, random_state=42, acq_func="EI")

    best = apply_trial(
        base,
        res.x[0],
        res.x[1],
        res.x[2],
        int(res.x[3]),
        res.x[4],
    )
    write_cfg(best, PARAMS)
    kill_gazebo()
    out = {
        "best_score": float(res.fun),
        "best_params": {d.name: float(v) if hasattr(v, "item") else int(v) for d, v in zip(space, res.x)},
    }
    with open(ROOT / "tune_mpc_results.json", "w", encoding="utf-8") as f:
        json.dump(out, f, indent=2)
    print(json.dumps(out, indent=2), flush=True)


if __name__ == "__main__":
    main()
