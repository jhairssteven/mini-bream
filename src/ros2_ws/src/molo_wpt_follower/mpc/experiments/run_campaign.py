#!/usr/bin/env python3
"""
Scientific campaign: hypothesis -> test (Gazebo) -> log -> plot -> aggregate.

Methodology documented in hypotheses.yaml per approach.
"""

from __future__ import annotations

import argparse
import copy
import json
import math
import signal
import subprocess
import sys
import time
from datetime import datetime, timezone
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from mpc import build_mission_path, load_config
from plot_results import plot_all

try:
    from tune_mpc import OPEN_WATER_LAUNCH, BASH, ensure_gazebo, gazebo_ready, kill_gazebo, ros_env
except ImportError:
    OPEN_WATER_LAUNCH = (
        "/workspace/ros2_ws/install/blueboat_sim/share/blueboat_sim/launch/open_water.launch.py"
    )
    BASH = "bash --noprofile --norc -lc"

    def ros_env() -> str:
        return (
            "source /opt/ros/humble/setup.bash && "
            "source /workspace/codebase/vrx_ws/install/setup.bash && "
            "source /workspace/codebase/mini-bream/src/ros2_ws/install/setup.bash"
        )

    def kill_gazebo() -> None:
        subprocess.run([*BASH.split(), "pkill -9 -f 'mpc/mpc.py'; pkill -9 -f 'gz sim'; sleep 2; true"], check=False)

    def gazebo_ready() -> bool:
        return subprocess.run(
            [*BASH.split(), f"{ros_env()} && timeout 4 ros2 topic echo /blueboat/sensors/gps/gps/fix --once 2>/dev/null | grep -q latitude"],
            capture_output=True,
        ).returncode == 0

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


def deep_merge(base: dict, patch: dict) -> dict:
    out = copy.deepcopy(base)
    for k, v in patch.items():
        if isinstance(v, dict) and isinstance(out.get(k), dict):
            out[k] = deep_merge(out[k], v)
        else:
            out[k] = copy.deepcopy(v)
    return out


def merge_traj_cfg(base: dict, traj_entry: dict) -> dict:
    cfg = copy.deepcopy(base)
    wp = cfg.setdefault("waypoints", {})
    wp["relative_to_start"] = True
    wp.pop("lemniscate", None)
    wp.pop("points", None)
    wp["trajectory"] = copy.deepcopy(traj_entry["trajectory"])
    for section in ("path", "ilos", "control", "guidance", "mpc"):
        if section in traj_entry:
            cfg.setdefault(section, {}).update(copy.deepcopy(traj_entry[section]))
    return cfg


def save_ref_path(cfg: dict, path: Path, origin_xy: tuple[float, float] = (0.0, 0.0)) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    samples, _ = build_mission_path(cfg, origin_xy)
    with open(path, "w", encoding="utf-8") as f:
        f.write("x,y,psi,kappa\n")
        for s in samples:
            f.write(f"{s.x:.6f},{s.y:.6f},{s.psi:.6f},{s.kappa:.6f}\n")


def wait_for_xte(timeout_s: float = 25.0) -> bool:
    cmd = (
        f"{ros_env()} && timeout {timeout_s} bash -c "
        "'until ros2 topic echo /molo_mpc/cross_track_error --once 2>/dev/null | grep -q data; do sleep 0.5; done'"
    )
    return subprocess.run([*BASH.split(), cmd], capture_output=True).returncode == 0


def run_evaluate(duration: float, skip_initial: float) -> float:
    cmd = f"{ros_env()} && cd {ROOT} && python3 evaluate_mpc.py --duration {duration} --skip-initial {skip_initial}"
    proc = subprocess.run([*BASH.split(), cmd], capture_output=True, text=True, timeout=duration + 90)
    for line in proc.stdout.splitlines():
        if line.startswith("SCORE "):
            return float(line.split()[1])
    return float("inf")


def run_trial(
    cfg: dict,
    run_dir: Path,
    warmup: float,
    duration: float,
    skip_initial: float,
    launch: str,
    restart_sim: bool,
) -> float:
    subprocess.run([*BASH.split(), f"{ros_env()} && pkill -9 -f 'mpc/mpc.py' 2>/dev/null; true"], check=False)
    time.sleep(1)
    if restart_sim:
        kill_gazebo()
        time.sleep(3)
        if not ensure_gazebo(launch):
            return float("inf")

    cfg = copy.deepcopy(cfg)
    cfg.setdefault("experiment", {})["log_csv"] = str(run_dir / "log.csv")

    tmp = run_dir / "config.yaml"
    with open(tmp, "w", encoding="utf-8") as f:
        yaml.safe_dump(cfg, f, sort_keys=False)

    err_log = open(run_dir / "mpc_stderr.log", "w", encoding="utf-8")
    mpc = subprocess.Popen(
        [*BASH.split(), f"{ros_env()} && cd {ROOT} && exec python3 mpc.py --config {tmp}"],
        stdout=subprocess.DEVNULL,
        stderr=err_log,
    )
    time.sleep(warmup)
    if not gazebo_ready():
        mpc.kill()
        err_log.close()
        return float("inf")
    origin_xy = (0.0, 0.0)
    origin_file = run_dir / "origin.json"
    if origin_file.exists():
        with open(origin_file, encoding="utf-8") as f:
            o = json.load(f)
            origin_xy = (float(o["x"]), float(o["y"]))
    save_ref_path(cfg, run_dir / "ref_path.csv", origin_xy)
    if not wait_for_xte(20.0):
        mpc.kill()
        err_log.close()
        return float("inf")
    rmse = run_evaluate(duration, skip_initial)
    err_log.close()
    mpc.send_signal(signal.SIGINT)
    try:
        mpc.wait(timeout=6)
    except subprocess.TimeoutExpired:
        mpc.kill()
    time.sleep(3)
    return rmse


def main() -> None:
    parser = argparse.ArgumentParser(description="MPC scientific experiment campaign")
    parser.add_argument("--hypotheses", default=str(ROOT / "experiments" / "hypotheses.yaml"))
    parser.add_argument("--params", default=str(ROOT / "params.yaml"))
    parser.add_argument("--trajectories", default=str(ROOT / "trajectories.yaml"))
    parser.add_argument("--out", default=None, help="Campaign output directory")
    parser.add_argument("--launch", default=OPEN_WATER_LAUNCH)
    parser.add_argument("--no-restart", action="store_true")
    parser.add_argument("--hypothesis", nargs="*", help="Subset of hypothesis ids")
    parser.add_argument("--plot-only", action="store_true")
    parser.add_argument(
        "--trajectory",
        nargs="*",
        help="Override curved_trajectories list (e.g. circle lemniscate)",
    )
    args = parser.parse_args()

    hyp_path = Path(args.hypotheses)
    with open(hyp_path, encoding="utf-8") as f:
        campaign = yaml.safe_load(f)
    with open(args.params, encoding="utf-8") as f:
        base_params = yaml.safe_load(f)
    with open(args.trajectories, encoding="utf-8") as f:
        traj_suite = yaml.safe_load(f)

    target = float(campaign.get("target_rmse_m", 0.1))
    warmup = float(campaign.get("warmup_s", 20.0))
    duration = float(campaign.get("evaluate_s", 55.0))
    skip_initial = float(campaign.get("skip_initial_s", 15.0))
    curved = args.trajectory or campaign.get(
        "curved_trajectories", ["circle", "lemniscate", "sine", "rectangle", "triangle"]
    )

    stamp = datetime.now(timezone.utc).strftime("%Y%m%d_%H%M%S")
    campaign_dir = Path(args.out) if args.out else ROOT / "experiments" / "results" / stamp
    campaign_dir.mkdir(parents=True, exist_ok=True)

    if args.plot_only:
        plot_all(campaign_dir)
        print(f"Plots written under {campaign_dir}")
        return

    traj_by_name = {t["name"]: t for t in traj_suite.get("trajectories", [])}
    run_trajs = [traj_by_name[n] for n in curved if n in traj_by_name]

    hypotheses = campaign.get("hypotheses", [])
    if args.hypothesis:
        allow = set(args.hypothesis)
        hypotheses = [h for h in hypotheses if h["id"] in allow]

    # Write methodology manifest
    with open(campaign_dir / "hypotheses.yaml", "w", encoding="utf-8") as f:
        yaml.safe_dump(campaign, f, sort_keys=False)

    kill_gazebo()
    time.sleep(2)
    if not ensure_gazebo(args.launch):
        raise SystemExit("Gazebo failed to start")

    runs: list = []
    for hyp in hypotheses:
        hid = hyp["id"]
        print(f"\n=== {hid}: {hyp.get('hypothesis', '')} ===", flush=True)
        cfg_hyp = deep_merge(base_params, hyp.get("config_patch", {}))

        for traj in run_trajs:
            tname = traj["name"]
            cfg = merge_traj_cfg(cfg_hyp, traj)
            run_dir = campaign_dir / hid / tname
            run_dir.mkdir(parents=True, exist_ok=True)

            meta = {
                "hypothesis_id": hid,
                "hypothesis": hyp.get("hypothesis", ""),
                "prediction": hyp.get("prediction", ""),
                "trajectory": tname,
                "run_dir": str(run_dir),
                "approach": cfg.get("control", {}).get("approach", ""),
            }
            with open(run_dir / "meta.json", "w", encoding="utf-8") as f:
                json.dump(meta, f, indent=2)

            print(f"  [{tname}] ...", flush=True)
            rmse = run_trial(
                cfg,
                run_dir,
                warmup,
                duration,
                skip_initial,
                args.launch,
                restart_sim=not args.no_restart,
            )
            meta["rmse"] = rmse
            meta["pass"] = math.isfinite(rmse) and rmse < target
            meta["target_rmse_m"] = target
            runs.append(meta)
            status = "PASS" if meta["pass"] else "FAIL"
            print(f"  [{tname}] rmse={rmse:.4f} m  {status}", flush=True)

    kill_gazebo()

    # Analysis notes per hypothesis
    analysis = []
    for hid in {r["hypothesis_id"] for r in runs}:
        sub = [r for r in runs if r["hypothesis_id"] == hid]
        rmses = [r["rmse"] for r in sub if math.isfinite(r["rmse"])]
        analysis.append(
            {
                "hypothesis_id": hid,
                "mean_rmse_curved": float(sum(rmses) / len(rmses)) if rmses else None,
                "worst_rmse_curved": float(max(rmses)) if rmses else None,
                "pass_count": sum(1 for r in sub if r.get("pass")),
                "n": len(sub),
            }
        )

    summary = {
        "timestamp": stamp,
        "target_rmse_m": target,
        "curved_trajectories": curved,
        "runs": runs,
        "analysis": analysis,
        "best_hypothesis": min(
            analysis,
            key=lambda a: a["mean_rmse_curved"] if a["mean_rmse_curved"] is not None else float("inf"),
        )["hypothesis_id"]
        if analysis
        else None,
    }
    with open(campaign_dir / "campaign_results.json", "w", encoding="utf-8") as f:
        json.dump(summary, f, indent=2)

    plot_all(campaign_dir)

    print(f"\nCampaign complete: {campaign_dir}", flush=True)
    print(json.dumps({"analysis": analysis, "best": summary["best_hypothesis"]}, indent=2), flush=True)


if __name__ == "__main__":
    main()
