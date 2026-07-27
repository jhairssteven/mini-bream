"""Shared ILOS experiment trial runner (used by run_ilos_experiment and tune_ilos)."""

from __future__ import annotations

import json
import math
import signal
import subprocess
import time
from pathlib import Path
from typing import List, Optional, Tuple

PKG_DIR = Path(__file__).resolve().parent
MOLO_DIR = PKG_DIR.parent
MPC_DIR = MOLO_DIR / "mpc"
EXP_DIR = MPC_DIR / "experiments"

import sys

for path in (str(PKG_DIR), str(MPC_DIR), str(EXP_DIR)):
    if path not in sys.path:
        sys.path.insert(0, path)

from config import prepare_run_config  # noqa: E402


def _mpc_imports():
    from mpc import build_mission_path

    return build_mission_path


def ros_env(cfg: Optional[dict] = None) -> str:
    parts = ["source /opt/ros/humble/setup.bash"]
    ws = Path("/workspace/ros2_ws/install/setup.bash")
    if ws.is_file():
        parts.append(f"source {ws}")
    vrx = Path("/workspace/vrx_ws/install/setup.bash")
    if vrx.is_file():
        parts.append(f"source {vrx}")
    local_ws = MOLO_DIR.parents[2] / "install" / "setup.bash"
    if local_ws.is_file():
        parts.append(f"source {local_ws}")
    if cfg and cfg.get("use_sim_time"):
        parts.append("export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}")
    return " && ".join(parts)


def bash_cmd(cmd: str, cfg: Optional[dict] = None) -> List[str]:
    return ["bash", "--noprofile", "--norc", "-lc", f"{ros_env(cfg)} && {cmd}"]


def kill_stack() -> None:
    subprocess.run(
        bash_cmd("pkill -f 'ilos_boat/stack_runner.py' 2>/dev/null; sleep 0.5; true"),
        capture_output=True,
    )


def save_ref_path(cfg: dict, path: Path, origin_xy: tuple[float, float] = (0.0, 0.0)) -> None:
    build_mission_path = _mpc_imports()
    path.parent.mkdir(parents=True, exist_ok=True)
    samples, _ = build_mission_path(cfg, origin_xy)
    with open(path, "w", encoding="utf-8") as f:
        f.write("x,y,psi,kappa\n")
        for s in samples:
            f.write(f"{s.x:.6f},{s.y:.6f},{s.psi:.6f},{s.kappa:.6f}\n")


def wait_for_xte(timeout_s: float, cfg: Optional[dict] = None) -> bool:
    cmd = (
        f"timeout {timeout_s} bash -c "
        "'until ros2 topic echo /molo_mpc/cross_track_error --once 2>/dev/null | grep -q data; do sleep 0.5; done'"
    )
    return subprocess.run(bash_cmd(cmd, cfg), capture_output=True).returncode == 0


def wait_for_gps(
    timeout_s: float,
    gps_topic: str,
    cfg: Optional[dict] = None,
) -> Optional[Tuple[float, float]]:
    cmd = (
        f"timeout {timeout_s} bash -c "
        f"'until ros2 topic echo {gps_topic} --once 2>/dev/null "
        "| grep -q latitude; do sleep 0.5; done'"
    )
    proc = subprocess.run(bash_cmd(cmd, cfg), capture_output=True, text=True)
    if proc.returncode != 0:
        return None

    read_cmd = f"ros2 topic echo {gps_topic} --once"
    proc = subprocess.run(
        bash_cmd(read_cmd, cfg), capture_output=True, text=True, timeout=timeout_s + 5
    )
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


def run_evaluate(duration: float, skip_initial: float, cfg: Optional[dict] = None) -> float:
    cmd = f"cd {MPC_DIR} && python3 evaluate_mpc.py --duration {duration} --skip-initial {skip_initial}"
    proc = subprocess.run(
        bash_cmd(cmd, cfg),
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


def start_process(cmd: str, cfg: Optional[dict] = None, stderr_path: Optional[Path] = None) -> subprocess.Popen:
    err = open(stderr_path, "w", encoding="utf-8") if stderr_path else subprocess.DEVNULL
    return subprocess.Popen(bash_cmd(cmd, cfg), stdout=subprocess.DEVNULL, stderr=err)


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
    wait_gps_timeout_s: float = 60.0,
    wait_xte_timeout_s: float = 45.0,
) -> float:
    """Run one scored ILOS trial; returns XTE RMSE (inf on failure)."""
    kill_stack()
    time.sleep(0.5)

    origin = cfg.get("origin", {})
    origin_lat = origin.get("lat")
    origin_lon = origin.get("lon")

    if origin_lat is None or origin_lon is None:
        gps_topic = cfg.get("topics", {}).get("gps", "/wamv/sensors/gps/gps/fix")
        fix = wait_for_gps(wait_gps_timeout_s, gps_topic, cfg)
        if fix is None:
            return float("inf")
        origin_lat, origin_lon = fix
        cfg = dict(cfg)
        cfg.setdefault("origin", {})
        cfg["origin"]["lat"] = origin_lat
        cfg["origin"]["lon"] = origin_lon

    cfg = prepare_run_config(cfg, run_dir)
    stack_cmd = f"cd {PKG_DIR} && exec python3 stack_runner.py --config {run_dir / 'config.yaml'}"
    proc = start_process(stack_cmd, cfg=cfg, stderr_path=run_dir / "stack_stderr.log")

    try:
        time.sleep(warmup_s)

        origin_file = run_dir / "origin.json"
        origin_xy = (0.0, 0.0)
        if origin_file.exists():
            with open(origin_file, encoding="utf-8") as f:
                o = json.load(f)
                origin_xy = (float(o["x"]), float(o["y"]))
        save_ref_path(cfg, run_dir / "ref_path.csv", origin_xy)

        if not wait_for_xte(wait_xte_timeout_s, cfg):
            rmse = rmse_from_log(run_dir / "log.csv", skip_initial_s)
            return rmse if rmse is not None else float("inf")

        rmse = run_evaluate(evaluate_s, skip_initial_s, cfg)
        if not math.isfinite(rmse):
            log_rmse = rmse_from_log(run_dir / "log.csv", skip_initial_s)
            if log_rmse is not None:
                rmse = log_rmse
        return rmse
    finally:
        stop_process(proc)
        kill_stack()
        time.sleep(0.5)
