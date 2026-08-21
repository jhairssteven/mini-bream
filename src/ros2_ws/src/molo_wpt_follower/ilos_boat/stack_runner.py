#!/usr/bin/env python3
"""Run ilos_boat helper nodes + ILOS follower in a single ROS process."""

from __future__ import annotations

import argparse
import os
import signal
import sys
import threading
import time
from pathlib import Path

import rclpy
import yaml
from rclpy.executors import MultiThreadedExecutor

PKG_DIR = Path(__file__).resolve().parent
H0_DIR = PKG_DIR.parent / "h0_boat"
for path in (str(PKG_DIR), str(H0_DIR)):
    if path not in sys.path:
        sys.path.insert(0, path)

from ilos_follower import IlosFollowerNode  # noqa: E402
from pose_trail_viz import PoseTrailViz  # noqa: E402


def load_yaml(path: Path | str) -> dict:
    with open(path, "r", encoding="utf-8") as f:
        return yaml.safe_load(f) or {}


def main() -> None:
    parser = argparse.ArgumentParser(description="Run ilos_boat ROS stack in one process")
    parser.add_argument("--config", required=True, help="Merged experiment config YAML")
    args = parser.parse_args()

    cfg = load_yaml(args.config)
    viz = cfg.get("viz", {})

    init_args = []
    if cfg.get("use_sim_time") or os.environ.get("MOLO_USE_SIM_TIME", "").lower() in (
        "1",
        "true",
        "yes",
    ):
        init_args = ["--ros-args", "-p", "use_sim_time:=true"]
    rclpy.init(args=init_args if init_args else None)
    executor = MultiThreadedExecutor(num_threads=6)
    nodes = [
        PoseTrailViz(
            viz.get("pose_topic", "/molo_mpc/vehicle_pose"),
            viz.get("recent_poses_topic", "/molo_ilos/recent_poses"),
            viz.get("frame_id", "map"),
            max_poses=int(viz.get("max_recent_poses", 40)),
        ),
        IlosFollowerNode(cfg),
    ]

    for node in nodes:
        executor.add_node(node)

    stop = threading.Event()

    def _shutdown(*_args):
        stop.set()
        executor.shutdown()

    signal.signal(signal.SIGINT, _shutdown)
    signal.signal(signal.SIGTERM, _shutdown)

    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    try:
        while not stop.is_set():
            time.sleep(0.2)
    finally:
        executor.shutdown()
        for node in nodes:
            node.destroy_node()
        rclpy.shutdown()
        spin_thread.join(timeout=2.0)


if __name__ == "__main__":
    main()
