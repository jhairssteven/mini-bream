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

from config import bridge_topics, stack_components  # noqa: E402
from ilos_follower import IlosFollowerNode  # noqa: E402
from pose_trail_viz import PoseTrailViz  # noqa: E402
from trial_runner import wait_for_gps  # noqa: E402
from velocity_odom import VelocityOdomNode  # noqa: E402


def load_yaml(path: Path | str) -> dict:
    with open(path, "r", encoding="utf-8") as f:
        return yaml.safe_load(f) or {}


def main() -> None:
    parser = argparse.ArgumentParser(description="Run ilos_boat ROS stack in one process")
    parser.add_argument("--config", required=True, help="Merged experiment config YAML")
    args = parser.parse_args()

    cfg = load_yaml(args.config)
    bridge = bridge_topics(cfg)
    components = stack_components(cfg)
    origin = cfg.get("origin", {})
    viz = cfg.get("viz", {})

    origin_lat = origin.get("lat")
    origin_lon = origin.get("lon")
    if origin_lat is None or origin_lon is None:
        gps_topic = bridge.get("gps_topic", "/wamv/sensors/gps/gps/fix")
        wait_s = float(cfg.get("experiment", {}).get("wait_for_gps_timeout_s", 60.0))
        fix = wait_for_gps(wait_s, gps_topic, cfg)
        if fix is None:
            raise RuntimeError(
                f"GPS origin unavailable on {gps_topic} after {wait_s:.0f}s; "
                "start frontseat moving_base_rtk or pass origin lat/lon."
            )
        origin_lat, origin_lon = fix
        cfg.setdefault("origin", {})
        cfg["origin"]["lat"] = origin_lat
        cfg["origin"]["lon"] = origin_lon

    init_args = []
    if cfg.get("use_sim_time") or os.environ.get("MOLO_USE_SIM_TIME", "").lower() in (
        "1",
        "true",
        "yes",
    ):
        init_args = ["--ros-args", "-p", "use_sim_time:=true"]
    rclpy.init(args=init_args if init_args else None)
    executor = MultiThreadedExecutor(num_threads=6)
    nodes = []

    if components["velocity_odom"]:
        nodes.append(
            VelocityOdomNode(
                bridge.get("gps_topic", "/wamv/sensors/gps/gps/fix"),
                bridge.get("imu_topic", "/wamv/sensors/imu/imu/data"),
                bridge.get("estimated_odom_topic", "/molo_boat/estimated_odometry"),
                float(origin_lat),
                float(origin_lon),
                frame_id="map",
                child_frame_id="base_link",
                window_s=float(bridge.get("velocity_window_s", 0.4)),
            )
        )
    nodes.append(
        PoseTrailViz(
            viz.get("pose_topic", "/molo_mpc/vehicle_pose"),
            viz.get("recent_poses_topic", "/molo_ilos/recent_poses"),
            viz.get("frame_id", "world"),
            max_poses=int(viz.get("max_recent_poses", 40)),
        )
    )
    nodes.append(IlosFollowerNode(cfg))

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
