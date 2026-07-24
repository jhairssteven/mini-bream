#!/usr/bin/env python3
"""Run h0_boat helper nodes + MPC in a single ROS process."""

from __future__ import annotations

import argparse
import signal
import sys
import threading
import time
from pathlib import Path

import rclpy
import yaml
from rclpy.executors import MultiThreadedExecutor

PKG_DIR = Path(__file__).resolve().parent
MPC_DIR = PKG_DIR.parent / "mpc"
for path in (str(PKG_DIR), str(MPC_DIR)):
    if path not in sys.path:
        sys.path.insert(0, path)

from config import bridge_topics, load_yaml  # noqa: E402
from pose_trail_viz import PoseTrailViz  # noqa: E402
from thrust_bridge import ThrustBridgeNode  # noqa: E402
from velocity_odom import VelocityOdomNode  # noqa: E402


def main() -> None:
    parser = argparse.ArgumentParser(description="Run h0_boat ROS stack in one process")
    parser.add_argument("--config", required=True, help="Merged experiment config YAML")
    args = parser.parse_args()

    cfg = load_yaml(args.config)
    bridge = bridge_topics(cfg)
    origin = cfg.get("origin", {})
    viz = cfg.get("viz", {})

    rclpy.init()
    executor = MultiThreadedExecutor(num_threads=6)
    nodes = []

    nodes.append(
        VelocityOdomNode(
            bridge.get("gps_topic", "/wamv/sensors/gps/gps/fix"),
            bridge.get("imu_topic", "/wamv/sensors/imu/imu/data"),
            bridge.get("estimated_odom_topic", "/molo_boat/estimated_odometry"),
            float(origin["lat"]),
            float(origin["lon"]),
            window_s=float(bridge.get("velocity_window_s", 0.4)),
        )
    )
    nodes.append(
        ThrustBridgeNode(
            bridge.get("thrust_input_left", "/molo_boat/thrust_left"),
            bridge.get("thrust_input_right", "/molo_boat/thrust_right"),
            bridge.get("thrust_mode", "pwm_topics"),
            bridge.get("pwm_left", "/pwm/left_thrust_cmd"),
            bridge.get("pwm_right", "/pwm/right_thrust_cmd"),
            bridge.get("pwm_daemon_host", "127.0.0.1"),
            int(bridge.get("pwm_daemon_port", 5600)),
        )
    )
    nodes.append(
        PoseTrailViz(
            viz.get("pose_topic", "/molo_mpc/vehicle_pose"),
            viz.get("recent_poses_topic", "/molo_h0/recent_poses"),
            viz.get("frame_id", "world"),
            max_poses=int(viz.get("max_recent_poses", 40)),
        )
    )

    from mpc import MpcFollowerNode  # noqa: E402

    nodes.append(MpcFollowerNode(cfg))

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
