#!/usr/bin/env python3
"""Publish a sample nav_msgs/Path for MPC topic-based reference testing."""

from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path as NavPath
from rclpy.node import Node

MPC_DIR = Path(__file__).resolve().parent
if str(MPC_DIR) not in sys.path:
    sys.path.insert(0, str(MPC_DIR))

from path_reference import sine_path_points


def s_curve_points(
    length_m: float = 40.0,
    amplitude_m: float = 6.0,
    waves: float = 2.0,
    n: int = 48,
) -> list[tuple[float, float]]:
    return sine_path_points(length_m, amplitude_m, waves, n)


def build_path_msg(frame_id: str, points: list[tuple[float, float]]) -> NavPath:
    msg = NavPath()
    msg.header.frame_id = frame_id
    for x, y in points:
        ps = PoseStamped()
        ps.header.frame_id = frame_id
        ps.pose.position.x = float(x)
        ps.pose.position.y = float(y)
        ps.pose.orientation.w = 1.0
        msg.poses.append(ps)
    return msg


class PathPublisher(Node):
    def __init__(self, topic: str, frame_id: str, points: list[tuple[float, float]], rate_hz: float):
        super().__init__("publish_reference_path")
        self._pub = self.create_publisher(NavPath, topic, 10)
        self._msg = build_path_msg(frame_id, points)
        period = 1.0 / max(rate_hz, 0.1)
        self._timer = self.create_timer(period, self._publish)
        self.get_logger().info(
            f"Publishing {len(points)}-pose S-curve on {topic} ({frame_id}) at {rate_hz:.1f} Hz"
        )

    def _publish(self) -> None:
        self._msg.header.stamp = self.get_clock().now().to_msg()
        for ps in self._msg.poses:
            ps.header.stamp = self._msg.header.stamp
        self._pub.publish(self._msg)


def main() -> None:
    p = argparse.ArgumentParser(description="Publish sample reference path for MPC follower")
    p.add_argument("--topic", default="/molo_mpc/reference_path")
    p.add_argument("--frame-id", default="world")
    p.add_argument("--length-m", type=float, default=40.0)
    p.add_argument("--amplitude-m", type=float, default=6.0)
    p.add_argument("--waves", type=float, default=2.0)
    p.add_argument("--num-points", type=int, default=48)
    p.add_argument("--rate-hz", type=float, default=1.0, help="Republish rate (latched testing)")
    p.add_argument("--once", action="store_true", help="Publish one message and exit")
    args = p.parse_args()

    points = s_curve_points(args.length_m, args.amplitude_m, args.waves, args.num_points)
    rclpy.init()
    node = PathPublisher(args.topic, args.frame_id, points, args.rate_hz)
    if args.once:
        node._publish()
        node.get_logger().info("Published once.")
        node.destroy_node()
        rclpy.shutdown()
        return
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
