#!/usr/bin/env python3
"""Publish recent boat poses as RViz markers for live experiment monitoring."""

from __future__ import annotations

import argparse
import math
from collections import deque
from typing import Deque, Tuple

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from std_msgs.msg import ColorRGBA, Header
from visualization_msgs.msg import Marker, MarkerArray


class PoseTrailViz(Node):
    def __init__(
        self,
        pose_topic: str,
        marker_topic: str,
        frame_id: str,
        max_poses: int = 40,
        marker_scale: float = 0.35,
    ):
        super().__init__("molo_h0_pose_trail_viz")
        self._frame_id = frame_id
        self._max_poses = max(2, max_poses)
        self._marker_scale = marker_scale
        self._poses: Deque[Tuple[float, float, float]] = deque(maxlen=self._max_poses)

        self.create_subscription(PoseStamped, pose_topic, self._pose_cb, 10)
        self._pub = self.create_publisher(MarkerArray, marker_topic, 10)
        self.create_timer(0.2, self._publish)

    def _pose_cb(self, msg: PoseStamped) -> None:
        q = msg.pose.orientation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )
        self._poses.append(
            (float(msg.pose.position.x), float(msg.pose.position.y), float(yaw))
        )

    def _hdr(self) -> Header:
        h = Header()
        h.stamp = self.get_clock().now().to_msg()
        h.frame_id = self._frame_id
        return h

    def _publish(self) -> None:
        if not self._poses:
            return

        arr = MarkerArray()
        n = len(self._poses)
        for i, (x, y, yaw) in enumerate(self._poses):
            m = Marker()
            m.header = self._hdr()
            m.ns = "recent_poses"
            m.id = i
            m.type = Marker.ARROW
            m.action = Marker.ADD
            m.pose.position.x = x
            m.pose.position.y = y
            m.pose.position.z = 0.05
            m.pose.orientation.z = math.sin(yaw * 0.5)
            m.pose.orientation.w = math.cos(yaw * 0.5)
            m.scale.x = self._marker_scale
            m.scale.y = self._marker_scale * 0.35
            m.scale.z = 0.08
            alpha = 0.25 + 0.75 * (i + 1) / n
            m.color = ColorRGBA(r=1.0, g=0.45, b=0.05, a=float(alpha))
            arr.markers.append(m)

        self._pub.publish(arr)


def main() -> None:
    parser = argparse.ArgumentParser(description="Recent pose trail markers for H0 boat experiment")
    parser.add_argument("--pose-topic", default="/molo_mpc/vehicle_pose")
    parser.add_argument("--marker-topic", default="/molo_h0/recent_poses")
    parser.add_argument("--frame-id", default="world")
    parser.add_argument("--max-poses", type=int, default=40)
    args = parser.parse_args()

    rclpy.init()
    node = PoseTrailViz(
        args.pose_topic,
        args.marker_topic,
        args.frame_id,
        max_poses=args.max_poses,
    )
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
