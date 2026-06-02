"""RViz visualization publishers for molo_wpt_follower."""

from __future__ import annotations

import math
from typing import List, Optional, Sequence

import rclpy
from geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion
from nav_msgs.msg import Path
from std_msgs.msg import Float32, Header
from visualization_msgs.msg import Marker, MarkerArray

from algorithms import PathPoint, Pose2D


def _yaw_to_quaternion(yaw: float) -> Quaternion:
    q = Quaternion()
    q.z = math.sin(yaw * 0.5)
    q.w = math.cos(yaw * 0.5)
    return q


class WptVisualizer:
    def __init__(self, node, topics: dict, frame_id: str, min_trajectory_step_m: float = 0.5):
        self.node = node
        self.frame_id = frame_id
        self.min_trajectory_step_m = min_trajectory_step_m
        self._traversed = Path()
        self._traversed.header.frame_id = frame_id

        self.path_pub = node.create_publisher(Path, topics.get("path", "/molo_wpt/path"), 10)
        self.mission_pub = node.create_publisher(
            Path, topics.get("mission_path", "/molo_wpt/mission_path"), 10
        )
        self.traversed_pub = node.create_publisher(
            Path, topics.get("traversed_path", "/molo_wpt/traversed_path"), 10
        )
        self.pose_pub = node.create_publisher(
            PoseStamped, topics.get("vehicle_pose", "/molo_wpt/vehicle_pose"), 10
        )
        self.heading_pub = node.create_publisher(
            Marker, topics.get("target_heading", "/molo_wpt/target_heading"), 10
        )
        self.xte_pub = node.create_publisher(
            Float32, topics.get("cross_track_error", "/molo_wpt/cross_track_error"), 10
        )
        self.head_err_pub = node.create_publisher(
            Float32, topics.get("heading_error_deg", "/molo_wpt/heading_error_deg"), 10
        )

    def _header(self) -> Header:
        h = Header()
        h.stamp = self.node.get_clock().now().to_msg()
        h.frame_id = self.frame_id
        return h

    def publish_path(self, points: Sequence[PathPoint]) -> None:
        msg = Path()
        msg.header = self._header()
        for pt in points:
            ps = PoseStamped()
            ps.header = msg.header
            ps.pose.position.x = pt.x
            ps.pose.position.y = pt.y
            ps.pose.orientation = _yaw_to_quaternion(pt.heading)
            msg.poses.append(ps)
        self.path_pub.publish(msg)

    def publish_mission(self, points: Sequence[PathPoint]) -> None:
        msg = Path()
        msg.header = self._header()
        for pt in points:
            ps = PoseStamped()
            ps.header = msg.header
            ps.pose.position.x = pt.x
            ps.pose.position.y = pt.y
            ps.pose.orientation = _yaw_to_quaternion(pt.heading)
            msg.poses.append(ps)
        self.mission_pub.publish(msg)

    def publish_vehicle_pose(self, pose: Pose2D) -> None:
        msg = PoseStamped()
        msg.header = self._header()
        msg.pose.position.x = pose.x
        msg.pose.position.y = pose.y
        msg.pose.orientation = _yaw_to_quaternion(pose.theta)
        self.pose_pub.publish(msg)

    def reset_traversed_path(self) -> None:
        self._traversed.poses = []

    def publish_traversed_path(self, pose: Pose2D) -> None:
        """Append pose to the traversed trajectory and publish as nav_msgs/Path."""
        if self._traversed.poses:
            last = self._traversed.poses[-1].pose.position
            if math.hypot(pose.x - last.x, pose.y - last.y) < self.min_trajectory_step_m:
                return

        ps = PoseStamped()
        ps.header = self._header()
        ps.pose.position.x = pose.x
        ps.pose.position.y = pose.y
        ps.pose.orientation = _yaw_to_quaternion(pose.theta)
        self._traversed.poses.append(ps)

        out = Path()
        out.header = ps.header
        out.poses = self._traversed.poses
        self.traversed_pub.publish(out)

    def publish_target_heading(
        self, pose: Pose2D, desired_heading: float, length: float = 5.0
    ) -> None:
        marker = Marker()
        marker.header = self._header()
        marker.ns = "target_heading"
        marker.id = 0
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        marker.pose.position.x = pose.x
        marker.pose.position.y = pose.y
        marker.pose.orientation = _yaw_to_quaternion(desired_heading)
        marker.scale.x = length
        marker.scale.y = 0.3
        marker.scale.z = 0.3
        marker.color.r = 0.1
        marker.color.g = 0.9
        marker.color.b = 0.2
        marker.color.a = 1.0
        self.heading_pub.publish(marker)

    def publish_debug(self, debug: dict) -> None:
        if "cross_track_error" in debug:
            self.xte_pub.publish(Float32(data=float(debug["cross_track_error"])))
        if "heading_error_rad" in debug:
            self.head_err_pub.publish(
                Float32(data=float(math.degrees(debug["heading_error_rad"])))
            )

    def publish_waypoint_markers(self, points: Sequence[PathPoint]) -> MarkerArray:
        arr = MarkerArray()
        header = self._header()
        for i, pt in enumerate(points):
            m = Marker()
            m.header = header
            m.ns = "mission_waypoints"
            m.id = i
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position.x = pt.x
            m.pose.position.y = pt.y
            m.pose.position.z = 0.5
            m.scale.x = m.scale.y = m.scale.z = 1.0
            m.color.r = 1.0
            m.color.g = 0.4
            m.color.b = 0.0
            m.color.a = 1.0
            arr.markers.append(m)
        return arr
