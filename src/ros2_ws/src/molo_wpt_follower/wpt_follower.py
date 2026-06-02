#!/usr/bin/env python3
"""Standalone waypoint follower — ROS I/O only; algorithms live in algorithms.py."""

from __future__ import annotations

import argparse
import math
import os
import sys
from pathlib import Path
from typing import List, Optional, Tuple

import rclpy
import utm
import yaml
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import Imu, NavSatFix
from std_msgs.msg import Float64, Float32
from tf_transformations import euler_from_quaternion
from visualization_msgs.msg import MarkerArray

# Allow running as: python3 wpt_follower.py
sys.path.insert(0, str(Path(__file__).resolve().parent))

from algorithms import PathPoint, Pose2D, WaypointController, heading_from_points
from viz import WptVisualizer


def load_config(path: str) -> dict:
    with open(path, "r", encoding="utf-8") as f:
        return yaml.safe_load(f)


def gps_to_local(lat: float, lon: float, origin_utm: Tuple[float, float]) -> Tuple[float, float]:
    x, y, _, _ = utm.from_latlon(lat, lon)
    return x - origin_utm[0], y - origin_utm[1]


class WptFollowerNode(Node):
    def __init__(self, config: dict):
        super().__init__("molo_wpt_follower")

        self.config = config
        self.frame_id = config.get("frame_id", "world")
        self.sim_enable = bool(config.get("sim_enable", True))
        topics = config.get("topics", {})

        origin = config.get("origin", {})
        self.origin_utm = utm.from_latlon(
            float(origin.get("lat", 40.448417)),
            float(origin.get("lon", -86.867750)),
        )[:2]

        qos_sensor = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=1,
        )
        qos_cmd = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=5,
        )

        self.create_subscription(
            NavSatFix, topics.get("gps", "/wamv/sensors/gps/gps/fix"), self._gps_cb, qos_cmd
        )
        self.create_subscription(
            Imu, topics.get("imu", "/wamv/sensors/imu/imu/data"), self._imu_cb, qos_sensor
        )
        self.create_subscription(
            Odometry,
            topics.get("odometry", "/wamv/sensors/position/ground_truth_odometry"),
            self._odom_cb,
            qos_sensor,
        )
        self.create_subscription(
            PoseStamped,
            topics.get("goal_pose", "/goal_pose"),
            self._goal_cb,
            qos_sensor,
        )

        thrust_type = Float64 if self.sim_enable else Float32
        scale = 1000.0 if self.sim_enable else 1.0
        self._thrust_scale = scale
        self._left_pub = self.create_publisher(
            thrust_type, topics.get("left_thrust", "/wamv/thrusters/left/thrust"), qos_cmd
        )
        self._right_pub = self.create_publisher(
            thrust_type, topics.get("right_thrust", "/wamv/thrusters/right/thrust"), qos_cmd
        )

        self._marker_pub = self.create_publisher(MarkerArray, "/molo_wpt/waypoint_markers", 10)
        traj_cfg = config.get("trajectory", {})
        self.viz = WptVisualizer(
            self,
            topics,
            self.frame_id,
            min_trajectory_step_m=float(traj_cfg.get("min_distance_m", 0.5)),
        )

        self.controller = WaypointController(config)
        self.controller.configure_ilos(config.get("ilos", {}))

        self._pose: Optional[Pose2D] = None
        self._speed = 0.0
        self._mission_ready = False
        self._mission_complete = False
        self._loop_count = 0

        rate = float(config.get("control_rate_hz", 10.0))
        self.create_timer(1.0 / rate, self._control_loop)

        self._init_mission_from_config()
        self.get_logger().info(
            f"molo_wpt_follower ready (mode={config.get('control_mode', 'ilos')}, frame={self.frame_id})"
        )

    def _yaw_from_imu(self, msg: Imu) -> float:
        q = (msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w)
        _, _, yaw = euler_from_quaternion(q)
        return float(yaw)

    def _gps_cb(self, msg: NavSatFix) -> None:
        if msg.latitude == 0.0 and msg.longitude == 0.0:
            return
        x, y = gps_to_local(msg.latitude, msg.longitude, self.origin_utm)
        theta = self._pose.theta if self._pose else 0.0
        self._pose = Pose2D(x, y, theta)

    def _imu_cb(self, msg: Imu) -> None:
        yaw = self._yaw_from_imu(msg)
        if self._pose is None:
            self._pose = Pose2D(0.0, 0.0, yaw)
        else:
            self._pose = Pose2D(self._pose.x, self._pose.y, yaw)

    def _odom_cb(self, msg: Odometry) -> None:
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        self._speed = math.hypot(vx, vy)

    def _goal_cb(self, msg: PoseStamped) -> None:
        goal = Pose2D(
            msg.pose.position.x,
            msg.pose.position.y,
            self._yaw_from_imu_msg(msg.pose.orientation),
        )
        if self.config.get("control_mode") == "polar":
            self.controller.set_polar_goal(goal)
            self.get_logger().info(f"Polar goal set: ({goal.x:.1f}, {goal.y:.1f})")
        else:
            self.get_logger().info(
                "goal_pose received — switch control_mode to 'polar' for single goals"
            )

    @staticmethod
    def _yaw_from_imu_msg(orientation) -> float:
        q = (orientation.x, orientation.y, orientation.z, orientation.w)
        _, _, yaw = euler_from_quaternion(q)
        return float(yaw)

    def _init_mission_from_config(self) -> None:
        wp_cfg = self.config.get("waypoints", {})
        raw_points = wp_cfg.get("points", [])
        if not raw_points:
            return

        relative = bool(wp_cfg.get("relative_to_start", False))
        if relative:
            self._pending_relative = [
                (float(p["x"]), float(p["y"]), p.get("heading")) for p in raw_points
            ]
            self._mission_ready = False
            self.get_logger().info("Waiting for first pose to resolve relative waypoints")
            return

        waypoints = heading_from_points(
            [(float(p["x"]), float(p["y"]), p.get("heading")) for p in raw_points]
        )
        self._activate_mission(waypoints)

    def _activate_mission(self, waypoints: List[PathPoint]) -> None:
        self.controller.set_mission(waypoints)
        self._mission_ready = True
        self.viz.reset_traversed_path()
        self.viz.publish_mission(waypoints)
        self.viz.publish_path(self.controller.planned_path)
        markers = self.viz.publish_waypoint_markers(waypoints)
        self._marker_pub.publish(markers)
        self.get_logger().info(f"Mission loaded with {len(waypoints)} waypoints")

    def _resolve_relative_waypoints(self) -> None:
        if self._pose is None:
            return
        pending = getattr(self, "_pending_relative", None)
        if pending is None:
            return

        absolute = [
            (self._pose.x + dx, self._pose.y + dy, h) for dx, dy, h in pending
        ]
        waypoints = heading_from_points(absolute)
        del self._pending_relative
        self._activate_mission(waypoints)

    def _publish_thrust(self, linear: float, angular: float) -> None:
        left = max(-1.0, min(1.0, linear - angular))
        right = max(-1.0, min(1.0, linear + angular))
        thrust_type = Float64 if self.sim_enable else Float32
        self._left_pub.publish(thrust_type(data=float(left * self._thrust_scale)))
        self._right_pub.publish(thrust_type(data=float(right * self._thrust_scale)))

    def _control_loop(self) -> None:
        if hasattr(self, "_pending_relative"):
            self._resolve_relative_waypoints()
            if not self._mission_ready:
                return

        if self._pose is None or not self._mission_ready or self._mission_complete:
            return

        linear, angular, complete, debug = self.controller.step(self._pose, self._speed)
        self._mission_complete = complete
        self._loop_count += 1

        if self._loop_count % 20 == 0 and self.controller.planned_path:
            self.viz.publish_path(self.controller.planned_path)
            if self.controller.mission_waypoints:
                self.viz.publish_mission(self.controller.mission_waypoints)

        self.viz.publish_vehicle_pose(self._pose)
        self.viz.publish_traversed_path(self._pose)
        self.viz.publish_debug(debug)
        if "desired_heading" in debug:
            self.viz.publish_target_heading(self._pose, debug["desired_heading"])

        if complete:
            self._publish_thrust(0.0, 0.0)
            self.get_logger().info("Mission complete — stopping")
            return

        self._publish_thrust(linear, angular)


def main() -> None:
    parser = argparse.ArgumentParser(description="molo waypoint follower")
    default_cfg = Path(__file__).resolve().parent / "params.yaml"
    parser.add_argument(
        "--config",
        default=str(default_cfg),
        help="Path to params.yaml",
    )
    args = parser.parse_args()

    if not os.path.isfile(args.config):
        print(f"Config not found: {args.config}", file=sys.stderr)
        sys.exit(1)

    config = load_config(args.config)
    rclpy.init()
    node = WptFollowerNode(config)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
