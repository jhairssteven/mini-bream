#!/usr/bin/env python3
"""
Mock frontseat sensor publishers for integration testing.

Publishes GPS + IMU on the same topics as moving_base_rtk so the Jetson-side
h0_boat stack can run without real RTK hardware on the Pi.
"""

from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path

import rclpy
import utm
import yaml
from geometry_msgs.msg import Quaternion, Vector3
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import Imu, NavSatFix
from std_msgs.msg import Float32, Float64
from tf_transformations import quaternion_from_euler

PKG_DIR = Path(__file__).resolve().parent
MOLO_DIR = PKG_DIR.parent
MPC_DIR = MOLO_DIR / "mpc"
for path in (str(PKG_DIR), str(MPC_DIR)):
    if path not in sys.path:
        sys.path.insert(0, path)

from config import build_h0_boat_config  # noqa: E402


def lemniscate_xy(t: float, scale_m: float) -> tuple[float, float, float]:
    """Gerono lemniscate; returns x, y, heading."""
    a = scale_m
    denom = 1.0 + math.sin(t) ** 2
    x = a * math.cos(t) / denom
    y = a * math.sin(t) * math.cos(t) / denom
    dt = 1e-4
    denom2 = 1.0 + math.sin(t + dt) ** 2
    x2 = a * math.cos(t + dt) / denom2
    y2 = a * math.sin(t + dt) * math.cos(t + dt) / denom2
    psi = math.atan2(y2 - y, x2 - x)
    return x, y, psi


class MockFrontseatNode(Node):
    def __init__(
        self,
        origin_lat: float,
        origin_lon: float,
        scale_m: float,
        speed_mps: float,
        gps_topic: str,
        imu_topic: str,
        odom_topic: str,
        thrust_topics: tuple[str, str],
        log_thrust: bool,
        frame_id: str = "map",
    ):
        super().__init__("mock_frontseat")
        self._origin_utm = utm.from_latlon(origin_lat, origin_lon)
        self._scale_m = scale_m
        self._speed = max(speed_mps, 0.05)
        self._t_path = 0.0
        self._x = self._y = self._psi = 0.0
        self._log_thrust = log_thrust
        self._frame_id = frame_id

        qos_sensor = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=1,
        )
        qos_gps = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=5,
        )
        self._gps_pub = self.create_publisher(NavSatFix, gps_topic, qos_gps)
        self._imu_pub = self.create_publisher(Imu, imu_topic, qos_sensor)
        self._odom_pub = self.create_publisher(Odometry, odom_topic, qos_sensor)

        self.create_subscription(Float32, thrust_topics[0], self._left_cb, 10)
        self.create_subscription(Float32, thrust_topics[1], self._right_cb, 10)
        self.create_subscription(Float64, thrust_topics[0], self._left_cb64, 10)
        self.create_subscription(Float64, thrust_topics[1], self._right_cb64, 10)
        self._last_thrust = (0.0, 0.0)

        self.create_timer(0.1, self._tick)
        self.get_logger().info(
            f"mock_frontseat origin=({origin_lat:.6f}, {origin_lon:.6f}) "
            f"scale={scale_m}m odom={odom_topic}"
        )

    def _left_cb(self, msg: Float32) -> None:
        self._last_thrust = (float(msg.data), self._last_thrust[1])

    def _right_cb(self, msg: Float32) -> None:
        self._last_thrust = (self._last_thrust[0], float(msg.data))

    def _left_cb64(self, msg: Float64) -> None:
        self._last_thrust = (float(msg.data), self._last_thrust[1])

    def _right_cb64(self, msg: Float64) -> None:
        self._last_thrust = (self._last_thrust[0], float(msg.data))

    def _local_to_gps(self, x: float, y: float) -> tuple[float, float]:
        east = self._origin_utm[0] + x
        north = self._origin_utm[1] + y
        lat, lon = utm.to_latlon(east, north, self._origin_utm[2], self._origin_utm[3])
        return lat, lon

    def _tick(self) -> None:
        self._t_path += 0.1 * self._speed / max(self._scale_m, 1.0)
        self._x, self._y, self._psi = lemniscate_xy(self._t_path, self._scale_m)

        lat, lon = self._local_to_gps(self._x, self._y)
        gps = NavSatFix()
        gps.header.stamp = self.get_clock().now().to_msg()
        gps.header.frame_id = "gps"
        gps.latitude = lat
        gps.longitude = lon
        gps.altitude = 0.0
        gps.status.status = 0
        gps.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN
        self._gps_pub.publish(gps)

        imu = Imu()
        imu.header.stamp = gps.header.stamp
        imu.header.frame_id = "imu_link"
        q = Quaternion()
        q.z = math.sin(self._psi * 0.5)
        q.w = math.cos(self._psi * 0.5)
        imu.orientation = q
        imu.angular_velocity = Vector3(z=self._speed / max(self._scale_m, 1.0) * 0.15)
        self._imu_pub.publish(imu)

        odom = Odometry()
        odom.header.stamp = gps.header.stamp
        odom.header.frame_id = self._frame_id
        odom.child_frame_id = "base_link"
        odom.pose.pose.position.x = self._x
        odom.pose.pose.position.y = self._y
        qx, qy, qz, qw = quaternion_from_euler(0.0, 0.0, self._psi)
        odom.pose.pose.orientation = Quaternion(x=qx, y=qy, z=qz, w=qw)
        odom.twist.twist.linear.x = self._speed
        odom.twist.twist.angular.z = float(imu.angular_velocity.z)
        self._odom_pub.publish(odom)

        if self._log_thrust and (abs(self._last_thrust[0]) > 1e-4 or abs(self._last_thrust[1]) > 1e-4):
            self.get_logger().info(
                f"thrust L={self._last_thrust[0]:+.3f} R={self._last_thrust[1]:+.3f} "
                f"pose=({self._x:.2f},{self._y:.2f})",
                throttle_duration_sec=2.0,
            )


def main() -> None:
    parser = argparse.ArgumentParser(description="Mock frontseat GPS/IMU for h0_boat testing")
    parser.add_argument("--config", default=None, help="Optional h0_boat merged config YAML")
    parser.add_argument("--origin-lat", type=float, default=None)
    parser.add_argument("--origin-lon", type=float, default=None)
    parser.add_argument("--scale-m", type=float, default=7.0)
    parser.add_argument("--speed-mps", type=float, default=0.14)
    parser.add_argument("--gps-topic", default="/wamv/sensors/gps/gps/fix")
    parser.add_argument("--imu-topic", default="/wamv/sensors/imu/imu/data")
    parser.add_argument("--odom-topic", default="/mock/odom")
    parser.add_argument("--thrust-left", default="/molo_boat/thrust_left")
    parser.add_argument("--thrust-right", default="/molo_boat/thrust_right")
    parser.add_argument("--no-thrust-log", action="store_true")
    args = parser.parse_args()

    origin_lat = args.origin_lat
    origin_lon = args.origin_lon
    scale_m = args.scale_m
    speed_mps = args.speed_mps
    gps_topic = args.gps_topic
    imu_topic = args.imu_topic
    odom_topic = args.odom_topic
    thrust_left = args.thrust_left
    thrust_right = args.thrust_right
    frame_id = "map"

    if args.config:
        with open(args.config, encoding="utf-8") as f:
            cfg = yaml.safe_load(f)
        origin = cfg.get("origin", {})
        origin_lat = float(origin.get("lat", origin_lat or 40.448417))
        origin_lon = float(origin.get("lon", origin_lon or -86.867750))
        traj = cfg.get("waypoints", {}).get("trajectory", {})
        scale_m = float(traj.get("scale_m", scale_m))
        speed_mps = float(cfg.get("path", {}).get("cruise_speed_mps", speed_mps))
        topics = cfg.get("topics", {})
        gps_topic = topics.get("gps", gps_topic)
        imu_topic = topics.get("imu", imu_topic)
        odom_topic = topics.get("odometry", odom_topic)
        thrust_left = topics.get("left_thrust", thrust_left)
        thrust_right = topics.get("right_thrust", thrust_right)
        frame_id = str(cfg.get("frame_id", frame_id))
    else:
        if origin_lat is None or origin_lon is None:
            cfg = build_h0_boat_config()
            origin_lat = float(cfg["origin"].get("lat") or 40.448417)
            origin_lon = float(cfg["origin"].get("lon") or -86.867750)

    rclpy.init()
    node = MockFrontseatNode(
        origin_lat,
        origin_lon,
        scale_m,
        speed_mps,
        gps_topic,
        imu_topic,
        odom_topic,
        (thrust_left, thrust_right),
        log_thrust=not args.no_thrust_log,
        frame_id=frame_id,
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
