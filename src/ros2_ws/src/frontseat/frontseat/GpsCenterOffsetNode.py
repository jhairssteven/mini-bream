#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import NavSatFix, Imu
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy

import utm
import math


class GpsCenterOffsetNode(Node):
    """
    Node that corrects GPS readings by accounting for the sensor's physical offset from the vehicle's center.
    It uses IMU yaw data to rotate the offset vector and adjusts the GPS coordinates (via UTM conversion)
    to represent the vehicle's actual center position.
    """
    def __init__(self):
        super().__init__('gps_center_offset_node')

        # ---------------------------------------------------------
        # Parameters: GPS offset relative to VEHICLE BASE LINK
        # ---------------------------------------------------------
        self.declare_parameter("gps_offset_x", -0.85)  # forward (+)
        self.declare_parameter("gps_offset_y", 0.0)    # left (+)

        self.gps_offset_x = self.get_parameter("gps_offset_x").value
        self.gps_offset_y = self.get_parameter("gps_offset_y").value

        # Current yaw angle (from IMU)
        self.yaw = 0.0

        qos_best_effort_volatile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=1
        )

        # ---------------------------------------------------------
        # Subscribers
        # ---------------------------------------------------------
        self.sub_gps = self.create_subscription(
            NavSatFix,
            '/wamv/sensors/gps/gps/fix',
            self.gps_callback,
            qos_best_effort_volatile
        )

        self.sub_imu = self.create_subscription(
            Imu,
            '/wamv/sensors/imu/imu/data',     # change to correct topic
            self.imu_callback,
            qos_best_effort_volatile
        )

        # ---------------------------------------------------------
        # Publisher
        # ---------------------------------------------------------
        self.pub = self.create_publisher(
            NavSatFix,
            '/wamv/sensors/gps/centered_gps/fix',
            qos_best_effort_volatile
        )

        self.get_logger().info("GPS Center Offset Node started (UTM + yaw transform).")

    # ============================================================
    # IMU callback → store yaw angle
    # ============================================================
    def imu_callback(self, msg: Imu):
        # quaternion → yaw
        q = msg.orientation
        siny_cosp = 2 * (q.w*q.z + q.x*q.y)
        cosy_cosp = 1 - 2 * (q.y*q.y + q.z*q.z)
        self.yaw = math.atan2(siny_cosp, cosy_cosp)

    # ============================================================
    # GPS callback
    # ============================================================
    def gps_callback(self, msg: NavSatFix):

        lat = msg.latitude
        lon = msg.longitude

        # 1. Convert GPS → UTM
        easting, northing, zone_number, zone_letter = utm.from_latlon(lat, lon)

        # 2. Rotate offset by vehicle yaw
        dx = self.gps_offset_x
        dy = self.gps_offset_y
        psi = self.yaw

        dx_earth = dx * math.cos(psi) - dy * math.sin(psi)
        dy_earth = dx * math.sin(psi) + dy * math.cos(psi)

        # 3. Compute center position (subtract GPS offset)
        easting_center = easting - dx_earth
        northing_center = northing - dy_earth

        # 4. Convert back to lat/lon
        lat_new, lon_new = utm.to_latlon(
            easting_center,
            northing_center,
            zone_number,
            zone_letter
        )

        # 5. Output NavSatFix
        new_msg = NavSatFix()
        new_msg.header = msg.header
        new_msg.latitude = lat_new
        new_msg.longitude = lon_new
        new_msg.altitude = msg.altitude

        self.pub.publish(new_msg)


def main(args=None):
    rclpy.init(args=args)
    node = GpsCenterOffsetNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
