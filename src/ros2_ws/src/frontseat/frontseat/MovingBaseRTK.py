#!/usr/bin/env python3
import math

import rclpy
import tf_transformations as tf
import utm
from geometry_msgs.msg import Quaternion
from rclpy.node import Node

from sensor_msgs.msg import NavSatFix, Imu
from std_msgs.msg import Float32
from visualization_msgs.msg import Marker

from frontseat.qos_profiles import best_effort_volatile_qos, reliable_volatile_qos

import numpy as np

""" Heading from dual GPS baseline (base → rover). Rover = bow, base = stern. """


class MovingBaseRTK(Node):
    def __init__(self, node_name='moving_base_rtk'):
        super().__init__(node_name)

        self.rover_msg = None
        self.base_msg = None

        self.create_subscription(NavSatFix, '/fix/rover', self.gps_rover_cbk, reliable_volatile_qos)
        self.create_subscription(NavSatFix, '/fix/base', self.gps_base_cbk, reliable_volatile_qos)

        self.heading_pub = self.create_publisher(Imu, '/baseline/heading', best_effort_volatile_qos)
        self.heading_deg_pub = self.create_publisher(Float32, '/heading/deg', reliable_volatile_qos)
        self.heading_marker_pub = self.create_publisher(Marker, '/baseline/heading/marker', 10)
        self.gps_center_pub = self.create_publisher(NavSatFix, '/fix/center/avg', best_effort_volatile_qos)

        self.baseline_heading_timer = self.create_timer(1.0 / 19.0, self.publish_baseline_heading)
        self.averaged_gps_timer = self.create_timer(1.0 / 19.0, self.publish_averaged_gps)

    def gps_rover_cbk(self, msg):
        self.rover_msg = msg

    def gps_base_cbk(self, msg):
        self.base_msg = msg

    def publish_baseline_heading(self):
        if self.rover_msg is None or self.base_msg is None:
            return

        rover_x, rover_y, _, _ = utm.from_latlon(self.rover_msg.latitude, self.rover_msg.longitude)
        base_x, base_y, _, _ = utm.from_latlon(self.base_msg.latitude, self.base_msg.longitude)
        # Forward = base → rover (stern to bow).
        yaw = math.atan2(rover_y - base_y, rover_x - base_x)

        q = tf.quaternion_from_euler(0.0, 0.0, yaw)
        imu_msg = Imu()
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = 'gps_baseline'
        imu_msg.orientation.x = q[0]
        imu_msg.orientation.y = q[1]
        imu_msg.orientation.z = q[2]
        imu_msg.orientation.w = q[3]

        self.heading_pub.publish(imu_msg)
        self.heading_deg_pub.publish(Float32(data=yaw * 180.0 / np.pi))
        self._publish_heading_marker()

    def _publish_heading_marker(self) -> None:
        # Pose is in gps_center_link; RViz transforms into its Fixed Frame via TF.
        marker = Marker()
        marker.header.frame_id = 'gps_center_link'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'moving_base_rtk'
        marker.id = 0
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        marker.pose.position.x = 0.0
        marker.pose.position.y = 0.0
        marker.pose.position.z = 0.2
        # Rover is at +X, base at -X: baseline heading is along +X.
        marker.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        marker.scale.x = 2.5
        marker.scale.y = 0.15
        marker.scale.z = 0.15
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        self.heading_marker_pub.publish(marker)

    def publish_averaged_gps(self):
        if self.rover_msg is None or self.base_msg is None:
            return

        lat = (self.rover_msg.latitude + self.base_msg.latitude) / 2.0
        lon = (self.rover_msg.longitude + self.base_msg.longitude) / 2.0
        alt = (self.rover_msg.altitude + self.base_msg.altitude) / 2.0

        cov1 = np.array(self.rover_msg.position_covariance).reshape(3, 3)
        cov2 = np.array(self.base_msg.position_covariance).reshape(3, 3)
        cov_avg = (cov1 + cov2) / 2.0

        # Build fused NavSatFix
        msg = NavSatFix()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'gps_center_link'
        msg.status.status = self.rover_msg.status.status
        msg.status.service = self.rover_msg.status.service
        msg.latitude = lat
        msg.longitude = lon
        msg.altitude = alt
        msg.position_covariance = cov_avg.flatten().tolist()
        msg.position_covariance_type = self.rover_msg.position_covariance_type

        self.gps_center_pub.publish(msg)


def main(args=None):
    rclpy.init()
    node = MovingBaseRTK()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
