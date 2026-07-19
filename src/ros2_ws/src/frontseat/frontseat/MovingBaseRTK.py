#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import NavSatFix, Imu
from std_msgs.msg import Float32
import tf_transformations as tf

from frontseat.qos_profiles import best_effort_volatile_qos, reliable_volatile_qos

import numpy as np

""" This node is used to estimate the heading of the vehicle using a dual GPS antenna with Moving Base RTK. 
 For the published heading to work correctly, the rover antenna should be at the front of the vehicle and 
 the base antenna should be at the rear of the vehicle. """
 
class MovingBaseRTK(Node):
    """ Heading estimation using a dual GPS antenna with Moving Base RTK"""
    def __init__(self, node_name='moving_base_rtk'):
        super().__init__(node_name)
        self.gps1_msg = None
        self.gps2_msg = None

        self.create_subscription(NavSatFix, '/fix/rover', self.gps_rover_cbk, reliable_volatile_qos)
        self.create_subscription(NavSatFix, '/fix/base', self.gps_base_cbk, reliable_volatile_qos)
        self.create_subscription(Imu, '/navheading', self.nav_heading_cbk, reliable_volatile_qos)

        self.heading_pub = self.create_publisher(Imu, '/baseline/heading', best_effort_volatile_qos)
        self.heading_deg_pub = self.create_publisher(Float32, '/heading/deg', reliable_volatile_qos)
        self.gps_center_pub = self.create_publisher(NavSatFix, '/fix/center/avg', best_effort_volatile_qos)

        self.averaged_gps_timer = self.create_timer(1.0 / 19.0, self.publish_averaged_gps)

    def gps_rover_cbk(self, msg):
        self.gps1_msg = msg

    def gps_base_cbk(self, msg):
        self.gps2_msg = msg

    def nav_heading_cbk(self, msg):
        self.heading_pub.publish(msg)

        qx = msg.orientation.x
        qy = msg.orientation.y
        qz = msg.orientation.z
        qw = msg.orientation.w

        _, _, yaw = tf.euler_from_quaternion([qx, qy, qz, qw])

        self.heading_deg_pub.publish(Float32(data=yaw*180/np.pi))

    def publish_averaged_gps(self):
        """ Take values from two different GPS sensors, average their values (including covariance) 
            and publish them. """
        if self.gps1_msg is None or self.gps2_msg is None:
            return  # wait until both are received

        # Average lat/lon/alt
        lat = (self.gps1_msg.latitude + self.gps2_msg.latitude) / 2.0
        lon = (self.gps1_msg.longitude + self.gps2_msg.longitude) / 2.0
        alt = (self.gps1_msg.altitude + self.gps2_msg.altitude) / 2.0

        # Average covariance matrices
        cov1 = np.array(self.gps1_msg.position_covariance).reshape(3, 3)
        cov2 = np.array(self.gps2_msg.position_covariance).reshape(3, 3)
        cov_avg = (cov1 + cov2) / 2.0

        # Build fused NavSatFix
        msg = NavSatFix()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "gps_center_link"

        msg.status.status = self.gps1_msg.status.status
        msg.status.service = self.gps1_msg.status.service

        msg.latitude = lat
        msg.longitude = lon
        msg.altitude = alt

        msg.position_covariance = cov_avg.flatten().tolist()
        msg.position_covariance_type = self.gps1_msg.position_covariance_type

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
