#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import NavSatFix, Imu
from std_msgs.msg import Float32
import tf_transformations as tf

from frontseat.qos_profiles import reliable_transient_local_qos, best_effort_volatile_qos, reliable_volatile_qos

from dataclasses import dataclass
import numpy as np
import utm

@dataclass
class GPS:
    lat: float
    lon: float
    utm_zone: str = ''

    def to_gps(self):
        self.gps_lat, self.gps_lon = utm.to_latlon(self.utm_x, self.utm_y, int(self.utm_zone[:-1]), self.utm_zone[-1])
    def to_utm(self):
        utm_x, utm_y, utm_zone_num, utm_zone_ltr = utm.from_latlon(self.lat, self.lon)
        self.utm_zone = '%d%s'%(utm_zone_num, utm_zone_ltr)
        return utm_x, utm_y
    
class MovingBaseRTK(Node):
    """ Heading estimation using a dual GPS antenna with Moving Base RTK"""
    def __init__(self, node_name='moving_base_rtk'):
        super().__init__(node_name)
        #self.origin_gps_lat, self.origin_gps_lon = 40.448417, -86.867750 #harner
        #self.origin_gps_lat, self.origin_gps_lon = 40.4476285, -86.86825809999999 #harner closer to the shore
        self.origin_gps_lat, self.origin_gps_lon = 40.40229852, -86.84558228 #kepner
        self.origin_gps = GPS(self.origin_gps_lat, self.origin_gps_lon)
        self.ox, self.oy = self.origin_gps.to_utm()
        self.gps1 = GPS(self.origin_gps_lat, self.origin_gps_lon)
        self.gps2 = GPS(self.origin_gps_lat, self.origin_gps_lon)
        self.gps1_msg = None # To save the GPS ros msg
        self.gps2_msg = None # To save the GPS ros msg
        
        self.create_subscription(NavSatFix, '/fix/rover', self.gps_rover_cbk, reliable_volatile_qos)
        self.create_subscription(NavSatFix, '/fix/base', self.gps_base_cbk, reliable_volatile_qos)
        self.create_subscription(Imu, '/navheading', self.nav_heading_cbk, reliable_volatile_qos)

        self.heading_pub = self.create_publisher(Imu, '/baseline/heading', best_effort_volatile_qos)
        self.heading_deg_pub = self.create_publisher(Float32, '/heading/deg', reliable_volatile_qos)
        self.gps_center_pub = self.create_publisher(NavSatFix, '/fix/center/avg', best_effort_volatile_qos)
        
        self.averaged_gps_timer = self.create_timer(1.0 / 19.0, self.publish_averaged_gps)


    def gps_rover_cbk(self, msg):
        self.gps1.lat, self.gps1.lon = msg.latitude, msg.longitude
        self.gps1_msg = msg

    def gps_base_cbk(self, msg):
        self.gps2.lat, self.gps2.lon = msg.latitude, msg.longitude
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
