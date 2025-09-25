#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import NavSatFix, Imu
from std_msgs.msg import String, Float32
from geometry_msgs.msg import Quaternion
import tf_transformations as tf

from frontseat.qos_profiles import reliable_transient_local_qos, best_effort_volatile_qos, reliable_volatile_qos

from dataclasses import dataclass
import numpy as np
import utm
import math

from visualization_msgs.msg import Marker
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path

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
    
class DualAntenna(Node):
    """ Heading estimation using a dual GPS antenna """
    def __init__(self, node_name='dual_antenna'):
        super().__init__(node_name)
        #self.origin_gps = GPS(40.448417, -86.867750) # Harner
        self.origin_gps = GPS(40.40229852, -86.84558228) # Kepner
        self.ox, self.oy = self.origin_gps.to_utm()
        self.gps1 = GPS(40.40229852, -86.84558228)
        self.gps2 = GPS(40.40229852, -86.84558228)
        self.gps1_msg = None # To save the GPS ros msg
        self.gps2_msg = None # To save the GPS ros msg
        self.topic_gps1 = '/fix1'
        self.topic_gps2 = '/fix2'
        self.baseline_heading_topic = '/baseline/heading' #'/wamv/sensors/imu/imu/data'

        self.create_subscription(NavSatFix, self.topic_gps1, self.gps1_cbk, reliable_volatile_qos)
        self.create_subscription(NavSatFix, self.topic_gps2, self.gps2_cbk, reliable_volatile_qos)
        
        self.heading_pub = self.create_publisher(Imu, self.baseline_heading_topic, best_effort_volatile_qos)
        self.gps_center_pub = self.create_publisher(NavSatFix, '/dA/gps/center/fix', best_effort_volatile_qos)
        self.stats_pub = self.create_publisher(String, '/dA/stats2', reliable_volatile_qos)
        
        self.timer = self.create_timer(1.0 / 3.0, self.compute_baseline_heading)
        self.timer = self.create_timer(1.0 / 19.0, self.publish_averaged_gps)

        # Debug
        self.heading_value_pub_deg = self.create_publisher(Float32, '/dA/heading/estimated/degrees/value', reliable_volatile_qos)
        self.gt_vehicle_heading_pub_deg = self.create_publisher(Float32, '/dA/heading/ground_truth/degrees/value', reliable_volatile_qos)
        self.diff_heading_value_pub_deg = self.create_publisher(Float32, '/dA/heading/abs_diff/degrees/value', reliable_volatile_qos)
        self.gpss_estimated_separation = self.create_publisher(Float32, '/dA/separation/estimated/value', reliable_volatile_qos)
        self.gt_gps_separation = self.create_publisher(Float32, '/dA/separation/ground_truth/value', reliable_volatile_qos)
        self.gps_gt_separation=0.96 # m
        self.prev_x, self.prev_y = None, None


        # Visuals
        self.marker_pub = self.create_publisher(Marker, '/dA/heading/marker', 10)
        self.gps1_path_pub = self.create_publisher(Path, '/gps1/path', 10)
        self.gps1_path = Path()
        self.gps1_path.header.frame_id = "world"
        
        self.gps2_path_pub = self.create_publisher(Path, '/gps2/path', 10)
        self.gps2_path = Path()
        self.gps2_path.header.frame_id = "world"

        # increment for next marker
        self.marker_id_counter = 0
        
    def gps1_cbk(self, msg):
        self.gps1.lat, self.gps1.lon = msg.latitude, msg.longitude
        self.gps1_msg = msg

    def gps2_cbk(self, msg):
        self.gps2.lat, self.gps2.lon = msg.latitude, msg.longitude
        self.gps2_msg = msg

    def get_ground_truth_heading(self, x, y, default_if_none):
        """ Estimate the ground truth heading by taking the 
            discrete derivative of the trajectory at current 
            position, with previous position. """
        if self.prev_x is None or self.prev_y is None:
            self.prev_x, self.prev_y = x, y
            return default_if_none
        dx = x - self.prev_x
        dy = y - self.prev_y
        gt_heading = np.arctan2(dy, dx)
        return gt_heading
    
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

        # Publish
        self.gps_center_pub.publish(msg)
        self.get_logger().info(f"Published center GPS at lat={lat}, lon={lon}")

    def compute_baseline_heading(self):
        gps1_x, gps1_y = self.gps1.to_utm()
        gps2_x, gps2_y = self.gps2.to_utm()
        # vector that goes from gps2 -> gps1
        Px = gps2_x-gps1_x
        Py = gps2_y-gps1_y
        baseline_heading = np.arctan2(Py, Px)# + (180+90+5)*np.pi/180 # 90 deg offset is necessary since the GPSs were mounted across the vehicle
        imu_msg = self.publish_heading_as_imu_msg(baseline_heading)
        gpss_estimated_separation = np.hypot(Px, Py)
        
        # publish stats summary for easy debugging
        stats = f'{gps1_x-self.ox}, {gps1_y-self.oy}, {gps2_x-self.ox}, {gps2_y-self.oy}, {Px}, {Py}, {baseline_heading}, {gpss_estimated_separation}'
        self.stats_pub.publish(String(data=stats))
        self.heading_value_pub_deg.publish(Float32(data=baseline_heading*180/np.pi))
        self.gpss_estimated_separation.publish(Float32(data=gpss_estimated_separation))
        
        gt_heading = self.get_ground_truth_heading(x=gps1_x - self.ox, y=gps1_y - self.oy, default_if_none=baseline_heading)

        self.prev_x, self.prev_y = gps1_x-self.ox, gps1_y-self.oy
        self.gt_vehicle_heading_pub_deg.publish(Float32(data=gt_heading*180/np.pi))
        self.diff_heading_value_pub_deg.publish(Float32(data=abs(abs(baseline_heading)-(abs(gt_heading)))*180/np.pi))
        self.gt_gps_separation.publish(Float32(data=self.gps_gt_separation)) # Plot a fixed value for easy comparison


        # Visuals
        # Visualize the estimated heading as an arrow marker, at the position of GPS1
        if self.marker_id_counter % 2 == 0:
            self.publish_arrow_marker_at(x=gps1_x - self.ox, y=gps1_y - self.oy, quaternion=imu_msg.orientation, color=(0.0, 0.0, 1.0, 1.0))
            self.publish_arrow_marker_at(x=gps1_x - self.ox, y=gps1_y - self.oy, quaternion=self.get_yaw_as_quaternion(gt_heading), color=(0.0,0.0,0.0,1.0))

        # Visualize GPS1 and GPS2 trajectories
        self.add_point_to_path_and_publish(self.gps1_path_pub, self.gps1_path, gps1_x-self.ox, gps1_y-self.oy)
        self.add_point_to_path_and_publish(self.gps2_path_pub, self.gps2_path, gps2_x-self.ox, gps2_y-self.oy)
    
    def get_yaw_as_quaternion(self, yaw):
        q = tf.quaternion_from_euler(0.0, 0.0, yaw)
        return Quaternion(x = q[0], y = q[1], z = q[2], w = q[3])

    def publish_heading_as_imu_msg(self, yaw_rad):
        
        imu_msg = Imu()
        imu_msg.orientation = self.get_yaw_as_quaternion(yaw_rad)
        
        # Set angular_velocity and linear_acceleration to 0
        imu_msg.angular_velocity.x = 0.0
        imu_msg.angular_velocity.y = 0.0
        imu_msg.angular_velocity.z = 0.0
        imu_msg.linear_acceleration.x = 0.0
        imu_msg.linear_acceleration.y = 0.0
        imu_msg.linear_acceleration.z = 0.0

        self.heading_pub.publish(imu_msg)
        #self.get_logger().info(f'Published IMU with yaw: {yaw_rad*180/np.pi} deg')
        return imu_msg

    def publish_arrow_marker_at(self, x, y, quaternion, color=(0.0, 1.0, 0.0, 1.0)):
        marker = Marker()
        marker.header.frame_id = "world"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "pose_marker"
        marker.id = self.marker_id_counter
        marker.type = Marker.ARROW
        marker.action = Marker.ADD

        # Set position
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = 0.0

        marker.pose.orientation = quaternion

        # Scale of the arrow (shaft length, shaft diameter, head diameter)
        marker.scale.x = 1.0
        marker.scale.y = 0.1
        marker.scale.z = 0.1

        # Color RGBA
        marker.color.r = color[0]
        marker.color.g = color[1]
        marker.color.b = color[2]
        marker.color.a = color[3]

        self.marker_pub.publish(marker)
        
        # increment for next marker
        self.marker_id_counter += 1

    def add_point_to_path_and_publish(self, path_publisher, path, x: float, y: float):
        """Append a new (x, y, yaw) pose to the path and publish it."""
        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = "world"
        pose_stamped.header.stamp = self.get_clock().now().to_msg()

        # Pose
        pose_stamped.pose.position.x = x
        pose_stamped.pose.position.y = y
        pose_stamped.pose.position.z = 0.0
        # Dummy yaw of 90deg
        q = tf.quaternion_from_euler(0.0, 0.0, np.pi/2)
        
        pose_stamped.pose.orientation.x = q[0]
        pose_stamped.pose.orientation.y = q[1]
        pose_stamped.pose.orientation.z = q[2]
        pose_stamped.pose.orientation.w = q[3]

        # Append new pose to path and publish updated path
        path.header.stamp = pose_stamped.header.stamp
        path.poses.append(pose_stamped)
        path_publisher.publish(path)

def main(args=None):
    rclpy.init()

    dualAntenna = DualAntenna()
    try:
        rclpy.spin(dualAntenna)
    except KeyboardInterrupt:
        pass
    finally:
        dualAntenna.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
