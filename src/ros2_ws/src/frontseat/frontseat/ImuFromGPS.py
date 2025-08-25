#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
from tf_transformations import quaternion_from_euler
from sensor_msgs.msg import NavSatFix, Imu
from geometry_msgs.msg import Quaternion
from nav_msgs.msg import Odometry
import utm
import math as m

class ImuFromGPSNode(Node):
    def __init__(self, node_name='imu_from_gps'):
        super().__init__(node_name)
        
        # For high-rate sensor data
        qos_best_effort_volatile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,   # Send ony once without retry
            durability=QoSDurabilityPolicy.VOLATILE,        # Do not store old messages
            depth=1
        )

        self.create_subscription(NavSatFix, '/wamv/sensors/gps/gps/fix', self.gps_cbk, qos_best_effort_volatile)
        self.fix_vel_sub = self.create_subscription(Odometry, '/wamv/sensors/position/ground_truth_odometry',
                                                        self.position_ground_truth, qos_best_effort_volatile)

        self.imu_pub = self.create_publisher(Imu, '/wamv/sensors/imu/imu/data/estimated', qos_best_effort_volatile)
        
        self.timer = self.create_timer(1.0 / 19.0, self.publish_heading)  # 3 Hz

        # variables
        self.cur_lat = None
        self.cur_lon = None
        self.prev_lat = None
        self.prev_lon = None
        self.curr_orientation = None
        self.gt_x = None
        self.gt_y = None
        self.prev_gt_x = None
        self.prev_gt_y = None
        self.d_gt_x = None
        self.d_gt_y = None
        self.use_ground_truth = False
        
        
        self.get_logger().info(f"Node '{node_name}' is ready")

    def position_ground_truth(self, msg):
        self.gt_x = msg.pose.pose.position.x
        self.gt_y = msg.pose.pose.position.y
        
        if self.prev_gt_y is None and self.prev_gt_x is None:    
            self.prev_gt_x = self.gt_x
            self.prev_gt_y = self.gt_y
        
        self.d_gt_x = self.gt_x - self.prev_gt_x
        self.d_gt_y = self.gt_y - self.prev_gt_y
        # update previous value
        self.prev_gt_x = self.gt_x
        self.prev_gt_y = self.gt_y
        
    def publish_heading(self):
        if self.curr_orientation is None:
            return
    
        imu_msg = Imu()
        imu_msg.orientation = self.curr_orientation
        
        # Following ROS convention set first element to -1 for other systems to ignore covariance values.
        imu_msg.orientation_covariance = [-1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        
        # Timestamp
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = "wamv/wamv/imu_wamv_link/imu_wamv_sensor"

        self.imu_pub.publish(imu_msg)

    def gps_cbk(self, msg):
        self.cur_lat, self.cur_lon = msg.latitude, msg.longitude
        if self.prev_lat is None:
            self.prev_lat, self.prev_lon = self.cur_lat, self.cur_lon
            return
        if self.cur_lat == self.prev_lat and self.cur_lon == self.prev_lon:
            return
        if self.gt_y is None and self.gt_x is None:
            return
        
        self.curr_orientation = self.estimate_heading()
        self.prev_lat, self.prev_lon = self.cur_lat, self.cur_lon

    def estimate_heading(self):
        # Calculate change in vector position (utm) and get its orientation to estimate heading
        x, y = self.to_utm(self.cur_lat, self.cur_lon)
        prev_x, prev_y = self.to_utm(self.prev_lat, self.prev_lon)
        dx, dy = x - prev_x, y - prev_y
        if dy == 0:
            dy = 0.0001
        
        if self.use_ground_truth:
            yaw = m.atan2(self.d_gt_y, self.d_gt_x)
        else:
            yaw = m.atan2(dy, dx)
        qx, qy, qz, qw = quaternion_from_euler(0.0, 0.0, yaw)
        return Quaternion(x=qx, y=qy, z=qz, w=qw)

    def to_utm(self, lat, lon):
        utm_x, utm_y, _, _ = utm.from_latlon(lat, lon)
        return [utm_x, utm_y]
    
def main(args=None):
    rclpy.init()

    imu_from_gps_node = ImuFromGPSNode()
    try:
        rclpy.spin(imu_from_gps_node)
    except KeyboardInterrupt:
        pass
    finally:
        imu_from_gps_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
