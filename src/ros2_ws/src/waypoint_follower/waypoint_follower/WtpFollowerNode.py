#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
from tf_transformations import quaternion_from_euler
from sensor_msgs.msg import NavSatFix, Imu
from std_msgs.msg import Float32, Float64
from geometry_msgs.msg import Quaternion
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
import tf_transformations as tf
import utm
import math as m
from waypoint_follower.CommonUtils import *

class WtpFollowerNode(Node):
    def __init__(self, node_name='wtp_follower'):
        super().__init__(node_name)
        
        # For high-rate sensor data
        qos_best_effort_volatile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,   # Send ony once without retry
            durability=QoSDurabilityPolicy.VOLATILE,        # Do not store old messages
            depth=1
        )
        # For critical real-time commands, reliable but no stored history
        qos_reliable_volatile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,  # Rety until success
            durability=QoSDurabilityPolicy.VOLATILE,    # Do not store old messages
            depth=5
        )
        
        self.create_subscription(NavSatFix, '/wamv/sensors/gps/gps/fix', self.gps_cbk, qos_reliable_volatile)
        self.imu_pub = self.create_subscription(Imu, '/wamv/sensors/imu/imu/data/estimated', self.imu_cbk, qos_best_effort_volatile)
        self.goal_sub = self.create_subscription(PoseStamped, '/goal_pose', self.goal_callback, qos_best_effort_volatile)
        
        # sim
        #self.left_pub = self.create_publisher(Float64, '/wamv/thrusters/left/thrust', qos_reliable_volatile)
        #self.right_pub = self.create_publisher(Float64, '/wamv/thrusters/right/thrust', qos_reliable_volatile)
        # real
        self.left_pub = self.create_publisher(Float32, '/pwm/left_thrust_cmd', qos_reliable_volatile)
        self.right_pub = self.create_publisher(Float32, '/pwm/right_thrust_cmd', qos_reliable_volatile)

        self.timer = self.create_timer(1.0 / 19.0, self.runner)  # 19 Hz

        # variables
        self.curr_x = self.curr_y = None
        self.curr_orientation = None
        self.goal_x = None
        self.goal_y = None
        self.goal_yaw = None
        
        self.declare_parameter("goal_lat", 40.448417)
        self.declare_parameter("goal_lon", -86.867750)
        origin_lat = self.get_parameter("goal_lat").value
        origin_lon = self.get_parameter("goal_lon").value
        self.origin_utm_x, self.origin_utm_y = self.to_utm(origin_lat, origin_lon)
        
        self.declare_parameter("waypoint_achieved_offset_distance", 2.0)
        self.WAYPOINT_ACHIEVED_OFFSET_DISTANCE = self.get_parameter("waypoint_achieved_offset_distance").value

        
        self.declare_parameter("kr", 0.07)
        self.declare_parameter("ka", 0.8)
        self.declare_parameter("kb", 0.1)
        self.declare_parameter("max_linear_x", 1.0)
        self.declare_parameter("max_angular_z", 0.5)
        self.kr = self.get_parameter("kr").value
        self.ka = self.get_parameter("ka").value
        self.kb = self.get_parameter("kb").value
        self.max_linear_x = self.get_parameter("max_linear_x").value
        self.max_angular_z = self.get_parameter("max_angular_z").value

    def goal_callback(self, msg: PoseStamped):
        x_utm = self.origin_utm_x + msg.pose.position.x
        y_utm = self.origin_utm_y + msg.pose.position.y
        yaw = self.get_yaw_from_quaternion(msg.pose.orientation)
        
        self.goal_x = x_utm
        self.goal_y = y_utm
        self.goal_yaw = yaw
        self.get_logger().info(f"New goal (x, y, yaw): {msg.pose.position.x}, {msg.pose.position.y}, {yaw}")

    def get_yaw_from_quaternion(self, orientation):
        q = (orientation.x, orientation.y, orientation.z, orientation.w)
        _, _, yaw = tf.euler_from_quaternion(q)
        return yaw
    def to_utm(self, lat, lon):
        utm_x, utm_y, _, _ = utm.from_latlon(lat, lon)
        return [utm_x, utm_y]
    
    def gps_cbk(self, msg):
        self.curr_x, self.curr_y = self.to_utm(msg.latitude, msg.longitude)

    def imu_cbk(self, msg):
        self.curr_orientation = msg.orientation
    
    def get_curr_pose(self):
        if self.curr_x == None or self.curr_y == None or self.curr_orientation == None:
            self.get_logger().warn("Current pose not yet available.")
            return Pose2D(x=0.0, y=0.0, theta=0.0)
        
        return Pose2D(x=float(self.curr_x), y=float(self.curr_y), theta=float(self.get_yaw_from_quaternion(self.curr_orientation)))
    
    def get_goal_pose(self):
        """ Get goal pose from Rviz if available, otherwise get 0,0, yaw= 0 Pose2D """
        if (self.goal_x == None or self.goal_y == None or self.goal_yaw == None):
            self.get_logger().warn("Goal pose not yet available.")
            return None
        return Pose2D(x=float(self.goal_x), y=float(self.goal_y), theta=float(self.goal_yaw))

    def runner(self):
        start_pose = self.get_curr_pose()
        goal_pose = self.get_goal_pose()
        if goal_pose == None:
            return
        rho, alpha, beta = get_rho_alpha_beta_cart(start_pose, goal_pose)
        
        if rho < self.WAYPOINT_ACHIEVED_OFFSET_DISTANCE:
            self.goal_x = self.goal_y = self.goal_yaw = None
            self.get_logger().info(f'Waypoint achieved at rho (m): {rho}')

        t = get_twist(rho, alpha, beta, kr=self.kr, ka=self.ka, kb=self.kb, max_linear_x=self.max_linear_x, max_angular_z=self.max_angular_z)
        self.pub_diff_drive(t.linear.x, t.angular.z)

    def pub_diff_drive(self, linear_vel, angular_vel, k=1.0):
        self.get_logger()
        left_mtr_vel =  linear_vel - k*angular_vel
        right_mtr_vel = linear_vel + k*angular_vel
        
        left_mtr_vel = np.clip(left_mtr_vel, -1, 1)
        right_mtr_vel = np.clip(right_mtr_vel, -1, 1)

        #self.left_pub.publish(Float64(data=float(1000*left_mtr_vel)))
        #self.right_pub.publish(Float64(data=float(1000*right_mtr_vel)))
        self.left_pub.publish(Float32(data=float(left_mtr_vel)))
        self.right_pub.publish(Float32(data=float(right_mtr_vel)))

def main(args=None):
    rclpy.init()

    wtp_follower_node = WtpFollowerNode()
    try:
        rclpy.spin(wtp_follower_node)
    except KeyboardInterrupt:
        pass
    finally:
        wtp_follower_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
