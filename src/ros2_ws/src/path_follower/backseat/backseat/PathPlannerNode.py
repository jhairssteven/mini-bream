#!/usr/bin/env python3

import utm
import numpy as np
import copy
import rclpy
import tf_transformations as tf

from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.executors import MultiThreadedExecutor


from visualization_msgs.msg import Marker
from geometry_msgs.msg import Vector3Stamped, Pose, PoseStamped
from sensor_msgs.msg import NavSatFix, Imu
from std_msgs.msg import Float32, Float64
from nav_msgs.msg import Path
from backseat_msgs.msg import Locg
from backseat_msgs.action import DoMission

from backseat.NavigationTools import *
from backseat.PathFollower import *
from backseat.DataLogger import *
import logging

class DebugOnlyFilter(logging.Filter):
    def filter(self, record):
        return record.levelno == logging.DEBUG

class PathPlannerNode(Node):
    def __init__(self):
        super().__init__('path_planner_node')
        self.get_logger().set_level(rclpy.logging.LoggingSeverity.ERROR)
        """ # Get node logger
        logger = self.get_logger()
        # Clear existing filters
        for h in logger.handlers:
            h.addFilter(DebugOnlyFilter()) """
        
        self.declare_parameter('max_linear_velocity', 3.0)
        self.declare_parameter('max_angular_velocity', 0.5)
        self.declare_parameter('max_linear_speed', 1.0)
        self.declare_parameter('min_linear_speed', 0.0)
        self.declare_parameter('sim_enable', False)
        self.declare_parameter('goal_lat', 40.448417)
        self.declare_parameter('goal_lon', -86.867750)
        self.declare_parameter('motor_thrust_scaling_factor', 1000.0)
        self.declare_parameter('publish_diff_drive', True)
        
        self.declare_parameter('kp', 3.6)
        self.declare_parameter('ki', 2.553)
        self.declare_parameter('kd', 3.381)
        self.declare_parameter('fw_speed', 0.1)

        self.max_linear_speed = self.get_parameter('max_linear_speed').value
        self.min_linear_speed = self.get_parameter('min_linear_speed').value
        self.sim_enable = self.get_parameter('sim_enable').value
        self.motor_thrust_scaling_factor = self.get_parameter('motor_thrust_scaling_factor').value
        self.max_linear_velocity = self.get_parameter('max_linear_velocity').value
        self.max_angular_velocity = self.get_parameter('max_angular_velocity').value

        self.publish_diff_drive = self.get_parameter('publish_diff_drive').get_parameter_value().bool_value
        self.goal_lat = self.get_parameter('goal_lat').value
        self.goal_lon = self.get_parameter('goal_lon').value

        self.mission = None
        self.path_follower = None
        self.current_wp = NavigationTools.Waypoint(gps_lat=0, gps_lon=0)
        self.tgt_heading = 0.0
        self.xte = 0.0
        self.tgt_wp = NavigationTools.Waypoint(gps_lat=0, gps_lon=0)
        self.head_u = 0
        self.head_i = 0
        self.head_prev_error = 0
        self.angle_idx = 0
        self.repulsion_heading = 0
        self.current_speed = 0
        self.wind_dir = 0
        self.wind_speed = 0
        self.mission_complete = False

        self.paths_published = False

        self._feedback = DoMission.Feedback()
        self._result = DoMission.Result()

        #self.data_logger = DataLogger(data_description='field_test_jun_07/Rendezvouz_paper',
        #                              headers=['head_err', 'tgt_heading', 'speed', 'wind_dir', 'wind_speed'])

        qos = QoSProfile(depth=10)
        
        # For high-rate, non-critical sensor data
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
        
        self.linear_velocity_pct_pub = self.create_publisher(Float64, '/wamv/linear_velocity_auto', qos_best_effort_volatile)
        self.angular_velocity_pct_pub = self.create_publisher(Float64, '/wamv/angular_velocity_auto', qos_best_effort_volatile)

        msg_type_thrusters = Float64
        qos_thrusters = qos_reliable_volatile

        # Publishers to control thrusters directly
        self.left_thruster_publisher = self.create_publisher(msg_type_thrusters, '/wamv/thrusters/left/thrust', qos_thrusters)
        self.right_thruster_publisher = self.create_publisher(msg_type_thrusters, '/wamv/thrusters/right/thrust', qos_thrusters)
        
        
        self.working_waypoint_pub = self.create_publisher(Marker, '/goal_marker', qos)
        self.xtrac_err_pub = self.create_publisher(Float32, '/xtrac_err_abs', qos)
        self.head_err_pub = self.create_publisher(Float32, '/head_err_deg', qos)
        self.lookahead_pub = self.create_publisher(Float32, '/lookahead', qos)
        self.way_gps_pub = self.create_publisher(Locg, '/way_gps', qos)
        self.wk_path_pub = self.create_publisher(Path, '/working_path', qos)
        self.vis_wk_path_pub = self.create_publisher(Path, '/log/working_path', qos)
        self.orig_path_pub = self.create_publisher(Path, '/original_path', qos)

        if self.sim_enable:
            self.create_subscription(NavSatFix, '/wamv/sensors/gps/gps/fix', self.__read_position_cbk, qos_best_effort_volatile)
            self.create_subscription(Vector3Stamped, '/wamv/sensors/gps/gps/fix_velocity', self.__read_speed_cbk, qos_best_effort_volatile)
            self.create_subscription(Imu, '/wamv/sensors/imu/imu/data', self.__read_imu_cbk, qos_best_effort_volatile)
            self.create_subscription(Float64, '/vrx/debug/wind/direction', self.__get_wind_dir_cbk, qos)
            self.create_subscription(Float64, '/vrx/debug/wind/speed', self.__get_wind_speed_cbk, qos)
        else:
            self.create_subscription(NavSatFix, '/ublox_gps/fix', self.__read_position_cbk, qos)
            self.create_subscription(Float32, '/compass_bearing_deg', self.__read_imu_cbk, qos)

        self.action_server = ActionServer(
            self,
            DoMission,
            'path_planner_action',
            execute_callback=self.__run,
            goal_callback=self.__goal_callback,
            cancel_callback=self.__cancel_callback)
        self.get_logger().info('Server initialized. Waiting for new mission...')

    def __goal_callback(self, goal_request):
        #self.get_logger().info('Received goal request')
        return GoalResponse.ACCEPT

    def __cancel_callback(self, goal_handle):
        self.get_logger().info('Received cancel request')
        
        return CancelResponse.ACCEPT

    def __read_position_cbk(self, msg):
        self.current_wp.pose.gps_lat = msg.latitude
        self.current_wp.pose.gps_lon = msg.longitude

    def __read_speed_cbk(self, msg):
        self.current_speed = np.linalg.norm([msg.vector.x, msg.vector.y])

    def __get_wind_dir_cbk(self, msg):
        self.wind_dir = np.deg2rad(msg.data)

    def __get_wind_speed_cbk(self, msg):
        self.wind_speed = msg.data

    def __read_imu_cbk(self, msg):
        if self.sim_enable:
            quaternion = (msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w)
            _, _, head = tf.euler_from_quaternion(quaternion)
            self.current_wp.pose.head = head
        else:
            head = NavigationTools.GpsCalculations().convert_2_angle(msg.data)
            self.current_wp.pose.head = head
        self.current_wp.depth = 0.0
    
    def build_thruster_msg(self, thrust_pct):
        """ thrust_pct: a Value between [-1, 1] """
        return Float64(data = self.motor_thrust_scaling_factor*thrust_pct)
    
    def to_diff_driv(self, linear_speed, angular_speed, k = 0.1, diff_drive=False):
        """! Internal call to transfor a speed and direction command to a
         differential drive command for the ASV.
        @param  k       Proportional coefficient to modulate how sharp to turn.
        @return None.
        """

        angular_velocity_pct = np.clip(angular_speed, -1, 1)
        linear_velocity_pct = np.clip(linear_speed, -1, 1)

        if (diff_drive):
            left =  linear_velocity_pct - k*angular_velocity_pct
            right = linear_velocity_pct + k*angular_velocity_pct
            left = np.clip(left, -1, 1)
            right = np.clip(right, -1, 1)
            
            self.left_thruster_publisher.publish(self.build_thruster_msg(left))
            self.right_thruster_publisher.publish(self.build_thruster_msg(right))
            return

        self.max_linear_vel = self.get_parameter('max_linear_velocity').value
        self.max_angular_vel = self.get_parameter('max_angular_velocity').value
        self.linear_velocity_pct_pub.publish(Float64(data=self.max_linear_vel*linear_velocity_pct))
        self.angular_velocity_pct_pub.publish(Float64(data=self.max_angular_vel*angular_velocity_pct))
        

    def __velocity_control_step(self):
        """Compute velocity control commands and publish them to actuators."""
        head_err = NavigationTools().GpsCalculations().angdiff(self.tgt_heading, self.current_wp.pose.head)
        #rclpy.loginfo(f'tgt_heading: {self.tgt_heading*180/np.pi}, {self.current_wp.pose.head*180/np.pi}, {head_err*180/np.pi}')
        self.head_err_pub.publish(Float32(data=float(head_err*180/np.pi)))
        way_gps_msg = Locg()
        way_gps_msg.dbear = float(self.tgt_heading)
        way_gps_msg.bear = float(self.current_wp.pose.head)
        self.way_gps_pub.publish(way_gps_msg)
        
        
        # PID Controller (Desired Heading (ILOS), Current heading)
        # ==========================
        self.head_i += (head_err + self.head_prev_error)
        self.head_i = np.clip(self.head_i,-0.1,0.1)
        derivative = head_err - self.head_prev_error
        KP = self.get_parameter("kp").value
        KI = self.get_parameter("ki").value
        KD = self.get_parameter("kd").value
        self.head_u = KP*head_err + KI*self.head_i + KD*derivative
        self.head_prev_error = head_err
        #self.get_logger().info(f'desired head: {self.head_u}, head_error: {self.head_prev_error}')
        
        linear_speed = self.get_cmd_velocity_from_error_state(head_err)

        self.to_diff_driv(linear_speed, self.head_u, k=1, diff_drive=self.publish_diff_drive)
    
    def get_cmd_velocity_from_error_state(self, head_err):
        # Adjust linear speed based on heading error (for bigger error, go slower)
        # Linear speed range will go from 0 to param 'max_linear_speed' value.
        # The vehicle will have 0.0 linear speed when heading error is too big.

        max_linear_speed = self.get_parameter("max_linear_speed").value
        min_linear_speed = 0.0
        angle_threshold_fast = 0.15 # rads
        angle_threshold_slow = 0.4  # rads
        
        m = (max_linear_speed-min_linear_speed)/(angle_threshold_fast-angle_threshold_slow)
        b = max_linear_speed - m*angle_threshold_fast
        angle_error = np.abs(head_err)
        linear_speed = m*angle_error + b

        linear_speed = np.clip(linear_speed, min_linear_speed, max_linear_speed)

        self.linear_velocity_pct_pub.publish(Float64(data=linear_speed))
        return linear_speed


    def __publish_paths(self, wk_path, orig_path):
        self.vis_wk_path_pub.publish(wk_path)
        self.wk_path_pub.publish(wk_path)
        self.orig_path_pub.publish(orig_path)
        self.paths_published = True

    def waypoints_to_ros_path(self, waypoints, x_ref, y_ref, frame_id='world'):
        """
        Convert a list of waypoints to a ROS Path message relative to a reference point.
        
        :param waypoints: List of waypoints, each with pose.utm_x and pose.utm_y attributes
        :param goal_lat: Latitude of the reference goal
        :param goal_lon: Longitude of the reference goal
        :param frame_id: Frame ID to set for the Path and poses
        :return: nav_msgs.msg.Path
        """
        path_ros = Path()
        path_ros.header.frame_id = frame_id

        for wp in waypoints:
            pose = PoseStamped()
            pose.header.frame_id = frame_id
            pose.pose.position.x = wp.pose.utm_x - x_ref
            pose.pose.position.y = wp.pose.utm_y - y_ref
            pose.pose.position.z = 0.0
            pose.pose.orientation.x = 0.0
            pose.pose.orientation.y = 0.0
            pose.pose.orientation.z = 0.0
            pose.pose.orientation.w = 1.0  # Identity quaternion
            path_ros.poses.append(pose)
        
        return path_ros

    def __update_follower(self, new_mission=True):
        """! Main loop intended to update the output of the path following algorithm.
        @param  None.
        @return None.
        """
        # Convert wk_path and orig_path from PathFollower into ROS Path messages 
        if new_mission or not self.paths_published or self.path_follower.replan_triggered:
            x_ref, y_ref, _, _ = utm.from_latlon(self.goal_lat, self.goal_lon)

            wk_path, orig_path = self.path_follower.get_generated_paths()
            wk_path_ros = self.waypoints_to_ros_path(wk_path, x_ref, y_ref)
            orig_path_ros = self.waypoints_to_ros_path(orig_path, x_ref, y_ref)
            self.__publish_paths(wk_path_ros, orig_path_ros)
            new_mission = False

        self.current_wp.ToUTM()

        mc, self.tgt_heading, self.xte, self.tgt_wp = self.path_follower.update(current_wp = self.current_wp,
                                                                                speed = self.current_speed)
        
        self.lookahead_pub.publish(Float32(data=float(self.path_follower.look_ahead)))
        #rclpy.loginfo('mc:{},wp_mode:{}'.format(mc,self.tgt_wp.wp_mode)) 

        self.publishWorkingWypt(self.path_follower.working_path[self.path_follower.work_index])
        self.xtrac_err_pub.publish(Float32(data=float(abs(self.path_follower.ye))))
        if (mc == False):
            self.__velocity_control_step()
        else:
            # Publish a zero velocity as a last command
            self.to_diff_driv(linear_speed=0.0, angular_speed=0.0, k=1, diff_drive=self.publish_diff_drive)
        self.mission_complete = mc

    def __load_mission(self, mission):
        wps = []
        for p in mission:
            quaternion = (p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w)
            _, _, head = tf.euler_from_quaternion(quaternion)
            wp = {'lat': p.position.latitude,
                  'lon': p.position.longitude,
                  'depth': 0.0,
                  'head': head,
                  'dive_mode': NavigationTools.DiveStyle['NONE'],
                  'wp_mode': NavigationTools.WayPointMode.REGULAR}
            
            wps.append(copy.deepcopy(wp))
        return NavigationTools.Mission(waypoints=wps)

    def __run(self, goal_handle):
        if goal_handle.request.filename:
            self.mission = NavigationTools.Mission(filename=goal_handle.request.filename)
        elif goal_handle.request.mission:
            self.mission = self.__load_mission(goal_handle.request.mission)
        else:
            goal_handle.abort()
            self.get_logger().error('No valid mission source provided.')
            return DoMission.Result(mission_complete=False)

        self.path_follower = PathFollower(self, mission=self.mission, path_creator=DubinsPath)
        self.new_mission = True
        self.mission_complete = False
        self.get_logger().info('New mission loaded. Executing...')
        while not self.mission_complete:
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().warn('Cancel requested')
                return DoMission.Result(mission_complete=False)
            
            self.__update_follower(new_mission=self.new_mission)
            self.new_mission = False

            self._feedback.xt_error = float(abs(self.path_follower.ye))
            goal_handle.publish_feedback(self._feedback)

        goal_handle.succeed()
        self.get_logger().info("Mission completed")
        
        return DoMission.Result(mission_complete=True)

    def getMarker(self, color=[1, 0, 0, 1], type=Marker.ARROW, lwh=None):
        mkr = Marker()
        mkr.header.frame_id = "world"
        mkr.header.stamp = self.get_clock().now().to_msg()
        mkr.type = type
        mkr.id = 0
        if lwh:
            mkr.scale.x, mkr.scale.y, mkr.scale.z = lwh
        elif type == Marker.CYLINDER:
            mkr.scale.x = mkr.scale.y = 0.2
            mkr.scale.z = 0.1
        elif type == Marker.ARROW:
            mkr.scale.x = 2.0
            mkr.scale.y = mkr.scale.z = 0.2
        elif type == Marker.LINE_STRIP:
            mkr.scale.x = mkr.scale.y = mkr.scale.z = 0.1
        else:
            mkr.scale.x = mkr.scale.y = 0.5
            mkr.scale.z = 0.1
        mkr.color.r, mkr.color.g, mkr.color.b, mkr.color.a = color
        return mkr

    def publishWorkingWypt(self, waypoint):
        x_ref, y_ref, _, _ = utm.from_latlon(self.get_parameter('goal_lat').value,
                                             self.get_parameter('goal_lon').value)
        x = waypoint.pose.utm_x - x_ref
        y = waypoint.pose.utm_y - y_ref
        yaw = waypoint.pose.head
        quat = tf.quaternion_from_euler(0, 0, yaw)
        p = Pose()
        p.position.x = x
        p.position.y = y
        p.position.z = 0.0
        p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = quat
        marker = self.getMarker(color=[1.0, 0.5, 1.0, 1.0], type=Marker.ARROW)
        marker.pose = p
        self.working_waypoint_pub.publish(marker)

    def __search_pattern(self, goal_wp, theta, delay):
        tgt_wp = NavigationTools.Waypoint(gps_lat=0, gps_lon=0)
        range_1 = np.linspace(-theta, theta, 10)
        range_2 = np.linspace(theta, 0, 10)
        angle_range = np.hstack((range_1, range_2))
        complete = False
        self.angle_idx += 1
        i = self.angle_idx // delay
        tgt_wp.pose.gps_lat = goal_wp.pose.gps_lat
        tgt_wp.pose.gps_lon = goal_wp.pose.gps_lon
        if i >= angle_range.shape[0]:
            self.angle_idx = 0
            complete = True
            tgt_wp.pose.head = goal_wp.pose.head
        else:
            tgt_wp.pose.head = goal_wp.pose.head + angle_range[i]
        return tgt_wp, complete

def main(args=None):
    rclpy.init(args=args)
    node = PathPlannerNode()
    # We use a MultiThreadedExecutor to handle incoming goal requests concurrently
    rclpy.spin(node, executor=MultiThreadedExecutor())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
