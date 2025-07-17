#!/usr/bin/env python3

import utm
import numpy as np
import copy
import rclpy
import tf_transformations as tf

from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.qos import QoSProfile

from backseat_msgs.msg import Locg
from backseat_msgs.action import DoMission
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Vector3Stamped, Pose, PoseStamped
from sensor_msgs.msg import NavSatFix, Imu
from std_msgs.msg import Float32, Float64
from nav_msgs.msg import Path

from backseat.NavigationTools import *
from backseat.PathFollower import *
from backseat.DataLogger import *

class PathPlannerNode(Node):
    def __init__(self):
        super().__init__('path_planner_node')

        self.declare_parameter('max_speed', 1.0)
        self.declare_parameter('min_speed', 0.2)
        self.declare_parameter('sim_enable', True)
        self.declare_parameter('goal_lat', 40.44707869787295)
        self.declare_parameter('goal_lon', -86.86853024869774)
        
        self.declare_parameter('kp', 3.6)
        self.declare_parameter('ki', 2.553)
        self.declare_parameter('kd', 3.381)
        self.declare_parameter('fw_speed', 0.1)

        self.max_speed = self.get_parameter('max_speed').value
        self.min_speed = self.get_parameter('min_speed').value
        self.sim_enable = self.get_parameter('sim_enable').value


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

        self._feedback = DoMission.Feedback()
        self._result = DoMission.Result()

        self.data_logger = DataLogger(data_description='field_test_jun_07/Rendezvouz_paper',
                                      headers=['head_err', 'tgt_heading', 'speed', 'wind_dir', 'wind_speed'])

        qos = QoSProfile(depth=10)
        self.left_pub = self.create_publisher(Float64, '/wamv/thrusters/left/thrust', qos)
        self.right_pub = self.create_publisher(Float64, '/wamv/thrusters/right/thrust', qos)
        self.working_waypoint_pub = self.create_publisher(Marker, '/goal_marker', qos)
        self.xtrac_err_pub = self.create_publisher(Float32, '/xtrac_err_abs', qos)
        self.head_err_pub = self.create_publisher(Float32, '/head_err_deg', qos)
        self.lookahead_pub = self.create_publisher(Float32, '/lookahead', qos)
        self.way_gps_pub = self.create_publisher(Locg, '/way_gps', qos)
        self.wk_path_pub = self.create_publisher(Path, '/working_path', qos)
        self.orig_path_pub = self.create_publisher(Path, '/original_path', qos)

        if self.sim_enable:
            self.create_subscription(NavSatFix, '/wamv/sensors/gps/gps/fix', self.__read_position_cbk, qos)
            self.create_subscription(Vector3Stamped, '/wamv/sensors/gps/gps/fix_velocity', self.__read_speed_cbk, qos)
            self.create_subscription(Imu, '/wamv/sensors/imu/imu/data', self.__read_imu_cbk, qos)
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
        self.get_logger().info('Received goal request')
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
    
    def __speeddir2diffdrive(self, speed, dir, k = 0.1):
        """! Internal call to transfor a speed and direction command to a
         differential drive command for the ASV.
        @param  speed   init_appForward speed of the vehicle.
        @param  dir     Direction where to move the vehicle, will determine the difference between the wheels speed.
        @param  k       Proportional coefficient to modulate how sharp to turn.
        @return None.
        """
        turn = np.clip(dir,-1,1)
        left =  speed - k*turn
        right = speed + k*turn
        left = np.clip(left,-1,1)
        right = np.clip(right,-1,1)

        self.left_pub.publish(Float64(data=float(1000*left)))
        self.right_pub.publish(Float64(data=float(1000*right)))

    def __publish_veh_output(self):
        """ Function to outuput the resulting controlled variables to the actuators."""
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
        up = self.get_parameter("max_speed").value
        low = self.get_parameter("min_speed").value
        x_up = 0.15
        x_low = 0.4
        m = (up-low)/(x_up-x_low)
        b = up - m*x_up
        x = np.abs(head_err)
        speed = np.clip(m*x+b,low,up)
        # TODO: use constant forward speed from parameter (how if .declare_parameter is mandatory?)
        # speed = self.get_parameter("fw_speed").value if self.has_parameter("fw_speed") else c_speed
        # ==========================

        self.data_logger.log_data([head_err, self.tgt_heading, speed, self.wind_dir, self.wind_speed])
        self.__speeddir2diffdrive(speed, self.head_u, k=1)
    

    def __publish_paths(self, wk_path, orig_path):
        self.wk_path_pub.publish(wk_path)
        self.orig_path_pub.publish(orig_path)

    def __update_follower(self):
        """! Main loop intended to update the output of the path following algorithm.
        @param  None.
        @return None.
        """
        # Convert wk_path and orig_path from PathFollower into ROS Path messages 
        wk_path,orig_path = self.path_follower.get_generated_paths()
        wk_path_ros = Path()
        wk_path_ros.header.frame_id = 'world'
        orig_path_ros = Path()
        orig_path_ros.header.frame_id = 'world'
        x_ref,y_ref,_,_ = utm.from_latlon(self.get_parameter('goal_lat').value, self.get_parameter('goal_lon').value)
        
        for i,wp in enumerate(wk_path):
            pose = PoseStamped()
            pose.pose.position.x = wp.pose.utm_x - x_ref
            pose.pose.position.y = wp.pose.utm_y - y_ref
            pose.pose.position.z = 0.0
            pose.pose.orientation.x = 0.0
            pose.pose.orientation.y = 0.0
            pose.pose.orientation.z = 0.0
            pose.pose.orientation.w = 0.0
            wk_path_ros.poses.append(pose)
        for i,wp in enumerate(orig_path):
            pose = PoseStamped()
            pose.pose.position.x = wp.pose.utm_x - x_ref
            pose.pose.position.y = wp.pose.utm_y - y_ref
            pose.pose.position.z = 0.0
            pose.pose.orientation.x = 0.0
            pose.pose.orientation.y = 0.0
            pose.pose.orientation.z = 0.0
            pose.pose.orientation.w = 0.0
            orig_path_ros.poses.append(pose)

        self.__publish_paths(wk_path_ros, orig_path_ros)
        self.current_wp.ToUTM()


        mc, self.tgt_heading, self.xte, self.tgt_wp = self.path_follower.update(current_wp = self.current_wp,
                                                                                speed = self.current_speed)
        
        self.lookahead_pub.publish(Float32(data=self.path_follower.look_ahead))
        #rclpy.loginfo('mc:{},wp_mode:{}'.format(mc,self.tgt_wp.wp_mode)) 

        self.publishWorkingWypt(self.path_follower.working_path[self.path_follower.work_index])
        self.xtrac_err_pub.publish(Float32(data=float(abs(self.path_follower.ye))))
        if (mc == False):
            self.__publish_veh_output()
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
        self.get_logger().info(f'Executing new mission: {goal_handle.request}')

        if goal_handle.request.filename:
            self.mission = NavigationTools.Mission(filename=goal_handle.request.filename)
        elif goal_handle.request.mission:
            self.mission = self.__load_mission(goal_handle.request.mission)
        else:
            goal_handle.abort()
            self.get_logger().error('No valid mission source provided.')
            return DoMissionResult(mission_complete=False)

        self.path_follower = PathFollower(self, mission=self.mission, path_creator=DubinsPath)

        while not self.mission_complete:
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                return DoMissionResult(mission_complete=False)

            self.__update_follower()
            self._feedback.xt_error = float(abs(self.path_follower.ye))
            goal_handle.publish_feedback(self._feedback)
            rclpy.spin_once(self, timeout_sec=0.1)

        result = DoMissionResult()
        result.mission_complete = True
        goal_handle.succeed()
        self.get_logger().info('Mission completed')
        return result

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
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
