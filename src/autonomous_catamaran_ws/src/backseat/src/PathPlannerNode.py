#!/usr/bin/env python3

import os
import numpy as np
import time
import pathlib
import copy

#from scipy.misc import derivative
from backseat.msg import locg
import rospy
#import actionlib
from nav_msgs.msg import Path
try:
    import tf
    from tf.transformations import quaternion_from_euler, euler_from_quaternion
except:
    pass

from geographic_msgs.msg import GeoPath,GeoPoseStamped,GeoPose
from geometry_msgs.msg import Vector3Stamped, Twist, Pose, PoseWithCovarianceStamped, Point, Quaternion, Twist, PoseStamped
from sensor_msgs.msg import NavSatFix, Imu
from std_msgs.msg import Float32, Float64

from geometry_msgs.msg import Pose
from visualization_msgs.msg import Marker
import tf.transformations as tf

#
from NavigationTools import *
from PathFollower import *
from DataLogger import *
#from mpc_control.msg import DoMissionResult, DoMissionFeedback, DoMissionGoal, DoMissionAction
#
# from mpc_control.msg import loc, head, telem
#from wamv_comms.msg import states, gpsImu, thruster

# try:
#     from robot_control.msg import radio
# except:
#     print('-'*30)
#     print('Error importing radio message')
#     print('-'*30)


class PathPlannerNode:
    """! The path planner node class.
    This class is the highest layer interface of the navigation algorithm. It receives a set of waypoints
    and defines a path to connect such waypoints. It uses the PathFollower and NavigationTools classes
    to perform the path following task of whole guidance strategy. The init_app method is the only public
    method of this class since it implements an action sever node, and it is not intended to be imported
    anywhere else in the system.
    """
    def __init__(self, node_name='PathPlanner'):
        """! Class constructor.
        @param  None.
        @return An instance of the PathPlannerNode class.
        """
        # Create the variables required to handle a mission
        self.mission = 0
        self.path_follower = 0
        self.current_wp = NavigationTools.Waypoint(gps_lat = 0, gps_lon = 0)
        # Controller output related variables
        self.mission_complete = False
        self.tgt_heading = 0.0
        self.xte = 0.0
        self.tgt_wp = NavigationTools.Waypoint(gps_lat = 0, gps_lon = 0)
        self.head_u = 0
        self.head_i = 0
        self.head_prev_error = 0
        self.angle_idx = 0
        self.max_speed = rospy.get_param("max_speed")
        self.min_speed = rospy.get_param("min_speed")
        self.repulsion_heading = 0
        self.current_speed = 0
        self.PWMl = Float32()
        self.PWMr = Float32()
        # Wind FF compensator
        self.wind_dir = 0
        self.wind_speed = 0
        # ROS related variables
        self.node_name = node_name
        self.rate = 0
        self._as = 0
        # create messages that are used to publish feedback/result
        #self._feedback = DoMissionFeedback()
        #self._result = DoMissionResult()
        # Configuration parameters
        self.sim_enable = rospy.get_param("sim_enable")
        self.data_logger = DataLogger(data_description='field_test_jun_07/Rendezvouz_paper', headers=['head_err', 'tgt_heading', 'speed', 'wind_dir', 'wind_speed'])
    
    def __load_mission(self, mission):
        """! Loads the external mission into a list of waypoints that can be 
        undestood by the path follower.
        @param  mission The list of GeoPoseStamped waypoints provided by the high level mission planner.
        @return None.
        """
        wps = []
        for p in mission:
            # Convert from quaternion to euler angles to extract the desired heading(yaw)
            quaternion = (p.pose.orientation.x,
                          p.pose.orientation.y,
                          p.pose.orientation.z,
                          p.pose.orientation.w)
            _,_,head = euler_from_quaternion(quaternion)
            # Create the waypoint in the required format
            wp = {'lat':p.pose.position.latitude,
                  'lon':p.pose.position.longitude,
                  'depth':0.0,
                  'head':head,
                  'dive_mode':NavigationTools.DiveStyle['NONE'],
                  'wp_mode':NavigationTools.WayPointMode[p.header.frame_id]}
            # Create a list of dictionaries making sure to avoid overwriting 
            wps.append(copy.deepcopy(wp))

        rospy.loginfo('Creating Mission')
        self.mission = NavigationTools.Mission(waypoints=wps)
        rospy.loginfo('Creating Follower Object')
        path_type = rospy.get_param("path_type")
        if path_type == 'dubins':
            self.path_follower = PathFollower(mission=self.mission, path_creator=DubinsPath)
        elif path_type == 'reeds_shepp':
            self.path_follower = PathFollower(mission=self.mission, path_creator=ReedsSheppPath)
        rospy.loginfo('Mission Loaded')
        
    def init_app(self):
        """! Action server initializer.
        @param  None
        @return None.
        """
        # ROS node initilization
        print("Init app")
        rospy.init_node(self.node_name, anonymous=False)
        self.rate = rospy.Rate(10)
        # Subscribers
        if self.sim_enable:
            rospy.Subscriber('/wamv/sensors/gps/gps/fix', NavSatFix, self.__read_position_cbk, queue_size=1)
            rospy.Subscriber('/wamv/sensors/gps/gps/fix_velocity', Vector3Stamped, self.__read_speed_cbk, queue_size=1)
            rospy.Subscriber('/wamv/sensors/imu/imu/data', Imu, self.__read_imu_cbk, queue_size=1)
            rospy.Subscriber("/vrx/debug/wind/direction",Float64, self.__get_wind_dir_cbk)
            rospy.Subscriber("/vrx/debug/wind/speed", Float64, self.__get_wind_speed_cbk)
        else:
            rospy.Subscriber('/ublox_gps/fix', NavSatFix,self.__read_position_cbk, queue_size=1)
            rospy.Subscriber('/compass_bearing_deg', Float32, self.__read_imu_cbk, queue_size=1)

        # Publishers
        self.left_pub = rospy.Publisher('/wamv/thrusters/left_thrust_cmd', Float32, queue_size=10)
        self.right_pub = rospy.Publisher('/wamv/thrusters/right_thrust_cmd', Float32, queue_size=10)
        # Debug publishers
        self.working_waypoint_pub = rospy.Publisher("/goal_marker", Marker, queue_size=2)
        self.xtrac_err_pub = rospy.Publisher('/xtrac_err_abs', Float32, queue_size=1)
        self.head_err_pub = rospy.Publisher('/head_err_deg', Float32, queue_size=1)
        self.lookahead_pub = rospy.Publisher('/lookahead', Float32, queue_size=1)
        self.way_gps_pub = rospy.Publisher('/way_gps', locg, queue_size=1)
        
        # Visuals
        self.wk_path_pub = rospy.Publisher("/working_path",Path,queue_size=1)
        self.orig_path_pub = rospy.Publisher("/original_path",Path,queue_size=1)
        self.__main()
        """ # Start action server
        self._as = actionlib.SimpleActionServer(self.node_name, 
                                                DoMissionAction, 
                                                execute_cb=self.__run, 
                                                auto_start = False)
        self._as.start() """


    def __speeddir2diffdrive(self, speed, dir, k = 0.1):
        """! Internal call to transfor a speed and direction command to a
         differential drive command for the ASV.
        @param  speed   init_appForward speed of the vehicle.
        @param  dir     Direction where to move the vehicle, this__speeddir2diffdrive will determine the difference between the wheels speed.
        @param  k       Proportional coefficient to modulate how sharp to turn.
        @return None.
        """
        turn = np.clip(dir,-1,1)
        left =  speed - k*turn
        right = speed + k*turn
        left = np.clip(left,-1,1)
        right = np.clip(right,-1,1)

        self.left_pub.publish(left)
        self.right_pub.publish(right)

    def __get_wind_dir_cbk(self,data):
        """! Callback to catch wind direction.
        @param  data    The wind direction.
        @return None.
        """
        self.wind_dir = np.deg2rad(data.data)

    def __get_wind_speed_cbk(self, data):
        """! Callback to catch wind speed.
        @param  data    The wind speed value.
        @return None.
        """
        self.wind_speed = data.data

    def __read_speed_cbk(self,data):
        """! Callback to catch speed readings.
        @param  data    The 3 axis speed
        @return None.
        """
        # Exctract the fields from the message and fill the Waypoint object 
        # with the corresponding values
        # Since the integral component of ILOS is not being used, the vehicle speed doesn't affect.
        self.current_speed = np.linalg.norm([data.vector.x,data.vector.y])
        #rospy.loginfo('current_speed:{}'.format(self.current_speed))

    def __read_position_cbk(self,data):
        """! Callback to catch sensor readings.
        @param  data    The position from the GPS
        @return None.
        """
        self.current_wp.pose.gps_lat = data.latitude
        self.current_wp.pose.gps_lon = data.longitude

    def __read_imu_cbk(self,data):
        """! Callback to catch sensor readings.
        @param  data    The position from the GPS
        @return None.
        """
        # Exctract the fields from the message and fill the Waypoint object 
        # with the corresponding values
        if self.sim_enable:
            quaternion = (  data.orientation.x,
                            data.orientation.y,
                            data.orientation.z,
                            data.orientation.w)
            _,_,head = euler_from_quaternion(quaternion)
            self.current_wp.pose.head = head    # ENU Frame angle (East = 0 deg, North = 90 deg)
            self.current_wp.depth = 0.0
        else:
            compass_bearing_deg = data.data
            head = NavigationTools.GpsCalculations().convert_2_angle(compass_bearing_deg)
            self.current_wp.pose.head = head    # ENU Frame angle (East = 0 deg, North = 90 deg)
            self.current_wp.depth = 0.0
    
    def __publish_veh_output(self,data):
        """! Function to outuput the resulting controlled variables to the actuators.
        @param  data    The vehile commanded thrust for each of the motors.
        @return None.
        """
        cmd = Twist()        
        head_err = NavigationTools().GpsCalculations().angdiff(self.tgt_heading,self.current_wp.pose.head)
        #rospy.loginfo(f'tgt_heading: {self.tgt_heading*180/np.pi}, {self.current_wp.pose.head*180/np.pi}, {head_err*180/np.pi}')
        self.head_err_pub.publish(head_err*180/np.pi)
        way_gps_msg = locg()
        way_gps_msg.dbear = self.tgt_heading
        way_gps_msg.bear = self.current_wp.pose.head
        self.way_gps_pub.publish(way_gps_msg)
        # Controller
        # ==========================
        self.head_i += (head_err + self.head_prev_error)
        self.head_i = np.clip(self.head_i,-0.1,0.1)
        derivative = head_err - self.head_prev_error
        KP = rospy.get_param("kp") if rospy.has_param("kp") else 3.6
        KI = rospy.get_param("ki") if rospy.has_param("ki") else 2.553
        KD = rospy.get_param("kd") if rospy.has_param("kd") else 3.381
        self.head_u = KP*head_err + KI*self.head_i + KD*derivative
        self.head_prev_error = head_err
        up = rospy.get_param("max_speed")
        low = rospy.get_param("min_speed")
        x_up = 0.15
        x_low = 0.4
        m = (up-low)/(x_up-x_low)
        b = up - m*x_up
        x = np.abs(head_err)
        c_speed = np.clip(m*x+b,low,up)
        speed = rospy.get_param("fw_speed") if rospy.has_param("fw_speed") else c_speed
        # ==========================
        self.data_logger.log_data([head_err, self.tgt_heading, speed, self.wind_dir, self.wind_speed])
        self.__speeddir2diffdrive(speed, self.head_u, k=1)
    
    def __search_pattern(self,goal_wp,theta,delay):
        tgt_wp = NavigationTools.Waypoint(gps_lat = 0, gps_lon = 0)
        range_1 = np.linspace(-theta,theta,10)
        range_2 = np.linspace(theta,0,10)
        angle_range = np.hstack((range_1,range_2)) 
        complete = False
        self.angle_idx = self.angle_idx + 1
        i = self.angle_idx//delay
        tgt_wp.pose.gps_lat = goal_wp.pose.gps_lat
        tgt_wp.pose.gps_lon = goal_wp.pose.gps_lon
        if i >= angle_range.shape[0]:
            self.angle_idx = 0
            complete = True
            tgt_wp.pose.head = goal_wp.pose.head
        else:
            tgt_wp.pose.head = goal_wp.pose.head + angle_range[i]
        return tgt_wp,complete
    
    def __publish_paths(self,wk_path,orig_path):
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
        x_ref,y_ref,_,_ = utm.from_latlon(rospy.get_param('goal_lat'),rospy.get_param('goal_lon'))
        
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

        self.__publish_paths(wk_path_ros,orig_path_ros)
        self.current_wp.ToUTM()


        mc, self.tgt_heading, self.xte, self.tgt_wp = self.path_follower.update(current_wp = self.current_wp,
                                                                                speed = self.current_speed)
        
        self.lookahead_pub.publish(self.path_follower.look_ahead)
        #rospy.loginfo('mc:{},wp_mode:{}'.format(mc,self.tgt_wp.wp_mode)) 

        self.publishWorkingWypt(self.path_follower.working_path[self.path_follower.work_index])
        self.xtrac_err_pub.publish(abs(self.path_follower.ye))
        if (mc == False):
            self.__publish_veh_output(self.tgt_heading)
        self.mission_complete = mc
    
    def getMarker(self, color = [1, 0, 0, 1], type = Marker.ARROW, lwh = None):
        mkr = Marker()

        mkr.header.frame_id = "world"
        mkr.header.stamp = rospy.Time.now()

        # Set the shape (Sphere in this case)
        mkr.type = type
        mkr.id = 0
        if lwh:
            [length, width, height] = lwh
            mkr.scale.x = length
            mkr.scale.y = width 
            mkr.scale.z = height
        else:
            # Set marker's dimensions
            if type == Marker.CYLINDER:
                mkr.scale.x = 0.2 # 20cm radious in x
                mkr.scale.y = 0.2 # 20cm radious in y
                mkr.scale.z = 0.1 # 10cm height
            elif type == Marker.ARROW:
                mkr.scale.x = 2 # length
                mkr.scale.y = 0.2 # width
                mkr.scale.z = 0.2 # height
            elif type == Marker.LINE_STRIP:
                mkr.scale.x = 0.1 # length
                mkr.scale.y = 0.1 # width
                mkr.scale.z = 0.1 # height
            else:
                mkr.scale.x = 0.5 # length
                mkr.scale.y = 0.5 # width
                mkr.scale.z = 0.1 # height
        
        # Set the color (red by default)
        mkr.color.r = color[0]
        mkr.color.g = color[1]
        mkr.color.b = color[2]
        mkr.color.a = color[3] # transparency
        
        return mkr

    def publishWorkingWypt(self, waypoint):
        """ @param waypoint: a NavigationTools.Waypoint object """
        """ arrow = {
            x:
            y:
            head:
        }
        work_wyp_pub.Publish(arrow) """
        x_ref,y_ref,_,_ = utm.from_latlon(rospy.get_param('goal_lat'),rospy.get_param('goal_lon'))
        x = waypoint.pose.utm_x - x_ref
        y = waypoint.pose.utm_y - y_ref
        yaw = waypoint.pose.head

        quat = tf.quaternion_from_euler(0, 0, yaw)

        p = Pose()
        p.position.x = x
        p.position.y = y
        p.position.z = 0
        p.orientation.x = quat[0]
        p.orientation.y = quat[1]
        p.orientation.z = quat[2]
        p.orientation.w = quat[3]
        market_prot = self.getMarker(color=[1.0, 0.5, 1.0, 1.0], type=Marker.ARROW) # Blue
        market_prot.header.stamp = rospy.Time.now()
        market_prot.pose = p
        self.working_waypoint_pub.publish(market_prot)

    def __run(self, goal):
        """! Main execution loop of the action server. The logic is implemented on the update_follower method,
         in that way, if this node is implemented NOT as an action server, it can be easily modified by
         changing only this loop.
        @param goal The list of waypoints for the vehicle to follow.
        @return _feedback,_result   The feedback and the final action result.
        """
        self.__load_mission(goal.mission)
        
        while not self.mission_complete:
            
            # Check that preempt has not been requested by the client
            if self._as.is_preempt_requested():
                rospy.loginfo('%s: Preempted' % self.node_name)
                self._as.set_preempted()
                self._result.mission_complete = False
                break

            # Call the main function of the path follower
            self.__update_follower()
            self._feedback.xt_error = 0.0
            self._feedback.mission_completion_perc = 0.0
            
            # Publish the feedback
            self._as.publish_feedback(self._feedback)
            self.rate.sleep() 
        # Set the correct responses whenever the mission is completed
        if self.mission_complete:
            self.mission_complete = False
            self._result.mission_complete = True
            rospy.loginfo('%s: Succeeded' % self.node_name)
            self._as.set_succeeded(self._result)

    def __main(self):
        """ Non action server implementation of the PathFollower """

        rospy.loginfo('Creating Mission')
        self.mission = NavigationTools.Mission(filename=rospy.get_param("mission"))
        rospy.loginfo('Creating Follower Object')
        self.path_follower = PathFollower(mission=self.mission, path_creator=DubinsPath)
        rate = rospy.Rate(10)
        import time as ti
        while not rospy.is_shutdown():
            try:
                #s = ti.time()
                self.__update_follower()
                #print("took: ", round((s-ti.time())*1000, 2), "ms")
                rate.sleep()
            except rospy.ROSInterruptException as ROSe:
                rospy.logwarn(ROSe)
                break

if __name__ == "__main__":
    pp = PathPlannerNode(node_name='PathPlanner')
    print("start PP")
    pp.init_app()
    #rospy.spin()
