""" 
    1. When a new message comes to /move_base_simple/goal topic of type geometry_msgs/PoseStamped
    2. Get current vehicle pose from topic /curr_marker" of type visualization_msgs/Marker
    3. Convert PoseStamped and current vehicle position to GeoPoseStamped message types
    4. Create a 2-waypoint mission p1->p2 where p1 is current position and p2 is target position. This is just a list of GeoPoseStamped messages. ie. [p1, p2]
    5. Call the PathPlanner action server with this new mission.
"""

#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker
from geographic_msgs.msg import GeoPoseStamped

import actionlib
from backseat.msg import DoMissionAction, DoMissionGoal, DoMissionResult, DoMissionFeedback

import utm
class Pose():
        def __init__(self, gps_lat = -1, gps_lon = -1, utm_x = -1, utm_y = -1, utm_zone = -1, head = -1):
            self.gps_lat = gps_lat
            self.gps_lon = gps_lon
            self.head = head
            self.utm_x, self.utm_y, self.utm_zone = utm_x, utm_y, utm_zone
            # Convert the position to both formats
            if utm_x != -1 and utm_y != -1 and utm_zone != -1:
                self.convert_to_gps()
            elif gps_lat != -1 and gps_lon != -1:
                self.convert_to_utm()
            else:
                raise ValueError(''' You need to provide at least one way to initialize the position of the Waypoint ''')

        def __str__(self):
            out_str = ''

            out_str = 'Pose(GPS[lat:{},lon:{},head:{}] - UTM[x:{},y:{},head:{},zone:{}])'.format(self.gps_lat,self.gps_lon,self.head,
                                                                                                    self.utm_x,self.utm_y,self.head,self.utm_zone)

            return out_str

        def convert_to_gps(self):
            self.gps_lat, self.gps_lon = utm.to_latlon(self.utm_x, self.utm_y, int(self.utm_zone[:-1]), self.utm_zone[-1])
        
        def convert_to_utm(self):
            self.utm_x, self.utm_y, utm_zone_num, utm_zone_ltr =  utm.from_latlon( self.gps_lat, self.gps_lon)
            self.utm_zone = '%d%s'%(utm_zone_num, utm_zone_ltr)

class ActionServerClient:
    def __init__(self):
        self._ac = actionlib.SimpleActionClient('PathPlanner_action_server', DoMissionAction)
        self._ac.wait_for_server()
    
    def send_goal(self, mission):
        """ @param mission: The list of GeoPoseStamped waypoints representing the mission. """
        
        rospy.loginfo("Cancelling all previous goals...")
        self._ac.cancel_all_goals()
        
        # Send a new goal
        goal = DoMissionGoal()
        goal.mission = mission
        
        self._ac.send_goal(goal, feedback_cb=self.feedback_cb, done_cb=self.done_cb)
        rospy.loginfo("New mission sent.")
        rospy.loginfo('-'*20)

    def done_cb(self, _, result):
        rospy.loginfo(f'Mission_complete={result.mission_complete}')
        rospy.loginfo('-'*20)
        rospy.loginfo('')

    def feedback_cb(self, feedback):
        def fbk_logger(data):
            rospy.loginfo(f'Action client [Feedback]: {data}')

        #fbk_logger(feedback.xt_error)
        #fbk_logger(feedback.mission_completion_perc)
        pass

class NavGoalToWaypoint:
    def __init__(self):
        rospy.init_node('nav_goal_to_waypoint', anonymous=True)
        
        self.goal_sub = rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.goal_callback)
        self.marker_sub = rospy.Subscriber('/curr_marker', Marker, self.marker_callback)
        
        self.current_marker = None
        self._pp_ac = ActionServerClient()

    def goal_callback(self, goal_msg):
        if self.current_marker is None:
            rospy.logwarn("Current marker not received yet.")
            return
        
        current_geo_pose = self.convert_to_geopose(self.current_marker.pose, self.current_marker.header)
        goal_geo_pose = self.convert_to_geopose(goal_msg.pose, goal_msg.header)
        
        mission = [current_geo_pose, goal_geo_pose]
        rospy.loginfo('')
        rospy.loginfo('-'*20)
        rospy.loginfo("New goal [lat, lon]: %s, %s", mission[1].pose.position.latitude, mission[1].pose.position.longitude)
        self._pp_ac.send_goal(mission)

    def marker_callback(self, marker_msg):
        self.current_marker = marker_msg
    
    def convert_to_geopose(self, pose, header=None):
        """ Convert @param pose which is in a local coordinate frame with origin at utm(goal_lat, goal_lon).
        To a UTM format, then get equivalent GPS."""
#        from backseat.src import NavigationTools as NavTools
        origin_lat = rospy.get_param("goal_lat")
        origin_lon = rospy.get_param("goal_lon")
        origin_wyp_pose = Pose(gps_lat=origin_lat, gps_lon=origin_lon)
        origin_wyp_pose.convert_to_utm()
        
        x_pose_utm = origin_wyp_pose.utm_x + pose.position.x
        y_pose_utm = origin_wyp_pose.utm_y + pose.position.y

        geoPose = Pose(utm_x=x_pose_utm, utm_y=y_pose_utm, utm_zone=origin_wyp_pose.utm_zone)
        geoPose.convert_to_gps()
        
        geo_pose_stamp = GeoPoseStamped()
        geo_pose_stamp.header = header if header else pose.header
        geo_pose_stamp.pose.position.latitude = geoPose.gps_lat
        geo_pose_stamp.pose.position.longitude = geoPose.gps_lon
        geo_pose_stamp.pose.position.altitude = 0
        geo_pose_stamp.pose.orientation = pose.orientation
        return geo_pose_stamp

if __name__ == '__main__':
    try:
        mission_planner = NavGoalToWaypoint()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
