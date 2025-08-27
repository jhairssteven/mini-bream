#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import NavSatFix, Imu
from geographic_msgs.msg import GeoPose
from nav_msgs.msg import Path
from backseat_msgs.action import DoMission
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
from typing import Callable
import utm


class ActionServerClient:
    def __init__(self, node: Node):
        self.node = node
        self._client = ActionClient(node, DoMission, 'path_planner_action')
        self.goal_ctr = 0
        self._goal_handle = None
        self.result = DoMission.Result()

    def log(self, msg: str, enable_logs=False):
        if enable_logs:
            self.node.get_logger().info(f"[GOAL ID: {self.goal_ctr}] {msg}")

    def send_goal(self, goal_msg: DoMission.Goal, after_done_callback: Callable[[DoMission.Result], None]):
        # Cancel previous goal if any
        self.cancel_goal()

        self.goal_ctr += 1
        goal_msg.id = self.goal_ctr

        #self.log("Waiting for action server...")
        self._client.wait_for_server()

        self.log("Sending new mission...", enable_logs=True)
        send_future = self._client.send_goal_async(goal_msg, feedback_callback=self.feedback_cb)
        send_future.add_done_callback(self.goal_response_cb)
        self.after_done_callback = after_done_callback

    def cancel_goal(self):
        if self._goal_handle is not None:
            self.log("Canceling current goal...")
            cancel_future = self._goal_handle.cancel_goal_async()
            cancel_future.add_done_callback(self.cancel_done_cb)
        else:
            self.log("No active goal to cancel.")

    def cancel_done_cb(self, future):
        cancel_response = future.result()
        if len(cancel_response.goals_canceling) > 0:
            self.log("Goal successfully canceled.")
        else:
            self.log("Failed to cancel goal or goal already completed.")

    def goal_response_cb(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.log("Goal was rejected.")
            return

        #self.log("Goal accepted.")
        self._goal_handle = goal_handle
        goal_handle.get_result_async().add_done_callback(self.done_cb)

    def done_cb(self, future):
        self.result = future.result().result
        self.log(f"Mission complete: {self.result.mission_complete}")
        self._goal_handle = None  # Clear handle once result is received
        self.after_done_callback(self.result)

    def feedback_cb(self, feedback_msg):
        # feedback = feedback_msg.feedback
        # self.log(f"Feedback: {feedback.xt_error}")
        pass


class Pose:
    def __init__(self, gps_lat=-1, gps_lon=-1, utm_x=-1, utm_y=-1, utm_zone=None, head=-1):
        self.gps_lat = gps_lat
        self.gps_lon = gps_lon
        self.head = head
        self.utm_x = utm_x
        self.utm_y = utm_y
        self.utm_zone = utm_zone

        if utm_x != -1 and utm_y != -1 and utm_zone is not None:
            self.convert_to_gps()
        elif gps_lat != -1 and gps_lon != -1:
            self.convert_to_utm()
        else:
            raise ValueError("Must provide either GPS or UTM coordinates.")

    def convert_to_gps(self):
        self.gps_lat, self.gps_lon = utm.to_latlon(self.utm_x, self.utm_y, int(self.utm_zone[:-1]), self.utm_zone[-1])

    def convert_to_utm(self):
        self.utm_x, self.utm_y, z_num, z_let = utm.from_latlon(self.gps_lat, self.gps_lon)
        self.utm_zone = f"{z_num}{z_let}"


class NavGoalToWaypoint:
    def __init__(self, node: Node, action_client: ActionServerClient):
        linc_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=1
        )
        
        # Subscribers
        self.goal_sub = node.create_subscription(PoseStamped, '/goal_pose', self.goal_callback, 10)
        self.gps_sub = node.create_subscription(NavSatFix, '/wamv/sensors/gps/gps/fix', self.gps_callback, linc_qos)
        self.imu_sub = node.create_subscription(Imu, '/wamv/sensors/imu/imu/data', self.imu_callback, linc_qos)
        self.send_mission_sub = node.create_subscription(Path, '/rviz_path', self.buildMissionFromPoseArray, linc_qos)

        self.node = node
        self._action_client = action_client
        self.current_marker = None
        self.curr_lat = self.curr_long = None
        origin_lat = self.node.get_parameter("goal_lat").value
        origin_lon = self.node.get_parameter("goal_lon").value
        self.origin_pose = Pose(gps_lat=origin_lat, gps_lon=origin_lon)
        self.origin_pose.convert_to_utm()

    def imu_callback(self, msg: Imu):
        self.current_orientation = msg.orientation

    def gps_callback(self, msg: NavSatFix):
        self.curr_lat = msg.latitude
        self.curr_long = msg.longitude

    def buildGeoPose(self, latitude, longitude, orientation):
        gp = GeoPose()
        gp.position.latitude = latitude
        gp.position.longitude = longitude
        gp.position.altitude = 0.0
        gp.orientation = orientation
        return gp

    def __local_to_geo(self, pose):
        self.node.get_logger().info(f"{self.origin_pose.utm_x} + {pose.position.x} /n: {self.origin_pose.utm_y} + {pose.position.y}")
        x_utm = self.origin_pose.utm_x + pose.position.x
        y_utm = self.origin_pose.utm_y + pose.position.y
        
        geo = Pose(utm_x=x_utm, utm_y=y_utm, utm_zone=self.origin_pose.utm_zone)
        geo.convert_to_gps()
        geo_pose = self.buildGeoPose(geo.gps_lat, geo.gps_lon, pose.orientation)
        return geo_pose

    def getCurrentGeoPosition(self):
        if self.curr_lat is None or self.curr_long is None:
            self.node.get_logger().warn("Current lat, long not yet received.")
            return None

        return self.buildGeoPose(self.curr_lat, self.curr_long, self.current_orientation)
    
    def goal_callback(self, msg: PoseStamped):
        current_geo = self.getCurrentGeoPosition()
        if not current_geo: return
        goal_geo = self.__local_to_geo(msg.pose)

        self.node.get_logger().info(f"New goal lat/lon: {goal_geo.position.latitude}, {goal_geo.position.longitude}")
        self.sendMissionToClient([current_geo, goal_geo])

    def sendMissionToClient(self, mission):
        """ mission: A list of geo waypoints (output of __local_to_geo()) """
        goal_msg = DoMission.Goal()
        goal_msg.mission = mission
        self._action_client.send_goal(goal_msg)

    def buildMissionFromPoseArray(self, msg: Path):
        """ Build a mission from a given list of Poses, appending (and starting from) the current position."""
        current_geo = self.getCurrentGeoPosition()
        if not current_geo: return

        mission = []
        mission.append(current_geo)

        for poseStamped in msg.poses:
            """ x = float(round(poseStamped.pose.position.x, 0))
            y = float(round(poseStamped.pose.position.y, 0))
            z = float(round(poseStamped.pose.position.z, 0))

            # Update the pose with the rounded position
            poseStamped.pose.position = Point(x=x, y=y, z=z) """

            mission.append(self.__local_to_geo(poseStamped.pose))
        
        self.sendMissionToClient(mission)

class MainNode(Node):
    def __init__(self):
        super().__init__('nav_goal_to_waypoint')
        self.declare_parameter("goal_lat", 40.448417)
        self.declare_parameter("goal_lon", -86.867750)
        self.declare_parameter("load_mission_from_file", False)
        self.declare_parameter("mission", '')
        self._action_client = ActionServerClient(self)


        if self.get_parameter('load_mission_from_file').get_parameter_value().bool_value:
            filename = self.get_parameter("mission").get_parameter_value().string_value
            if filename:
                self.get_logger().info(f'Loading mission from file {filename}')

                goal = DoMission.Goal()
                goal.filename = filename
                self._action_client.send_goal(goal)

        self._mission_interface = NavGoalToWaypoint(self, self._action_client)


def main(args=None):
    rclpy.init(args=args)
    node = MainNode()

    def on_shutdown():
        node.get_logger().info("Shutting down, canceling goal (if any).")
        node._action_client.cancel_goal()

    rclpy.get_default_context().on_shutdown(on_shutdown)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
