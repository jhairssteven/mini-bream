#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import NavSatFix, Imu, Image
from geographic_msgs.msg import GeoPose
from std_msgs.msg import String

from cv_bridge import CvBridge
from mission_planner.moloplanner import Moloplanner
import math, threading, time
import utm

from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
from mission_planner.nav_goal_to_waypoint import ActionServerClient
from geographic_msgs.msg import GeoPose, GeoPoint
from backseat_msgs.action import DoMission
import tf2_ros
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
import struct
import numpy as np

class MoloAstarPlannerNode(Node):

    def __init__(self):
        super().__init__('molo_astar_planner_node')

        self.bridge = CvBridge()
        
        # --- Declare ROS parameters
        self.declare_parameter("config_path", '')
        self.declare_parameter("overrides", None)
        self.declare_parameter("goal_lat", 40.448417)
        self.declare_parameter("goal_lon", -86.867750)

        # --- Read parameters
        config_path = self.get_parameter("config_path").get_parameter_value().string_value
        overrides = self.get_parameter("overrides").get_parameter_value().string_array_value
        self.goal_lat = self.get_parameter("goal_lat").value
        self.goal_lon = self.get_parameter("goal_lon").value

        if config_path == '':
            raise RuntimeError(
                "Parameter 'config_path' is required but was not provided. "
                "Please pass it via launch file."
            )
        elif not overrides:
            raise RuntimeError(
                "Parameter 'config_path' is required but was not provided. "
                "Please pass it via launch file."
            )
        
        self.get_logger().info('Loading Moloplanner...')
        self.molo_planner = Moloplanner(
            config_path=config_path,
            overrides=overrides
        )

        # shared state (updated by subs)
        self.next_waypoint_gps = None
        self.camera_origin_gps = None
        self.boat_heading_deg = None
        self.input_img_array = None
        self.img_id = None
        self.current_gps = None
        self.local_path_gps_goal = None

        self._action_client = ActionServerClient(self)

        # For critical real-time commands, reliable but no stored history
        reliable_volatile_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,  # Rety until success
            durability=QoSDurabilityPolicy.VOLATILE,    # Do not store old messages
            depth=1
        )
        
        # Visualization
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.vis_pub_current = self.create_publisher(PointCloud2, '/moloplanner/vis/cloud_current', reliable_volatile_qos)
        self.vis_pub_history = self.create_publisher(PointCloud2, '/moloplanner/vis/cloud_history', reliable_volatile_qos)

        self.lock = threading.Lock()

        
        self.create_subscription(GeoPose, '/next_waypoint/geo_pose', self.next_waypoint_geopose_cb, 10)
        self.create_subscription(NavSatFix, '/camera_origin/gps', self.camera_origin_cb, qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=1
        ))
        
        self.create_subscription(Imu, '/heading/imu/data', self.imu_cb, 10)
        self.create_subscription(Image, '/camera/input_image', self.image_cb, 10)
        self.create_subscription(String, '/camera/image_id', self.imgid_cb, 10)
        self.create_subscription(NavSatFix, '/wamv/sensors/gps/centered_gps/fix', self.current_gps_cb, qos_profile=QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=1
        ))

        # Run local planner in its own thread
        self.worker_thread = threading.Thread(target=self.planner_loop, daemon=True)
        self.worker_thread.start()

        self.get_logger().info("Moloplanner node ready.")

    # ---------- Callbacks ----------
    def next_waypoint_geopose_cb(self, msg):
        with self.lock:
            self.next_waypoint_gps = (msg.position.latitude, msg.position.longitude)

    def camera_origin_cb(self, msg):
        with self.lock:
            self.camera_origin_gps = (msg.latitude, msg.longitude)

    def imgid_cb(self, msg):
        with self.lock:
            self.img_id = msg.data

    def current_gps_cb(self, msg):
        with self.lock:
            self.current_gps = (msg.latitude, msg.longitude)

    def image_cb(self, msg):
        try:
            img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            with self.lock:
                self.input_img_array = img
        except Exception as e:
            self.get_logger().error(f"Image conversion error: {e}")

    def imu_cb(self, msg):
        def quaternion_to_yaw(x, y, z, w):
            t3 = +2.0 * (w * z + x * y)
            t4 = +1.0 - 2.0 * (y * y + z * z)
            return math.atan2(t3, t4)
        
        yaw_rad = quaternion_to_yaw(
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
            msg.orientation.w
        )

        with self.lock:
            self.boat_heading_deg = math.degrees(yaw_rad)

    def planner_loop(self):
        while rclpy.ok():
            # Copy the latest state safely
            with self.lock:
                ready = all([
                    self.next_waypoint_gps,
                    self.camera_origin_gps,
                    self.boat_heading_deg is not None,
                    self.input_img_array is not None,
                    self.img_id is not None,
                    self.current_gps is not None
                ])

                if not ready:
                    self.get_logger().info('Not ready to plan. Missing variables from state', 
                                           throttle_duration_sec=3.0)
                    continue

                next_wp = self.next_waypoint_gps
                cam_origin = self.camera_origin_gps
                heading = self.boat_heading_deg
                img = self.input_img_array.copy()
                img_id = self.img_id
                current_gps = self.current_gps
                local_path_gps_goal = self.local_path_gps_goal
                

            # Check if we need to plan
            should_plan = False
            if local_path_gps_goal is None:
                should_plan = True
                self.get_logger().info("First plan execution.")
            else:
                # Calculate distance to end of path
                # Using UTM for distance in meters
                try:
                    curr_utm = utm.from_latlon(current_gps[0], current_gps[1])
                    end_utm = utm.from_latlon(local_path_gps_goal[0], local_path_gps_goal[1])
                    dist = math.sqrt((curr_utm[0] - end_utm[0])**2 + (curr_utm[1] - end_utm[1])**2)
                    
                    # Threshold for replanning (e.g., 5 meters)
                    REPLAN_THRESHOLD = 3.0 
                    if dist < REPLAN_THRESHOLD:
                        should_plan = True; self.local_path_gps_goal = None
                        self.get_logger().info(f"Close to path end by {dist:.2f}m. Stoping vehicle and replanning...")
                        self.get_logger().info(f"   ")
                        
                except Exception as e:
                    self.get_logger().error(f"Error calculating distance: {e}")
            
            if not should_plan:
                time.sleep(0.5)
                continue

            try:
                start = time.perf_counter()
                path, pcl_xz_coordinates, pcd = self.molo_planner.get_gps_local_astart_path(
                    img_id,
                    img,
                    next_wp,
                    cam_origin,
                    heading
                )
                
                elapsed = time.perf_counter() - start

                d_min = 0.2
                subsampled_path = self.molo_planner.resample_min_distance(path=path, d_min=d_min)

                self.get_logger().info(f"Got new path in: {elapsed:.6f} s, Subsampled path length: {len(subsampled_path)}")

                self.publish_path_clouds(pcd, pcl_xz_coordinates, cam_origin, heading)
                
                if len(subsampled_path) > 0:
                    self.execute_path(subsampled_path)
                    with self.lock:
                        self.local_path_gps_goal = subsampled_path[-1]
                else:
                    self.get_logger().warn("Generated path is empty.")

            except Exception as e:
                self.get_logger().error(f"Planner error: {e}")
                import traceback
                self.get_logger().error(traceback.format_exc())

    
    def publish_path_clouds(self, pcd, pcl_xz_coordinates, cam_origin, heading):
        """
        Publish PointClouds for local path planning visualization
        """
        # --- Visualization ---
        # 1. Broadcast TF for the camera frame
        # Calculate translation: UTM(cam) - UTM(goal)
        cam_utm = utm.from_latlon(cam_origin[0], cam_origin[1])
        goal_utm = utm.from_latlon(self.goal_lat, self.goal_lon)
        
        # Translation
        t_x = cam_utm[0] - goal_utm[0]
        t_y = cam_utm[1] - goal_utm[1]
        t_z = 0.0 # Assuming water level

        # Rotation
        # boat_heading_deg is 0=North, 90=East (Compass)
        # We need to convert to standard math angle (0=East, 90=North) for TF?
        # Actually, let's look at moloplanner logic:
        # camera_frame_orientation_deg = boat_heading_deg - 90
        # This suggests boat_heading_deg is standard compass (0=N, 90=E).
        # And camera x-axis is rotated -90 deg relative to boat heading.
        # Let's just use the same logic as moloplanner to be consistent.
        
        # However, TF expects rotation from Parent (World) to Child (Camera).
        # World is usually ENU (East-North-Up).
        # If boat_heading is compass (0=N, 90=E), then in ENU:
        # N -> +Y, E -> +X.
        # So 0 deg compass = +Y axis.
        # Standard yaw is 0 at +X (East).
        # So yaw = 90 - compass_heading.
        
        # Boat heading in ENU:
        yaw_boat_enu = math.radians(heading)
        
        # Camera frame is rotated -90 deg (CW) relative to boat?
        # "camera's x axis is 90deg CW rotated w.r.t the boat's heading"
        # So yaw_cam = yaw_boat - 90 deg
        yaw_cam_enu = yaw_boat_enu - math.radians(90)

        # Create quaternion
        import tf_transformations
        q = tf_transformations.quaternion_from_euler(0, math.radians(180), yaw_cam_enu)

        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'world'
        t.child_frame_id = 'moloplanner_camera_frame'
        t.transform.translation.x = t_x
        t.transform.translation.y = t_y
        t.transform.translation.z = t_z
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        self.tf_broadcaster.sendTransform(t)

        # 2. Publish PointClouds
        if pcd is not None:
            # Current (Bright)
            cloud_msg_current = self.create_colored_cloud(
                pcd, pcl_xz_coordinates, 'current', 'moloplanner_camera_frame')
            self.vis_pub_current.publish(cloud_msg_current)

            # History (Dull)
            cloud_msg_history = self.create_colored_cloud(
                pcd, pcl_xz_coordinates, 'history', 'moloplanner_camera_frame')
            self.vis_pub_history.publish(cloud_msg_history)
        
        
        
    def create_colored_cloud(self, pcd, path_coords, scheme, frame_id):
        """
        Create a PointCloud2 message from Open3D pcd and path coordinates.
        scheme: 'current' or 'history'
        """
        points = np.asarray(pcd.points)
        
        # Define colors
        if scheme == 'history':
            # White/Natural for PCD, Red for Path
            color_pcd = [255, 255, 255] 
            color_path = [255, 0, 0]
        else:
            # Gray for PCD, Dark Red for Path
            color_pcd = [100, 100, 100]
            color_path = [139, 0, 0]

        # Pack PCD points
        # We need to pack RGB into a float. 
        # RGB is usually packed as (r << 16) | (g << 8) | b
        
        def pack_rgb(r, g, b):
            rgb = (int(r) << 16) | (int(g) << 8) | int(b)
            return struct.unpack('f', struct.pack('I', rgb))[0]

        pcd_rgb = pack_rgb(*color_pcd)
        path_rgb = pack_rgb(*color_path)

        data = []
        
        # Add PCD points
        # PCL Z -> ROS Y, PCL Y -> ROS Z, PCL -X -> ROS X
        for p in points:
            data.append([-p[0], p[2], p[1], pcd_rgb])

        # Add Path points (convert XZ to XYZ)
        # path_coords are (x, z) in PCL. Y is 0.
        # PCL point: (x, 0, z)
        # ROS X = -x
        # ROS Y = z
        # ROS Z = 0
        
        if path_coords is not None:
            for p in path_coords:
                data.append([-p[0], p[1], 0.0, path_rgb])

        # Create PointCloud2
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = frame_id

        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='rgb', offset=12, datatype=PointField.FLOAT32, count=1),
        ]

        import sensor_msgs_py.point_cloud2 as pc2
        # If sensor_msgs_py is not available, we might need manual packing.
        # But it should be in standard ROS2 python.
        
        # Flatten data
        # pc2.create_cloud expects list of lists or similar
        return pc2.create_cloud(header, fields, data)

    def stop_vehicle(self, current_gps):
        # Send a mission with just the current position to stop/hold
        self.execute_path([current_gps])

    def execute_path(self, gps_local_path : list):
        # Always include current position of vehicle as first waypoint
        gps_local_path.insert(0, self.current_gps)
        geopose_path = [
            GeoPose(
                position=GeoPoint(latitude=lat, longitude=lon, altitude=0.0)
            )
            for lat, lon in gps_local_path
        ]

        goal_msg = DoMission.Goal()
        goal_msg.mission = geopose_path
        
        self._action_client.send_goal(goal_msg)


def main(args=None):
    rclpy.init(args=args)
    node = MoloAstarPlannerNode()
    rclpy.spin(node)
    rclpy.shutdown()