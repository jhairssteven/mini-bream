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

class MoloAstarPlannerNode(Node):

    def __init__(self):
        super().__init__('molo_astar_planner_node')

        self.bridge = CvBridge()
        
        # --- Declare ROS parameters
        self.declare_parameter("config_path", '')
        self.declare_parameter("overrides", None)

        # --- Read parameters
        config_path = self.get_parameter("config_path").get_parameter_value().string_value
        overrides = self.get_parameter("overrides").get_parameter_value().string_array_value

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
                    self.img_id is not None
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

            self.get_logger().info("Getting local path...")

            try:
                path = self.molo_planner.get_gps_local_astart_path(
                    img_id,
                    img,
                    next_wp,
                    cam_origin,
                    heading
                )
                
                self.get_logger().info(
                    f"Planner finished: Path with {len(path)} points."
                )
                
                d_min = 0.2
                subsampled_path = self.resample_min_distance(path=path, d_min=d_min)

                self.get_logger().info(f'New path subsampled at {d_min} (m), Length: {len(subsampled_path)}')
                self.execute_path(subsampled_path)
                time.sleep(5)

            except Exception as e:
                self.get_logger().error(f"Planner error: {e}")
                import traceback
                self.get_logger().error(traceback.format_exc())

    def execute_path(self, gps_local_path : list):
        from mission_planner.nav_goal_to_waypoint import ActionServerClient
        from geographic_msgs.msg import GeoPose, GeoPoint
        from backseat_msgs.action import DoMission
        geopose_path = [
            GeoPose(
                position=GeoPoint(latitude=lat, longitude=lon, altitude=0.0)
            )
            for lat, lon in gps_local_path
        ]

        self._action_client = ActionServerClient(self)
        goal_msg = DoMission.Goal()
        goal_msg.mission = geopose_path
        
        self._action_client.send_goal(goal_msg)

    def resample_min_distance(self, path, d_min):
            """
            Resample GPS waypoints enforcing a minimum spacing d_min (meters).
            Args:
            path: List [(lat, lon), (lat, lon), ...]
            d_min: (m) spacing distance in meters to subsample the path.

            Returns subsampled path with 'd_min' minimum separation for each waypoint [(lat_resampled, lon_resampled), ...].
            """
            
            # Convert path to UTM
            utm_points = [utm.from_latlon(lat, lon) for (lat, lon) in path]
            xy = [(p[0], p[1]) for p in utm_points]
            
            new_xy = [xy[0]]
            last_x, last_y = xy[0]
            
            for i in range(1, len(xy)):
                x, y = xy[i]
                
                # distance from last kept point
                dx = x - last_x
                dy = y - last_y
                dist = math.hypot(dx, dy)
                
                if dist < d_min:
                    continue  # not far enough
                
                # we may need several points if distance is large
                n_points = int(dist // d_min)
                
                for k in range(1, n_points + 1):
                    alpha = (k * d_min) / dist
                    new_x = last_x + alpha * dx
                    new_y = last_y + alpha * dy
                    new_xy.append((new_x, new_y))
                
                # update last reference point
                last_x, last_y = new_xy[-1]
            
            # Convert back to lat/lon
            zone_number = utm_points[0][2]
            zone_letter = utm_points[0][3]
            
            subsampled_path = []
            for x, y in new_xy:
                lat, lon = utm.to_latlon(x, y, zone_number, zone_letter)
                subsampled_path.append((lat, lon))
            
            return subsampled_path

def main(args=None):
    rclpy.init(args=args)
    node = MoloAstarPlannerNode()
    rclpy.spin(node)
    rclpy.shutdown()