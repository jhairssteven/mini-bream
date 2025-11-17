#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import NavSatFix, Imu, Image
from std_msgs.msg import String

from cv_bridge import CvBridge
from mission_planner.moloplanner import Moloplanner
import math, threading, time


class MoloAstarPlannerNode(Node):

    def __init__(self):
        super().__init__('molo_astar_planner_node')

        self.bridge = CvBridge()
        
        # --- Declare ROS parameters
        self.declare_parameter("config_path", None)
        self.declare_parameter("overrides", None)

        # --- Read parameters
        config_path = self.get_parameter("config_path").get_parameter_value().string_value
        overrides = self.get_parameter("overrides").get_parameter_value().string_array_value

        if not config_path:
            raise RuntimeError(
                "Parameter 'config_path' is required but was not provided. "
                "Please pass it via launch file."
            )
        
        # --- Create Moloplanner with ROS params
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

        # === Subscriptions ===
        self.create_subscription(NavSatFix, '/next_waypoint/gps', self.next_waypoint_cb, 10)
        self.create_subscription(NavSatFix, '/camera_origin/gps', self.camera_origin_cb, 10)
        self.create_subscription(Imu, '/heading/imu/data', self.imu_cb, 10)
        self.create_subscription(Image, '/camera/input_image', self.image_cb, 10)
        self.create_subscription(String, '/camera/image_id', self.imgid_cb, 10)

        # Start background planning thread
        self.worker_thread = threading.Thread(target=self.planner_loop, daemon=True)
        self.worker_thread.start()

        self.get_logger().info("Moloplanner node started.")

    # ---------- Callbacks ----------
    def next_waypoint_cb(self, msg):
        with self.lock:
            self.next_waypoint_gps = (msg.latitude, msg.longitude)

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
        yaw_rad = self.quaternion_to_yaw(
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
            msg.orientation.w
        )
        with self.lock:
            self.boat_heading_deg = math.degrees(yaw_rad)

    # ---------- Utility: quaternion to yaw ----------
    def quaternion_to_yaw(self, x, y, z, w):
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        return math.atan2(t3, t4)

    # ---------- Continuous planner loop ----------
    def planner_loop(self):
        while rclpy.ok():
            time.sleep(0.05)  # No-busy-wait loop (20 Hz polling)

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
                    continue

                next_wp = self.next_waypoint_gps
                cam_origin = self.camera_origin_gps
                heading = self.boat_heading_deg
                img = self.input_img_array.copy()
                img_id = self.img_id

            # ---------- Run the planner ----------
            self.get_logger().info("Running planner cycle...")

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

            except Exception as e:
                self.get_logger().error(f"Planner error: {e}")
                import traceback
                self.get_logger().error(traceback.format_exc())

def main(args=None):
    rclpy.init(args=args)
    node = MoloAstarPlannerNode()
    rclpy.spin(node)
    rclpy.shutdown()