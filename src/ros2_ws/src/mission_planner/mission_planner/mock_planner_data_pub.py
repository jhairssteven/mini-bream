#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import NavSatFix, Imu, Image
from std_msgs.msg import String

import math
import numpy as np
import cv2
from cv_bridge import CvBridge
import random
import time

from geographic_msgs.msg import GeoPose

class MockPlannerDataPublisher(Node):
    def __init__(self):
        super().__init__('mock_planner_data_publisher')

        self.bridge = CvBridge()

        # Subscribers
        self.create_subscription(GeoPose, '/goal_geopose', self.goal_geopose_callback, 10)
        # Publishers
        self.pub_wp = self.create_publisher(NavSatFix, '/next_waypoint/gps', 10)
        #self.pub_cam = self.create_publisher(NavSatFix, '/camera_origin/gps', qos_profile = QoSProfile(
        #    reliability=QoSReliabilityPolicy.BEST_EFFORT,
        #    durability=QoSDurabilityPolicy.VOLATILE,
        #    depth=1
        #))
        #self.pub_imu = self.create_publisher(Imu, '/heading/imu/data', 10)
        self.pub_img = self.create_publisher(Image, '/camera/input_image', 10)
        self.pub_imgid = self.create_publisher(String, '/camera/image_id', 10)

        self.timer = self.create_timer(0.5, self.publish_mock_data)  # 2 Hz

        #self.yaw = 0.0
        self.get_logger().info("Mock data publisher started.")
    
    def goal_geopose_callback(self, msg: GeoPose):
        wp = NavSatFix()
        wp.latitude = msg.position.latitude
        wp.longitude = msg.position.longitude
        self.pub_wp.publish(wp)


    def publish_mock_data(self):

        # --- 2. Mock camera origin GPS ---
        #cam = NavSatFix()
        #cam.latitude = 40.443100 + random.uniform(-0.0001, 0.0001)
        #cam.longitude = -86.763300 + random.uniform(-0.0001, 0.0001)
        #self.pub_cam.publish(cam)

        # --- 3. Mock IMU yaw ---
        #imu = Imu()
        #self.yaw += 10*180/np.pi  # slowly rotate 10deg
        #imu.orientation.w = math.cos(self.yaw * 0.5)
        #imu.orientation.z = math.sin(self.yaw * 0.5)
        #self.pub_imu.publish(imu)

        # --- 4. Mock image ---
        #the_img = cv2.imread("/workspace/codebase/mini-bream/src/ros2_ws/src/mission_planner/mission_planner/moloplanner/assets/input_imgs/frames_output/frame_2.png")
        #img_msg = self.bridge.cv2_to_imgmsg(the_img, encoding='bgr8')
        #self.pub_img.publish(img_msg)

        # --- 5. Mock image ID ---
        img_id = String()
        img_id.data = f"frame_{int(time.time())}"
        self.pub_imgid.publish(img_id)


def main(args=None):
    rclpy.init(args=args)
    node = MockPlannerDataPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
