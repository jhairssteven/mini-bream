#!/usr/bin/env python3

"""
Every 2 seconds, publish a PoseStamped with random x,y in the range [-10, 10].
Used for testing real-time goal setting performance of PathPlanner's action server.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import random


class RandomGoalPublisher(Node):
    def __init__(self):
        super().__init__('random_goal_publisher')
        self.publisher_ = self.create_publisher(PoseStamped, '/move_base_simple/goal', 10)
        self.timer = self.create_timer(2.0, self.timer_callback)  # every 2 seconds
        self.get_logger().info("RandomGoalPublisher started. Publishing goals every 2s.")

    def timer_callback(self):
        goal = PoseStamped()
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.header.frame_id = "map"

        goal.pose.position.x = random.uniform(-10.0, 10.0)
        goal.pose.position.y = random.uniform(-10.0, 10.0)
        goal.pose.position.z = 0.0

        goal.pose.orientation.x = 0.0
        goal.pose.orientation.y = 0.0
        goal.pose.orientation.z = 0.0
        goal.pose.orientation.w = 1.0

        self.get_logger().info(f"Publishing new goal: x={goal.pose.position.x:.2f}, y={goal.pose.position.y:.2f}")
        self.publisher_.publish(goal)


def main(args=None):
    rclpy.init(args=args)
    node = RandomGoalPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
