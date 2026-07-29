#!/usr/bin/env python3
"""Bridge RViz goals to Nav2 ComputePathToPose and publish /plan for external followers."""

from __future__ import annotations

import math
import sys
from typing import Optional

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import ComputePathToPose
from nav_msgs.msg import Path
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy


class GoalPathPlanner(Node):
  def __init__(self) -> None:
    super().__init__("goal_path_planner")
    self.declare_parameter("planner_id", "GridBased")
    self.declare_parameter("goal_topic", "/goal_pose")
    self.declare_parameter("plan_topic", "/plan")
    self.declare_parameter("replan_hz", 1.0)

    self._planner_id = str(self.get_parameter("planner_id").value)
    self._goal_topic = str(self.get_parameter("goal_topic").value)
    self._plan_topic = str(self.get_parameter("plan_topic").value)
    self._replan_hz = float(self.get_parameter("replan_hz").value)

    qos = QoSProfile(
      depth=1,
      reliability=ReliabilityPolicy.RELIABLE,
      durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )
    self._plan_pub = self.create_publisher(Path, self._plan_topic, qos)
    self.create_subscription(PoseStamped, self._goal_topic, self._goal_cb, 10)

    self._client = ActionClient(self, ComputePathToPose, "compute_path_to_pose")
    self._goal: Optional[PoseStamped] = None
    self._busy = False

    if self._replan_hz > 0.0:
      self.create_timer(1.0 / self._replan_hz, self._replan_tick)

    self.get_logger().info(
      f"Goal path planner: {self._goal_topic} -> {self._plan_topic} "
      f"(planner_id={self._planner_id}, replan={self._replan_hz} Hz)"
    )

  def _goal_cb(self, msg: PoseStamped) -> None:
    self._goal = msg
    self.get_logger().info(
      f"New goal ({msg.header.frame_id}): "
      f"({msg.pose.position.x:.2f}, {msg.pose.position.y:.2f})"
    )
    self._request_path()

  def _replan_tick(self) -> None:
    if self._goal is not None:
      self._request_path()

  def _request_path(self) -> None:
    if self._goal is None or self._busy:
      return
    if not self._client.wait_for_server(timeout_sec=2.0):
      self.get_logger().warning(
        "compute_path_to_pose action server not ready", throttle_duration_sec=5.0
      )
      return

    goal = ComputePathToPose.Goal()
    goal.goal = self._goal
    goal.planner_id = self._planner_id
    goal.use_start = False

    self._busy = True
    send_future = self._client.send_goal_async(goal)
    send_future.add_done_callback(self._goal_response_cb)

  def _goal_response_cb(self, future) -> None:
    goal_handle = future.result()
    if not goal_handle.accepted:
      self.get_logger().warning("ComputePathToPose goal rejected")
      self._busy = False
      return
    result_future = goal_handle.get_result_async()
    result_future.add_done_callback(self._result_cb)

  def _result_cb(self, future) -> None:
    self._busy = False
    result = future.result().result
    status = future.result().status
    if status != GoalStatus.STATUS_SUCCEEDED or result.path.poses is None:
      self.get_logger().warning("Path planning failed")
      return
    if len(result.path.poses) < 2:
      self.get_logger().warning("Planner returned a path with fewer than 2 poses")
      return
    self._plan_pub.publish(result.path)
    start = result.path.poses[0].pose.position
    end = result.path.poses[-1].pose.position
    dist = math.hypot(end.x - start.x, end.y - start.y)
    self.get_logger().info(
      f"Published plan with {len(result.path.poses)} poses "
      f"(length ~{dist:.1f} m, frame={result.path.header.frame_id})"
    )


def main(argv: list[str] | None = None) -> None:
  rclpy.init(args=argv)
  node = GoalPathPlanner()
  try:
    rclpy.spin(node)
  except KeyboardInterrupt:
    pass
  finally:
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
  main(sys.argv)
