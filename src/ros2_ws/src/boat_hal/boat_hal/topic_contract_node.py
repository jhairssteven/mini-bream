#!/usr/bin/env python3
"""Publish HAL topic contract as ROS parameters for launch-time discovery."""

from __future__ import annotations

import argparse

import rclpy
from rclpy.node import Node

from boat_hal.factory import load_hal_config


class TopicContractNode(Node):
  def __init__(self, config_path: str | None) -> None:
    super().__init__("boat_hal_topic_contract")
    hal = load_hal_config(config_path=config_path)
    for key, value in hal.topics.as_dict().items():
      self.declare_parameter(key, value)
    self.declare_parameter("mode", hal.mode)
    self.declare_parameter("use_sim_time", hal.use_sim_time)
    self.get_logger().info(f"HAL mode={hal.mode} topics loaded")


def main() -> None:
  parser = argparse.ArgumentParser()
  parser.add_argument("--config", default=None, help="Optional HAL YAML path")
  args, _ = parser.parse_known_args()
  rclpy.init()
  node = TopicContractNode(args.config)
  try:
    rclpy.spin(node)
  except KeyboardInterrupt:
    pass
  finally:
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
  main()
