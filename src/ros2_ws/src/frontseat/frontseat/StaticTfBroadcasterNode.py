#!/usr/bin/env python3
"""Publish static transforms from a YAML extrinsics file."""

import os

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from tf_transformations import quaternion_from_euler
import tf2_ros


class StaticTfBroadcaster(Node):
    def __init__(self, node_name='static_tf_broadcaster'):
        super().__init__(node_name)

        default_config = os.path.join(
            get_package_share_directory('frontseat'),
            'config', 'tf', 'blueboat_extrinsics.yaml',
        )
        self.declare_parameter('config_path', default_config)
        config_path = self.get_parameter('config_path').get_parameter_value().string_value

        transforms = self._load_transforms(config_path)
        self._broadcaster = tf2_ros.StaticTransformBroadcaster(self)
        self._broadcaster.sendTransform(transforms)
        self.get_logger().info(
            f'Published {len(transforms)} static transform(s) from {config_path}'
        )

    def _load_transforms(self, config_path: str) -> list:
        with open(config_path, 'r', encoding='utf-8') as config_file:
            data = yaml.safe_load(config_file)

        transforms = []
        for entry in data.get('transforms', []):
            tf_msg = TransformStamped()
            tf_msg.header.stamp = self.get_clock().now().to_msg()
            tf_msg.header.frame_id = entry['parent']
            tf_msg.child_frame_id = entry['child']

            translation = entry['translation']
            tf_msg.transform.translation.x = float(translation['x'])
            tf_msg.transform.translation.y = float(translation['y'])
            tf_msg.transform.translation.z = float(translation['z'])

            roll, pitch, yaw = entry.get('rpy', [0.0, 0.0, 0.0])
            quaternion = quaternion_from_euler(float(roll), float(pitch), float(yaw))
            tf_msg.transform.rotation.x = quaternion[0]
            tf_msg.transform.rotation.y = quaternion[1]
            tf_msg.transform.rotation.z = quaternion[2]
            tf_msg.transform.rotation.w = quaternion[3]
            transforms.append(tf_msg)

        return transforms


def main(args=None):
    rclpy.init(args=args)
    node = StaticTfBroadcaster()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
