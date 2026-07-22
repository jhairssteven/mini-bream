"""Publish BlueBoat sensor / thruster static transforms from YAML extrinsics."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
  config_path = os.path.join(
    get_package_share_directory('frontseat'),
    'config', 'tf', 'blueboat_extrinsics.yaml',
  )

  return LaunchDescription([
    Node(
      package='frontseat',
      executable='static_tf_broadcaster',
      name='static_tf_broadcaster',
      output='screen',
      parameters=[{'config_path': config_path}],
    ),
  ])
