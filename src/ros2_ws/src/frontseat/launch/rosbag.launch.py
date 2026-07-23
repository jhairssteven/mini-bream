"""Record selected mini-bream topics to field_tests/rosbags.

Usage:
  ros2 launch frontseat rosbag.launch.py
  ros2 launch frontseat rosbag.launch.py bag_storage:=mcap bag_suffix:=zed_lidar
  ros2 launch frontseat rosbag.launch.py record_bag:=false   # dry run / args only

On Jetson, source overlays before recording (or use this launch file, which does it):
  source /opt/zed_ws/install/setup.bash
"""

from __future__ import annotations

import shlex
from datetime import datetime
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration

TOPICS_TO_RECORD = [
    # Motors / teleop
    '/pwm/left_thrust_cmd',
    '/pwm/right_thrust_cmd',
    '/wamv/thrusters/left/thrust',
    '/wamv/thrusters/right/thrust',
    '/radio/thrusters/left_cmd',
    '/radio/thrusters/right_cmd',
    '/radio/on_ctrl',
    # GPS / RTK (apt ublox driver + frontseat remaps)
    '/fix/rover',
    '/fix/base',
    '/fix/center/avg',
    '/wamv/sensors/gps/gps/fix',
    '/wamv/sensors/gps/centered_gps/fix',
    '/wamv/sensors/imu/imu/data',
    '/wamv/sensors/imu/imu/data/estimated',
    '/baseline/heading',
    '/heading/deg',
    '/navstatus',
    '/navsvin',
    '/navrelposned',
    '/navheading',
    # LiDAR (Jetson perception)
    '/rslidar_points',
    '/rslidar_packets',
    '/rslidar_imu_data',
    # ZED 2i (Jetson perception — namespace/camera_name both "zed", zed_wrapper v5.x)
    '/zed/zed/rgb/color/rect/image',
    '/zed/zed/rgb/color/rect/camera_info',
    '/zed/zed/point_cloud/cloud_registered',
    # Frames
    '/tf',
    '/tf_static',
]


def _bag_root() -> str:
    """Persistent bag directory (mounted in Docker at /workspace/field_tests/rosbags)."""
    container_root = '/workspace/field_tests/rosbags'
    if os.path.isdir(container_root):
        return container_root

    launch_dir = os.path.dirname(os.path.abspath(__file__))
    # .../ros2_ws/src/frontseat/launch -> .../src/field_tests/rosbags
    src_dir = os.path.realpath(os.path.join(launch_dir, '..', '..', '..', '..'))
    return os.path.join(src_dir, 'field_tests', 'rosbags')


def _bag_output_path(suffix: str) -> str:
    timestamp = datetime.now().strftime('%Y-%m-%d_%H-%M-%S')
    day = datetime.now().strftime('%Y-%m-%d')
    stem = f'rosbag_{timestamp}'
    if suffix:
        stem = f'{stem}_{suffix}'
    bag_dir = os.path.join(_bag_root(), day, stem)
    # ros2 bag record creates the output folder; only ensure parent exists.
    os.makedirs(os.path.dirname(bag_dir), exist_ok=True)
    return bag_dir


def _record_shell_command(storage: str, bag_name: str) -> str:
    """Build a shell command that sources ROS overlays then runs ros2 bag record."""
    topic_args = ' '.join(shlex.quote(t) for t in TOPICS_TO_RECORD)
    parts = [
        'set -e',
        'source /opt/ros/humble/setup.bash',
        '[[ -f /opt/zed_ws/install/setup.bash ]] && source /opt/zed_ws/install/setup.bash',
        '[[ -f /workspace/ros2_ws/install/setup.bash ]] && source /workspace/ros2_ws/install/setup.bash',
        (
            f'exec ros2 bag record -s {shlex.quote(storage)} '
            f'-o {shlex.quote(bag_name)} {topic_args}'
        ),
    ]
    return ' && '.join(parts)


def _launch_setup(context, *args, **kwargs):
    record_bag = LaunchConfiguration('record_bag').perform(context).strip().lower()
    if record_bag not in ('true', '1', 'yes'):
        return [LogInfo(msg='[rosbag] record_bag=false — not recording')]

    suffix = LaunchConfiguration('bag_suffix').perform(context).strip()
    if suffix:
        suffix = suffix.replace(' ', '_').replace('/', '_')

    storage = LaunchConfiguration('bag_storage').perform(context).strip().lower()
    if storage not in ('sqlite3', 'mcap'):
        raise RuntimeError(f"bag_storage must be 'sqlite3' or 'mcap', got '{storage}'")

    bag_name = _bag_output_path(suffix)
    record_cmd = ['bash', '-lc', _record_shell_command(storage, bag_name)]

    return [
        LogInfo(msg=[f'[rosbag] Recording to: {bag_name} (storage: {storage})']),
        ExecuteProcess(cmd=record_cmd, output='screen'),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'record_bag',
            default_value='true',
            description='Set false to launch without starting ros2 bag record.',
        ),
        DeclareLaunchArgument(
            'bag_suffix',
            default_value='',
            description='Optional suffix after timestamp in bag folder name (e.g. zed_lidar).',
        ),
        DeclareLaunchArgument(
            'bag_storage',
            default_value='mcap',
            description="rosbag2 storage backend: 'mcap' (default) or 'sqlite3'.",
        ),
        OpaqueFunction(function=_launch_setup),
    ])
