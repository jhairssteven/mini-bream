"""Record selected mini-bream topics to field_tests/rosbags.

Usage:
  ros2 launch frontseat rosbag.launch.py
  ros2 launch frontseat rosbag.launch.py bag_storage:=mcap bag_suffix:=zed_lidar
  ros2 launch frontseat rosbag.launch.py record_perception:=false bag_suffix:=rtk_only
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

CORE_TOPICS = [
    '/pwm/left_thrust_cmd',
    '/pwm/right_thrust_cmd',
    # GPS / RTK
    '/fix/rover',
    '/fix/base',
    '/wamv/sensors/gps/gps/fix',
    '/wamv/sensors/imu/imu/data',
    '/heading/deg',
    '/navheading',
    '/navrelposned',
    '/navstatus',
    # TF
    '/tf',
    '/tf_static',
    # Odometry (autonomy)
    '/molo_boat/estimated_odometry',
]

LIDAR_TOPICS = [
    '/rslidar_points',
    '/rslidar_imu_data',
    '/rslidar_points/filtered',
]

ZED_TOPICS = [
    '/zed/zed/rgb/color/rect/image',
    '/zed/zed/rgb/color/rect/camera_info',
    '/zed/zed/point_cloud/cloud_registered',
]

PERCEPTION_TOPICS = LIDAR_TOPICS + ZED_TOPICS


def _topics_to_record(record_perception: bool) -> list[str]:
    topics = list(CORE_TOPICS)
    if record_perception:
        topics.extend(PERCEPTION_TOPICS)
    return topics


def _parse_bool(value: str, name: str) -> bool:
    normalized = value.strip().lower()
    if normalized in ('true', '1', 'yes'):
        return True
    if normalized in ('false', '0', 'no'):
        return False
    raise RuntimeError(f"{name} must be 'true' or 'false', got '{value}'")


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


def _record_shell_command(storage: str, bag_name: str, topics: list[str]) -> str:
    """Build a shell command that sources ROS overlays then runs ros2 bag record."""
    topic_args = ' '.join(shlex.quote(t) for t in topics)
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

    record_perception = _parse_bool(
        LaunchConfiguration('record_perception').perform(context),
        'record_perception',
    )
    topics = _topics_to_record(record_perception)

    bag_name = _bag_output_path(suffix)
    record_cmd = ['bash', '-lc', _record_shell_command(storage, bag_name, topics)]

    perception_note = 'with perception' if record_perception else 'without perception (LiDAR + ZED)'
    return [
        LogInfo(msg=[
            f'[rosbag] Recording to: {bag_name} (storage: {storage}, {perception_note}, '
            f'{len(topics)} topics)',
        ]),
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
        DeclareLaunchArgument(
            'record_perception',
            default_value='true',
            description='Record LiDAR and ZED topics. Set false for smaller GPS/RTK-only bags.',
        ),
        OpaqueFunction(function=_launch_setup),
    ])
