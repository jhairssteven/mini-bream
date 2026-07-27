"""Shared VRX simulation launch logic for BlueBoat worlds."""

import os

from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from blueboat_sim.vrx import launch as vrx_launch
from blueboat_sim.vrx.model import Model


def _default_blueboat_urdf():
    return os.path.join(
        get_package_share_directory('blueboat_sim'),
        'urdf',
        'blueboat_sim.urdf.xacro',
    )


def _hal_relays():
    return [
        Node(
            package='topic_tools',
            executable='relay',
            name='lidar_points_relay',
            parameters=[{
                'input_topic': '/blueboat/sensors/lidars/lidar_blueboat_sensor/points',
                'output_topic': '/rslidar_points',
                'lazy': True,
            }],
        ),
        Node(
            package='topic_tools',
            executable='relay',
            name='zed_rgb_relay',
            parameters=[{
                'input_topic': '/blueboat/sensors/cameras/front_camera_sensor/image_raw',
                'output_topic': '/zed/zed/rgb/color/rect/image',
                'lazy': True,
            }],
        ),
    ]


def launch(context, *args, **kwargs):
    config_file = LaunchConfiguration('config_file').perform(context)
    world_name = LaunchConfiguration('world').perform(context)
    sim_mode = LaunchConfiguration('sim_mode').perform(context)
    bridge_competition_topics = (
        LaunchConfiguration('bridge_competition_topics').perform(context).lower() == 'true'
    )
    robot = LaunchConfiguration('robot').perform(context)
    headless = LaunchConfiguration('headless').perform(context).lower() == 'true'
    robot_urdf = LaunchConfiguration('urdf').perform(context)
    gz_paused = LaunchConfiguration('paused').perform(context).lower() == 'true'
    competition_mode = LaunchConfiguration('competition_mode').perform(context).lower() == 'true'
    extra_gz_args = LaunchConfiguration('extra_gz_args').perform(context)
    spawn_x = float(LaunchConfiguration('spawn_x').perform(context))
    spawn_y = float(LaunchConfiguration('spawn_y').perform(context))
    spawn_z = float(LaunchConfiguration('spawn_z').perform(context))
    spawn_yaw = float(LaunchConfiguration('spawn_yaw').perform(context))

    if not robot_urdf:
        robot_urdf = _default_blueboat_urdf()

    launch_processes = []
    if config_file:
        with open(config_file, 'r') as stream:
            models = Model.FromConfig(stream)
    else:
        model = Model('blueboat', 'blueboat', [spawn_x, spawn_y, spawn_z, 0, 0, spawn_yaw])
        model.set_urdf(robot_urdf)
        models = [model]

    world_name, _ext = os.path.splitext(world_name)
    launch_processes.extend(
        vrx_launch.simulation(world_name, headless, gz_paused, extra_gz_args))
    world_name_base = os.path.basename(world_name)
    launch_processes.extend(vrx_launch.spawn(sim_mode, world_name_base, models, robot))

    if sim_mode in ('bridge', 'full') and bridge_competition_topics:
        launch_processes.extend(
            vrx_launch.competition_bridges(world_name_base, competition_mode))

    if sim_mode in ('bridge', 'full'):
        launch_processes.extend(_hal_relays())

    return launch_processes


def common_launch_arguments(default_world: str, world_description: str):
    return [
        DeclareLaunchArgument('world', default_value=default_world, description=world_description),
        DeclareLaunchArgument('sim_mode', default_value='full'),
        DeclareLaunchArgument('bridge_competition_topics', default_value='True'),
        DeclareLaunchArgument('config_file', default_value=''),
        DeclareLaunchArgument('robot', default_value=''),
        DeclareLaunchArgument('headless', default_value='False'),
        DeclareLaunchArgument(
            'urdf',
            default_value='',
            description='BlueBoat xacro (default: blueboat_sim.urdf.xacro)',
        ),
        DeclareLaunchArgument('paused', default_value='False'),
        DeclareLaunchArgument('competition_mode', default_value='False'),
        DeclareLaunchArgument('extra_gz_args', default_value=''),
        DeclareLaunchArgument('spawn_x', default_value='0.0'),
        DeclareLaunchArgument('spawn_y', default_value='0.0'),
        DeclareLaunchArgument('spawn_z', default_value='0.15'),
        DeclareLaunchArgument('spawn_yaw', default_value='0.0'),
        OpaqueFunction(function=launch),
    ]
