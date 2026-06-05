"""Launch VRX WAM-V in open water (no shore/dock collisions) for MPC and path-following tests."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
import os

import vrx_gz.launch
from vrx_gz.model import Model


def launch(context, *args, **kwargs):
    config_file = LaunchConfiguration("config_file").perform(context)
    world_name = LaunchConfiguration("world").perform(context)
    sim_mode = LaunchConfiguration("sim_mode").perform(context)
    bridge_competition_topics = (
        LaunchConfiguration("bridge_competition_topics").perform(context).lower() == "true"
    )
    robot = LaunchConfiguration("robot").perform(context)
    headless = LaunchConfiguration("headless").perform(context).lower() == "true"
    robot_urdf = LaunchConfiguration("urdf").perform(context)
    gz_paused = LaunchConfiguration("paused").perform(context).lower() == "true"
    competition_mode = LaunchConfiguration("competition_mode").perform(context).lower() == "true"
    extra_gz_args = LaunchConfiguration("extra_gz_args").perform(context)
    spawn_x = float(LaunchConfiguration("spawn_x").perform(context))
    spawn_y = float(LaunchConfiguration("spawn_y").perform(context))
    spawn_z = float(LaunchConfiguration("spawn_z").perform(context))
    spawn_yaw = float(LaunchConfiguration("spawn_yaw").perform(context))

    launch_processes = []
    models = []
    if config_file and config_file != "":
        with open(config_file, "r") as stream:
            models = Model.FromConfig(stream)
    else:
        m = Model("wamv", "wam-v", [spawn_x, spawn_y, spawn_z, 0, 0, spawn_yaw])
        if robot_urdf and robot_urdf != "":
            m.set_urdf(robot_urdf)
        models.append(m)

    world_name, _ext = os.path.splitext(world_name)
    launch_processes.extend(
        vrx_gz.launch.simulation(world_name, headless, gz_paused, extra_gz_args)
    )
    world_name_base = os.path.basename(world_name)
    launch_processes.extend(vrx_gz.launch.spawn(sim_mode, world_name_base, models, robot))

    if (sim_mode == "bridge" or sim_mode == "full") and bridge_competition_topics:
        launch_processes.extend(
            vrx_gz.launch.competition_bridges(world_name_base, competition_mode)
        )

    return launch_processes


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "world",
                default_value="open_water_harner",
                description="Open-water world (no shore/dock obstacles)",
            ),
            DeclareLaunchArgument("sim_mode", default_value="full"),
            DeclareLaunchArgument("bridge_competition_topics", default_value="True"),
            DeclareLaunchArgument("config_file", default_value=""),
            DeclareLaunchArgument("robot", default_value=""),
            DeclareLaunchArgument("headless", default_value="False"),
            DeclareLaunchArgument("urdf", default_value=""),
            DeclareLaunchArgument("paused", default_value="False"),
            DeclareLaunchArgument("competition_mode", default_value="False"),
            DeclareLaunchArgument("extra_gz_args", default_value=""),
            DeclareLaunchArgument("spawn_x", default_value="0.0"),
            DeclareLaunchArgument("spawn_y", default_value="0.0"),
            DeclareLaunchArgument("spawn_z", default_value="0.2"),
            DeclareLaunchArgument("spawn_yaw", default_value="0.0"),
            OpaqueFunction(function=launch),
        ]
    )
