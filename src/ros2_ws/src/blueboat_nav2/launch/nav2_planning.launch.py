"""Launch Nav2 planning stack (costmap + Smac planner) for BlueBoat."""

from __future__ import annotations

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter
from nav2_common.launch import RewrittenYaml


def _launch_setup(context, *args, **kwargs):
    platform = LaunchConfiguration("platform").perform(context)
    use_sim = LaunchConfiguration("use_sim_time").perform(context).lower() in (
        "1",
        "true",
        "yes",
    )

    pkg_share = Path(get_package_share_directory("blueboat_nav2"))
    params_file = (
        pkg_share / "config" / "nav2_blueboat_sim.yaml"
        if platform == "blueboat_sim"
        else pkg_share / "config" / "nav2_blueboat_real.yaml"
    )

    use_sim_str = "true" if use_sim else "false"

    configured_params = RewrittenYaml(
        source_file=str(params_file),
        root_key="",
        param_rewrites={"use_sim_time": use_sim_str},
        convert_types=True,
    )

    remappings = [("/tf", "tf"), ("/tf_static", "tf_static")]

    actions = []
    if use_sim:
        actions.append(SetParameter(name="use_sim_time", value=True))

    actions.extend(
        [
            Node(
                package="pointcloud_to_laserscan",
                executable="pointcloud_to_laserscan_node",
                name="pointcloud_to_laserscan",
                output="screen",
                parameters=[configured_params],
                remappings=[
                    ("cloud_in", "/rslidar_points"),
                    ("scan", "/scan"),
                ],
            ),
            Node(
                package="nav2_planner",
                executable="planner_server",
                name="planner_server",
                output="screen",
                parameters=[configured_params],
                remappings=remappings,
            ),
            Node(
                package="nav2_lifecycle_manager",
                executable="lifecycle_manager",
                name="lifecycle_manager_planning",
                output="screen",
                parameters=[configured_params],
            ),
            Node(
                package="blueboat_nav2",
                executable="goal_path_planner.py",
                name="goal_path_planner",
                output="screen",
                parameters=[configured_params],
            ),
        ]
    )
    return actions


def generate_launch_description():
    return LaunchDescription(
        [
            SetEnvironmentVariable("RCUTILS_LOGGING_BUFFERED_STREAM", "1"),
            DeclareLaunchArgument("platform", default_value="blueboat_sim"),
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            OpaqueFunction(function=_launch_setup),
        ]
    )
