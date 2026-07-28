"""Nav2 planning stack only (no path follower)."""

import sys
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import SetParameter

_LAUNCH_DIR = Path(__file__).resolve().parent
if str(_LAUNCH_DIR) not in sys.path:
    sys.path.insert(0, str(_LAUNCH_DIR))

from molo_tf_launch import tf_bridge_action  # noqa: E402


def _setup(context, *args, **kwargs):
    platform = LaunchConfiguration("platform").perform(context)
    use_sim = LaunchConfiguration("use_sim_time").perform(context).lower() in (
        "1",
        "true",
        "yes",
    )

    actions = []
    if use_sim:
        actions.append(SetParameter(name="use_sim_time", value=True))

    tf_action = tf_bridge_action(platform, use_sim_time=use_sim)
    if tf_action is not None:
        actions.append(tf_action)

    actions.append(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(
                    Path(get_package_share_directory("blueboat_nav2"))
                    / "launch"
                    / "nav2_planning.launch.py"
                )
            ),
            launch_arguments={
                "platform": platform,
                "use_sim_time": LaunchConfiguration("use_sim_time"),
            }.items(),
        )
    )
    return actions


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("platform", default_value="blueboat_sim"),
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            OpaqueFunction(function=_setup),
        ]
    )
