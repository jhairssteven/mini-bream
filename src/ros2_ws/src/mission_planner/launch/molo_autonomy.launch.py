"""Full perception-planning-control stack: Nav2 planning + ILOS/H0 follower."""

import sys
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import SetParameter

_LAUNCH_DIR = Path(__file__).resolve().parent
if str(_LAUNCH_DIR) not in sys.path:
    sys.path.insert(0, str(_LAUNCH_DIR))

from molo_tf_launch import molo_process_env, tf_bridge_action  # noqa: E402


def _path_follower_stack_runner() -> Path:
    candidates = [
        Path("/workspace/ros2_ws/src/molo_wpt_follower/path_follower/stack_runner.py"),
        _LAUNCH_DIR.parent.parent / "molo_wpt_follower/path_follower/stack_runner.py",
    ]
    for path in candidates:
        if path.is_file():
            return path
    raise RuntimeError(
        "path_follower/stack_runner.py not found. "
        "Start follower manually: "
        "python3 src/ros2_ws/src/molo_wpt_follower/path_follower/stack_runner.py "
        "--controller ilos --platform sim"
    )


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

    nav2_launch = IncludeLaunchDescription(
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
    actions.append(nav2_launch)
    return actions


def _follower_node(context, *args, **kwargs):
    if LaunchConfiguration("enable_follower").perform(context).lower() not in (
        "1",
        "true",
        "yes",
    ):
        return []

    controller = LaunchConfiguration("controller").perform(context)
    platform = LaunchConfiguration("platform").perform(context)
    follower_platform = "sim" if platform == "blueboat_sim" else platform
    use_sim = LaunchConfiguration("use_sim_time").perform(context).lower() in (
        "1",
        "true",
        "yes",
    )
    return [
        ExecuteProcess(
            cmd=[
                "python3",
                str(_path_follower_stack_runner()),
                "--controller",
                controller,
                "--platform",
                follower_platform,
            ],
            name="path_follower",
            output="screen",
            additional_env=molo_process_env(use_sim),
        )
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("platform", default_value="blueboat_sim"),
            DeclareLaunchArgument(
                "controller",
                default_value="ilos",
                description="Path follower controller: ilos | h0 | mpc",
            ),
            DeclareLaunchArgument("enable_follower", default_value="true"),
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            OpaqueFunction(function=_setup),
            OpaqueFunction(function=_follower_node),
        ]
    )
