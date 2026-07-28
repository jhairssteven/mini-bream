"""Full perception-planning-control stack for BlueBoat sim or field profiles."""

import sys
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter

_LAUNCH_DIR = Path(__file__).resolve().parent
if str(_LAUNCH_DIR) not in sys.path:
    sys.path.insert(0, str(_LAUNCH_DIR))

from molo_planning_paths import platform_config_dir, script_path  # noqa: E402
from molo_tf_launch import molo_process_env, tf_bridge_action  # noqa: E402


def _path_follower_stack_runner() -> Path:
    """Resolve stack_runner.py in source tree (install layout has no molo_wpt_follower)."""
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


def _planning_nodes(context, *args, **kwargs):
    platform = LaunchConfiguration("platform").perform(context)
    use_rgbd = LaunchConfiguration("enable_rgbd").perform(context).lower() in (
        "1",
        "true",
        "yes",
    )
    use_sim_time = LaunchConfiguration("use_sim_time").perform(context)

    cfg_dir = platform_config_dir(platform)
    costmap_cfg = cfg_dir / "pointcloud_to_costmap.json"
    planner_cfg = cfg_dir / "local_planner.json"
    rgbd_cfg = cfg_dir / "rgbd_filter.json"

    use_sim = use_sim_time.lower() in ("1", "true", "yes")
    proc_env = molo_process_env(use_sim)

    actions = []
    if use_sim:
        actions.append(SetParameter(name="use_sim_time", value=True))

    tf_action = tf_bridge_action(platform, use_sim_time=use_sim)
    if tf_action is not None:
        actions.append(tf_action)

    if use_rgbd:
        actions.append(
            ExecuteProcess(
                cmd=["python3", str(script_path("rgbd_filter")), str(rgbd_cfg)],
                name="rgbd_water_filter",
                output="screen",
                additional_env=proc_env,
            )
        )

    actions.extend(
        [
            ExecuteProcess(
                cmd=["python3", str(script_path("pointcloud_to_costmap")), str(costmap_cfg)],
                name="pointcloud_to_costmap",
                output="screen",
                additional_env=proc_env,
            ),
            ExecuteProcess(
                cmd=["python3", str(script_path("local_planner")), str(planner_cfg)],
                name="molo_local_planner",
                output="screen",
                additional_env=proc_env,
            ),
        ]
    )
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
    stack_runner = _path_follower_stack_runner()
    return [
        ExecuteProcess(
            cmd=[
                "python3",
                str(stack_runner),
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
            DeclareLaunchArgument("enable_rgbd", default_value="false"),
            DeclareLaunchArgument("enable_follower", default_value="true"),
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            OpaqueFunction(function=_planning_nodes),
            OpaqueFunction(function=_follower_node),
        ]
    )
