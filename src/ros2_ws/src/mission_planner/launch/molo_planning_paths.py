"""Launch helpers for the molo_map local planning pipeline."""

from __future__ import annotations

import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory


def molo_map_dir() -> Path:
    """Return the molo_map folder (rgbd, costmap, planner scripts)."""
    env = os.environ.get("MOLO_MAP_DIR")
    if env:
        return Path(env)

    candidates = [
        Path("/workspace/ros2_ws/src/mission_planner/mission_planner/moloplanner/molo_map"),
        Path(__file__).resolve().parents[1]
        / "mission_planner"
        / "moloplanner"
        / "molo_map",
    ]
    for candidate in candidates:
        if candidate.is_dir():
            return candidate

    pkg_share = Path(get_package_share_directory("mission_planner"))
    installed = pkg_share / "molo_map"
    if installed.is_dir():
        return installed

    raise RuntimeError(
        "molo_map scripts not found. Set MOLO_MAP_DIR or run from the source workspace."
    )


def molo_wpt_dir() -> Path:
    """Return molo_wpt_follower source tree (path_follower stack_runner)."""
    env = os.environ.get("MOLO_WPT_DIR")
    if env:
        return Path(env)
    candidates = [
        Path("/workspace/ros2_ws/src/molo_wpt_follower"),
        Path(__file__).resolve().parents[2] / "molo_wpt_follower",
    ]
    for candidate in candidates:
        if (candidate / "path_follower" / "stack_runner.py").is_file():
            return candidate
    raise RuntimeError(
        "molo_wpt_follower not found. Set MOLO_WPT_DIR or run from the source workspace."
    )


def platform_config_dir(platform: str) -> Path:
    return Path(get_package_share_directory("mission_planner")) / "config" / platform


def script_path(name: str) -> Path:
    folder = {
        "rgbd_filter": "rgbd_water_filter",
        "pointcloud_to_costmap": "ros_bev_costmap",
        "local_planner": "molo_plan_v2",
        "tf_bridge": "tf_bridge",
    }
    script_names = {
        "rgbd_filter": "rgbd_filter.py",
        "pointcloud_to_costmap": "pointcloud_to_costmap.py",
        "local_planner": "local_planner_node.py",
        "tf_bridge": "tf_bridge_node.py",
    }
    return molo_map_dir() / folder[name] / script_names[name]


def tf_bridge_script_path() -> Path:
    return script_path("tf_bridge")
