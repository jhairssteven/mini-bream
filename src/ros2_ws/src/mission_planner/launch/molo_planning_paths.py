"""Launch helpers for BlueBoat autonomy (TF bridge + path resolution)."""

from __future__ import annotations

import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory


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


def tf_bridge_script_path() -> Path:
    candidates = [
        Path(
            "/workspace/ros2_ws/src/mission_planner/mission_planner/moloplanner/molo_map/tf_bridge/tf_bridge_node.py"
        ),
        Path(__file__).resolve().parents[1]
        / "mission_planner"
        / "moloplanner"
        / "molo_map"
        / "tf_bridge"
        / "tf_bridge_node.py",
    ]
    for candidate in candidates:
        if candidate.is_file():
            return candidate
    raise RuntimeError("tf_bridge_node.py not found in mission_planner/molo_map/tf_bridge")
