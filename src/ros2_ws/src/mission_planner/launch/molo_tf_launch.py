"""Shared TF bridge launch helpers for molo planning/autonomy."""

from __future__ import annotations

from pathlib import Path

from launch.actions import ExecuteProcess

from molo_planning_paths import platform_config_dir, tf_bridge_script_path


def molo_process_env(use_sim_time: bool) -> dict[str, str]:
    if use_sim_time:
        return {"MOLO_USE_SIM_TIME": "1"}
    return {}


def tf_bridge_action(platform: str, *, use_sim_time: bool = False) -> ExecuteProcess | None:
    """Return ExecuteProcess for the platform TF bridge, or None if config is missing."""
    cfg = platform_config_dir(platform) / "tf_bridge.json"
    if not cfg.is_file():
        return None
    return ExecuteProcess(
        cmd=["python3", str(tf_bridge_script_path()), str(cfg)],
        name="molo_tf_bridge",
        output="screen",
        additional_env=molo_process_env(use_sim_time),
    )
