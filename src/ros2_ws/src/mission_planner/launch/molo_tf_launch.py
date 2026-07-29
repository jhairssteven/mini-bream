"""Shared TF bridge launch helpers for molo planning/autonomy."""

from __future__ import annotations

import subprocess
from pathlib import Path

from launch.actions import ExecuteProcess

from molo_planning_paths import platform_config_dir, tf_bridge_script_path


def _is_jetson_field() -> bool:
    try:
        ips = subprocess.check_output(["hostname", "-I"], text=True, timeout=2).split()
    except (OSError, subprocess.SubprocessError):
        return False
    return "192.168.0.102" in ips


def autonomy_launch_env(use_sim_time: bool) -> dict[str, str]:
    env: dict[str, str] = {"RMW_IMPLEMENTATION": "rmw_cyclonedds_cpp"}
    if use_sim_time:
        env["MOLO_USE_SIM_TIME"] = "1"
    if _is_jetson_field() and Path("/etc/cyclonedds.jetson.xml").is_file():
        env["CYCLONEDDS_URI"] = "file:///etc/cyclonedds.jetson.xml"
    return env


def molo_process_env(use_sim_time: bool) -> dict[str, str]:
    return autonomy_launch_env(use_sim_time)


def tf_bridge_action(
    platform: str,
    *,
    use_sim_time: bool = False,
    additional_env: dict[str, str] | None = None,
) -> ExecuteProcess | None:
    """Return ExecuteProcess for the platform TF bridge, or None if config is missing."""
    cfg = platform_config_dir(platform) / "tf_bridge.json"
    if not cfg.is_file():
        return None
    env = autonomy_launch_env(use_sim_time)
    if additional_env:
        env.update(additional_env)
    return ExecuteProcess(
        cmd=["python3", str(tf_bridge_script_path()), str(cfg)],
        name="molo_tf_bridge",
        output="screen",
        additional_env=env,
    )
