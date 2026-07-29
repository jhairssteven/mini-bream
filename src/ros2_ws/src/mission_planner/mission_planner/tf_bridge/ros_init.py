"""Shared rclpy init helpers for mission_planner nodes launched via ExecuteProcess."""

from __future__ import annotations

import os
import sys

import rclpy


def init_rclpy(argv: list[str] | None = None, *, use_sim_time: bool | None = None) -> None:
    """Initialize rclpy, honoring MOLO_USE_SIM_TIME when use_sim_time is not explicit."""
    args = list(argv if argv is not None else sys.argv)
    if use_sim_time is None:
        use_sim_time = os.environ.get("MOLO_USE_SIM_TIME", "").lower() in ("1", "true", "yes")
    if use_sim_time and not any("use_sim_time" in arg for arg in args):
        args.extend(["--ros-args", "-p", "use_sim_time:=true"])
    rclpy.init(args=args)


def execute_process_env(use_sim_time: bool) -> dict[str, str]:
    """Environment for ExecuteProcess nodes so they honor simulation time."""
    if not use_sim_time:
        return {}
    return {"MOLO_USE_SIM_TIME": "1"}
