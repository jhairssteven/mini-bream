#!/usr/bin/env python3
"""ILOS + PID heading control with differential thrust (no MPC, no boat model)."""

from __future__ import annotations

import argparse
import json
import math
import os
import sys
from pathlib import Path
from typing import List, Optional, Tuple

import numpy as np
import rclpy
import yaml
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from std_msgs.msg import Float64, Float32
from tf_transformations import euler_from_quaternion

MPC_DIR = Path(__file__).resolve().parent.parent / "mpc"
MOLO_DIR = MPC_DIR.parent
if str(MPC_DIR) not in sys.path:
    sys.path.insert(0, str(MPC_DIR))
if str(MOLO_DIR) not in sys.path:
    sys.path.append(str(MOLO_DIR))

from algorithms import (  # noqa: E402
    DubinsPlanner,
    HeadingPID,
    ILOSFollower,
    PathPoint,
    Pose2D,
    angdiff,
    speed_from_heading_error,
)
from mpc import build_mission_path  # noqa: E402
from path_activation import PathActivationManager  # noqa: E402
from path_reference import PathSample, closest_index  # noqa: E402
from viz import MpcVisualizer  # noqa: E402


def load_config(path: str) -> dict:
    with open(path, "r", encoding="utf-8") as f:
        return yaml.safe_load(f)


class IlosFollowerNode(Node):
    def __init__(self, config: dict):
        super().__init__("molo_ilos_follower")
        self.cfg = config
        self.frame_id = config.get("frame_id", "world")
        self.sim_enable = bool(config.get("sim_enable", True))
        topics = config.get("topics", {})

        dubins_cfg = config.get("dubins", {})
        self._dubins = DubinsPlanner(
            float(dubins_cfg.get("turning_radius_m", 5.5)),
            float(dubins_cfg.get("step_size_m", 0.5)),
        )
        ilos_cfg = config.get("ilos", {})
        self._ilos_kwargs = {
            "lookahead_min": float(ilos_cfg.get("lookahead_min_m", 1.0)),
            "lookahead_max": float(ilos_cfg.get("lookahead_max_m", 3.0)),
            "conv_rate": float(ilos_cfg.get("conv_rate", 8.0)),
            "gamma": float(ilos_cfg.get("gamma", 0.0)),
            "replan_dist": float(ilos_cfg.get("replan_dist_m", 26.0)),
            "replan_lookahead": float(ilos_cfg.get("replan_lookahead_m", 8.0)),
            "handover_offset": int(ilos_cfg.get("handover_offset", 1)),
            "no_of_laps": int(ilos_cfg.get("no_of_laps", 1)),
        }
        pid_cfg = config.get("pid", {})
        self._heading_pid = HeadingPID(
            float(pid_cfg.get("kp", 1.2)),
            float(pid_cfg.get("ki", 0.0)),
            float(pid_cfg.get("kd", 0.35)),
            float(pid_cfg.get("integral_limit", 0.15)),
        )
        guidance = config.get("guidance", {})
        self._max_r = float(guidance.get("max_yaw_rate_rad_s", 0.45))
        self._kappa_speed_scale = float(guidance.get("kappa_speed_scale", 2.5))

        thrust_cfg = config.get("thrust", {})
        self._yaw_mix = float(thrust_cfg.get("yaw_mix_gain", 0.85))
        self._surge_gain = float(thrust_cfg.get("surge_gain", 1.0))
        self._min_surge = float(thrust_cfg.get("min_surge_norm", 0.12))

        speed_cfg = config.get("speed", {})
        self._angle_fast = float(speed_cfg.get("angle_threshold_fast_rad", 0.12))
        self._angle_slow = float(speed_cfg.get("angle_threshold_slow_rad", 0.45))
        self._min_speed_scale = float(speed_cfg.get("min_speed_scale", 0.35))
        self._cruise_ref = float(config.get("path", {}).get("cruise_speed_mps", 0.25))

        self._sim_scale = 1000.0 if self.sim_enable else 1.0
        self._path_closed = True
        exp = config.get("experiment", {})
        self._log_path = exp.get("log_csv") if exp else None
        self._log_file = None
        self._t0 = None
        if self._log_path:
            Path(self._log_path).parent.mkdir(parents=True, exist_ok=True)
            self._log_file = open(self._log_path, "w", encoding="utf-8")
            self._log_file.write(
                "t,x,y,psi,u,v,r,xte,u_cmd,r_ref,v_ref,kappa,approach,thrust_l_N,thrust_r_N\n"
            )

        qos_s = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=1,
        )
        qos_c = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=5,
        )

        odom_topic = topics.get("odometry", "/odom")
        self.create_subscription(Odometry, odom_topic, self._odom_cb, qos_s)

        thrust_t = Float64 if self.sim_enable else Float32
        self._left_pub = self.create_publisher(
            thrust_t, topics.get("left_thrust", "/wamv/thrusters/left/thrust"), qos_c
        )
        self._right_pub = self.create_publisher(
            thrust_t, topics.get("right_thrust", "/wamv/thrusters/right/thrust"), qos_c
        )

        traj_cfg = config.get("trajectory", {})
        self.viz = MpcVisualizer(self, self.frame_id, topics)
        self._traj_step = float(traj_cfg.get("min_distance_m", 0.25))

        self._x = self._y = self._psi = None
        self._u = self._v = self._r = 0.0
        self._path: List[PathSample] = []
        self._ilos: Optional[ILOSFollower] = None
        self._path_ready = False
        self._path_idx = 0

        self._path_activation = PathActivationManager(
            self,
            config,
            self._on_path_ready,
            build_mission_path,
        )
        self._pending_origin = self._path_activation.pending

        self._rate = float(config.get("control_rate_hz", 10.0))
        self.create_timer(1.0 / self._rate, self._control_loop)
        self.get_logger().info(
            f"molo_ilos_follower ready (ILOS+PID, pose from {odom_topic})"
        )

    def _odom_cb(self, msg: Odometry) -> None:
        self._x = float(msg.pose.pose.position.x)
        self._y = float(msg.pose.pose.position.y)
        q = msg.pose.pose.orientation
        _, _, yaw = euler_from_quaternion((q.x, q.y, q.z, q.w))
        self._psi = float(yaw)
        self._u = float(msg.twist.twist.linear.x)
        self._v = float(msg.twist.twist.linear.y)
        self._r = float(msg.twist.twist.angular.z)
        if self._path_activation.pending:
            self._path_activation.notify_pose(self._x, self._y, self._psi)

    def _on_path_ready(
        self,
        samples: List[PathSample],
        closed: bool,
        origin_xy: Tuple[float, float],
        idx0: int,
        viz_samples: List[PathSample] | None = None,
    ) -> None:
        soft_update = self._path_ready and self._ilos is not None
        display_samples = viz_samples if viz_samples is not None else samples
        self._path = samples
        self._path_closed = closed
        self._path_ready = len(self._path) >= 2
        self._pending_origin = False
        if self._log_path:
            origin_file = Path(self._log_path).parent / "origin.json"
            with open(origin_file, "w", encoding="utf-8") as f:
                json.dump({"x": origin_xy[0], "y": origin_xy[1]}, f)
        ilos_kw = dict(self._ilos_kwargs)
        if self._path_closed:
            ilos_kw["no_of_laps"] = max(ilos_kw.get("no_of_laps", 1), 50)
        points = [PathPoint(s.x, s.y, s.psi) for s in self._path]
        self._ilos = ILOSFollower(points, dubins_planner=self._dubins, **ilos_kw)
        if soft_update and self._x is not None and self._y is not None:
            idx0 = closest_index(self._path, self._x, self._y)
        self._ilos.work_index = idx0
        self._ilos.orig_index = idx0
        if not soft_update:
            self._heading_pid.reset()
        self._path_idx = idx0
        stamp = self._path_activation._last_path_stamp
        self.viz.publish_ref(display_samples, stamp=stamp)
        action = "updated" if soft_update else "loaded"
        self.get_logger().info(
            f"ILOS path {action} ({len(self._path)} samples, closed={self._path_closed})"
        )

    def _speed_scale(self, head_err: float, kappa: float) -> float:
        head_scale = speed_from_heading_error(
            head_err,
            1.0,
            self._angle_fast,
            self._angle_slow,
        )
        head_scale = max(self._min_speed_scale, head_scale)
        turn_scale = 1.0 / (1.0 + self._kappa_speed_scale * abs(kappa))
        return head_scale * turn_scale

    def _publish_thrust(self, left: float, right: float) -> None:
        tl = float(np.clip(left, -1.0, 1.0) * self._sim_scale)
        tr = float(np.clip(right, -1.0, 1.0) * self._sim_scale)
        t = Float64 if self.sim_enable else Float32
        self._left_pub.publish(t(data=tl))
        self._right_pub.publish(t(data=tr))

    def _control_loop(self) -> None:
        if (
            self._x is None
            or self._psi is None
            or not self._path_ready
            or self._ilos is None
        ):
            return

        if self._t0 is None:
            self._t0 = self.get_clock().now().nanoseconds * 1e-9

        path_cfg = self.cfg.get("path", {})
        cruise = float(path_cfg.get("cruise_speed_mps", 0.25))
        pose = Pose2D(self._x, self._y, self._psi)
        complete = self._ilos.update(pose, cruise)

        idx_c = closest_index(self._path, self._x, self._y)
        if path_cfg.get("monotonic_progress", False) or self._path_closed:
            self._path_idx = max(self._path_idx, idx_c)
        else:
            self._path_idx = idx_c

        kappa = float(self._path[self._path_idx].kappa) if self._path else 0.0
        xte = abs(self._ilos.ye)
        desired = self._ilos.desired_heading
        head_err = angdiff(desired, self._psi)
        r_ref = float(
            np.clip(self._heading_pid.step(desired, self._psi), -self._max_r, self._max_r)
        )

        speed_scale = self._speed_scale(head_err, kappa)
        u_cmd = cruise * speed_scale
        surge_norm = float(
            np.clip(
                self._surge_gain * u_cmd / max(self._cruise_ref, 0.05),
                self._min_surge if not complete else 0.0,
                1.0,
            )
        )
        yaw_norm = float(np.clip(self._yaw_mix * r_ref / max(self._max_r, 0.05), -1.0, 1.0))
        left = surge_norm - yaw_norm
        right = surge_norm + yaw_norm

        if complete:
            left = right = 0.0
            u_cmd = r_ref = 0.0

        self._publish_thrust(left, right)

        app = self.cfg.get("control", {}).get("approach", "ilos_pid")
        if self._log_file:
            t = self.get_clock().now().nanoseconds * 1e-9 - self._t0
            self._log_file.write(
                f"{t:.4f},{self._x:.5f},{self._y:.5f},{self._psi:.5f},"
                f"{self._u:.4f},{self._v:.4f},{self._r:.4f},{xte:.5f},"
                f"{u_cmd:.4f},{r_ref:.4f},0.0000,{kappa:.5f},{app},"
                f"{left * self._sim_scale:.2f},{right * self._sim_scale:.2f}\n"
            )
            self._log_file.flush()

        self.viz.publish_pose(self._x, self._y, self._psi)
        self.viz.publish_xte(xte)
        if self._path_ready and self._path:
            stamp = self._path_activation._last_path_stamp
            display = getattr(self._path_activation, "_viz_samples", None)
            if display is None:
                display = self._path
            self.viz.publish_ref(display, stamp=stamp)
        self.viz.publish_traversed(self._x, self._y, self._psi, self._traj_step)


def main() -> None:
    parser = argparse.ArgumentParser(description="molo ILOS+PID follower (no MPC)")
    parser.add_argument("--config", required=True, help="Path to experiment config YAML")
    args = parser.parse_args()
    if not os.path.isfile(args.config):
        print(f"Config not found: {args.config}", file=sys.stderr)
        sys.exit(1)

    config = load_config(args.config)
    rclpy.init()
    node = IlosFollowerNode(config)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node._log_file:
            node._log_file.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
