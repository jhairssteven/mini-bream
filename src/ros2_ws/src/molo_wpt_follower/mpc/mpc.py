#!/usr/bin/env python3
"""MPC trajectory follower: ILOS guidance + velocity MPC thrust allocation."""

from __future__ import annotations

import argparse
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

MPC_DIR = Path(__file__).resolve().parent
PARENT = MPC_DIR.parent
if str(MPC_DIR) not in sys.path:
    sys.path.insert(0, str(MPC_DIR))

from boat_model import BoatParameters
from control_law import ControlLaw
from path_activation import PathActivationManager
from path_reference import (
    PathSample,
    _smooth_and_finish_samples,
    attach_curvature,
    closest_index,
    generate_raw_points,
    horizon_reference,
    resample_polyline,
    rotate_path_to_index,
    signed_cross_track_error,
)
from spatial_mpc import SpatialMPC
from velocity_mpc import VelocityMPC
from viz import MpcVisualizer

if str(PARENT) not in sys.path:
    sys.path.append(str(PARENT))
from algorithms import DubinsPlanner, HeadingPID, ILOSFollower, PathPoint, Pose2D


def load_config(path: str) -> dict:
    with open(path, "r", encoding="utf-8") as f:
        return yaml.safe_load(f)


def build_mission_path(cfg: dict, origin_xy: Tuple[float, float]) -> Tuple[List[PathSample], bool]:
    wp = cfg.get("waypoints", {})
    path_cfg = cfg.get("path", {})
    step = float(path_cfg.get("resample_step_m", 0.5))
    cruise = float(path_cfg.get("cruise_speed_mps", 0.85))

    if wp.get("points"):
        raw = [(float(p["x"]), float(p["y"])) for p in wp["points"]]
        closed = bool(path_cfg.get("closed", False))
    else:
        traj = wp.get("trajectory", wp)
        raw, closed = generate_raw_points(traj)
        if "closed" in traj:
            closed = bool(traj["closed"])
        if "closed" in path_cfg:
            closed = bool(path_cfg["closed"])

    raw = [(origin_xy[0] + x, origin_xy[1] + y) for x, y in raw]
    samples = resample_polyline(raw, step, closed=closed)
    finished = _smooth_and_finish_samples(samples, cfg, closed, cruise)
    return finished, closed


def path_to_ilos_points(samples: List[PathSample]) -> List[PathPoint]:
    return [PathPoint(s.x, s.y, s.psi) for s in samples]


class MpcFollowerNode(Node):
    def __init__(self, config: dict):
        super().__init__("molo_mpc_follower")
        self.cfg = config
        self.frame_id = config.get("frame_id", "world")
        self.sim_enable = bool(config.get("sim_enable", True))
        topics = config.get("topics", {})

        boat = BoatParameters.from_dict(config.get("boat", {}))
        mpc_cfg = config.get("mpc", {})
        self._mpc_mode = str(mpc_cfg.get("mode", "velocity")).lower()
        self._mpc_dt = float(mpc_cfg.get("dt", 0.1))
        self._path_idx = 0
        if self._mpc_mode == "spatial":
            self.mpc = SpatialMPC(
                boat,
                horizon=int(mpc_cfg.get("horizon", 16)),
                dt=self._mpc_dt,
                Q_diag=np.array(
                    mpc_cfg.get("Q_spatial_diag", [800.0, 800.0, 180.0, 4.0, 4.0, 60.0]),
                    dtype=float,
                ),
                R_diag=np.array(mpc_cfg.get("R_diag", [0.04, 0.04]), dtype=float),
                Q_terminal_scale=float(mpc_cfg.get("Q_terminal_scale", 6.0)),
            )
            self._velocity_mpc = None
        else:
            self.mpc = VelocityMPC(
                boat,
                horizon=int(mpc_cfg.get("horizon", 12)),
                dt=self._mpc_dt,
                Q_diag=np.array(
                    mpc_cfg.get("Q_vel_diag", mpc_cfg.get("Q_diag", [12, 0.5, 80])),
                    dtype=float,
                ),
                R_diag=np.array(mpc_cfg.get("R_diag", [0.02, 0.02]), dtype=float),
                Q_terminal_scale=float(mpc_cfg.get("Q_terminal_scale", 5.0)),
            )
            self._velocity_mpc = self.mpc
        self._thrust_max = boat.max_thrust_N
        self._sim_scale = 1000.0 if self.sim_enable else 1.0

        dubins_cfg = config.get("dubins", {})
        self._dubins = DubinsPlanner(
            float(dubins_cfg.get("turning_radius_m", 5.5)),
            float(dubins_cfg.get("step_size_m", 0.5)),
        )
        ilos_cfg = config.get("ilos", {})
        self._ilos_kwargs = {
            "lookahead_min": float(ilos_cfg.get("lookahead_min_m", 1.2)),
            "lookahead_max": float(ilos_cfg.get("lookahead_max_m", 4.0)),
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
        self._max_r = float(config.get("guidance", {}).get("max_yaw_rate_rad_s", 0.45))
        self._path_closed = True
        self._control_law: Optional[ControlLaw] = None
        exp = config.get("experiment", {})
        self._log_path = exp.get("log_csv") if exp else None
        self._log_file = None
        self._t0 = None
        if self._log_path:
            Path(self._log_path).parent.mkdir(parents=True, exist_ok=True)
            self._log_file = open(self._log_path, "w", encoding="utf-8")
            self._log_file.write(
                "t,x,y,psi,u,v,r,xte,u_cmd,r_ref,v_ref,kappa,approach,"
                "u_mpc_ref,v_mpc_ref,r_mpc_ref,thrust_l_N,thrust_r_N\n"
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

        self._path_activation = PathActivationManager(
            self,
            config,
            self._on_path_ready,
            build_mission_path,
        )
        self._pending_origin = self._path_activation.pending

        self._rate = float(config.get("control_rate_hz", 10.0))
        self.create_timer(1.0 / self._rate, self._control_loop)
        approach = config.get("control", {}).get("approach", "curvature_ff_ilos")
        self.get_logger().info(
            f"molo_mpc_follower ready (mode={self._mpc_mode}, approach={approach}, "
            f"pose from {odom_topic})"
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
        display = viz_samples if viz_samples is not None else samples
        self._path = samples
        self._path_closed = closed
        self._path_ready = len(self._path) >= 2
        self._pending_origin = False
        if self._log_path:
            origin_file = Path(self._log_path).parent / "origin.json"
            import json

            with open(origin_file, "w", encoding="utf-8") as f:
                json.dump({"x": origin_xy[0], "y": origin_xy[1]}, f)
        ilos_kw = dict(self._ilos_kwargs)
        if self._path_closed:
            ilos_kw["no_of_laps"] = max(ilos_kw.get("no_of_laps", 1), 50)
        points = path_to_ilos_points(self._path)
        self._ilos = ILOSFollower(points, dubins_planner=self._dubins, **ilos_kw)
        idx = closest_index(self._path, origin_xy[0], origin_xy[1])
        self._ilos.work_index = idx
        self._ilos.orig_index = idx
        self._heading_pid.reset()
        mpc_cfg = self.cfg.get("mpc", {})
        self._control_law = ControlLaw(
            self.cfg,
            self._path,
            self._path_closed,
            float(mpc_cfg.get("dt", 0.1)),
            int(mpc_cfg.get("horizon", 12)),
        )
        self._control_law.attach_ilos(self._ilos, self._heading_pid)
        self._control_law.reset_index(idx)
        self._path_idx = idx
        self.viz.publish_ref(self._path)
        self.get_logger().info(
            f"MPC path loaded ({len(self._path)} samples, closed={self._path_closed})"
        )

    def _publish_thrust(self, thrust: np.ndarray) -> None:
        tl = float(np.clip(thrust[0] / self._thrust_max, -1.0, 1.0) * self._sim_scale)
        tr = float(np.clip(thrust[1] / self._thrust_max, -1.0, 1.0) * self._sim_scale)
        t = Float64 if self.sim_enable else Float32
        self._left_pub.publish(t(data=tl))
        self._right_pub.publish(t(data=tr))

    def _control_loop(self) -> None:
        if (
            self._x is None
            or self._psi is None
            or not self._path_ready
            or self._ilos is None
            or self._control_law is None
        ):
            return

        if self._t0 is None:
            self._t0 = self.get_clock().now().nanoseconds * 1e-9

        pose = Pose2D(self._x, self._y, self._psi)
        path_cfg = self.cfg.get("path", {})
        cruise = float(path_cfg.get("cruise_speed_mps", 0.45))
        complete = self._ilos.update(pose, cruise)

        z = np.array([self._x, self._y, self._psi, self._u, self._v, self._r])
        step_m = float(path_cfg.get("resample_step_m", 0.4))
        u_cmd = r_ref = v_ref = kappa = 0.0
        u_mpc_ref = v_mpc_ref = r_mpc_ref = 0.0
        app = self.cfg.get("control", {}).get("approach", "")

        if self._mpc_mode == "spatial":
            idx_c = closest_index(self._path, self._x, self._y)
            if path_cfg.get("monotonic_progress", False) or self._path_closed:
                self._path_idx = max(self._path_idx, idx_c)
            else:
                self._path_idx = idx_c
            z_ref = horizon_reference(
                self._path,
                self._path_idx,
                self.mpc.N,
                self._mpc_dt,
                step_m,
                closed=self._path_closed,
            )
            thrust, _, pred = self.mpc.solve(z, z_ref)
            xte = abs(signed_cross_track_error(self._path, self._x, self._y))
            if self._path:
                p = self._path[self._path_idx]
                u_cmd = float(p.u_ref)
                kappa = float(p.kappa)
            u_mpc_ref, v_mpc_ref, r_mpc_ref = (
                float(z_ref[0, 3]),
                float(z_ref[0, 4]),
                float(z_ref[0, 5]),
            )
        else:
            out = self._control_law.compute(
                self._x, self._y, self._psi, path_cfg, ilos_heading=self._ilos.desired_heading
            )
            xte = out.xte
            u_cmd, r_ref, v_ref, kappa = out.u_cmd, out.r_ref, out.v_ref, out.kappa
            u_mpc_ref, v_mpc_ref, r_mpc_ref = (
                float(out.nu_horizon[0, 0]),
                float(out.nu_horizon[0, 1]),
                float(out.nu_horizon[0, 2]),
            )
            thrust, pred = self.mpc.solve(z, out.nu_horizon)

        if complete:
            thrust = np.zeros(2)

        self._publish_thrust(thrust)

        if self._log_file:
            t = self.get_clock().now().nanoseconds * 1e-9 - self._t0
            self._log_file.write(
                f"{t:.4f},{self._x:.5f},{self._y:.5f},{self._psi:.5f},"
                f"{self._u:.4f},{self._v:.4f},{self._r:.4f},{xte:.5f},"
                f"{u_cmd:.4f},{r_ref:.4f},{v_ref:.4f},{kappa:.5f},{app},"
                f"{u_mpc_ref:.4f},{v_mpc_ref:.4f},{r_mpc_ref:.4f},"
                f"{thrust[0]:.2f},{thrust[1]:.2f}\n"
            )
            self._log_file.flush()

        self.viz.publish_pose(self._x, self._y, self._psi)
        self.viz.publish_xte(xte)
        if self._path_ready and self._path:
            self.viz.publish_ref(self._path)
        self.viz.publish_traversed(self._x, self._y, self._psi, self._traj_step)
        if pred is not None and len(pred) > 1 and self._path:
            if pred.shape[1] >= 3:
                pred_xy = pred[:, :3]
            else:
                pred_xy = np.zeros((len(pred), 3))
                px, py, ps = self._x, self._y, self._psi
                dt = self._mpc_dt
                for k in range(len(pred)):
                    u, v, r = pred[k]
                    ps = ps + r * dt
                    px += (u * math.cos(ps) - v * math.sin(ps)) * dt
                    py += (u * math.sin(ps) + v * math.cos(ps)) * dt
                    pred_xy[k] = [px, py, ps]
            self.viz.publish_pred(pred_xy)


def main() -> None:
    parser = argparse.ArgumentParser(description="molo MPC follower")
    parser.add_argument(
        "--config",
        default=str(MPC_DIR / "params.yaml"),
        help="Path to MPC params.yaml",
    )
    args = parser.parse_args()
    if not os.path.isfile(args.config):
        print(f"Config not found: {args.config}", file=sys.stderr)
        sys.exit(1)

    config = load_config(args.config)
    rclpy.init()
    node = MpcFollowerNode(config)
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
