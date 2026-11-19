#!/usr/bin/env python3
"""Ground-speed MPC → ILOS → PI/PD → allocation → thruster commands.

Mission path: same fixed shapes as h0/ilos (lemniscate, etc.) via build_mission_path.
No RRT* — global reference is the experiment trajectory; MPC redraws a short
local collision-free polyline for ILOS to track.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import sys
import threading
from pathlib import Path
from typing import List, Optional, Tuple

import numpy as np
import rclpy
import yaml
from nav_msgs.msg import Odometry
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from std_msgs.msg import Float32, Float64
from tf_transformations import euler_from_quaternion

PKG_DIR = Path(__file__).resolve().parent
MPC_DIR = PKG_DIR.parent / "mpc"
MOLO_DIR = PKG_DIR.parent
# MPC_DIR first so mpc/viz.py wins over molo_wpt_follower/viz.py
if str(MPC_DIR) not in sys.path:
    sys.path.insert(0, str(MPC_DIR))
if str(PKG_DIR) not in sys.path:
    sys.path.insert(0, str(PKG_DIR))
if str(MOLO_DIR) not in sys.path:
    sys.path.append(str(MOLO_DIR))

from ground_speed_mpc import (  # noqa: E402
    GroundSpeedMPC,
    GroundSpeedMpcConfig,
    Obstacle,
    window_reference,
)
from inner_control import (  # noqa: E402
    Allocator,
    LocalILOS,
    LocalILOSConfig,
    SurgePI,
    ThrusterMap,
    YawPD,
    angdiff,
    platform_thrust_commands,
    speed_scale_from_heading,
)
from mpc import build_mission_path  # noqa: E402
from path_activation import PathActivationManager  # noqa: E402
from path_reference import PathSample, closest_index  # noqa: E402
from viz import MpcVisualizer  # noqa: E402


def load_config(path: str) -> dict:
    with open(path, "r", encoding="utf-8") as f:
        return yaml.safe_load(f)


class GsMpcIlosFollowerNode(Node):
    def __init__(self, config: dict):
        super().__init__("molo_gs_mpc_ilos")
        self.cfg = config
        self.frame_id = config.get("frame_id", "world")
        self.sim_enable = bool(config.get("sim_enable", True))
        topics = config.get("topics", {})

        mpc_cfg = config.get("gs_mpc", {})
        self._mpc = GroundSpeedMPC(
            GroundSpeedMpcConfig(
                horizon=int(mpc_cfg.get("horizon", 12)),
                dt=float(mpc_cfg.get("dt", 0.15)),
                u_min=float(mpc_cfg.get("u_min", 0.05)),
                u_max=float(mpc_cfg.get("u_max", 0.45)),
                r_max=float(mpc_cfg.get("r_max", 0.55)),
                du_max=float(mpc_cfg.get("du_max", 0.08)),
                dr_max=float(mpc_cfg.get("dr_max", 0.20)),
                u_nom=float(config.get("path", {}).get("cruise_speed_mps", 0.25)),
                w_path=float(mpc_cfg.get("w_path", 40.0)),
                w_u=float(mpc_cfg.get("w_u", 2.0)),
                w_r=float(mpc_cfg.get("w_r", 0.5)),
                w_du=float(mpc_cfg.get("w_du", 1.0)),
                w_dr=float(mpc_cfg.get("w_dr", 0.5)),
                w_obs=float(mpc_cfg.get("w_obs", 80.0)),
                boat_radius_m=float(mpc_cfg.get("boat_radius_m", 0.6)),
                margin_m=float(mpc_cfg.get("margin_m", 0.8)),
                max_iter=int(mpc_cfg.get("max_iter", 40)),
            )
        )
        self._mpc_ref_ahead = int(mpc_cfg.get("ref_ahead_points", 48))
        self._mpc_period = float(mpc_cfg.get("period_s", mpc_cfg.get("dt", 0.15)))

        ilos_cfg = config.get("ilos", {})
        self._ilos = LocalILOS(
            LocalILOSConfig(
                delta_m=float(ilos_cfg.get("delta_m", ilos_cfg.get("lookahead_min_m", 1.5))),
                sigma=float(ilos_cfg.get("sigma", ilos_cfg.get("gamma", 0.15))),
                e_i_limit=float(ilos_cfg.get("e_i_limit", 5.0)),
                advance_eps_m=float(ilos_cfg.get("advance_eps_m", 0.15)),
            )
        )

        surge_cfg = config.get("surge_pi", {})
        self._surge = SurgePI(
            kp=float(surge_cfg.get("kp", 40.0)),
            ki=float(surge_cfg.get("ki", 8.0)),
            k_ff=float(surge_cfg.get("k_ff", 35.0)),
            x_min=float(surge_cfg.get("x_min", -200.0)),
            x_max=float(surge_cfg.get("x_max", 200.0)),
            i_limit=float(surge_cfg.get("i_limit", 60.0)),
            dx_max=float(surge_cfg.get("dx_max", 200.0)),
            u_lpf_tau_s=float(surge_cfg.get("u_lpf_tau_s", 0.25)),
            ff_overspeed_fade_mps=float(surge_cfg.get("ff_overspeed_fade_mps", 0.40)),
            brake_x_min=float(surge_cfg.get("brake_x_min", -10.0)),
            brake_kp_scale=float(surge_cfg.get("brake_kp_scale", 0.4)),
        )
        yaw_cfg = config.get("yaw_pd", {})
        self._yaw = YawPD(
            kp=float(yaw_cfg.get("kp", 35.0)),
            kd=float(yaw_cfg.get("kd", 12.0)),
            n_max=float(yaw_cfg.get("n_max", 50.0)),
        )
        alloc_cfg = config.get("allocation", {})
        t_max = float(alloc_cfg.get("t_max", 100.0))
        self._alloc = Allocator(
            thruster_sep_m=float(alloc_cfg.get("thruster_sep_m", 0.59)),
            t_min=float(alloc_cfg.get("t_min", -t_max)),
            t_max=t_max,
            surge_priority=bool(alloc_cfg.get("surge_priority", True)),
            surge_priority_x_min=float(alloc_cfg.get("surge_priority_x_min", 40.0)),
        )
        speed_cfg = config.get("speed", {})
        self._angle_fast = float(speed_cfg.get("angle_threshold_fast_rad", 0.15))
        self._angle_slow = float(speed_cfg.get("angle_threshold_slow_rad", 0.6))
        self._min_speed_scale = float(speed_cfg.get("min_speed_scale", 0.35))
        # Never crawl at MPC u_min — that + aggressive PI caused fwd/back chatter.
        self._u_d_floor_frac = float(speed_cfg.get("u_d_floor_frac_of_cruise", 0.7))
        fmap = config.get("thruster_map", {})
        self._fmap = ThrusterMap(
            t_max=float(fmap.get("t_max", t_max)),
            n_db=float(fmap.get("n_db", 0.02)),
            c1=float(fmap.get("c1", t_max)),
            c2=float(fmap.get("c2", 0.0)),
        )

        self._u_min = float(self._mpc.cfg.u_min)
        self._u_max = float(self._mpc.cfg.u_max)
        # Signed surge feedback (body-x). Unsigned |U_g| loses direction and
        # makes the PI drive harder reverse when already backing (unstable).
        surge_mode = str(config.get("surge_pi", {}).get("speed_feedback", "body_u"))
        self._speed_feedback = surge_mode  # body_u | ground_long | ground_mag

        exp = config.get("experiment", {})
        self._log_path = exp.get("log_csv") if exp else None
        self._log_file = None
        self._t0 = None
        if self._log_path:
            Path(self._log_path).parent.mkdir(parents=True, exist_ok=True)
            self._log_file = open(self._log_path, "w", encoding="utf-8")
            self._log_file.write(
                "t,x,y,psi,u,v,r,xte,u_cmd,r_ref,v_ref,kappa,approach,"
                "thrust_l_N,thrust_r_N,u_g,u_mpc,psi_d,X,N\n"
            )

        qos_s = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=1,
        )
        odom_topic = topics.get("odometry", "/odom")
        self.create_subscription(Odometry, odom_topic, self._odom_cb, qos_s)

        thrust_t = Float64 if self.sim_enable else Float32
        self._left_pub = self.create_publisher(
            thrust_t, topics.get("left_thrust", "/wamv/thrusters/left/thrust"), qos_s
        )
        self._right_pub = self.create_publisher(
            thrust_t, topics.get("right_thrust", "/wamv/thrusters/right/thrust"), qos_s
        )

        traj_cfg = config.get("trajectory", {})
        self.viz = MpcVisualizer(self, self.frame_id, topics)
        self._traj_step = float(traj_cfg.get("min_distance_m", 0.25))
        self._viz_lock = threading.Lock()
        self._viz_snap: Optional[Tuple[float, float, float, float]] = None
        self._viz_pred = None
        self._viz_display: Optional[List] = None
        self._viz_ref_stamp = None
        self._viz_ref_dirty = False
        self._viz_ref_mono = 0.0
        self._viz_group = MutuallyExclusiveCallbackGroup()

        self._x = self._y = self._psi = None
        self._u = self._v = self._r = 0.0
        self._u_g = 0.0
        self._chi = 0.0
        self._path: List[PathSample] = []
        self._path_xy: List[Tuple[float, float]] = []
        self._path_ready = False
        self._path_idx = 0
        self._path_closed = True

        self._u_mpc = float(config.get("path", {}).get("cruise_speed_mps", 0.25))
        self._p_loc: List[Tuple[float, float]] = []
        self._obstacles: List[Obstacle] = []
        self._last_mpc_t = -1e9

        self._path_activation = PathActivationManager(
            self,
            config,
            self._on_path_ready,
            build_mission_path,
        )
        self._pending_origin = self._path_activation.pending

        self._rate = float(config.get("control_rate_hz", 10.0))
        self._dt = 1.0 / self._rate
        self.create_timer(self._dt, self._control_loop)
        # Paths and pose markers go out on another thread so a stalled subscriber
        # cannot delay the next thrust command.
        self.create_timer(0.1, self._publish_viz, callback_group=self._viz_group)
        self.get_logger().info(
            f"molo_gs_mpc_ilos ready (GS-MPC→ILOS→PI/PD, pose from {odom_topic})"
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
        # Ground speed from body velocity (GT odom / GPS-derived odom)
        self._u_g = math.hypot(self._u, self._v)
        xd = self._u * math.cos(self._psi) - self._v * math.sin(self._psi)
        yd = self._u * math.sin(self._psi) + self._v * math.cos(self._psi)
        self._chi = math.atan2(yd, xd) if self._u_g > 0.05 else self._psi
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
        soft = self._path_ready
        display_samples = viz_samples if viz_samples is not None else samples
        self._path = samples
        self._path_xy = [(s.x, s.y) for s in samples]
        self._path_closed = closed
        self._path_ready = len(self._path) >= 2
        self._pending_origin = False
        if self._log_path:
            origin_file = Path(self._log_path).parent / "origin.json"
            with open(origin_file, "w", encoding="utf-8") as f:
                json.dump({"x": origin_xy[0], "y": origin_xy[1]}, f)
        if soft and self._x is not None and self._y is not None:
            idx0 = closest_index(self._path, self._x, self._y)
        self._path_idx = idx0
        if not soft:
            self._surge.reset()
            self._ilos.reset()
            self._mpc.reset()
        stamp = self._path_activation._last_path_stamp
        with self._viz_lock:
            self._viz_display = display_samples
            self._viz_ref_stamp = stamp
            self._viz_ref_dirty = True
        action = "updated" if soft else "loaded"
        self.get_logger().info(
            f"GS-MPC path {action} ({len(self._path)} samples, closed={self._path_closed})"
        )

    def _run_mpc(self) -> None:
        assert self._x is not None and self._psi is not None
        ref = window_reference(
            self._path_xy, self._x, self._y, ahead=self._mpc_ref_ahead
        )
        if self._path_closed and len(ref) < 4:
            # wrap a bit more of the loop
            ref = window_reference(
                self._path_xy + self._path_xy[: self._mpc_ref_ahead],
                self._x,
                self._y,
                ahead=self._mpc_ref_ahead,
            )
        cruise = float(self.cfg.get("path", {}).get("cruise_speed_mps", self._u_mpc))
        sol = self._mpc.solve(
            self._x,
            self._y,
            self._psi,
            ref,
            obstacles=self._obstacles,
            u_nom=cruise,
        )
        if sol.success or self._last_mpc_t < 0:
            # ILOS tracks a local window of the global mission path (stable).
            # MPC prediction is published for RViz / obstacle-aware replans;
            # when obstacles are present, prefer the MPC polyline.
            if self._obstacles:
                self._p_loc = [(float(p[0]), float(p[1])) for p in sol.path_xy]
            else:
                self._p_loc = window_reference(
                    self._path_xy, self._x, self._y, ahead=self._mpc_ref_ahead
                )
                if len(self._p_loc) < 2:
                    self._p_loc = [(float(p[0]), float(p[1])) for p in sol.path_xy]
            self._u_mpc = float(np.clip(sol.u_mpc, self._u_min, self._u_max))
            self._ilos.set_path(self._p_loc, soft=True, pose_xy=(self._x, self._y))
            states = np.column_stack([sol.path_xy, sol.path_psi])
            with self._viz_lock:
                self._viz_pred = states
        # else keep last P_loc / U_MPC

    def _publish_thrust(self, left_n: float, right_n: float) -> Tuple[float, float]:
        """Publish thruster cmds; return allocated forces [N] for logging."""
        cmd_l, cmd_r, force_l, force_r = platform_thrust_commands(
            left_n, right_n, self._fmap, sim_enable=self.sim_enable
        )
        t = Float64 if self.sim_enable else Float32
        self._left_pub.publish(t(data=float(cmd_l)))
        self._right_pub.publish(t(data=float(cmd_r)))
        return force_l, force_r

    def _control_loop(self) -> None:
        if self._x is None or self._psi is None or not self._path_ready:
            return

        now = self.get_clock().now().nanoseconds * 1e-9
        if self._t0 is None:
            self._t0 = now

        if now - self._last_mpc_t >= self._mpc_period:
            self._run_mpc()
            self._last_mpc_t = now

        if len(self._p_loc) < 2:
            # First tick before MPC: seed from global segment
            ref = window_reference(self._path_xy, self._x, self._y, ahead=12)
            self._p_loc = ref if len(ref) >= 2 else self._path_xy[:12]
            self._ilos.set_path(self._p_loc, soft=False)

        if self._speed_feedback == "ground_mag":
            u_g_meas = self._u_g
        elif self._speed_feedback == "ground_long":
            u_g_meas = self._u_g * math.cos(self._chi - self._psi)
        else:
            # body-x surge (signed) — preferred with odometry
            u_g_meas = self._u

        cruise = float(self.cfg.get("path", {}).get("cruise_speed_mps", 0.3))
        # Speed hint: floor U_MPC near cruise so we never hold crawl speed with a
        # high-gain PI (that pattern reverses the boat and chatters).
        u_hint = max(float(self._u_mpc), cruise * self._u_d_floor_frac)
        psi_d, _ = self._ilos.update(self._x, self._y, self._dt, u_hint)
        head_err = angdiff(psi_d, self._psi)
        # Heading scale reduces speed in large turns, but keep a real cruise
        # floor (not crawl) so the surge loop does not chatter around ~0.
        u_floor = cruise * self._u_d_floor_frac
        u_d = float(
            np.clip(
                u_hint
                * speed_scale_from_heading(
                    head_err,
                    self._angle_fast,
                    self._angle_slow,
                    self._min_speed_scale,
                ),
                u_floor,
                self._u_max,
            )
        )

        x_force = self._surge.step(u_d, u_g_meas, self._dt)
        n_moment = self._yaw.step(psi_d, self._psi, self._r)
        t_l, t_r = self._alloc.allocate(x_force, n_moment)
        force_l, force_r = self._publish_thrust(t_l, t_r)

        # XTE vs global path (scoring / evaluate_mpc): always use geometric
        # closest sample. Monotonic index is only for optional progress logs.
        idx_c = closest_index(self._path, self._x, self._y)
        path_cfg = self.cfg.get("path", {})
        if path_cfg.get("monotonic_progress", False):
            self._path_idx = max(self._path_idx, idx_c)
        else:
            self._path_idx = idx_c
        ref = self._path[idx_c]
        dx = self._x - ref.x
        dy = self._y - ref.y
        xte_global = abs(-math.sin(ref.psi) * dx + math.cos(ref.psi) * dy)
        kappa = float(ref.kappa) if self._path else 0.0

        app = self.cfg.get("control", {}).get("approach", "gs_mpc_ilos")
        if self._log_file:
            t = now - self._t0
            self._log_file.write(
                f"{t:.4f},{self._x:.5f},{self._y:.5f},{self._psi:.5f},"
                f"{self._u:.4f},{self._v:.4f},{self._r:.4f},{xte_global:.5f},"
                f"{u_d:.4f},0.0000,0.0000,{kappa:.5f},{app},"
                f"{force_l:.2f},{force_r:.2f},"
                f"{u_g_meas:.4f},{self._u_mpc:.4f},{psi_d:.5f},{x_force:.3f},{n_moment:.3f}\n"
            )
            self._log_file.flush()

        with self._viz_lock:
            self._viz_snap = (self._x, self._y, self._psi, xte_global)

    def _publish_viz(self) -> None:
        with self._viz_lock:
            snap = self._viz_snap
            pred = self._viz_pred
            self._viz_pred = None
            display = self._viz_display
            stamp = self._viz_ref_stamp
            dirty = self._viz_ref_dirty
            self._viz_ref_dirty = False
        if snap is not None:
            x, y, psi, xte = snap
            self.viz.publish_pose(x, y, psi)
            self.viz.publish_xte(xte)
            self.viz.publish_traversed(x, y, psi, self._traj_step)
        if pred is not None:
            self.viz.publish_pred(pred)
        now = self.get_clock().now().nanoseconds * 1e-9
        if display and (dirty or now - self._viz_ref_mono >= 1.0):
            self.viz.publish_ref(display, stamp=stamp)
            self._viz_ref_mono = now


def main() -> None:
    parser = argparse.ArgumentParser(description="Ground-speed MPC + ILOS follower")
    parser.add_argument("--config", required=True, help="Path to experiment config YAML")
    args = parser.parse_args()
    if not os.path.isfile(args.config):
        print(f"Config not found: {args.config}", file=sys.stderr)
        sys.exit(1)

    config = load_config(args.config)
    rclpy.init()
    node = GsMpcIlosFollowerNode(config)
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        if node._log_file:
            node._log_file.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
