"""Waypoint-following algorithms in a local Cartesian world frame (x, y, theta).

All geometry uses ENU heading: 0 rad = East, pi/2 = North.
No GPS or UTM types appear here — only plain floats.
"""

from __future__ import annotations

import copy
import math
from dataclasses import dataclass
from typing import List, Optional, Sequence, Tuple

import dubins
import numpy as np


@dataclass
class Pose2D:
    x: float
    y: float
    theta: float


@dataclass
class PathPoint:
    x: float
    y: float
    heading: float


def norm(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle <= -math.pi:
        angle += 2.0 * math.pi
    return angle


def angdiff(target: float, current: float) -> float:
    return norm(target - current)


def distance(a: Pose2D, b: Pose2D) -> float:
    return math.hypot(a.x - b.x, a.y - b.y)


def point_to_segment_distance(
    px: float, py: float, ax: float, ay: float, bx: float, by: float
) -> float:
    dx, dy = bx - ax, by - ay
    len_sq = dx * dx + dy * dy
    if len_sq < 1e-9:
        return math.hypot(px - ax, py - ay)
    t = max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / len_sq))
    return math.hypot(px - (ax + t * dx), py - (ay + t * dy))


def bearing(from_pose: Pose2D, to_pose: Pose2D) -> float:
    return math.atan2(to_pose.y - from_pose.y, to_pose.x - from_pose.x)


def get_alpha(start: Pose2D, goal: Pose2D) -> float:
    return norm(bearing(start, goal) - start.theta)


def get_beta(start: Pose2D, goal: Pose2D) -> float:
    return norm(norm(goal.theta - start.theta) - get_alpha(start, goal))


def get_rho_alpha_beta(start: Pose2D, goal: Pose2D) -> Tuple[float, float, float]:
    return distance(start, goal), get_alpha(start, goal), get_beta(start, goal)


def polar_twist(
    rho: float,
    alpha: float,
    beta: float,
    kr: float = 0.07,
    ka: float = 0.8,
    kb: float = 0.1,
    max_linear: float = 0.5,
    max_angular: float = 0.2,
) -> Tuple[float, float]:
    linear = np.clip(kr * rho, -max_linear, max_linear)
    angular = np.clip(ka * alpha + kb * beta, -max_angular, max_angular)
    return float(linear), float(angular)


def diff_drive(linear: float, angular: float, k: float = 1.0) -> Tuple[float, float]:
    left = float(np.clip(linear - k * angular, -1.0, 1.0))
    right = float(np.clip(linear + k * angular, -1.0, 1.0))
    return left, right


def heading_from_points(
    points: Sequence[Tuple[float, float, Optional[float]]],
) -> List[PathPoint]:
    """Fill missing headings from point-to-point bearings."""
    if not points:
        return []

    n = len(points)
    out: List[PathPoint] = []
    for i, (x, y, h) in enumerate(points):
        if h is not None:
            heading = float(h)
        elif i < n - 1:
            heading = math.atan2(points[i + 1][1] - y, points[i + 1][0] - x)
        else:
            heading = out[-1].heading if out else 0.0
        out.append(PathPoint(x, y, heading))
    return out


class DubinsPlanner:
    def __init__(self, turning_radius: float, step_size: float):
        self.turning_radius = turning_radius
        self.step_size = step_size

    def plan(self, waypoints: Sequence[PathPoint]) -> List[PathPoint]:
        if len(waypoints) < 2:
            return list(waypoints)

        dense: List[PathPoint] = []
        q0 = (waypoints[0].x, waypoints[0].y, waypoints[0].heading)
        dense.append(waypoints[0])

        for wp in waypoints[1:]:
            q1 = (wp.x, wp.y, wp.heading)
            seg_dist = math.hypot(q1[0] - q0[0], q1[1] - q0[1])
            if seg_dist > 2.0 * self.step_size:
                path = dubins.shortest_path(q0, q1, self.turning_radius)
                configs, _ = path.sample_many(self.step_size)
                for x, y, h in configs[1:]:
                    dense.append(PathPoint(x, y, h))
            else:
                dense.append(PathPoint(q1[0], q1[1], q1[2]))
            q0 = q1

        return dense


class ILOSFollower:
    """Integral LOS path follower (from backseat PathFollower)."""

    def __init__(
        self,
        path: Sequence[PathPoint],
        *,
        lookahead_min: float = 1.0,
        lookahead_max: float = 3.0,
        conv_rate: float = 8.0,
        gamma: float = 0.0,
        replan_dist: float = 26.0,
        replan_lookahead: float = 8.0,
        handover_offset: int = 1,
        no_of_laps: int = 1,
        dubins_planner: Optional[DubinsPlanner] = None,
    ):
        self.original_path = list(path)
        self.working_path = copy.deepcopy(self.original_path)
        self.dubins = dubins_planner or DubinsPlanner(0.1, 0.5)

        self.lookahead_min = lookahead_min
        self.lookahead_max = lookahead_max
        self.conv_rate = conv_rate
        self.gamma = gamma
        self.replan_dist = replan_dist
        self.replan_lookahead = replan_lookahead
        self.handover_offset = handover_offset
        self.no_of_laps = no_of_laps

        self.work_index = 0
        self.orig_index = 0
        self.beta_hat = 0.0
        self.ye = 0.0
        self.proj_heading = 0.0
        self.look_ahead = lookahead_min
        self.mission_complete = False
        self.lap_ctr = 0
        self.replan_triggered = False
        self.desired_heading = 0.0

    @staticmethod
    def _dist(a: PathPoint, b: PathPoint) -> float:
        return math.hypot(a.x - b.x, a.y - b.y)

    def _dist_to_path(
        self, pose: Pose2D, path: Sequence[PathPoint], current_index: int, rng: int = 10
    ) -> Tuple[float, float, int]:
        pt = PathPoint(pose.x, pose.y, pose.theta)
        best_dist = self._dist(pt, path[current_index])
        path_index = min(max(current_index, 0), len(path) - 1)
        proj_heading = path[path_index].heading

        for index in range(current_index - rng, current_index + rng):
            index = min(max(index, 0), len(path) - 1)
            d = self._dist(pt, path[index])
            if d < best_dist:
                best_dist = d
                path_index = index
                proj_heading = path[index].heading

        ref = path[path_index]
        dx = pose.x - ref.x
        dy = pose.y - ref.y
        ye = -math.sin(proj_heading) * dx + math.cos(proj_heading) * dy
        return ye, proj_heading, path_index

    def _ilos(
        self, ye: float, beta_hat: float, proj_heading: float, speed: float, delta: float
    ) -> Tuple[float, float]:
        u = max(speed, 0.1)
        beta_hat += (ye * self.gamma * u * delta) / math.sqrt(
            delta * delta + (ye + delta * beta_hat) ** 2
        )
        heading = math.atan(-beta_hat - ye / delta) + proj_heading
        return heading, beta_hat

    def _variable_lookahead(self, ye: float) -> float:
        return (self.lookahead_max - self.lookahead_min) * math.exp(
            -self.conv_rate * abs(ye)
        ) + self.lookahead_min

    def _replan(self, pose: Pose2D) -> None:
        step = self.dubins.step_size
        target_idx = int(
            min(
                self.orig_index + math.floor(self.replan_lookahead / step),
                len(self.original_path) - math.floor(25.0 / step),
            )
        )
        target_idx = max(0, min(target_idx, len(self.original_path) - 1))
        start = PathPoint(pose.x, pose.y, pose.theta)
        end = copy.deepcopy(self.original_path[target_idx])
        recovery = self.dubins.plan([start, end])
        recovery.extend(copy.deepcopy(self.original_path[target_idx:]))
        self.working_path = recovery
        self.work_index = 0
        self.replan_triggered = True

    def update(self, pose: Pose2D, speed: float) -> bool:
        """Advance one control step. Returns True when mission is complete."""
        if self.mission_complete or len(self.working_path) < 2:
            self.mission_complete = True
            return True

        self.ye, self.proj_heading, self.work_index = self._dist_to_path(
            pose, self.working_path, self.work_index
        )
        _, _, self.orig_index = self._dist_to_path(
            pose, self.original_path, self.orig_index
        )

        if self.work_index >= len(self.working_path) - self.handover_offset:
            self.lap_ctr += 1
            if self.lap_ctr >= self.no_of_laps:
                self.mission_complete = True
                self.ye = 0.0
                return True
            self.orig_index = 1
            self.work_index = 1
            self.ye = 0.0

        if abs(self.ye) > self.replan_dist:
            self._replan(pose)
        else:
            self.replan_triggered = False

        self.look_ahead = self._variable_lookahead(self.ye)
        self.desired_heading, self.beta_hat = self._ilos(
            self.ye, self.beta_hat, self.proj_heading, speed, self.look_ahead
        )
        return False


class HeadingPID:
    def __init__(self, kp: float, ki: float, kd: float, integral_limit: float = 0.1):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.integral_limit = integral_limit
        self.integral = 0.0
        self.prev_error = 0.0

    def reset(self) -> None:
        self.integral = 0.0
        self.prev_error = 0.0

    def step(self, desired: float, current: float) -> float:
        err = angdiff(desired, current)
        self.integral = float(
            np.clip(self.integral + err, -self.integral_limit, self.integral_limit)
        )
        derivative = err - self.prev_error
        self.prev_error = err
        return self.kp * err + self.ki * self.integral + self.kd * derivative


def speed_from_heading_error(
    head_err: float,
    max_linear_pct: float,
    angle_threshold_fast: float = 0.15,
    angle_threshold_slow: float = 0.4,
) -> float:
    m = (max_linear_pct - 0.0) / (angle_threshold_fast - angle_threshold_slow)
    b = max_linear_pct - m * angle_threshold_fast
    return float(np.clip(m * abs(head_err) + b, 0.0, max_linear_pct))


class WaypointController:
    """High-level controller selecting polar or ILOS mode."""

    def __init__(self, config: dict):
        self.mode = config.get("control_mode", "ilos")
        self.polar_cfg = config.get("polar", {})
        self.speed_cfg = config.get("speed", {})
        self.arrival_radius = float(self.polar_cfg.get("arrival_radius_m", 2.0))

        dubins_cfg = config.get("dubins", {})
        self.dubins = DubinsPlanner(
            float(dubins_cfg.get("turning_radius_m", 0.1)),
            float(dubins_cfg.get("step_size_m", 0.5)),
        )

        ilos_cfg = config.get("ilos", {})
        pid_cfg = config.get("pid", {})
        self.pid = HeadingPID(
            float(pid_cfg.get("kp", 1.0)),
            float(pid_cfg.get("ki", 0.0)),
            float(pid_cfg.get("kd", 0.2)),
            float(pid_cfg.get("integral_limit", 0.1)),
        )

        self.ilos_follower: Optional[ILOSFollower] = None
        self.mission_waypoints: List[PathPoint] = []
        self.planned_path: List[PathPoint] = []

        self._polar_goal: Optional[Pose2D] = None
        self._polar_index = 0

    def set_mission(self, waypoints: Sequence[PathPoint]) -> None:
        self.mission_waypoints = list(waypoints)
        self._polar_index = 0
        self._polar_goal = None
        self.pid.reset()

        if self.mode == "ilos" and len(waypoints) >= 2:
            self.planned_path = self.dubins.plan(waypoints)
            ilos_cfg = self.config_ilos if hasattr(self, "config_ilos") else {}
            self.ilos_follower = ILOSFollower(
                self.planned_path,
                dubins_planner=self.dubins,
                **ilos_cfg,
            )
        else:
            if len(waypoints) >= 2:
                self.planned_path = self.dubins.plan(waypoints)
            else:
                self.planned_path = list(waypoints)
            self.ilos_follower = None

    def configure_ilos(self, ilos_cfg: dict) -> None:
        self.config_ilos = {
            "lookahead_min": float(ilos_cfg.get("lookahead_min_m", 1.0)),
            "lookahead_max": float(ilos_cfg.get("lookahead_max_m", 3.0)),
            "conv_rate": float(ilos_cfg.get("conv_rate", 8.0)),
            "gamma": float(ilos_cfg.get("gamma", 0.0)),
            "replan_dist": float(ilos_cfg.get("replan_dist_m", 26.0)),
            "replan_lookahead": float(ilos_cfg.get("replan_lookahead_m", 8.0)),
            "handover_offset": int(ilos_cfg.get("handover_offset", 1)),
            "no_of_laps": int(ilos_cfg.get("no_of_laps", 1)),
        }

    def set_polar_goal(self, goal: Pose2D) -> None:
        self._polar_goal = goal

    def step(
        self, pose: Pose2D, speed: float
    ) -> Tuple[float, float, bool, dict]:
        """Returns (linear_cmd, angular_cmd, mission_complete, debug)."""
        debug: dict = {"mode": self.mode}

        if self.mode == "ilos" and self.ilos_follower is not None:
            complete = self.ilos_follower.update(pose, speed)
            desired = self.ilos_follower.desired_heading
            head_err = angdiff(desired, pose.theta)
            angular = self.pid.step(desired, pose.theta)
            linear_pct = speed_from_heading_error(
                head_err,
                float(self.speed_cfg.get("max_linear_pct", 0.01)),
                float(self.speed_cfg.get("angle_threshold_fast_rad", 0.15)),
                float(self.speed_cfg.get("angle_threshold_slow_rad", 0.4)),
            )
            max_ang = float(self.speed_cfg.get("max_angular_pct", 0.3))
            debug.update(
                {
                    "desired_heading": desired,
                    "cross_track_error": self.ilos_follower.ye,
                    "heading_error_rad": head_err,
                    "lookahead": self.ilos_follower.look_ahead,
                    "path_index": self.ilos_follower.work_index,
                }
            )
            if complete:
                return 0.0, 0.0, True, debug
            return linear_pct, float(np.clip(angular, -max_ang, max_ang)), False, debug

        # Polar mode — sequential waypoints or a single RViz goal
        goal = self._polar_goal
        prev_wp: Optional[Pose2D] = None
        if goal is None and self.mission_waypoints:
            idx = min(self._polar_index, len(self.mission_waypoints) - 1)
            wp = self.mission_waypoints[idx]
            goal = Pose2D(wp.x, wp.y, wp.heading)
            if idx > 0:
                p = self.mission_waypoints[idx - 1]
                prev_wp = Pose2D(p.x, p.y, p.heading)

        if goal is None:
            return 0.0, 0.0, True, debug

        rho, alpha, beta = get_rho_alpha_beta(pose, goal)
        if prev_wp is not None:
            debug["cross_track_error"] = point_to_segment_distance(
                pose.x, pose.y, prev_wp.x, prev_wp.y, goal.x, goal.y
            )
        debug.update({"rho": rho, "alpha": alpha, "beta": beta})

        if rho < self.arrival_radius:
            if self._polar_goal is not None:
                self._polar_goal = None
                return 0.0, 0.0, True, debug
            self._polar_index += 1
            if self._polar_index >= len(self.mission_waypoints):
                return 0.0, 0.0, True, debug
            wp = self.mission_waypoints[self._polar_index]
            goal = Pose2D(wp.x, wp.y, wp.heading)
            rho, alpha, beta = get_rho_alpha_beta(pose, goal)

        linear, angular = polar_twist(
            rho,
            alpha,
            beta,
            kr=float(self.polar_cfg.get("kr", 0.07)),
            ka=float(self.polar_cfg.get("ka", 0.8)),
            kb=float(self.polar_cfg.get("kb", 0.1)),
            max_linear=float(self.polar_cfg.get("max_linear", 0.5)),
            max_angular=float(self.polar_cfg.get("max_angular", 0.2)),
        )
        return linear, angular, False, debug
