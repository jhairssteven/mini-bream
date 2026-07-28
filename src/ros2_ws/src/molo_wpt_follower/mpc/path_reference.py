"""Reference path sampling for spatial MPC."""

from __future__ import annotations

import math
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import List, Sequence, Tuple

import numpy as np

_PARENT = Path(__file__).resolve().parent.parent
if str(_PARENT) not in sys.path:
    sys.path.append(str(_PARENT))


@dataclass
class PathSample:
    x: float
    y: float
    psi: float
    u_ref: float
    kappa: float = 0.0


def _wrap(a: float) -> float:
    while a > math.pi:
        a -= 2.0 * math.pi
    while a <= -math.pi:
        a += 2.0 * math.pi
    return a


def lemniscate_points(a: float = 10.0, n: int = 32) -> List[Tuple[float, float]]:
    pts = []
    for i in range(n):
        t = 2.0 * math.pi * i / n
        d = 1.0 + math.sin(t) ** 2
        pts.append((a * math.cos(t) / d, a * math.sin(t) * math.cos(t) / d))
    return pts


def circle_points(radius: float = 10.0, n: int = 48) -> List[Tuple[float, float]]:
    pts = []
    for i in range(n):
        t = 2.0 * math.pi * i / n
        pts.append((radius * math.cos(t), radius * math.sin(t)))
    return pts


def line_points(length_m: float = 40.0, n: int = 2) -> List[Tuple[float, float]]:
    return [(0.0, 0.0), (length_m, 0.0)] if n <= 2 else [
        (length_m * i / (n - 1), 0.0) for i in range(n)
    ]


def rectangle_points(
    width_m: float = 12.0, height_m: float = 8.0, n_per_edge: int = 8
) -> List[Tuple[float, float]]:
    w, h = width_m / 2.0, height_m / 2.0
    raw: List[Tuple[float, float]] = []
    for i in range(n_per_edge):
        t = i / n_per_edge
        raw.append((w * (2.0 * t - 1.0), h))
    for i in range(n_per_edge):
        t = i / n_per_edge
        raw.append((w, h * (1.0 - 2.0 * t)))
    for i in range(n_per_edge):
        t = i / n_per_edge
        raw.append((w * (1.0 - 2.0 * t), -h))
    for i in range(n_per_edge):
        t = i / n_per_edge
        raw.append((-w, -h * (2.0 * t - 1.0)))
    return raw


def sine_path_points(
    length_m: float = 50.0, amplitude_m: float = 6.0, waves: float = 2.0, n: int = 64
) -> List[Tuple[float, float]]:
    pts = []
    for i in range(n):
        t = i / max(1, n - 1)
        x = length_m * t
        y = amplitude_m * math.sin(2.0 * math.pi * waves * t)
        pts.append((x, y))
    return pts


def triangle_points(size_m: float = 14.0) -> List[Tuple[float, float]]:
    h = size_m * math.sqrt(3) / 2.0
    return [(0.0, 0.0), (size_m, 0.0), (size_m / 2.0, h), (0.0, 0.0)]


def generate_raw_points(traj: dict) -> Tuple[List[Tuple[float, float]], bool]:
    """Return polyline vertices and whether the path is a closed loop."""
    kind = str(traj.get("type", "lemniscate")).lower()
    if kind == "lemniscate":
        p = traj.get("lemniscate", traj)
        return lemniscate_points(float(p.get("scale_m", 10.0)), int(p.get("num_points", 32))), True
    if kind == "circle":
        return circle_points(float(traj.get("radius_m", 10.0)), int(traj.get("num_points", 48))), True
    if kind == "line":
        return line_points(float(traj.get("length_m", 40.0)), int(traj.get("num_points", 2))), False
    if kind == "rectangle":
        return (
            rectangle_points(
                float(traj.get("width_m", 12.0)),
                float(traj.get("height_m", 8.0)),
                int(traj.get("n_per_edge", 8)),
            ),
            True,
        )
    if kind in ("sine", "sine_path"):
        return (
            sine_path_points(
                float(traj.get("length_m", 50.0)),
                float(traj.get("amplitude_m", 6.0)),
                float(traj.get("waves", 2.0)),
                int(traj.get("num_points", 64)),
            ),
            False,
        )
    if kind == "triangle":
        return triangle_points(float(traj.get("size_m", 14.0))), True
    if kind == "points" and traj.get("vertices"):
        verts = [(float(v[0]), float(v[1])) for v in traj["vertices"]]
        closed = bool(traj.get("closed", verts[0] == verts[-1]))
        return verts, closed
    raise ValueError(f"Unknown trajectory type: {kind}")


def resample_polyline(
    points: Sequence[Tuple[float, float]], step_m: float, closed: bool = False
) -> List[PathSample]:
    if len(points) < 2:
        return []
    dense: List[Tuple[float, float]] = [points[0]]
    nseg = len(points) if closed else len(points) - 1
    for i in range(nseg):
        x0, y0 = points[i]
        x1, y1 = points[(i + 1) % len(points)]
        seg = math.hypot(x1 - x0, y1 - y0)
        n = max(1, int(seg / step_m))
        for k in range(1, n + 1):
            t = k / n
            dense.append((x0 + t * (x1 - x0), y0 + t * (y1 - y0)))

    out: List[PathSample] = []
    cruise_u = 0.8
    for i, (x, y) in enumerate(dense):
        if i < len(dense) - 1:
            dx = dense[i + 1][0] - x
            dy = dense[i + 1][1] - y
            psi = math.atan2(dy, dx)
        else:
            psi = out[-1].psi if out else 0.0
        out.append(PathSample(x, y, psi, cruise_u))
    attach_curvature(out, closed)
    return out


def attach_curvature(path: List[PathSample], closed: bool) -> None:
    """In-place curvature kappa = d(psi)/ds along polyline."""
    n = len(path)
    if n < 2:
        return
    for i in range(n):
        j = (i + 1) % n if closed else min(i + 1, n - 1)
        ds = math.hypot(path[j].x - path[i].x, path[j].y - path[i].y)
        dpsi = _wrap(path[j].psi - path[i].psi)
        path[i].kappa = dpsi / ds if ds > 1e-6 else 0.0
    if not closed:
        path[-1].kappa = path[-2].kappa if n > 1 else 0.0


def path_curvature(path: Sequence[PathSample], idx: int, closed: bool) -> float:
    if not path:
        return 0.0
    idx = max(0, min(idx, len(path) - 1))
    return float(path[idx].kappa)


def advance_path_index(
    path: Sequence[PathSample],
    idx: int,
    advance: int,
    closed: bool,
) -> int:
    n = len(path)
    if n == 0:
        return 0
    if closed:
        return (idx + advance) % n
    return min(idx + advance, n - 1)


def velocity_horizon_reference(
    path: Sequence[PathSample],
    start_idx: int,
    horizon: int,
    dt: float,
    step_m: float,
    cruise: float,
    closed: bool,
    max_r: float,
) -> np.ndarray:
    """(horizon+1, 3) references [u, v, r] with curvature feedforward r = u*kappa."""
    ref = np.zeros((horizon + 1, 3))
    if not path:
        return ref
    idx = start_idx
    n = len(path)
    for k in range(horizon + 1):
        p = path[idx]
        kappa = path_curvature(path, idx, closed)
        u = cruise / (1.0 + 2.0 * abs(kappa))
        u = max(0.12, u)
        r = float(np.clip(u * kappa, -max_r, max_r))
        ref[k] = [u, 0.0, r]
        advance = max(1, int(max(u, 0.15) * dt / step_m))
        idx = advance_path_index(path, idx, advance, closed)
    return ref


def rotate_path_to_index(path: List[PathSample], idx: int) -> List[PathSample]:
    """Rotate closed/open polyline so index `idx` becomes the first sample."""
    if not path or idx <= 0:
        return path
    idx = idx % len(path)
    if idx == 0:
        return path
    return path[idx:] + path[:idx]


def closest_index(path: Sequence[PathSample], x: float, y: float) -> int:
    best_i, best_d = 0, float("inf")
    for i, p in enumerate(path):
        d = (p.x - x) ** 2 + (p.y - y) ** 2
        if d < best_d:
            best_d, best_i = d, i
    return best_i


def cross_track_error(path: Sequence[PathSample], x: float, y: float) -> float:
    if len(path) < 2:
        return 0.0 if path else float("inf")
    i = closest_index(path, x, y)
    i0 = max(0, i - 1)
    i1 = min(len(path) - 1, i + 1)
    ax, ay = path[i0].x, path[i0].y
    bx, by = path[i1].x, path[i1].y
    dx, dy = bx - ax, by - ay
    l2 = dx * dx + dy * dy
    if l2 < 1e-9:
        return math.hypot(x - ax, y - ay)
    t = max(0.0, min(1.0, ((x - ax) * dx + (y - ay) * dy) / l2))
    px, py = ax + t * dx, ay + t * dy
    return math.hypot(x - px, y - py)


def signed_cross_track_error(path: Sequence[PathSample], x: float, y: float) -> float:
    """Signed lateral error (positive = left of path tangent)."""
    if len(path) < 2:
        return 0.0
    i = closest_index(path, x, y)
    i0 = max(0, i - 1)
    i1 = min(len(path) - 1, i + 1)
    ax, ay = path[i0].x, path[i0].y
    bx, by = path[i1].x, path[i1].y
    dx, dy = bx - ax, by - ay
    l2 = math.hypot(dx, dy)
    if l2 < 1e-9:
        return 0.0
    cross = dx * (y - ay) - dy * (x - ax)
    sign = 1.0 if cross >= 0.0 else -1.0
    return sign * cross_track_error(path, x, y)


def _smooth_and_finish_samples(
    samples: List[PathSample],
    cfg: dict,
    closed: bool,
    cruise: float,
) -> List[PathSample]:
    """Optional Dubins smoothing, cruise speed, and curvature attachment."""
    path_cfg = cfg.get("path", {})
    step = float(path_cfg.get("resample_step_m", 0.5))
    if path_cfg.get("smooth_dubins", True) and len(samples) >= 2:
        from algorithms import DubinsPlanner, PathPoint

        dubins_cfg = cfg.get("dubins", {})
        planner = DubinsPlanner(
            float(dubins_cfg.get("turning_radius_m", 4.0)),
            float(dubins_cfg.get("step_size_m", step)),
        )
        pts = [PathPoint(s.x, s.y, s.psi) for s in samples]
        if closed and len(pts) > 2:
            pts.append(PathPoint(pts[0].x, pts[0].y, pts[0].heading))
        smooth = planner.plan(pts)
        if closed and len(smooth) > 1:
            smooth = smooth[:-1]
        samples = [PathSample(p.x, p.y, p.heading, cruise) for p in smooth]
    for s in samples:
        s.u_ref = cruise
    attach_curvature(samples, closed)
    return samples


def samples_from_nav_path(
    msg,
    cfg: dict,
    origin_xy: Tuple[float, float],
) -> Tuple[List[PathSample], bool]:
    """Convert ``nav_msgs/Path`` poses into ``PathSample`` list."""
    path_cfg = cfg.get("path", {})
    wp_cfg = cfg.get("waypoints", {})
    relative = bool(
        path_cfg.get("relative_to_start", wp_cfg.get("relative_to_start", False))
    )
    step = float(path_cfg.get("resample_step_m", 0.5))
    cruise = float(path_cfg.get("cruise_speed_mps", 0.45))
    closed = bool(path_cfg.get("closed", False))

    if path_cfg.get("pass_through", False):
        raw_pts: List[Tuple[float, float]] = []
        for ps in msg.poses:
            x = float(ps.pose.position.x)
            y = float(ps.pose.position.y)
            if relative:
                x += origin_xy[0]
                y += origin_xy[1]
            raw_pts.append((x, y))
        if len(raw_pts) < 2:
            return [], closed
        samples: List[PathSample] = []
        for i, (x, y) in enumerate(raw_pts):
            if i < len(raw_pts) - 1:
                dx = raw_pts[i + 1][0] - x
                dy = raw_pts[i + 1][1] - y
                psi = math.atan2(dy, dx)
            elif samples:
                psi = samples[-1].psi
            else:
                psi = 0.0
            samples.append(PathSample(x, y, psi, cruise))
        attach_curvature(samples, closed)
        return samples, closed

    raw: List[Tuple[float, float]] = []
    for ps in msg.poses:
        x = float(ps.pose.position.x)
        y = float(ps.pose.position.y)
        if relative:
            x += origin_xy[0]
            y += origin_xy[1]
        raw.append((x, y))

    if len(raw) < 2:
        return [], closed

    if (
        not closed
        and len(raw) >= 3
        and math.hypot(raw[0][0] - raw[-1][0], raw[0][1] - raw[-1][1]) < step * 0.5
    ):
        closed = True
        raw = raw[:-1]

    samples = resample_polyline(raw, step, closed=closed)
    if not samples:
        return [], closed
    finished = _smooth_and_finish_samples(samples, cfg, closed, cruise)
    return finished, closed


def horizon_reference(
    path: Sequence[PathSample],
    start_idx: int,
    horizon: int,
    dt: float,
    step_m: float = 0.5,
    closed: bool = True,
) -> np.ndarray:
    """Return (horizon+1, 6) reference [x,y,psi,u,v,r] along arc length."""
    ref = np.zeros((horizon + 1, 6))
    if not path:
        return ref
    idx = start_idx
    n = len(path)
    for k in range(horizon + 1):
        p = path[min(idx, n - 1)]
        ref[k] = [p.x, p.y, p.psi, p.u_ref, 0.0, 0.0]
        advance = max(1, int(max(p.u_ref, 0.2) * dt / step_m))
        nxt = idx + advance
        if closed:
            idx = nxt % n
        else:
            idx = min(nxt, n - 1)
    return ref
