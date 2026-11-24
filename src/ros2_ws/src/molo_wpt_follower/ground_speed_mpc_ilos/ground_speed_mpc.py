"""Local ground-speed kinematic MPC (path generator).

Decision variables per horizon step: ground speed U_k and yaw rate r_k.
Prediction (no current term)::

    x_{k+1} = x_k + dt * U_k * cos(psi_k)
    y_{k+1} = y_k + dt * U_k * sin(psi_k)
    psi_{k+1} = psi_k + dt * r_k

Published interface (see pipeline.tex): local polyline P_loc and first
optimal speed U_MPC. Yaw-rate sequence is internal only.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import List, Optional, Sequence, Tuple

import numpy as np
from scipy.optimize import minimize


def _point_to_segment_distance(
    px: float, py: float, ax: float, ay: float, bx: float, by: float
) -> float:
    dx, dy = bx - ax, by - ay
    len_sq = dx * dx + dy * dy
    if len_sq < 1e-9:
        return math.hypot(px - ax, py - ay)
    t = max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / len_sq))
    return math.hypot(px - (ax + t * dx), py - (ay + t * dy))


@dataclass
class Obstacle:
    x: float
    y: float
    vx: float = 0.0
    vy: float = 0.0
    radius: float = 1.0


@dataclass
class GroundSpeedMpcConfig:
    horizon: int = 12
    dt: float = 0.15
    u_min: float = 0.05
    u_max: float = 1.8
    r_max: float = 0.55
    du_max: float = 0.08
    dr_max: float = 0.20
    u_nom: float = 0.25
    w_path: float = 40.0
    w_u: float = 2.0
    w_r: float = 0.5
    w_du: float = 1.0
    w_dr: float = 0.5
    w_obs: float = 80.0
    boat_radius_m: float = 0.6
    margin_m: float = 0.8
    max_iter: int = 40


@dataclass
class MpcSolution:
    path_xy: np.ndarray  # (N+1, 2)
    path_psi: np.ndarray  # (N+1,)
    u_seq: np.ndarray  # (N,)
    r_seq: np.ndarray  # (N,)
    u_mpc: float
    success: bool
    cost: float


@dataclass
class GroundSpeedMPC:
    cfg: GroundSpeedMpcConfig = field(default_factory=GroundSpeedMpcConfig)
    _u_prev: Optional[np.ndarray] = field(default=None, init=False, repr=False)
    _r_prev: Optional[np.ndarray] = field(default=None, init=False, repr=False)
    _last: Optional[MpcSolution] = field(default=None, init=False, repr=False)

    def reset(self) -> None:
        self._u_prev = None
        self._r_prev = None
        self._last = None

    def solve(
        self,
        x0: float,
        y0: float,
        psi0: float,
        ref_xy: Sequence[Tuple[float, float]],
        obstacles: Optional[Sequence[Obstacle]] = None,
        u_nom: Optional[float] = None,
    ) -> MpcSolution:
        cfg = self.cfg
        n = int(cfg.horizon)
        dt = float(cfg.dt)
        u_nom = float(cfg.u_nom if u_nom is None else u_nom)
        obstacles = list(obstacles or [])
        ref = np.asarray(ref_xy, dtype=float)
        if ref.ndim != 2 or ref.shape[0] < 2:
            return self._fallback(x0, y0, psi0, u_nom)

        u0 = (
            np.full(n, u_nom)
            if self._u_prev is None
            else np.clip(self._u_prev, cfg.u_min, cfg.u_max)
        )
        r0 = np.zeros(n) if self._r_prev is None else np.clip(self._r_prev, -cfg.r_max, cfg.r_max)
        z0 = np.concatenate([u0, r0])

        bounds = [(cfg.u_min, cfg.u_max)] * n + [(-cfg.r_max, cfg.r_max)] * n

        def unpack(z: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
            return z[:n], z[n:]

        def predict(u: np.ndarray, r: np.ndarray) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
            xs = np.empty(n + 1)
            ys = np.empty(n + 1)
            ps = np.empty(n + 1)
            xs[0], ys[0], ps[0] = x0, y0, psi0
            for k in range(n):
                xs[k + 1] = xs[k] + dt * u[k] * math.cos(ps[k])
                ys[k + 1] = ys[k] + dt * u[k] * math.sin(ps[k])
                ps[k + 1] = ps[k] + dt * r[k]
            return xs, ys, ps

        def path_cost(xs: np.ndarray, ys: np.ndarray) -> float:
            d = _distances_to_polyline(xs, ys, ref)
            return float(np.dot(d, d))

        def obstacle_cost(xs: np.ndarray, ys: np.ndarray) -> float:
            if not obstacles:
                return 0.0
            pen = 0.0
            for k in range(n + 1):
                t = k * dt
                for obs in obstacles:
                    ox = obs.x + obs.vx * t
                    oy = obs.y + obs.vy * t
                    r_safe = obs.radius + cfg.boat_radius_m + cfg.margin_m
                    d = math.hypot(xs[k] - ox, ys[k] - oy)
                    if d < r_safe:
                        pen += (r_safe - d) ** 2
            return pen

        def objective(z: np.ndarray) -> float:
            u, r = unpack(z)
            xs, ys, _ = predict(u, r)
            du = np.diff(u, prepend=u[0])
            dr = np.diff(r, prepend=r[0])
            return float(
                cfg.w_path * path_cost(xs, ys)
                + cfg.w_u * np.sum((u - u_nom) ** 2)
                + cfg.w_r * np.sum(r ** 2)
                + cfg.w_du * np.sum(du ** 2)
                + cfg.w_dr * np.sum(dr ** 2)
                + cfg.w_obs * obstacle_cost(xs, ys)
            )

        # Rate limits enforced softly via w_du / w_dr (and clipped warm-start).
        # Hard |ΔU|,|Δr| inequalities with abs() are poorly behaved in SLSQP.
        try:
            res = minimize(
                objective,
                z0,
                method="SLSQP",
                bounds=bounds,
                options={"maxiter": cfg.max_iter, "ftol": 1e-4, "disp": False},
            )
            z_opt = res.x if res.success else z0
            success = bool(res.success)
            cost = float(res.fun) if res.success else float(objective(z_opt))
        except Exception:
            z_opt = z0
            success = False
            cost = float("inf")

        u, r = unpack(z_opt)
        xs, ys, ps = predict(u, r)
        sol = MpcSolution(
            path_xy=np.column_stack([xs, ys]),
            path_psi=ps,
            u_seq=u.copy(),
            r_seq=r.copy(),
            u_mpc=float(np.clip(u[0], cfg.u_min, cfg.u_max)),
            success=success,
            cost=cost,
        )
        self._u_prev = u.copy()
        self._r_prev = r.copy()
        self._last = sol
        return sol

    def _fallback(self, x0: float, y0: float, psi0: float, u_nom: float) -> MpcSolution:
        cfg = self.cfg
        n = cfg.horizon
        dt = cfg.dt
        u = np.full(n, float(np.clip(u_nom, cfg.u_min, cfg.u_max)))
        r = np.zeros(n)
        xs = np.empty(n + 1)
        ys = np.empty(n + 1)
        ps = np.empty(n + 1)
        xs[0], ys[0], ps[0] = x0, y0, psi0
        for k in range(n):
            xs[k + 1] = xs[k] + dt * u[k] * math.cos(ps[k])
            ys[k + 1] = ys[k] + dt * u[k] * math.sin(ps[k])
            ps[k + 1] = ps[k]
        sol = MpcSolution(
            path_xy=np.column_stack([xs, ys]),
            path_psi=ps,
            u_seq=u,
            r_seq=r,
            u_mpc=float(u[0]),
            success=False,
            cost=float("inf"),
        )
        self._last = sol
        return sol


def _dist_to_polyline(px: float, py: float, ref: np.ndarray) -> float:
    d = _distances_to_polyline(np.asarray([px]), np.asarray([py]), ref)
    return float(d[0]) if d.size else float("inf")


def _distances_to_polyline(xs: np.ndarray, ys: np.ndarray, ref: np.ndarray) -> np.ndarray:
    """Min distance from each query point to a polyline. Shape (len(xs),)."""
    xs = np.asarray(xs, dtype=float)
    ys = np.asarray(ys, dtype=float)
    ref = np.asarray(ref, dtype=float)
    if ref.shape[0] < 2:
        if ref.shape[0] == 1:
            return np.hypot(xs - ref[0, 0], ys - ref[0, 1])
        return np.full(xs.shape, np.inf)
    ax = ref[:-1, 0]
    ay = ref[:-1, 1]
    dx = ref[1:, 0] - ax
    dy = ref[1:, 1] - ay
    len_sq = dx * dx + dy * dy
    px = xs[:, None] - ax
    py = ys[:, None] - ay
    t = (px * dx + py * dy) / np.maximum(len_sq, 1e-9)
    t = np.clip(t, 0.0, 1.0)
    t = np.where(len_sq < 1e-9, 0.0, t)
    cx = ax + t * dx
    cy = ay + t * dy
    return np.hypot(xs[:, None] - cx, ys[:, None] - cy).min(axis=1)


def window_reference(
    samples_xy: Sequence[Tuple[float, float]],
    x: float,
    y: float,
    ahead: int = 40,
) -> List[Tuple[float, float]]:
    """Take a local window of the global path around the closest point."""
    if not samples_xy:
        return []
    pts = list(samples_xy)
    best_i = 0
    best_d = float("inf")
    for i, (px, py) in enumerate(pts):
        d = (px - x) ** 2 + (py - y) ** 2
        if d < best_d:
            best_d = d
            best_i = i
    end = min(len(pts), best_i + max(ahead, 2))
    window = pts[best_i:end]
    if len(window) < 2 and best_i > 0:
        window = pts[max(0, best_i - 1) : end]
    # Closed-path wrap
    if len(window) < 2 and pts:
        window = pts[best_i:] + pts[: max(2, ahead - (len(pts) - best_i))]
    return window
