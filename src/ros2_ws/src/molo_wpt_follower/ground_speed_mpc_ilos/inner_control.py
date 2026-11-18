"""ILOS path follower + surge/yaw PI/PD + dual-thruster allocation + f^{-1}.

Matches pipeline.tex sections 4–7. Allocation follows in-repo BlueBoat /
boat_model yaw sign (positive N → more starboard thrust).
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import List, Sequence, Tuple

import numpy as np


def _norm(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle <= -math.pi:
        angle += 2.0 * math.pi
    return angle


def _angdiff(target: float, current: float) -> float:
    return _norm(target - current)


angdiff = _angdiff  # public alias


@dataclass
class LocalILOSConfig:
    delta_m: float = 1.5
    sigma: float = 0.15  # integral gain (m/s) — river current rejection
    e_i_limit: float = 5.0
    advance_eps_m: float = 0.15


@dataclass
class LocalILOS:
    """ILOS on a short local polyline (MPC output or global segment)."""

    cfg: LocalILOSConfig = field(default_factory=LocalILOSConfig)
    e_i: float = 0.0
    seg_idx: int = 0
    ye: float = 0.0
    alpha: float = 0.0
    psi_d: float = 0.0

    def reset(self) -> None:
        self.e_i = 0.0
        self.seg_idx = 0
        self.ye = 0.0

    def set_path(
        self,
        path_xy: Sequence[Tuple[float, float]],
        soft: bool = False,
        pose_xy: Tuple[float, float] | None = None,
    ) -> None:
        self._path = [(float(p[0]), float(p[1])) for p in path_xy]
        if not soft:
            self.reset()
        if pose_xy is not None and len(self._path) >= 2:
            self.seg_idx = _closest_segment(self._path, pose_xy[0], pose_xy[1])
        else:
            self.seg_idx = min(self.seg_idx, max(0, len(self._path) - 2))

    def update(self, x: float, y: float, dt: float, u_mpc: float) -> Tuple[float, float]:
        """Return (psi_d, U_d)."""
        path = getattr(self, "_path", [])
        if len(path) < 2:
            return self.psi_d, float(u_mpc)

        self.seg_idx = _advance_segment(path, self.seg_idx, x, y, self.cfg.advance_eps_m)
        ax, ay = path[self.seg_idx]
        bx, by = path[min(self.seg_idx + 1, len(path) - 1)]
        self.alpha = math.atan2(by - ay, bx - ax)
        self.ye = _signed_cross_track(x, y, ax, ay, bx, by)

        delta = max(self.cfg.delta_m, 1e-3)
        sigma = self.cfg.sigma
        denom = delta * delta + (self.ye + sigma * self.e_i) ** 2
        self.e_i += dt * (delta * self.ye) / max(denom, 1e-9)
        self.e_i = float(np.clip(self.e_i, -self.cfg.e_i_limit, self.cfg.e_i_limit))
        if abs(self.ye) > 4.0 * delta:
            self.e_i *= 0.95

        self.psi_d = _norm(self.alpha - math.atan2(self.ye + sigma * self.e_i, delta))
        return self.psi_d, float(u_mpc)


@dataclass
class SurgePI:
    """Feedforward + mild PI on filtered surge, with output rate limit.

    k_ff must be near the plant steady-state gain X/U (sim ≈ few N per m/s).
    Large k_ff (e.g. 40) makes open-loop FF dominate: PI cannot brake until
    U_meas ≫ U_d (empirically U_g stayed ~0.75 m/s vs U_d ~0.25 with X>0).
    FF is faded under overspeed so feedback can actually decelerate.

    When u_d > 0 (forward cruise), brake force is clamped to brake_x_min so the
    loop can slow gently without driving the hull through zero into reverse.
    """

    kp: float = 40.0
    ki: float = 8.0
    k_ff: float = 35.0  # X_ff ≈ k_ff * U_d; plant X/U ≈ 25+40U (~35 near cruise)
    x_min: float = -200.0
    x_max: float = 200.0
    i_limit: float = 60.0
    dx_max: float = 200.0  # N/s slew (~1 s to full X_max)
    u_lpf_tau_s: float = 0.25
    # Fade FF to 0 once overspeed reaches this (m/s); wider = softer brake onset
    ff_overspeed_fade_mps: float = 0.40
    # Floor on X while commanding forward speed (N). 0 = coast-only; <0 = light brake.
    brake_x_min: float = -10.0
    # Scale kp when err < 0 (overspeed) so accel and brake are asymmetric.
    brake_kp_scale: float = 0.4
    integral: float = 0.0
    _u_filt: float = 0.0
    _x_prev: float = 0.0
    _filt_init: bool = False

    def reset(self) -> None:
        self.integral = 0.0
        self._u_filt = 0.0
        self._x_prev = 0.0
        self._filt_init = False

    def step(self, u_d: float, u_meas: float, dt: float) -> float:
        dt = max(float(dt), 1e-3)
        if not self._filt_init:
            self._u_filt = float(u_meas)
            self._filt_init = True
        else:
            alpha = dt / max(self.u_lpf_tau_s + dt, 1e-3)
            self._u_filt = (1.0 - alpha) * self._u_filt + alpha * float(u_meas)

        u_d = float(u_d)
        err = u_d - self._u_filt
        self.integral = float(np.clip(self.integral + err * dt, -self.i_limit, self.i_limit))

        # Fade feedforward when faster than commanded so PI can decelerate.
        overspeed = max(0.0, self._u_filt - u_d)
        fade = max(self.ff_overspeed_fade_mps, 1e-3)
        ff_scale = float(np.clip(1.0 - overspeed / fade, 0.0, 1.0))

        kp_eff = self.kp
        if err < 0.0:
            kp_eff *= float(np.clip(self.brake_kp_scale, 0.0, 1.0))

        x_raw = self.k_ff * u_d * ff_scale + kp_eff * err + self.ki * self.integral
        x_lo, x_hi = self.x_min, self.x_max
        # Forward command: do not allow hard reverse thrust (avoids U crossing 0).
        if u_d > 0.05:
            x_lo = max(x_lo, float(self.brake_x_min))
        x_sat = float(np.clip(x_raw, x_lo, x_hi))
        if abs(x_raw - x_sat) > 1e-9:
            self.integral *= 0.95

        # Slew-rate limit — kills thruster chatter
        dx = float(np.clip(x_sat - self._x_prev, -self.dx_max * dt, self.dx_max * dt))
        x_out = self._x_prev + dx
        self._x_prev = x_out
        return float(x_out)


@dataclass
class YawPD:
    kp: float = 35.0
    kd: float = 12.0
    n_max: float = 50.0

    def step(self, psi_d: float, psi: float, r: float) -> float:
        e = _angdiff(psi_d, psi)
        n = self.kp * e - self.kd * r
        return float(np.clip(n, -self.n_max, self.n_max))


@dataclass
class ThrusterMap:
    """Grey-box T = f(n); inverse via Newton / clamp. Linear default for sim."""

    t_max: float = 100.0
    n_db: float = 0.02
    c1: float = 100.0
    c2: float = 0.0

    def forward(self, n: float) -> float:
        if abs(n) < self.n_db:
            return 0.0
        return self.c1 * n + self.c2 * n * abs(n)

    def inverse(self, thrust_n: float) -> float:
        t = float(np.clip(thrust_n, -self.t_max, self.t_max))
        if abs(self.c2) < 1e-9:
            n = t / max(self.c1, 1e-6)
        else:
            s = 1.0 if t >= 0.0 else -1.0
            a = self.c2
            b = self.c1
            c = -abs(t)
            disc = b * b - 4.0 * a * c
            n_abs = (-b + math.sqrt(max(disc, 0.0))) / (2.0 * a) if a != 0 else abs(t) / b
            n = s * n_abs
        if abs(n) < self.n_db and abs(t) > 1e-6:
            n = math.copysign(self.n_db, t)
        return float(np.clip(n, -1.0, 1.0))


def platform_thrust_commands(
    left_n: float,
    right_n: float,
    fmap: ThrusterMap,
    *,
    sim_enable: bool,
) -> Tuple[float, float, float, float]:
    """Map allocated thruster forces to platform topic commands.

    Returns ``(cmd_left, cmd_right, force_left_N, force_right_N)``.

    - **sim (Gazebo):** publish force in Newtons (``gz-sim`` Thruster
      ``max_thrust_cmd`` units). Do **not** apply ``n*1000`` or the PWM
      deadband bump — that quantized small PI forces into ±20 N bang-bang.
    - **real:** publish normalized ``[-1, 1]`` via ``fmap.inverse`` for
      ``motor_controller`` / ``/pwm/*_thrust_cmd``.
    """
    fl = float(np.clip(left_n, -fmap.t_max, fmap.t_max))
    fr = float(np.clip(right_n, -fmap.t_max, fmap.t_max))
    if sim_enable:
        return fl, fr, fl, fr
    return fmap.inverse(fl), fmap.inverse(fr), fl, fr


@dataclass
class Allocator:
    """Dual-thruster allocation with surge priority.

    boat_model: N = L*(T_R - T_L). With B=2L:
        T_L = X/2 - N/B ,  T_R = X/2 + N/B

    Surge priority: clip |N| so both thrusters keep the sign of X (or zero).
    That stops the common forward/back bug where a saturated yaw moment
    drives one thruster reverse and the hull starts backing.
    """

    thruster_sep_m: float = 0.59
    t_min: float = -100.0
    t_max: float = 100.0
    surge_priority: bool = True
    # Only enforce no-reverse when |X| is large enough; below this, allow
    # full yaw (otherwise path following starves at low surge).
    surge_priority_x_min: float = 40.0

    def allocate(self, x_force: float, n_moment: float) -> Tuple[float, float]:
        b = max(self.thruster_sep_m, 1e-3)
        x_force = float(x_force)
        n_moment = float(n_moment)

        if self.surge_priority and abs(x_force) >= self.surge_priority_x_min:
            # |N| <= B/2 * |X|  ⇒  min(T_L, T_R) keeps sign of X
            n_lim = 0.5 * b * abs(x_force)
            n_moment = float(np.clip(n_moment, -n_lim, n_lim))

        t_l = x_force / 2.0 - n_moment / b
        t_r = x_force / 2.0 + n_moment / b
        t_l = float(np.clip(t_l, self.t_min, self.t_max))
        t_r = float(np.clip(t_r, self.t_min, self.t_max))
        return t_l, t_r


def speed_scale_from_heading(
    head_err: float,
    angle_fast: float = 0.15,
    angle_slow: float = 0.6,
    min_scale: float = 0.35,
) -> float:
    """Reduce surge command when heading error is large (ilos_boat style)."""
    ae = abs(float(head_err))
    if ae <= angle_fast:
        return 1.0
    if ae >= angle_slow:
        return float(min_scale)
    # linear between
    t = (ae - angle_fast) / max(angle_slow - angle_fast, 1e-6)
    return float(1.0 - t * (1.0 - min_scale))


def _signed_cross_track(
    px: float, py: float, ax: float, ay: float, bx: float, by: float
) -> float:
    dx, dy = bx - ax, by - ay
    length = math.hypot(dx, dy)
    if length < 1e-9:
        return 0.0
    return ((px - ax) * (-dy) + (py - ay) * dx) / length


def _closest_segment(path: List[Tuple[float, float]], x: float, y: float) -> int:
    best_i = 0
    best_d = float("inf")
    for i in range(max(0, len(path) - 1)):
        ax, ay = path[i]
        bx, by = path[min(i + 1, len(path) - 1)]
        mx, my = 0.5 * (ax + bx), 0.5 * (ay + by)
        d = (mx - x) ** 2 + (my - y) ** 2
        if d < best_d:
            best_d = d
            best_i = i
    return best_i


def _advance_segment(
    path: List[Tuple[float, float]],
    idx: int,
    x: float,
    y: float,
    eps: float,
) -> int:
    n = len(path)
    if n < 2:
        return 0
    idx = min(max(idx, 0), n - 2)
    while idx < n - 2:
        ax, ay = path[idx]
        bx, by = path[idx + 1]
        seg_len = math.hypot(bx - ax, by - ay)
        if seg_len < 1e-9:
            idx += 1
            continue
        progress = ((x - ax) * (bx - ax) + (y - ay) * (by - ay)) / seg_len
        if progress > seg_len - eps:
            idx += 1
        else:
            break
    return idx
