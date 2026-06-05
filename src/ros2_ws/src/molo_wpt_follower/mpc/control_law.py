"""Pluggable guidance + reference generation for MPC experiments."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import List, Optional, Sequence, Tuple

import numpy as np

from path_reference import (
    PathSample,
    advance_path_index,
    closest_index,
    path_curvature,
    signed_cross_track_error,
    velocity_horizon_reference,
)


def _wrap(a: float) -> float:
    while a > math.pi:
        a -= 2.0 * math.pi
    while a <= -math.pi:
        a += 2.0 * math.pi
    return a


def _sat(x: float, phi: float) -> float:
    """Boundary-layer substitute for sign(x)."""
    if phi < 1e-6:
        phi = 1e-6
    return math.tanh(x / phi)


# All non-learning approaches registered for benchmarking
APPROACH_NAMES = (
    "baseline_ilos_velocity",
    "curvature_ff_ilos",
    "stanley",
    "aggressive_ff",
    "geometric_ff",
    "pure_pursuit",
    "smc_ilos",
    "smc_geometric",
    "frenet_ff",
    "backstepping",
    "contouring",
)


@dataclass
class ControlOutput:
    """References for velocity MPC and logging."""

    u_cmd: float
    r_ref: float
    v_ref: float
    nu_horizon: np.ndarray  # (N+1, 3)
    xte: float
    xte_signed: float
    path_idx: int
    desired_heading: float
    kappa: float


class ControlLaw:
    def __init__(self, cfg: dict, path: Sequence[PathSample], closed: bool, dt: float, horizon: int):
        self.cfg = cfg.get("control", cfg)
        self._full_cfg = cfg
        self.path = list(path)
        self.closed = closed
        self.dt = dt
        self.horizon = horizon
        self.approach = str(self.cfg.get("approach", "curvature_ff_ilos")).lower()
        self._path_idx = 0
        self._int_ey = 0.0
        g = cfg.get("guidance", {})
        self._max_r = float(g.get("max_yaw_rate_rad_s", 0.65))
        self._k_cte = float(g.get("stanley_k", 1.2))
        self._k_lat_v = float(g.get("lateral_v_gain", 0.0))
        self._kappa_u_scale = float(g.get("kappa_speed_scale", 2.5))
        self._kappa_ff_gain = float(g.get("kappa_ff_gain", 1.0))
        self._pp_steps = int(g.get("pure_pursuit_steps", 5))
        self._use_ilos_heading = bool(self.cfg.get("use_ilos_heading", True))
        # SMC / Frenet / backstepping gains
        self._smc_k_r = float(g.get("smc_k_r", 1.4))
        self._smc_k_v = float(g.get("smc_k_v", 0.9))
        self._smc_phi = float(g.get("smc_phi", 0.12))
        self._smc_lambda_i = float(g.get("smc_lambda_i", 0.35))
        self._los_delta = float(g.get("los_delta_m", 2.0))
        self._frenet_k_y = float(g.get("frenet_k_y", 1.0))
        self._frenet_k_psi = float(g.get("frenet_k_psi", 1.2))
        self._bs_k1 = float(g.get("backstep_k1", 1.1))
        self._bs_k2 = float(g.get("backstep_k2", 2.0))
        self._contour_k_d = float(g.get("contour_k_d", 1.3))
        self._contour_k_dd = float(g.get("contour_k_dd", 0.5))
        self._ilos = None
        self._heading_pid = None

    def attach_ilos(self, ilos, heading_pid) -> None:
        self._ilos = ilos
        self._heading_pid = heading_pid

    def reset_index(self, idx: int) -> None:
        self._path_idx = idx
        self._int_ey = 0.0

    def _cruise(self, path_cfg: dict) -> float:
        return float(path_cfg.get("cruise_speed_mps", 0.35))

    def _speed_from_curvature(self, cruise: float, kappa: float, xte: float = 0.0) -> float:
        allow_rev = bool(self.cfg.get("allow_reverse", False))
        xte_scale = float(np.exp(-2.2 * min(abs(xte), 1.5)))
        u = cruise * max(0.3, xte_scale) / (1.0 + self._kappa_u_scale * abs(kappa))
        if not allow_rev:
            u = max(0.12, u)
        return float(u)

    def _geometric_errors(
        self, x: float, y: float, psi: float, path_cfg: dict
    ) -> Tuple[float, float, int, float]:
        idx = closest_index(self.path, x, y)
        if path_cfg.get("monotonic_progress", False):
            self._path_idx = max(self._path_idx, idx)
        else:
            self._path_idx = idx
        xte_s = signed_cross_track_error(self.path, x, y)
        kappa = path_curvature(self.path, self._path_idx, self.closed)
        return abs(xte_s), xte_s, self._path_idx, kappa

    def _heading_target(
        self, p: PathSample, xte_s: float, u_cmd: float, ilos_heading: Optional[float]
    ) -> float:
        if self.approach in ("baseline_ilos_velocity", "baseline") and ilos_heading is not None:
            return ilos_heading
        if (
            self._use_ilos_heading
            and ilos_heading is not None
            and self.approach
            not in ("pure_geometric", "geometric_ff", "smc_geometric", "frenet_ff", "backstepping", "contouring")
        ):
            return ilos_heading
        return p.psi

    def _horizon(
        self,
        idx: int,
        step_m: float,
        u_cmd: float,
        r_ref: float,
        v_ref: float,
        r_fb_scale: float = 0.0,
        r_fb: float = 0.0,
    ) -> np.ndarray:
        nu_h = velocity_horizon_reference(
            self.path, idx, self.horizon, self.dt, step_m, u_cmd, self.closed, self._max_r
        )
        if abs(v_ref) > 1e-6:
            nu_h[:, 1] = v_ref
        if r_fb_scale != 0.0:
            nu_h[:, 2] = np.clip(nu_h[:, 2] + r_fb_scale * r_fb, -self._max_r, self._max_r)
        return nu_h

    def _los_heading(self, p: PathSample, e_d: float, u_cmd: float) -> float:
        """Integral LOS correction on path tangent."""
        u_s = max(abs(u_cmd), 0.15)
        return _wrap(p.psi + math.atan2(-self._smc_lambda_i * e_d - self._int_ey * 0.0, self._los_delta + u_s))

    def _smc_rates(
        self, psi_des: float, psi: float, e_d: float, u_cmd: float, kappa: float
    ) -> Tuple[float, float]:
        """Boundary-layer SMC on lateral and heading surfaces."""
        self._int_ey += e_d * self.dt
        self._int_ey = float(np.clip(self._int_ey, -3.0, 3.0))
        s_y = e_d + self._smc_lambda_i * self._int_ey
        s_psi = _wrap(psi_des - psi)
        r_ff = self._kappa_ff_gain * u_cmd * kappa
        r_ref = r_ff + self._smc_k_r * _sat(s_psi, self._smc_phi)
        v_ref = -self._smc_k_v * _sat(s_y, self._smc_phi)
        return r_ref, v_ref

    def compute(
        self,
        x: float,
        y: float,
        psi: float,
        path_cfg: dict,
        ilos_heading: Optional[float] = None,
    ) -> ControlOutput:
        cruise = self._cruise(path_cfg)
        xte, xte_s, idx, kappa = self._geometric_errors(x, y, psi, path_cfg)
        p = self.path[idx]
        step_m = float(path_cfg.get("resample_step_m", 0.4))
        psi_des = self._heading_target(p, xte_s, 0.0, ilos_heading)
        u_cmd = self._speed_from_curvature(cruise, kappa, xte)
        e_d = xte_s
        psi_e = _wrap(p.psi - psi)
        u_s = max(abs(u_cmd), 0.15)

        r_ref = 0.0
        v_ref = 0.0
        nu_h: np.ndarray

        if self.approach in ("baseline_ilos_velocity", "baseline"):
            psi_des = ilos_heading if ilos_heading is not None else p.psi
            r_pid = self._heading_pid.step(psi_des, psi) if self._heading_pid else 0.0
            r_ref = float(np.clip(r_pid, -self._max_r, self._max_r))
            nu_h = np.tile([u_cmd, 0.0, r_ref], (self.horizon + 1, 1))

        elif self.approach in ("curvature_ff", "curvature_ff_ilos", "h1_curvature_ff"):
            psi_des = self._heading_target(p, xte_s, u_cmd, ilos_heading)
            r_ff = self._kappa_ff_gain * u_cmd * kappa
            r_fb = self._heading_pid.step(psi_des, psi) if self._heading_pid else 0.0
            r_ref = float(np.clip(r_ff + r_fb, -self._max_r, self._max_r))
            lat = float(self.cfg.get("lateral_v_gain", self._k_lat_v))
            v_ref = float(np.clip(-lat * e_d, -0.35, 0.35))
            nu_h = self._horizon(idx, step_m, u_cmd, r_ref, v_ref, 0.3, r_fb)

        elif self.approach in ("stanley", "h2_stanley"):
            psi_des = p.psi
            psi_stanley = math.atan2(self._k_cte * e_d, u_s) + p.psi
            r_ref = float(np.clip(_wrap(psi_stanley - psi) / self.dt, -self._max_r, self._max_r))
            r_ref = float(np.clip(0.7 * u_cmd * kappa + 0.3 * r_ref, -self._max_r, self._max_r))
            v_ref = float(np.clip(-self._k_lat_v * e_d, -0.3, 0.3))
            nu_h = self._horizon(idx, step_m, u_cmd, r_ref, v_ref)

        elif self.approach in ("pure_pursuit", "pp_mpc", "h8_pure_pursuit"):
            la_idx = advance_path_index(self.path, idx, self._pp_steps, self.closed)
            tg = self.path[la_idx]
            psi_pp = math.atan2(tg.y - y, tg.x - x)
            r_pp = _wrap(psi_pp - psi) / max(self.dt, 0.05)
            r_ff = self._kappa_ff_gain * u_cmd * kappa
            r_fb = self._heading_pid.step(psi_des, psi) if self._heading_pid else 0.0
            r_ref = float(np.clip(0.5 * r_pp + 0.5 * r_ff + 0.25 * r_fb, -self._max_r, self._max_r))
            v_ref = float(np.clip(-0.5 * e_d, -0.25, 0.25))
            nu_h = self._horizon(idx, step_m, u_cmd, r_ref, v_ref)

        elif self.approach in ("geometric_ff", "h5_geometric"):
            psi_des = p.psi
            u_cmd = self._speed_from_curvature(cruise * 0.85, kappa, xte)
            r_ff = u_cmd * kappa
            r_cte = self._k_cte * e_d / u_s
            r_fb = self._heading_pid.step(psi_des, psi) if self._heading_pid else 0.0
            r_ref = float(np.clip(r_ff + r_cte + 0.5 * r_fb, -self._max_r, self._max_r))
            v_ref = float(np.clip(-1.0 * e_d, -0.4, 0.4))
            nu_h = self._horizon(idx, step_m, u_cmd, r_ref, v_ref)
            nu_h[:, 2] = np.clip(nu_h[:, 2] + r_cte, -self._max_r, self._max_r)

        elif self.approach in ("aggressive_ff", "h3_aggressive"):
            psi_des = self._heading_target(p, xte_s, u_cmd, ilos_heading)
            u_cmd = self._speed_from_curvature(cruise * 0.9, kappa, xte)
            r_ff = u_cmd * kappa
            r_fb = (
                2.0 * self._k_cte * e_d
                if self._heading_pid is None
                else self._heading_pid.step(psi_des, psi)
            )
            r_ref = float(np.clip(r_ff + r_fb, -self._max_r * 1.05, self._max_r * 1.05))
            v_ref = float(np.clip(-0.8 * e_d, -0.35, 0.35))
            nu_h = self._horizon(idx, step_m, u_cmd, r_ref, v_ref)

        elif self.approach in ("smc_ilos", "h12_smc_ilos"):
            psi_des = ilos_heading if ilos_heading is not None else self._los_heading(p, e_d, u_cmd)
            r_ref, v_ref = self._smc_rates(psi_des, psi, e_d, u_cmd, kappa)
            r_ref = float(np.clip(r_ref, -self._max_r, self._max_r))
            v_ref = float(np.clip(v_ref, -0.4, 0.4))
            nu_h = self._horizon(idx, step_m, u_cmd, r_ref, v_ref)

        elif self.approach in ("smc_geometric", "h13_smc_geometric"):
            psi_des = self._los_heading(p, e_d, u_cmd)
            r_ref, v_ref = self._smc_rates(psi_des, psi, e_d, u_cmd, kappa)
            r_ref = float(np.clip(r_ref, -self._max_r, self._max_r))
            v_ref = float(np.clip(v_ref, -0.4, 0.4))
            nu_h = self._horizon(idx, step_m, u_cmd, r_ref, v_ref)

        elif self.approach in ("frenet_ff", "h14_frenet_ff"):
            psi_des = p.psi
            r_ff = self._kappa_ff_gain * u_cmd * kappa
            r_ref = r_ff + self._frenet_k_y * e_d / u_s + self._frenet_k_psi * psi_e
            r_ref = float(np.clip(r_ref, -self._max_r, self._max_r))
            v_ref = float(np.clip(-self._frenet_k_y * 0.8 * e_d, -0.35, 0.35))
            nu_h = self._horizon(idx, step_m, u_cmd, r_ref, v_ref)

        elif self.approach in ("backstepping", "h15_backstepping"):
            psi_des = p.psi
            alpha = _wrap(p.psi - math.atan2(self._bs_k1 * e_d, self._los_delta + u_s))
            z2 = _wrap(psi - alpha)
            r_ref = (-self._bs_k2 * z2 - z2 / self.dt) + self._kappa_ff_gain * u_cmd * kappa
            r_ref = float(np.clip(r_ref, -self._max_r, self._max_r))
            v_ref = float(np.clip(-0.7 * e_d, -0.3, 0.3))
            nu_h = self._horizon(idx, step_m, u_cmd, r_ref, v_ref)

        elif self.approach in ("contouring", "h16_contouring"):
            psi_des = p.psi
            e_d_dot = u_cmd * math.sin(_wrap(psi - p.psi))
            r_ff = self._kappa_ff_gain * u_cmd * kappa
            r_ref = r_ff + self._contour_k_d * e_d + self._contour_k_dd * e_d_dot
            r_ref = float(np.clip(r_ref, -self._max_r, self._max_r))
            v_ref = float(np.clip(-self._contour_k_d * 0.9 * e_d, -0.35, 0.35))
            nu_h = self._horizon(idx, step_m, u_cmd, r_ref, v_ref)

        else:
            psi_des = self._heading_target(p, xte_s, u_cmd, ilos_heading)
            r_ff = self._kappa_ff_gain * u_cmd * kappa
            r_fb = self._heading_pid.step(psi_des, psi) if self._heading_pid else 0.0
            r_ref = float(np.clip(r_ff + r_fb, -self._max_r, self._max_r))
            v_ref = float(np.clip(-0.5 * e_d, -0.2, 0.2))
            nu_h = self._horizon(idx, step_m, u_cmd, r_ref, v_ref)

        return ControlOutput(
            u_cmd=u_cmd,
            r_ref=r_ref,
            v_ref=v_ref,
            nu_horizon=nu_h,
            xte=xte,
            xte_signed=xte_s,
            path_idx=idx,
            desired_heading=psi_des,
            kappa=kappa,
        )
