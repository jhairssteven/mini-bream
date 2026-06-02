"""WAMV 3-DOF hydrodynamics + 6-DOF pose kinematics (fixed parameters).

Model (body frame velocities u,v,r; world pose x,y,psi):

  [x_dot, y_dot, psi_dot] = R(psi) @ [u, v] + [0, 0, r]
  M @ [u_dot, v_dot, r_dot] = B_thruster @ [T_l, T_r] - C(nu)*nu - D*nu - d_nl(nu)

Change `boat:` in params.yaml to retune for another vessel with the same structure.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Tuple

import numpy as np
from scipy.linalg import expm


@dataclass
class BoatParameters:
    """Hydrodynamic and geometry parameters (WAMV-16 / LINC nominal defaults)."""

  # Rigid body
    mass_kg: float = 320.4012
    yaw_inertia_kgm2: float = 570.4965
    cg_x_m: float = 0.0
    cg_y_m: float = 0.0

    # Added mass (diagonal)
    added_mass_surge_kg: float = 21.5914
    added_mass_sway_kg: float = 43.1522
    added_mass_yaw_kgm2: float = 5.5560
    added_mass_y_r_kgm: float = 8.0
    added_mass_n_v_kgm: float = 10.0

    # Linear damping
    damp_surge_Ns_per_m: float = 124.8553
    damp_sway_Ns_per_m: float = 138.1975
    damp_yaw_Nms_per_rad: float = 789.8261

    # Quadratic damping
    damp_surge_quad_Ns2_per_m2: float = 144.4452
    damp_sway_quad_Ns2_per_m2: float = 119.0375
    damp_yaw_quad_Nms2_per_rad2: float = 243.6211

    # Thrusters (differential)
    thruster_arm_m: float = 0.7562
    thruster_eff_left: float = 1.0
    thruster_eff_right: float = 1.0
    max_thrust_N: float = 250.0

    @classmethod
    def from_dict(cls, d: dict) -> "BoatParameters":
        return cls(**{k: v for k, v in d.items() if k in cls.__dataclass_fields__})


class LinearizedUSV6DOF:
    """6-state [x, y, psi, u, v, r] model linearized at operating point z0."""

    def __init__(self, boat: BoatParameters, dt: float = 0.1):
        self.p = boat
        self.dt = dt
        self._build_mass_and_allocation()

    def _build_mass_and_allocation(self) -> None:
        p = self.p
        self.M = np.diag(
            [
                p.mass_kg + p.added_mass_surge_kg,
                p.mass_kg + p.added_mass_sway_kg,
                p.yaw_inertia_kgm2 + p.added_mass_yaw_kgm2,
            ]
        )
        self.Minv = np.linalg.inv(self.M)
        self.D = np.diag([p.damp_surge_Ns_per_m, p.damp_sway_Ns_per_m, p.damp_yaw_Nms_per_rad])
        L = p.thruster_arm_m
        el, er = p.thruster_eff_left, p.thruster_eff_right
        self.B_thr = np.array([[el, er], [0.0, 0.0], [-L * el, L * er]])
        self.B_accel = self.Minv @ self.B_thr  # 3x2: thrust -> [u_dot, v_dot, r_dot]

    def _coriolis(self, u: float, v: float, r: float) -> np.ndarray:
        p = self.p
        c13 = p.mass_kg * (p.cg_x_m * r + v) + p.added_mass_sway_kg * v + 0.5 * (
            p.added_mass_y_r_kgm + p.added_mass_n_v_kgm
        ) * r
        c23 = p.mass_kg * (u - p.cg_y_m * r) - p.added_mass_surge_kg * u - 0.5 * (
            p.added_mass_yaw_kgm2 - p.added_mass_surge_kg
        ) * r
        return np.array([[0.0, 0.0, -c13], [0.0, 0.0, c23], [c13, -c23, 0.0]])

    def _nonlinear_damp_vec(self, u: float, v: float, r: float) -> np.ndarray:
        p = self.p
        return np.array(
            [
                p.damp_surge_quad_Ns2_per_m2 * abs(u) * u,
                p.damp_sway_quad_Ns2_per_m2 * abs(v) * v,
                p.damp_yaw_quad_Nms2_per_rad2 * abs(r) * r,
            ]
        )

    def continuous_matrices(self, z: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        """Return A_c (6x6), B_c (6x2) for dz = A_c z + B_c u."""
        _, _, psi, u, v, r = z
        c, s = float(np.cos(psi)), float(np.sin(psi))

        # Kinematic block
        A = np.zeros((6, 6))
        A[0, 2] = -u * s - v * c
        A[0, 3] = c
        A[0, 4] = -s
        A[1, 2] = u * c - v * s
        A[1, 3] = s
        A[1, 4] = c
        A[2, 5] = 1.0

        nu = np.array([u, v, r])
        C = self._coriolis(u, v, r)
        d_nl = self._nonlinear_damp_vec(u, v, r)
        dD = np.diag(
            [
                self.p.damp_surge_quad_Ns2_per_m2 * abs(u),
                self.p.damp_sway_quad_Ns2_per_m2 * abs(v),
                self.p.damp_yaw_quad_Nms2_per_rad2 * abs(r),
            ]
        )
        A_vel = -self.Minv @ (C + self.D + dD)
        A[3:6, 3:6] = A_vel

        B = np.zeros((6, 2))
        B[3:6, :] = self.B_accel
        return A, B

    def discretize(self, z: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        A_c, B_c = self.continuous_matrices(z)
        n, m = 6, 2
        M = np.zeros((n + m, n + m))
        M[:n, :n] = A_c
        M[:n, n:] = B_c
        Md = expm(M * self.dt)
        return Md[:n, :n], Md[:n, n:]

    def steady_state_thrust(self, u_ref: float, v_ref: float = 0.0, r_ref: float = 0.0) -> np.ndarray:
        """Feedforward [T_l, T_r] to hold reference body velocities."""
        nu = np.array([u_ref, v_ref, r_ref])
        C = self._coriolis(u_ref, v_ref, r_ref)
        force = C @ nu + self.D @ nu + self._nonlinear_damp_vec(u_ref, v_ref, r_ref)
        thrust = np.linalg.pinv(self.B_thr) @ force
        tmax = self.p.max_thrust_N
        return np.clip(thrust, -tmax, tmax)
