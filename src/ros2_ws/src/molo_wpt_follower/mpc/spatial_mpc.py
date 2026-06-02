"""Spatial trajectory-tracking MPC (OSQP) on [x, y, psi, u, v, r]."""

from __future__ import annotations

import math
from typing import Optional, Tuple

import numpy as np
import osqp
from scipy import sparse

from boat_model import BoatParameters, LinearizedUSV6DOF


def _wrap(a: float) -> float:
    while a > math.pi:
        a -= 2.0 * math.pi
    while a <= -math.pi:
        a += 2.0 * math.pi
    return a


class SpatialMPC:
    def __init__(
        self,
        boat: BoatParameters,
        *,
        horizon: int = 20,
        dt: float = 0.1,
        Q_diag: Optional[np.ndarray] = None,
        R_diag: Optional[np.ndarray] = None,
        Q_terminal_scale: float = 5.0,
    ):
        self.model = LinearizedUSV6DOF(boat, dt=dt)
        self.N = horizon
        self.dt = dt
        self.nx = 6
        self.nu = 2
        self.p = boat

        if Q_diag is None:
            Q_diag = np.array([500.0, 500.0, 120.0, 2.0, 2.0, 40.0])
        if R_diag is None:
            R_diag = np.array([0.05, 0.05])

        self.Q = np.diag(Q_diag)
        self.R = np.diag(R_diag)
        self.Qf = Q_terminal_scale * self.Q

        self._prob: Optional[osqp.OSQP] = None
        self._last_u = np.zeros(2)

    def _build_qp(self, Ad: np.ndarray, Bd: np.ndarray) -> None:
        N, nx, nu = self.N, self.nx, self.nu
        n_vars = (N + 1) * nx + N * nu
        n_eq = (N + 1) * nx
        n_ineq = (N + 1) * nx + N * nu

        P_blocks = [sparse.kron(sparse.eye(N), self.Q), self.Qf, sparse.kron(sparse.eye(N), self.R)]
        self.P = sparse.block_diag(P_blocks, format="csc")
        self.q = np.zeros(n_vars)

        Ad_block = sparse.kron(sparse.eye(N + 1), -sparse.eye(nx)) + sparse.kron(
            sparse.eye(N + 1, k=-1), sparse.csc_matrix(Ad)
        )
        Bd_rows = []
        for i in range(N + 1):
            if i == 0:
                Bd_rows.append(sparse.csc_matrix((nx, N * nu)))
            else:
                Bd_rows.append(
                    sparse.hstack(
                        [
                            sparse.csc_matrix((nx, (i - 1) * nu)),
                            sparse.csc_matrix(Bd),
                            sparse.csc_matrix((nx, (N - i) * nu)),
                        ]
                    )
                )
        Bd_block = sparse.vstack(Bd_rows)
        self.A_eq = sparse.hstack([Ad_block, Bd_block])

        tmax = self.p.max_thrust_N
        u_min = np.array([-tmax, -tmax])
        u_max = np.array([tmax, tmax])
        state_lb = np.array([-1e6, -1e6, -math.pi, -2.0, -1.5, -1.2])
        state_ub = np.array([1e6, 1e6, math.pi, 3.0, 1.5, 1.2])

        self.l = np.concatenate(
            [np.zeros(n_eq), np.tile(state_lb, N + 1), np.tile(u_min, N)]
        )
        self.u = np.concatenate(
            [np.zeros(n_eq), np.tile(state_ub, N + 1), np.tile(u_max, N)]
        )
        self.A = sparse.vstack([self.A_eq, sparse.eye(n_ineq)], format="csc")
        self._n_vars = n_vars

    def solve(
        self, z_meas: np.ndarray, z_ref: np.ndarray
    ) -> Tuple[np.ndarray, float, np.ndarray]:
        """
        z_meas, z_ref: shape (6,) or (N+1, 6).
        Returns (thrust [T_l,T_r] in N, cross_track_hint, predicted states).
        """
        z_meas = np.asarray(z_meas, dtype=float).reshape(6)
        if z_ref.ndim == 1:
            z_ref = np.tile(z_ref.reshape(1, 6), (self.N + 1, 1))
        else:
            z_ref = np.asarray(z_ref, dtype=float).reshape(self.N + 1, 6)

        for k in range(self.N + 1):
            z_ref[k, 2] = z_meas[2] + _wrap(z_ref[k, 2] - z_meas[2])

        Ad, Bd = self.model.discretize(z_meas)
        self._build_qp(Ad, Bd)

        N, nx = self.N, self.nx
        n_vars = self._n_vars

        self.l[:nx] = z_meas
        self.u[:nx] = z_meas

        Q_full = sparse.block_diag([self.Q] * N + [self.Qf])
        q_state = -Q_full @ z_ref.reshape(-1, order="C")
        self.q[: (N + 1) * nx] = q_state

        self._prob = osqp.OSQP()
        self._prob.setup(
            P=self.P,
            q=self.q,
            A=self.A,
            l=self.l,
            u=self.u,
            verbose=False,
            warm_start=True,
            max_iter=4000,
            eps_abs=1e-4,
            eps_rel=1e-4,
            polish=False,
        )
        res = self._prob.solve()
        if res.info.status not in ("solved", "solved inaccurate"):
            thrust = self.model.steady_state_thrust(max(z_ref[0, 3], 0.3), 0.0, 0.0)
            return thrust, float("inf"), z_meas.reshape(1, 6)

        u0 = res.x[(N + 1) * nx : (N + 1) * nx + self.nu].copy()
        ff = self.model.steady_state_thrust(z_ref[0, 3], z_ref[0, 4], z_ref[0, 5])
        thrust = np.clip(u0 + ff, -self.p.max_thrust_N, self.p.max_thrust_N)
        pred = res.x[: (N + 1) * nx].reshape(N + 1, nx)
        self._last_u = thrust
        xte = math.hypot(z_meas[0] - z_ref[0, 0], z_meas[1] - z_ref[0, 1])
        return thrust, xte, pred
