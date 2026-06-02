"""Velocity-space MPC (u, v, r) with differential thrust — linearized from Fossen model."""

from __future__ import annotations

from typing import Optional, Tuple

import numpy as np
import osqp
from scipy import sparse

from boat_model import BoatParameters, LinearizedUSV6DOF


class VelocityMPC:
    def __init__(
        self,
        boat: BoatParameters,
        *,
        horizon: int = 12,
        dt: float = 0.1,
        Q_diag: Optional[np.ndarray] = None,
        R_diag: Optional[np.ndarray] = None,
        Q_terminal_scale: float = 5.0,
    ):
        self.model = LinearizedUSV6DOF(boat, dt=dt)
        self.N = horizon
        self.dt = dt
        self.nx = 3
        self.nu = 2
        self.p = boat

        if Q_diag is None:
            Q_diag = np.array([12.0, 0.5, 80.0])
        if R_diag is None:
            R_diag = np.array([0.02, 0.02])

        self.Q = np.diag(Q_diag)
        self.R = np.diag(R_diag)
        self.Qf = Q_terminal_scale * self.Q

    def _build_qp(self, Ad: np.ndarray, Bd: np.ndarray) -> None:
        N, nx, nu = self.N, self.nx, self.nu
        n_vars = (N + 1) * nx + N * nu
        n_eq = (N + 1) * nx
        n_ineq = (N + 1) * nx + N * nu

        self.P = sparse.block_diag(
            [sparse.kron(sparse.eye(N), self.Q), self.Qf, sparse.kron(sparse.eye(N), self.R)],
            format="csc",
        )
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
        state_lb = np.array([-0.5, -1.2, -1.0])
        state_ub = np.array([1.2, 1.2, 1.0])

        self.l = np.concatenate(
            [np.zeros(n_eq), np.tile(state_lb, N + 1), np.tile(u_min, N)]
        )
        self.u = np.concatenate(
            [np.zeros(n_eq), np.tile(state_ub, N + 1), np.tile(u_max, N)]
        )
        self.A = sparse.vstack([self.A_eq, sparse.eye(n_ineq)], format="csc")
        self._n_vars = n_vars

    def solve(
        self, z_pose_vel: np.ndarray, nu_ref: np.ndarray
    ) -> Tuple[np.ndarray, np.ndarray]:
        """
        z_pose_vel: [x,y,psi,u,v,r] for linearization.
        nu_ref: (3,) or (N+1, 3) reference [u, v, r].
        Returns (thrust [T_l,T_r], predicted nu trajectory).
        """
        z_pose_vel = np.asarray(z_pose_vel, dtype=float).reshape(6)
        nu_meas = z_pose_vel[3:6]

        if nu_ref.ndim == 1:
            nu_ref = np.tile(nu_ref.reshape(1, 3), (self.N + 1, 1))
        else:
            nu_ref = np.asarray(nu_ref, dtype=float).reshape(self.N + 1, 3)

        Ad_full, Bd_full = self.model.discretize(z_pose_vel)
        Ad = Ad_full[3:6, 3:6]
        Bd = Bd_full[3:6, :]
        self._build_qp(Ad, Bd)

        N, nx = self.N, self.nx
        self.l[:nx] = nu_meas
        self.u[:nx] = nu_meas

        Q_full = sparse.block_diag([self.Q] * N + [self.Qf])
        self.q[: (N + 1) * nx] = -Q_full @ nu_ref.reshape(-1, order="C")

        prob = osqp.OSQP()
        prob.setup(
            P=self.P,
            q=self.q,
            A=self.A,
            l=self.l,
            u=self.u,
            verbose=False,
            warm_start=True,
            max_iter=3000,
            eps_abs=1e-4,
            eps_rel=1e-4,
            polish=False,
        )
        res = prob.solve()
        if res.info.status not in ("solved", "solved inaccurate"):
            ff = self.model.steady_state_thrust(max(nu_ref[0, 0], 0.2), 0.0, nu_ref[0, 2])
            return ff, nu_meas.reshape(1, 3)

        u0 = res.x[(N + 1) * nx : (N + 1) * nx + self.nu].copy()
        ff = self.model.steady_state_thrust(nu_ref[0, 0], nu_ref[0, 1], nu_ref[0, 2])
        thrust = np.clip(u0 + ff, -self.p.max_thrust_N, self.p.max_thrust_N)
        pred = res.x[: (N + 1) * nx].reshape(N + 1, nx)
        return thrust, pred
