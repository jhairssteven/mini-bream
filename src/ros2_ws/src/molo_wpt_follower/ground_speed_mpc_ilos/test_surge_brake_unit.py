#!/usr/bin/env python3
"""Unit checks for surge brake softener + surge-priority allocator (Test A)."""
from __future__ import annotations

import sys
from pathlib import Path

import numpy as np

PKG = Path(__file__).resolve().parent
sys.path.insert(0, str(PKG))

from inner_control import (  # noqa: E402
    Allocator,
    SurgePI,
    ThrusterMap,
    platform_thrust_commands,
)


def test_surge_overspeed_brake_clamped() -> None:
    """At Ud>0 and large overspeed, X must stay >= brake_x_min (no slam reverse)."""
    legacy = SurgePI(
        kp=9.12,
        ki=0.63,
        k_ff=3.92,
        dx_max=70.0,
        ff_overspeed_fade_mps=0.12,
        brake_x_min=-26.0,
        brake_kp_scale=1.0,
        u_lpf_tau_s=0.05,
    )
    patched = SurgePI(
        kp=9.12,
        ki=0.63,
        k_ff=3.92,
        dx_max=25.0,
        ff_overspeed_fade_mps=0.40,
        brake_x_min=-10.0,
        brake_kp_scale=0.4,
        u_lpf_tau_s=0.05,
    )
    ud, u_fast, dt = 0.35, 0.70, 0.1
    # Warm filters / allow slew to settle toward steady command
    x_leg = x_pat = 0.0
    for _ in range(40):
        x_leg = legacy.step(ud, u_fast, dt)
        x_pat = patched.step(ud, u_fast, dt)
    assert x_pat >= -10.0 - 1e-6, f"patched X={x_pat} below brake floor"
    assert x_leg < x_pat - 0.5, f"legacy should brake harder than patched ({x_leg} vs {x_pat})"
    print(f"OK overspeed brake: legacy X={x_leg:.2f}  patched X={x_pat:.2f}")


def test_allocator_priority_arms_at_low_x() -> None:
    """With x_min=1.5, yaw cannot reverse one thruster at X=3 N."""
    old = Allocator(surge_priority=True, surge_priority_x_min=8.0)
    new = Allocator(surge_priority=True, surge_priority_x_min=1.5)
    x, n = 3.0, 6.0  # large yaw vs surge
    tl_old, tr_old = old.allocate(x, n)
    tl_new, tr_new = new.allocate(x, n)
    assert (tl_old < 0) or (tr_old < 0), "legacy should allow opposing thruster"
    assert tl_new >= -1e-9 and tr_new >= -1e-9, f"patched opposing: {tl_new}, {tr_new}"
    print(f"OK allocator: legacy Tl/Tr={tl_old:.2f}/{tr_old:.2f}  patched={tl_new:.2f}/{tr_new:.2f}")


def test_forward_tracking_still_pushes() -> None:
    """When slow, patched loop still commands forward X."""
    pi = SurgePI(kp=40.0, ki=8.0, k_ff=35.0, dx_max=200.0, brake_x_min=-10.0)
    x = 0.0
    for _ in range(30):
        x = pi.step(0.35, 0.10, 0.1)
    assert x > 5.0, f"expected forward thrust, got {x}"
    print(f"OK underspeed accel: X={x:.2f}")


def test_sim_publishes_newtons_not_scaled_norm() -> None:
    """Gazebo thruster topics are force [N]; tiny X must not become ±20 via n*1000."""
    fmap = ThrusterMap(t_max=100.0, n_db=0.02, c1=100.0, c2=0.0)
    # Allocated ~0.5 N/side — old path deadband→n=0.02→pub=20
    cmd_l, cmd_r, fl, fr = platform_thrust_commands(
        0.5, 0.5, fmap, sim_enable=True
    )
    assert abs(cmd_l - 0.5) < 1e-9 and abs(cmd_r - 0.5) < 1e-9
    assert abs(fl - 0.5) < 1e-9 and abs(fr - 0.5) < 1e-9
    # Real boat still gets normalized via f^{-1} (with deadband floor)
    n_l, n_r, _, _ = platform_thrust_commands(0.5, 0.5, fmap, sim_enable=False)
    assert abs(n_l - 0.02) < 1e-9 and abs(n_r - 0.02) < 1e-9  # n_db bump
    # Full stick maps to plant max
    n_full, _, _, _ = platform_thrust_commands(100.0, 100.0, fmap, sim_enable=False)
    assert abs(n_full - 1.0) < 1e-9
    print(f"OK platform cmds: sim={cmd_l:.2f} N  real n={n_l:.3f}  full n={n_full:.1f}")


def test_plant_limits_allow_near_2mps() -> None:
    """X_max=200 N and T_max=100 N are consistent with U_ss≈1.95 m/s hydro."""
    x_max, t_max, b = 200.0, 100.0, 0.59
    assert abs(x_max - 2.0 * t_max) < 1e-9
    # Steady drag X = 25U + 40U^2 at U=1.94 → ~200 N
    u = 1.94
    x_drag = 25.0 * u + 40.0 * u * u
    assert 195.0 <= x_drag <= 205.0, f"unexpected drag X={x_drag}"
    n_diamond = b * t_max
    assert 55.0 <= n_diamond <= 60.0
    print(f"OK plant diamond: X_max={x_max:.0f}  T_max={t_max:.0f}  N_diam≈{n_diamond:.1f}")


def main() -> int:
    test_surge_overspeed_brake_clamped()
    test_allocator_priority_arms_at_low_x()
    test_forward_tracking_still_pushes()
    test_sim_publishes_newtons_not_scaled_norm()
    test_plant_limits_allow_near_2mps()
    print("Test A PASSED")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
