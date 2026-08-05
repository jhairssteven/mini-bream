"""Angle utilities for heading estimation."""

from __future__ import annotations

import math


def normalize_angle(angle_rad: float) -> float:
    """Wrap angle to [-pi, pi]."""
    return math.atan2(math.sin(angle_rad), math.cos(angle_rad))


def yaw_from_quaternion(qx: float, qy: float, qz: float, qw: float) -> float:
    """Extract yaw (about +Z) from a quaternion (REP-103 / tf_transformations convention)."""
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny_cosp, cosy_cosp)
