"""Unit tests for NAV-RELPOSNED heading parsing."""

import math

import pytest

from frontseat.heading.relpos import (
    ned_heading_deg_to_enu_yaw_rad,
    heading_accuracy_variance_rad2,
)


def test_ned_south_to_enu_yaw():
    # NED 180 deg (south) -> ENU yaw -90 deg.
    yaw = ned_heading_deg_to_enu_yaw_rad(180.0)
    assert yaw == pytest.approx(-math.pi / 2.0, abs=1e-6)


def test_heading_accuracy_variance():
    # 1 deg (1e5 in 1e-5 deg units) -> ~1 deg variance in rad^2
    var = heading_accuracy_variance_rad2(100_000)
    assert var == pytest.approx(math.radians(1.0) ** 2, rel=1e-3)
