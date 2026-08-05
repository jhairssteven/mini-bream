"""Unit tests for heading filters."""

import math

import pytest

from frontseat.heading.angles import normalize_angle, yaw_from_quaternion
from frontseat.heading.filters.ekf import HeadingEkf
from frontseat.heading.filters.low_pass import LowPassHeadingFilter


def test_normalize_angle():
    assert normalize_angle(3.5 * math.pi) == pytest.approx(-0.5 * math.pi, abs=1e-6)


def test_yaw_from_quaternion_identity():
    assert yaw_from_quaternion(0.0, 0.0, 0.0, 1.0) == pytest.approx(0.0, abs=1e-6)


def test_low_pass_initializes_on_first_sample():
    filt = LowPassHeadingFilter(alpha=0.5)
    assert filt.update(1.0) == 1.0
    assert filt.initialized


def test_low_pass_wraps_angles():
    filt = LowPassHeadingFilter(alpha=1.0)
    filt.update(math.pi - 0.1)
    yaw = filt.update(-math.pi + 0.1)
    assert abs(yaw) < 0.2


def test_ekf_gps_update_initializes():
    ekf = HeadingEkf(process_noise_var=0.01)
    state = ekf.update(0.5, 0.1)
    assert state is not None
    assert state.yaw_rad == pytest.approx(0.5, abs=1e-6)


def test_ekf_predict_then_update():
    ekf = HeadingEkf(process_noise_var=0.0002, max_variance_rad2=0.01, gps_variance_scale=0.05)
    ekf.update(0.0, 0.1)
    ekf.predict(0.1, 0.1)
    state = ekf.update(0.2, 0.1)
    assert state is not None
    assert 0.0 < state.yaw_rad < 0.2
