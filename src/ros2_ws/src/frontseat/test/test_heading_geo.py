"""Tests for local ENU conversion."""

from frontseat.heading.geo import latlon_to_local_enu


def test_origin_is_zero():
    east, north, up = latlon_to_local_enu(40.0, -86.0, 200.0, 40.0, -86.0, 200.0)
    assert east == 0.0
    assert north == 0.0
    assert up == 0.0


def test_north_displacement():
    east, north, _ = latlon_to_local_enu(40.0001, -86.0, 0.0, 40.0, -86.0, 0.0)
    assert east == 0.0
    assert north > 0.0
