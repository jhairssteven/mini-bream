"""Local tangent-plane conversions for GPS map origin."""

from __future__ import annotations

import math

# WGS84 semi-major axis (m).
_WGS84_A = 6378137.0


def latlon_to_local_enu(
    lat_deg: float,
    lon_deg: float,
    alt_m: float,
    origin_lat_deg: float,
    origin_lon_deg: float,
    origin_alt_m: float,
) -> tuple[float, float, float]:
    """Convert WGS84 lat/lon/alt to local ENU meters relative to an origin."""
    dlat = math.radians(lat_deg - origin_lat_deg)
    dlon = math.radians(lon_deg - origin_lon_deg)
    cos_lat = math.cos(math.radians(origin_lat_deg))
    east = dlon * cos_lat * _WGS84_A
    north = dlat * _WGS84_A
    up = alt_m - origin_alt_m
    return east, north, up
