"""Parse u-blox NAV-RELPOSNED9 into ENU heading measurements."""

from __future__ import annotations

import math
from dataclasses import dataclass

from ublox_msgs.msg import NavRELPOSNED9

# u-blox NAV-RELPOSNED9 flags (HPG 9, interface spec).
FLAGS_REL_POS_VALID = 4
FLAGS_REL_POS_HEAD_VALID = 256


@dataclass(frozen=True)
class RelPosHeadingMeasurement:
    """Heading measurement in ROS ENU convention (yaw about +Z, CCW from East)."""

    yaw_enu_rad: float
    variance_rad2: float
    heading_valid: bool
    position_valid: bool


def ned_heading_deg_to_enu_yaw_rad(heading_ned_deg: float) -> float:
    """Convert u-blox NED azimuth (0=N, 90=E, clockwise) to ROS ENU yaw."""
    return math.pi / 2.0 - math.radians(heading_ned_deg)


def heading_accuracy_variance_rad2(acc_heading: int, min_variance_rad2: float = 1e-6) -> float:
    """acc_heading is u-blox accuracy in units of 1e-5 deg (1-sigma)."""
    sigma_rad = math.radians(acc_heading * 1e-5)
    return max(sigma_rad * sigma_rad, min_variance_rad2)


def parse_nav_relposned9(
    msg: NavRELPOSNED9,
    min_variance_rad2: float = 1e-6,
) -> RelPosHeadingMeasurement:
    heading_ned_deg = msg.rel_pos_heading * 1e-5
    return RelPosHeadingMeasurement(
        yaw_enu_rad=ned_heading_deg_to_enu_yaw_rad(heading_ned_deg),
        variance_rad2=heading_accuracy_variance_rad2(msg.acc_heading, min_variance_rad2),
        heading_valid=bool(msg.flags & FLAGS_REL_POS_HEAD_VALID),
        position_valid=bool(msg.flags & FLAGS_REL_POS_VALID),
    )
