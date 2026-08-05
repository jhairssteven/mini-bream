"""Build standard ROS messages for heading outputs."""

from __future__ import annotations

import numpy as np
import tf_transformations as tf
from geometry_msgs.msg import Quaternion
from sensor_msgs.msg import Imu, NavSatFix
from visualization_msgs.msg import Marker


def yaw_to_quaternion(yaw_rad: float) -> Quaternion:
    qx, qy, qz, qw = tf.quaternion_from_euler(0.0, 0.0, yaw_rad)
    return Quaternion(x=qx, y=qy, z=qz, w=qw)


def build_imu_msg(
    stamp,
    frame_id: str,
    yaw_rad: float,
    yaw_variance_rad2: float,
) -> Imu:
    msg = Imu()
    msg.header.stamp = stamp
    msg.header.frame_id = frame_id
    msg.orientation = yaw_to_quaternion(yaw_rad)
    # orientation_covariance is row-major 3x3; index 8 = yaw variance about Z.
    msg.orientation_covariance[0] = -1.0
    msg.orientation_covariance[4] = -1.0
    msg.orientation_covariance[8] = yaw_variance_rad2
    return msg


def build_heading_marker(
    stamp,
    frame_id: str,
    yaw_rad: float,
    *,
    marker_id: int = 0,
    namespace: str = 'heading',
    arrow_length: float = 3.0,
    shaft_width: float = 0.25,
    color: tuple[float, float, float, float] = (0.1, 0.8, 0.2, 1.0),
    position_offset: tuple[float, float, float] = (0.0, 0.0, 0.0),
    body_aligned: bool = False,
) -> Marker:
    """Build an RViz ARROW marker.

    By default yaw_rad is applied in frame_id (ENU geographic heading in that frame).
    With body_aligned=True, yaw_rad is a small body-frame offset from +X (bow); identity
  means the arrow points along the boat traversal axis.
    """
    marker = Marker()
    marker.header.stamp = stamp
    marker.header.frame_id = frame_id
    marker.ns = namespace
    marker.id = marker_id
    marker.type = Marker.ARROW
    marker.action = Marker.ADD
    marker.pose.position.x = position_offset[0]
    marker.pose.position.y = position_offset[1]
    marker.pose.position.z = position_offset[2]
    marker.pose.orientation = yaw_to_quaternion(yaw_rad)
    marker.scale.x = arrow_length
    marker.scale.y = shaft_width
    marker.scale.z = shaft_width
    marker.color.r = color[0]
    marker.color.g = color[1]
    marker.color.b = color[2]
    marker.color.a = color[3]
    return marker


def build_center_fix(
    stamp,
    frame_id: str,
    rover_fix: NavSatFix,
    base_fix: NavSatFix,
) -> NavSatFix:
    lat = (rover_fix.latitude + base_fix.latitude) / 2.0
    lon = (rover_fix.longitude + base_fix.longitude) / 2.0
    alt = (rover_fix.altitude + base_fix.altitude) / 2.0

    cov1 = np.array(rover_fix.position_covariance).reshape(3, 3)
    cov2 = np.array(base_fix.position_covariance).reshape(3, 3)
    cov_avg = (cov1 + cov2) / 2.0

    msg = NavSatFix()
    msg.header.stamp = stamp
    msg.header.frame_id = frame_id
    msg.status.status = rover_fix.status.status
    msg.status.service = rover_fix.status.service
    msg.latitude = lat
    msg.longitude = lon
    msg.altitude = alt
    msg.position_covariance = cov_avg.flatten().tolist()
    msg.position_covariance_type = rover_fix.position_covariance_type
    return msg
