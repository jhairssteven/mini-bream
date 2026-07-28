"""RViz helpers for molo MPC follower."""

from __future__ import annotations

import math
from typing import Sequence

from geometry_msgs.msg import PoseStamped, Quaternion
from nav_msgs.msg import Path
from std_msgs.msg import Float32, Header
from visualization_msgs.msg import Marker

from path_reference import PathSample


def _yaw_q(yaw: float) -> Quaternion:
    q = Quaternion()
    q.z = math.sin(yaw * 0.5)
    q.w = math.cos(yaw * 0.5)
    return q


class MpcVisualizer:
    def __init__(self, node, frame_id: str, topics: dict):
        self.node = node
        self.frame_id = frame_id
        t = topics
        self.ref_pub = node.create_publisher(Path, t.get("ref_path", "/molo_mpc/ref_path"), 10)
        self.pred_pub = node.create_publisher(Path, t.get("pred_path", "/molo_mpc/pred_path"), 10)
        self.pose_pub = node.create_publisher(
            PoseStamped, t.get("vehicle_pose", "/molo_mpc/vehicle_pose"), 10
        )
        self.xte_pub = node.create_publisher(
            Float32, t.get("cross_track_error", "/molo_mpc/cross_track_error"), 10
        )
        self._traj = Path()
        self._traj.header.frame_id = frame_id
        self.trav_pub = node.create_publisher(
            Path, t.get("traversed_path", "/molo_mpc/traversed_path"), 10
        )

    def _hdr(self, stamp=None) -> Header:
        h = Header()
        h.stamp = stamp if stamp is not None else self.node.get_clock().now().to_msg()
        h.frame_id = self.frame_id
        return h

    def publish_ref(self, path: Sequence[PathSample], stamp=None) -> None:
        msg = Path()
        msg.header = self._hdr(stamp)
        for p in path:
            ps = PoseStamped()
            ps.header = msg.header
            ps.pose.position.x = p.x
            ps.pose.position.y = p.y
            ps.pose.orientation = _yaw_q(p.psi)
            msg.poses.append(ps)
        self.ref_pub.publish(msg)

    def publish_pred(self, states) -> None:
        msg = Path()
        msg.header = self._hdr()
        for row in states:
            ps = PoseStamped()
            ps.header = msg.header
            ps.pose.position.x = float(row[0])
            ps.pose.position.y = float(row[1])
            ps.pose.orientation = _yaw_q(float(row[2]))
            msg.poses.append(ps)
        self.pred_pub.publish(msg)

    def publish_pose(self, x: float, y: float, psi: float) -> None:
        ps = PoseStamped()
        ps.header = self._hdr()
        ps.pose.position.x = x
        ps.pose.position.y = y
        ps.pose.orientation = _yaw_q(psi)
        self.pose_pub.publish(ps)

    def publish_xte(self, xte: float) -> None:
        self.xte_pub.publish(Float32(data=float(xte)))

    def publish_traversed(self, x: float, y: float, psi: float, min_step: float = 0.3) -> None:
        if self._traj.poses:
            last = self._traj.poses[-1].pose.position
            if math.hypot(x - last.x, y - last.y) < min_step:
                return
        ps = PoseStamped()
        ps.header = self._hdr()
        ps.pose.position.x = x
        ps.pose.position.y = y
        ps.pose.orientation = _yaw_q(psi)
        self._traj.poses.append(ps)
        out = Path()
        out.header = ps.header
        out.poses = self._traj.poses
        self.trav_pub.publish(out)
