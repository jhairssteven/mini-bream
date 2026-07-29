#!/usr/bin/env python3
"""Broadcast map/world odometry to TF and publish configured static frame bridges."""

from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster


def _quat_from_rpy(roll: float, pitch: float, yaw: float) -> tuple[float, float, float, float]:
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy
    return qx, qy, qz, qw


def _identity_transform(parent: str, child: str, stamp) -> TransformStamped:
    msg = TransformStamped()
    msg.header.stamp = stamp
    msg.header.frame_id = parent
    msg.child_frame_id = child
    msg.transform.rotation.w = 1.0
    return msg


def _static_transform(cfg: dict, stamp) -> TransformStamped:
    parent = str(cfg["parent"])
    child = str(cfg["child"])
    translation = cfg.get("translation", [0.0, 0.0, 0.0])
    if isinstance(translation, dict):
        tx = float(translation.get("x", 0.0))
        ty = float(translation.get("y", 0.0))
        tz = float(translation.get("z", 0.0))
    else:
        tx, ty, tz = (float(v) for v in translation)

    if "rotation_quaternion" in cfg:
        qx, qy, qz, qw = (float(v) for v in cfg["rotation_quaternion"])
    else:
        rpy = cfg.get("rotation_rpy", [0.0, 0.0, 0.0])
        qx, qy, qz, qw = _quat_from_rpy(*(float(v) for v in rpy))

    msg = TransformStamped()
    msg.header.stamp = stamp
    msg.header.frame_id = parent
    msg.child_frame_id = child
    msg.transform.translation.x = tx
    msg.transform.translation.y = ty
    msg.transform.translation.z = tz
    msg.transform.rotation.x = qx
    msg.transform.rotation.y = qy
    msg.transform.rotation.z = qz
    msg.transform.rotation.w = qw
    return msg


class TfBridgeNode(Node):
    def __init__(self, cfg: dict):
        super().__init__("molo_tf_bridge")
        self._world_frame = str(cfg.get("world_frame", "map"))
        aliases = cfg.get("parent_frame_aliases", ["world", "odom"])
        self._parent_aliases = {str(a) for a in aliases}
        self._parent_aliases.add(self._world_frame)
        self._broadcast_odom_tf = bool(cfg.get("broadcast_odom_tf", True))
        self._child_frame_override = str(cfg.get("child_frame_override", "")).strip()

        self._tf_broadcaster = TransformBroadcaster(self)
        self._static_broadcaster = StaticTransformBroadcaster(self)

        stamp = self.get_clock().now().to_msg()
        static_msgs: list[TransformStamped] = []

        for alias in cfg.get("map_world_aliases", []):
            alias_name = str(alias).strip().lstrip("/")
            if alias_name and alias_name != self._world_frame:
                static_msgs.append(_identity_transform(self._world_frame, alias_name, stamp))

        for item in cfg.get("static_transforms", []):
            static_msgs.append(_static_transform(item, stamp))

        if static_msgs:
            self._static_broadcaster.sendTransform(static_msgs)
            self.get_logger().info(
                f"Published {len(static_msgs)} static transform(s): "
                + ", ".join(f"{m.header.frame_id}->{m.child_frame_id}" for m in static_msgs)
            )

        if self._broadcast_odom_tf:
            topic = str(cfg["odometry_topic"])
            qos = QoSProfile(
                depth=int(cfg.get("qos_depth", 10)),
                reliability=QoSReliabilityPolicy.BEST_EFFORT,
                durability=DurabilityPolicy.VOLATILE,
            )
            self.create_subscription(Odometry, topic, self._on_odom, qos)
            self.get_logger().info(
                f"Broadcasting TF from {topic!r} with world frame {self._world_frame!r}"
            )
        else:
            self.get_logger().info(
                f"Odom TF disabled; connected {self._world_frame!r} to Gazebo world frame alias(es)"
            )

    def _normalize_parent(self, frame_id: str) -> str:
        norm = frame_id.strip().lstrip("/")
        if norm in self._parent_aliases or not norm:
            return self._world_frame
        return norm

    def _on_odom(self, msg: Odometry) -> None:
        parent = self._normalize_parent(str(msg.header.frame_id))
        child = self._child_frame_override or str(msg.child_frame_id).strip().lstrip("/")
        if not child:
            return

        tf_msg = TransformStamped()
        tf_msg.header.stamp = msg.header.stamp
        tf_msg.header.frame_id = parent
        tf_msg.child_frame_id = child
        tf_msg.transform.translation.x = float(msg.pose.pose.position.x)
        tf_msg.transform.translation.y = float(msg.pose.pose.position.y)
        tf_msg.transform.translation.z = float(msg.pose.pose.position.z)
        tf_msg.transform.rotation = msg.pose.pose.orientation
        self._tf_broadcaster.sendTransform(tf_msg)


def main() -> None:
    parser = argparse.ArgumentParser(description="Odometry + static TF bridge for molo autonomy")
    parser.add_argument("config", help="Path to tf_bridge.json")
    args = parser.parse_args()

    config_path = Path(args.config)
    if not config_path.is_file():
        print(f"Config not found: {config_path}", file=sys.stderr)
        raise SystemExit(1)

    with open(config_path, encoding="utf-8") as f:
        cfg = json.load(f)

    _BRIDGE_DIR = Path(__file__).resolve().parent
    if str(_BRIDGE_DIR) not in sys.path:
        sys.path.insert(0, str(_BRIDGE_DIR))
    from ros_init import init_rclpy  # noqa: E402

    init_rclpy()
    node = TfBridgeNode(cfg)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
