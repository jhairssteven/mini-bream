#!/usr/bin/env python3
"""Filter boat hull and LiDAR mount hardware from RoboSense point clouds."""

from __future__ import annotations

import os
from typing import Optional

import numpy as np
import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Point, Pose, Quaternion, Vector3
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2 as pc2
from std_msgs.msg import ColorRGBA, Header
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

from frontseat.self_filter import (
    build_keep_mask,
    load_filter_boxes,
    read_xyz_grid,
)


class SelfFilterNode(Node):
    def __init__(self) -> None:
        super().__init__('self_filter')

        default_config = os.path.join(
            get_package_share_directory('frontseat'),
            'config', 'self_filter', 'blueboat_self_filter.yaml',
        )
        self.declare_parameter('config_path', default_config)
        self.declare_parameter('debug_enabled', False)

        config_path = self.get_parameter('config_path').get_parameter_value().string_value
        if not config_path:
            config_path = default_config
        self._config = self._load_config(config_path)

        debug_cfg = self._config.get('debug', {})
        param_debug = self.get_parameter('debug_enabled').get_parameter_value().bool_value
        self._debug_enabled = param_debug or bool(debug_cfg.get('enabled', False))
        self._publish_markers = self._debug_enabled and bool(
            debug_cfg.get('publish_markers', debug_cfg.get('enabled', False)),
        )
        self._publish_removed = self._debug_enabled and bool(
            debug_cfg.get('publish_removed_cloud', False),
        )

        self._filter_frame = str(self._config.get('output_frame', 'base_link'))
        margin = float(self._config.get('filter_margin', 0.0))
        self._boxes = load_filter_boxes(self._config, margin)

        sensor_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=5,
        )
        marker_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            depth=1,
        )

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        self._filtered_publishers: dict[str, rclpy.publisher.Publisher] = {}
        self._removed_publishers: dict[str, rclpy.publisher.Publisher] = {}
        self._cloud_subscriptions = []
        for entry in self._config.get('point_clouds', []):
            input_topic = entry['input_topic']
            output_topic = entry['output_topic']
            self._filtered_publishers[input_topic] = self.create_publisher(
                PointCloud2, output_topic, sensor_qos,
            )
            if self._publish_removed:
                removed_topic = entry.get('removed_topic')
                if not removed_topic:
                    die_msg = (
                        f'removed_topic required for {input_topic} when '
                        'debug.publish_removed_cloud is enabled'
                    )
                    raise ValueError(die_msg)
                self._removed_publishers[input_topic] = self.create_publisher(
                    PointCloud2, removed_topic, sensor_qos,
                )
            self._cloud_subscriptions.append(self.create_subscription(
                PointCloud2,
                input_topic,
                lambda msg, topic=input_topic: self._cloud_callback(msg, topic),
                sensor_qos,
            ))

        self._marker_pub = None
        if self._publish_markers:
            markers_topic = str(
                debug_cfg.get('markers_topic', '/self_filter/debug_markers'),
            )
            self._marker_pub = self.create_publisher(MarkerArray, markers_topic, marker_qos)
            self._marker_timer = self.create_timer(1.0, self._publish_debug_markers)
            self._publish_debug_markers()

        self.get_logger().info(
            f'Loaded {len(self._boxes)} filter volume(s) from {config_path}; '
            f'filter_frame={self._filter_frame}; '
            f'debug_enabled={self._debug_enabled}; '
            f'debug_markers={self._publish_markers}; '
            f'debug_removed_cloud={self._publish_removed}'
        )

    def _load_config(self, config_path: str) -> dict:
        with open(config_path, 'r', encoding='utf-8') as config_file:
            return yaml.safe_load(config_file)

    def _lookup_transform_matrix(
        self,
        target_frame: str,
        source_frame: str,
        stamp: Time,
    ) -> Optional[np.ndarray]:
        if target_frame == source_frame:
            return np.eye(4, dtype=np.float64)

        try:
            transform = self._tf_buffer.lookup_transform(
                target_frame,
                source_frame,
                stamp,
                timeout=rclpy.duration.Duration(seconds=0.2),
            )
        except TransformException as exc:
            self.get_logger().warning(
                f'TF {source_frame} -> {target_frame} unavailable: {exc}',
                throttle_duration_sec=5.0,
            )
            return None

        translation = transform.transform.translation
        rotation = transform.transform.rotation
        matrix = _quaternion_to_matrix(
            rotation.x, rotation.y, rotation.z, rotation.w,
        )
        matrix[0, 3] = translation.x
        matrix[1, 3] = translation.y
        matrix[2, 3] = translation.z
        return matrix

    def _cloud_callback(self, msg: PointCloud2, input_topic: str) -> None:
        xyz = read_xyz_grid(msg)
        if xyz.size == 0:
            self._filtered_publishers[input_topic].publish(msg)
            return

        stamp = Time.from_msg(msg.header.stamp)
        transform = self._lookup_transform_matrix(
            self._filter_frame, msg.header.frame_id, stamp,
        )
        if transform is None:
            return

        valid = np.isfinite(xyz).all(axis=1)
        keep = np.zeros(len(xyz), dtype=bool)

        if np.any(valid):
            points_h = np.hstack((xyz[valid], np.ones((int(valid.sum()), 1), dtype=np.float64)))
            points_base = (transform @ points_h.T).T[:, :3]
            keep[valid] = build_keep_mask(points_base, self._boxes)

        filtered = _select_pointcloud2(msg, keep, msg.header.frame_id)
        if filtered is None:
            self.get_logger().error(
                'Point count mismatch while filtering; passing cloud through unmodified',
                throttle_duration_sec=5.0,
            )
            self._filtered_publishers[input_topic].publish(msg)
            return

        self._filtered_publishers[input_topic].publish(filtered)

        if self._publish_removed:
            removed_mask = valid & ~keep
            removed = _select_pointcloud2(msg, removed_mask, msg.header.frame_id)
            if removed is not None:
                self._removed_publishers[input_topic].publish(removed)

    def _publish_debug_markers(self) -> None:
        if self._marker_pub is None:
            return

        stamp = self.get_clock().now().to_msg()
        markers = MarkerArray()
        for index, box in enumerate(self._boxes):
            marker = Marker()
            marker.header = Header(stamp=stamp, frame_id=self._filter_frame)
            marker.ns = 'self_filter'
            marker.id = index
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            marker.pose = Pose(
                position=Point(
                    x=float(box.center[0]),
                    y=float(box.center[1]),
                    z=float(box.center[2]),
                ),
                orientation=_matrix_to_quaternion(box.rotation),
            )
            marker.scale = Vector3(
                x=float(2.0 * box.half_extents[0]),
                y=float(2.0 * box.half_extents[1]),
                z=float(2.0 * box.half_extents[2]),
            )
            marker.color = ColorRGBA(r=1.0, g=0.2, b=0.1, a=0.35)
            marker.lifetime.sec = 0
            markers.markers.append(marker)

        self._marker_pub.publish(markers)


def _select_pointcloud2(
    msg: PointCloud2, select: np.ndarray, frame_id: str,
) -> Optional[PointCloud2]:
    raw_points = list(pc2.read_points(msg, skip_nans=False))
    if len(raw_points) != len(select):
        return None

    selected_points = [point for point, picked in zip(raw_points, select) if picked]
    header = Header(stamp=msg.header.stamp, frame_id=frame_id)
    return pc2.create_cloud(header, msg.fields, selected_points)


def _quaternion_to_matrix(x: float, y: float, z: float, w: float) -> np.ndarray:
    from tf_transformations import quaternion_matrix

    return quaternion_matrix([x, y, z, w])


def _matrix_to_quaternion(rotation: np.ndarray) -> Quaternion:
    from tf_transformations import quaternion_from_matrix

    matrix = np.eye(4)
    matrix[:3, :3] = rotation
    q = quaternion_from_matrix(matrix)
    return Quaternion(x=float(q[0]), y=float(q[1]), z=float(q[2]), w=float(q[3]))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SelfFilterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
