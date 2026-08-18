#!/usr/bin/env python3
"""ROS 2 node wrapping DBSCAN point-cloud noise filtering.

See ``README.md`` in this package for the pipeline, topics, and YAML knobs.
"""

from __future__ import annotations

import colorsys
import os
import time
from typing import Optional

import numpy as np
import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Point, Pose, Quaternion, Vector3
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import ColorRGBA, Header
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

from frontseat.lidar_filtering.pc_clustering.dbscan import (
    ClusterBox,
    ClusterParams,
    cluster_points,
)
from frontseat.lidar_filtering.ransac.plane import radial_mask
from frontseat.lidar_filtering.self_filter import read_xyz_grid


class PcClusteringNode(Node):
    def __init__(self) -> None:
        super().__init__('pc_clustering')

        default_config = os.path.join(
            get_package_share_directory('frontseat'),
            'config', 'lidar_filtering', 'pc_clustering', 'clustering.yaml',
        )
        self.declare_parameter('config_path', default_config)
        self.declare_parameter('debug_enabled', False)

        config_path = (
            self.get_parameter('config_path').get_parameter_value().string_value
        )
        if not config_path:
            config_path = default_config
        self._config = self._load_config(config_path)

        debug_cfg = self._config.get('debug', {})
        param_debug = (
            self.get_parameter('debug_enabled').get_parameter_value().bool_value
        )
        self._debug_enabled = param_debug or bool(
            debug_cfg.get('enabled', False),
        )
        self._publish_markers = self._debug_enabled and bool(
            debug_cfg.get('publish_markers', True),
        )
        self._publish_removed = self._debug_enabled and bool(
            debug_cfg.get('publish_removed_cloud', True),
        )

        self._input_topic = str(
            self._config.get('input_topic', '/rslidar_points/waterline_removed'),
        )
        self._output_topic = str(
            self._config.get('output_topic', '/rslidar_points/filtered'),
        )
        cluster_cfg = self._config.get('cluster', {})
        eps_default = cluster_cfg.get('cluster_tolerance', 0.45)
        self._params = ClusterParams(
            eps=float(cluster_cfg.get('eps', eps_default)),
            min_samples=int(cluster_cfg.get('min_samples', 4)),
            min_cluster_size=int(cluster_cfg.get('min_cluster_size', 10)),
            max_cluster_size=int(cluster_cfg.get('max_cluster_size', 0)),
            voxel_size=float(cluster_cfg.get('voxel_size', 0.0)),
            max_radius=float(self._config.get(
                'max_radius', cluster_cfg.get('max_radius', 4.0),
            )),
        )
        self._radius_frame = str(
            self._config.get('radius_frame', 'base_link'),
        )

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

        self._filtered_pub = self.create_publisher(
            PointCloud2, self._output_topic, sensor_qos,
        )
        self._removed_pub = None
        if self._publish_removed:
            removed_topic = str(
                self._config.get(
                    'removed_topic', '/rslidar_points/clustered_removed',
                ),
            )
            self._removed_pub = self.create_publisher(
                PointCloud2, removed_topic, sensor_qos,
            )
        self._marker_pub = None
        if self._publish_markers:
            markers_topic = str(self._config.get(
                'markers_topic', '/pc_clustering/debug_markers',
            ))
            self._marker_pub = self.create_publisher(
                MarkerArray, markers_topic, marker_qos,
            )

        self.create_subscription(
            PointCloud2, self._input_topic, self._cloud_callback, sensor_qos,
        )
        self.get_logger().info(
            f'DBSCAN clustering listening on {self._input_topic}; '
            f'output={self._output_topic}; '
            f'eps={self._params.eps:.2f} m; '
            f'min_samples={self._params.min_samples}; '
            f'min_cluster_size={self._params.min_cluster_size}; '
            f'voxel_size={self._params.voxel_size:.2f} m; '
            f'max_radius={self._params.max_radius:.2f} m; '
            f'radius_frame={self._radius_frame}; '
            f'debug_enabled={self._debug_enabled}; '
            f'config={config_path}'
        )
        if self._params.min_samples < 2:
            self.get_logger().warning(
                'cluster.min_samples < 2 makes every point a DBSCAN core; '
                'sparse leftover returns merge into one cluster and are kept'
            )

    def _load_config(self, config_path: str) -> dict:
        with open(config_path, 'r', encoding='utf-8') as config_file:
            return yaml.safe_load(config_file) or {}

    def _cloud_callback(self, msg: PointCloud2) -> None:
        t0 = time.perf_counter()
        xyz = read_xyz_grid(msg)
        header = Header(stamp=msg.header.stamp, frame_id=msg.header.frame_id)
        if xyz.size == 0:
            self._publish_empty(header)
            return

        valid = np.isfinite(xyz).all(axis=1)
        if not np.any(valid):
            self._publish_empty(header)
            return

        xyz_valid = xyz[valid]
        intensity = _read_intensity(msg)
        intensity_valid = None if intensity is None else intensity[valid]

        candidate = self._radius_mask(
            xyz_valid, msg.header.frame_id, Time.from_msg(msg.header.stamp),
        )
        result = cluster_points(
            xyz_valid, self._params, candidate_mask=candidate,
        )
        keep = result.keep_mask
        self._filtered_pub.publish(
            _create_xyzi_cloud(
                header,
                xyz_valid[keep],
                None if intensity_valid is None else intensity_valid[keep],
            )
        )
        if self._removed_pub is not None:
            removed = ~keep
            self._removed_pub.publish(
                _create_xyzi_cloud(
                    header,
                    xyz_valid[removed],
                    None if intensity_valid is None else
                    intensity_valid[removed],
                )
            )
        if self._marker_pub is not None:
            self._marker_pub.publish(_cluster_markers(header, result.boxes))

        elapsed_ms = (time.perf_counter() - t0) * 1000.0
        self.get_logger().info(
            f'in={len(xyz_valid)} candidates={result.candidate_count} '
            f'kept={result.kept_count} removed={result.removed_count} '
            f'clusters={result.cluster_count} '
            f'radius={self._params.max_radius:.1f} m time_ms={elapsed_ms:.1f}',
            throttle_duration_sec=2.0,
        )

    def _radius_mask(
        self,
        xyz: np.ndarray,
        source_frame: str,
        stamp: Time,
    ) -> np.ndarray:
        """XY cylinder in ``radius_frame``; ``max_radius <= 0`` keeps all."""
        if self._params.max_radius <= 0.0:
            return np.ones(len(xyz), dtype=bool)

        transform = self._lookup_transform_matrix(
            self._radius_frame, source_frame, stamp,
        )
        if transform is None:
            return radial_mask(xyz, self._params.max_radius)

        points_h = np.hstack((
            xyz, np.ones((len(xyz), 1), dtype=np.float64),
        ))
        xyz_radius = (transform @ points_h.T).T[:, :3]
        return radial_mask(xyz_radius, self._params.max_radius)

    def _lookup_transform_matrix(
        self,
        target_frame: str,
        source_frame: str,
        stamp: Time,
    ) -> Optional[np.ndarray]:
        if target_frame == source_frame:
            return np.eye(4, dtype=np.float64)

        for lookup_time in (stamp, Time()):
            try:
                transform = self._tf_buffer.lookup_transform(
                    target_frame,
                    source_frame,
                    lookup_time,
                    timeout=rclpy.duration.Duration(seconds=0.2),
                )
                return _transform_to_matrix(transform)
            except TransformException:
                continue

        self.get_logger().warning(
            f'TF {source_frame} -> {target_frame} unavailable; '
            'radius gate uses the cloud frame',
            throttle_duration_sec=5.0,
        )
        return None

    def _publish_empty(self, header: Header) -> None:
        empty = np.zeros((0, 3), dtype=np.float64)
        self._filtered_pub.publish(_create_xyzi_cloud(header, empty, None))
        if self._removed_pub is not None:
            self._removed_pub.publish(_create_xyzi_cloud(header, empty, None))
        if self._marker_pub is not None:
            self._marker_pub.publish(_cluster_markers(header, []))


def _cluster_markers(header: Header, boxes: list[ClusterBox]) -> MarkerArray:
    """AABB cube, wireframe, and label per kept cluster."""
    markers = MarkerArray()
    markers.markers.append(_delete_marker(header, 0))
    lifetime_ns = 400000000
    for index, box in enumerate(boxes):
        hue = (index * 0.61803398875) % 1.0
        red, green, blue = colorsys.hsv_to_rgb(hue, 0.85, 0.95)
        color = ColorRGBA(r=float(red), g=float(green), b=float(blue), a=0.35)
        id_base = 10 * index
        markers.markers.append(
            _box_cube(header, box, id_base + 1, color, lifetime_ns),
        )
        markers.markers.append(
            _box_wireframe(header, box, id_base + 2, color, lifetime_ns),
        )
        markers.markers.append(_box_label(header, box, id_base + 3, lifetime_ns))
    return markers


def _box_cube(
    header: Header,
    box: ClusterBox,
    marker_id: int,
    color: ColorRGBA,
    lifetime_ns: int,
) -> Marker:
    center = box.center
    size = box.size
    marker = Marker()
    marker.header = header
    marker.ns = 'pc_clustering'
    marker.id = marker_id
    marker.type = Marker.CUBE
    marker.action = Marker.ADD
    marker.pose = Pose(
        position=Point(
            x=float(center[0]),
            y=float(center[1]),
            z=float(center[2]),
        ),
        orientation=Quaternion(w=1.0),
    )
    marker.scale = Vector3(
        x=float(size[0]), y=float(size[1]), z=float(size[2]),
    )
    marker.color = color
    marker.lifetime.nanosec = lifetime_ns
    return marker


def _box_wireframe(
    header: Header,
    box: ClusterBox,
    marker_id: int,
    color: ColorRGBA,
    lifetime_ns: int,
) -> Marker:
    xmin, ymin, zmin = box.min_xyz
    xmax, ymax, zmax = box.max_xyz
    corners = [
        (xmin, ymin, zmin), (xmax, ymin, zmin),
        (xmax, ymax, zmin), (xmin, ymax, zmin),
        (xmin, ymin, zmax), (xmax, ymin, zmax),
        (xmax, ymax, zmax), (xmin, ymax, zmax),
    ]
    edges = (
        (0, 1), (1, 2), (2, 3), (3, 0),
        (4, 5), (5, 6), (6, 7), (7, 4),
        (0, 4), (1, 5), (2, 6), (3, 7),
    )
    marker = Marker()
    marker.header = header
    marker.ns = 'pc_clustering'
    marker.id = marker_id
    marker.type = Marker.LINE_LIST
    marker.action = Marker.ADD
    marker.pose.orientation.w = 1.0
    marker.scale.x = 0.04
    marker.color = ColorRGBA(
        r=min(color.r + 0.2, 1.0),
        g=min(color.g + 0.2, 1.0),
        b=min(color.b + 0.2, 1.0),
        a=1.0,
    )
    marker.lifetime.nanosec = lifetime_ns
    for i, j in edges:
        for idx in (i, j):
            cx, cy, cz = corners[idx]
            marker.points.append(Point(x=float(cx), y=float(cy), z=float(cz)))
    return marker


def _box_label(
    header: Header, box: ClusterBox, marker_id: int, lifetime_ns: int,
) -> Marker:
    center = box.center
    text = Marker()
    text.header = header
    text.ns = 'pc_clustering'
    text.id = marker_id
    text.type = Marker.TEXT_VIEW_FACING
    text.action = Marker.ADD
    text.pose.position.x = float(center[0])
    text.pose.position.y = float(center[1])
    text.pose.position.z = float(box.max_xyz[2] + 0.3)
    text.pose.orientation.w = 1.0
    text.scale.z = 0.4
    text.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=0.9)
    text.text = f'id={box.label} n={box.count}'
    text.lifetime.nanosec = lifetime_ns
    return text


def _delete_marker(header: Header, marker_id: int) -> Marker:
    marker = Marker()
    marker.header = header
    marker.ns = 'pc_clustering'
    marker.id = marker_id
    marker.action = Marker.DELETEALL
    return marker


def _read_intensity(msg: PointCloud2) -> Optional[np.ndarray]:
    names = {field.name for field in msg.fields}
    if 'intensity' not in names:
        return None
    from sensor_msgs_py import point_cloud2 as pc2
    try:
        values = pc2.read_points_numpy(
            msg, field_names=('intensity',), skip_nans=False,
        )
    except (AttributeError, TypeError, ValueError):
        return None
    intensity = np.asarray(values)
    if intensity.dtype.names and 'intensity' in intensity.dtype.names:
        intensity = intensity['intensity']
    return np.asarray(intensity, dtype=np.float32).reshape(-1)


def _create_xyzi_cloud(
    header: Header, xyz: np.ndarray, intensity: Optional[np.ndarray],
) -> PointCloud2:
    count = int(len(xyz))
    if intensity is None:
        dtype = np.dtype({
            'names': ['x', 'y', 'z'],
            'formats': ['<f4', '<f4', '<f4'],
            'itemsize': 12,
        })
        packed = np.zeros(count, dtype=dtype)
        if count:
            packed['x'] = xyz[:, 0]
            packed['y'] = xyz[:, 1]
            packed['z'] = xyz[:, 2]
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        return _numpy_cloud(header, packed, fields, 12)

    dtype = np.dtype({
        'names': ['x', 'y', 'z', 'intensity'],
        'formats': ['<f4', '<f4', '<f4', '<f4'],
        'itemsize': 16,
    })
    packed = np.zeros(count, dtype=dtype)
    if count:
        packed['x'] = xyz[:, 0]
        packed['y'] = xyz[:, 1]
        packed['z'] = xyz[:, 2]
        packed['intensity'] = intensity
    fields = [
        PointField(
            name='x', offset=0, datatype=PointField.FLOAT32, count=1,
        ),
        PointField(
            name='y', offset=4, datatype=PointField.FLOAT32, count=1,
        ),
        PointField(
            name='z', offset=8, datatype=PointField.FLOAT32, count=1,
        ),
        PointField(
            name='intensity', offset=12, datatype=PointField.FLOAT32, count=1,
        ),
    ]
    return _numpy_cloud(header, packed, fields, 16)


def _numpy_cloud(
    header: Header,
    packed: np.ndarray,
    fields: list[PointField],
    point_step: int,
) -> PointCloud2:
    msg = PointCloud2()
    msg.header = header
    msg.height = 1
    msg.width = int(len(packed))
    msg.fields = fields
    msg.is_bigendian = False
    msg.point_step = point_step
    msg.row_step = point_step * int(len(packed))
    msg.is_dense = True
    msg.data = packed.tobytes()
    return msg


def _transform_to_matrix(transform) -> np.ndarray:
    from tf_transformations import quaternion_matrix

    rotation = transform.transform.rotation
    matrix = quaternion_matrix(
        [rotation.x, rotation.y, rotation.z, rotation.w],
    )
    translation = transform.transform.translation
    matrix[0, 3] = translation.x
    matrix[1, 3] = translation.y
    matrix[2, 3] = translation.z
    return matrix


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PcClusteringNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
