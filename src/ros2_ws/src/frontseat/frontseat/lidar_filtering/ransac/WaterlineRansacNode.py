#!/usr/bin/env python3
"""ROS 2 node wrapping sequential waterline RANSAC.

See ``README.md`` in this package for the pipeline, topics, and YAML knobs.
"""

from __future__ import annotations

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

from frontseat.lidar_filtering.ransac.plane import (
    PlaneFit,
    RansacParams,
    fit_sequential_planes,
    inlier_extents,
    radial_mask,
)
from frontseat.lidar_filtering.self_filter import read_xyz_grid
from frontseat.qos_profiles import best_effort_volatile_qos


class WaterlineRansacNode(Node):
    def __init__(self) -> None:
        super().__init__('waterline_ransac')

        default_config = os.path.join(
            get_package_share_directory('frontseat'),
            'config', 'lidar_filtering', 'ransac', 'waterline.yaml',
        )
        self.declare_parameter('config_path', default_config)
        self.declare_parameter('debug_enabled', False)
        config_path = self.get_parameter('config_path').get_parameter_value().string_value
        if not config_path:
            config_path = default_config
        self._config = self._load_config(config_path)

        self._input_topic = str(
            self._config.get('input_topic', '/rslidar_points/self_filtered'),
        )
        self._fit_frame = str(self._config.get('fit_frame', 'base_link'))
        self._up_axis = np.array(
            self._config.get('up_axis', [0.0, 0.0, 1.0]), dtype=np.float64,
        )
        self._max_radius = float(self._config.get('max_radius', 4.0))

        ransac_cfg = self._config.get('ransac', {})
        self._num_planes = int(ransac_cfg.get('num_planes', ransac_cfg.get('num_passes', 2)))
        self._plane_params = _plane_params_from_config(ransac_cfg, self._num_planes)
        self._z_min = float(ransac_cfg.get('z_min', -4.0))
        self._z_max = float(ransac_cfg.get('z_max', 0.35))
        seed = int(ransac_cfg.get('random_seed', -1))
        self._rng = None if seed < 0 else np.random.default_rng(seed)

        debug_cfg = self._config.get('debug', {})
        param_debug = self.get_parameter('debug_enabled').get_parameter_value().bool_value
        self._debug_enabled = param_debug or bool(debug_cfg.get('enabled', False))
        self._max_marker_points = int(debug_cfg.get('max_marker_points', 2500))
        self._plane_thickness = float(debug_cfg.get('plane_thickness', 0.04))
        self._water_rgb = _rgb_uint32(debug_cfg.get('water_rgb', [30, 144, 255]))
        self._second_rgb = _rgb_uint32(debug_cfg.get('second_rgb', [50, 205, 50]))
        self._third_rgb = _rgb_uint32(debug_cfg.get('third_rgb', [255, 215, 0]))
        self._other_rgb = _rgb_uint32(debug_cfg.get('other_rgb', [255, 140, 0]))
        self._plane_rgbs = [self._water_rgb, self._second_rgb, self._third_rgb]

        cloud_sub_qos = best_effort_volatile_qos
        cloud_pub_qos = best_effort_volatile_qos
        marker_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            depth=1,
        )

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._colored_pub = None
        if self._debug_enabled:
            self._colored_pub = self.create_publisher(
                PointCloud2,
                str(self._config.get('colored_topic', '/rslidar_points/waterline_colored')),
                cloud_pub_qos,
            )
        self._removed_pub = self.create_publisher(
            PointCloud2,
            str(self._config.get('removed_topic', '/rslidar_points/waterline_removed')),
            cloud_pub_qos,
        )
        self._marker_pub = None
        if self._debug_enabled:
            self._marker_pub = self.create_publisher(
                MarkerArray,
                str(self._config.get('markers_topic', '/waterline_ransac/debug_markers')),
                marker_qos,
            )

        self.create_subscription(
            PointCloud2, self._input_topic, self._cloud_callback, cloud_sub_qos,
        )
        self.get_logger().info(
            f'Waterline RANSAC listening on {self._input_topic}; '
            f'fit_frame={self._fit_frame}; max_radius={self._max_radius:.2f} m; '
            f'num_planes={self._num_planes}; debug_enabled={self._debug_enabled}; '
            f'config={config_path}'
        )

    def _load_config(self, config_path: str) -> dict:
        with open(config_path, 'r', encoding='utf-8') as config_file:
            return yaml.safe_load(config_file) or {}

    def _cloud_callback(self, msg: PointCloud2) -> None:
        t0 = time.perf_counter()
        xyz = read_xyz_grid(msg)
        if xyz.size == 0:
            return

        stamp = Time.from_msg(msg.header.stamp)
        transform = self._lookup_transform_matrix(
            self._fit_frame, msg.header.frame_id, stamp,
        )
        if transform is None:
            return

        valid = np.isfinite(xyz).all(axis=1)
        colored_header = Header(stamp=msg.header.stamp, frame_id=msg.header.frame_id)
        marker_header = Header(stamp=msg.header.stamp, frame_id=self._fit_frame)
        if not np.any(valid):
            self._publish_empty(colored_header, marker_header)
            return

        xyz_valid = xyz[valid]
        intensity = _read_intensity(msg)
        intensity_valid = None if intensity is None else intensity[valid]
        points_h = np.hstack((
            xyz_valid, np.ones((len(xyz_valid), 1), dtype=np.float64),
        ))
        xyz_fit = (transform @ points_h.T).T[:, :3]

        candidate = (
            (xyz_fit[:, 2] >= self._z_min)
            & (xyz_fit[:, 2] <= self._z_max)
            & radial_mask(xyz_fit, self._max_radius)
        )
        fits: list[PlaneFit] = []
        if int(candidate.sum()) >= 3:
            rng = self._rng if self._rng is not None else np.random.default_rng()
            local_fits = fit_sequential_planes(
                xyz_fit[candidate],
                num_planes=self._num_planes,
                params=self._plane_params,
                up_axis=self._up_axis,
                rng=rng,
            )
            for local in local_fits:
                full_mask = np.zeros(len(xyz_valid), dtype=bool)
                full_mask[candidate] = local.inlier_mask
                fits.append(PlaneFit(
                    normal=local.normal,
                    offset=local.offset,
                    inlier_mask=full_mask,
                    centroid=local.centroid,
                ))

        stripped = np.zeros(len(xyz_valid), dtype=bool)
        for fit in fits:
            stripped |= fit.inlier_mask

        leftover = ~stripped
        self._removed_pub.publish(
            _create_xyzi_cloud(
                colored_header,
                xyz_valid[leftover],
                None if intensity_valid is None else intensity_valid[leftover],
            )
        )
        if self._colored_pub is not None:
            rgb = np.full(len(xyz_valid), self._other_rgb, dtype=np.uint32)
            for index, fit in enumerate(fits):
                color = (
                    self._plane_rgbs[index]
                    if index < len(self._plane_rgbs)
                    else self._other_rgb
                )
                rgb[fit.inlier_mask] = color
            self._colored_pub.publish(
                _create_xyzrgb_cloud(colored_header, xyz_valid, rgb),
            )
        if self._marker_pub is not None:
            self._marker_pub.publish(
                self._build_markers(marker_header, xyz_fit, fits)
            )

        elapsed_ms = (time.perf_counter() - t0) * 1000.0
        if fits:
            parts = []
            for index, fit in enumerate(fits):
                height = fit.height_at_origin()
                height_txt = 'n/a' if height is None else f'{height:.3f} m'
                label = 'water' if index == 0 else f'plane{index + 1}'
                parts.append(
                    f'{label}={fit.inlier_count} '
                    f'n=({fit.normal[0]:.3f}, {fit.normal[1]:.3f}, {fit.normal[2]:.3f}) '
                    f'z0={height_txt}'
                )
            self.get_logger().info(
                f'{"; ".join(parts)} candidates={int(candidate.sum())}/{len(xyz_valid)} '
                f'radius={self._max_radius:.1f} m time_ms={elapsed_ms:.1f}',
                throttle_duration_sec=2.0,
            )
        else:
            self.get_logger().warning(
                f'No waterline plane this frame time_ms={elapsed_ms:.1f}',
                throttle_duration_sec=2.0,
            )

    def _publish_empty(self, cloud_header: Header, marker_header: Header) -> None:
        empty = np.zeros((0, 3), dtype=np.float64)
        self._removed_pub.publish(_create_xyzi_cloud(cloud_header, empty, None))
        if self._colored_pub is not None:
            self._colored_pub.publish(
                _create_xyzrgb_cloud(cloud_header, empty, np.zeros(0, dtype=np.uint32)),
            )
        if self._marker_pub is not None:
            self._marker_pub.publish(
                self._build_markers(marker_header, empty, []),
            )

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
            f'TF {source_frame} -> {target_frame} unavailable',
            throttle_duration_sec=5.0,
        )
        return None

    def _build_markers(
        self,
        header: Header,
        xyz_fit: np.ndarray,
        fits: list[PlaneFit],
    ) -> MarkerArray:
        markers = MarkerArray()
        markers.markers.append(_delete_marker(header, 0))
        water_fit = fits[0] if fits else None
        markers.markers.extend(self._radius_markers(header, water_fit))
        plane_colors = [
            ColorRGBA(r=0.12, g=0.56, b=1.0, a=0.35),
            ColorRGBA(r=0.2, g=0.8, b=0.2, a=0.35),
            ColorRGBA(r=1.0, g=0.84, b=0.0, a=0.35),
        ]
        point_colors = [
            ColorRGBA(r=0.12, g=0.56, b=1.0, a=0.9),
            ColorRGBA(r=0.2, g=1.0, b=0.3, a=0.9),
            ColorRGBA(r=1.0, g=0.84, b=0.0, a=0.9),
        ]
        for index, fit in enumerate(fits):
            mask = fit.inlier_mask
            if not np.any(mask):
                continue
            id_base = 10 * index
            inliers = xyz_fit[mask]
            markers.markers.extend(_plane_debug_markers(
                header,
                fit,
                inliers,
                id_base=id_base,
                plane_color=plane_colors[min(index, len(plane_colors) - 1)],
                point_color=point_colors[min(index, len(point_colors) - 1)],
                thickness=self._plane_thickness,
                max_points=self._max_marker_points,
            ))
        return markers

    def _radius_markers(self, header: Header, fit) -> list[Marker]:
        if self._max_radius <= 0.0:
            return []

        height = 0.0 if fit is None else fit.height_at_origin()
        if height is None:
            height = 0.0
        z_lo = min(self._z_min, height)
        z_hi = max(self._z_max, height)
        z_span = max(z_hi - z_lo, 0.2)

        cylinder = Marker()
        cylinder.header = header
        cylinder.ns = 'waterline_ransac'
        cylinder.id = 5
        cylinder.type = Marker.CYLINDER
        cylinder.action = Marker.ADD
        cylinder.pose = Pose(
            position=Point(x=0.0, y=0.0, z=float(0.5 * (z_lo + z_hi))),
            orientation=Quaternion(w=1.0),
        )
        cylinder.scale = Vector3(
            x=float(2.0 * self._max_radius),
            y=float(2.0 * self._max_radius),
            z=float(z_span),
        )
        cylinder.color = ColorRGBA(r=0.2, g=1.0, b=0.4, a=0.12)
        cylinder.lifetime.sec = 0
        cylinder.lifetime.nanosec = 400000000

        ring = Marker()
        ring.header = header
        ring.ns = 'waterline_ransac'
        ring.id = 6
        ring.type = Marker.LINE_STRIP
        ring.action = Marker.ADD
        ring.scale.x = 0.06
        ring.color = ColorRGBA(r=0.2, g=1.0, b=0.4, a=1.0)
        ring.pose.orientation.w = 1.0
        ring.lifetime.sec = 0
        ring.lifetime.nanosec = 400000000
        angles = np.linspace(0.0, 2.0 * np.pi, 65)
        ring.points = [
            Point(
                x=float(self._max_radius * np.cos(angle)),
                y=float(self._max_radius * np.sin(angle)),
                z=float(height),
            )
            for angle in angles
        ]
        return [cylinder, ring]


def _plane_debug_markers(
    header: Header,
    fit: PlaneFit,
    inliers: np.ndarray,
    id_base: int,
    plane_color: ColorRGBA,
    point_color: ColorRGBA,
    thickness: float,
    max_points: int,
) -> list[Marker]:
    u, v, half_u, half_v = inlier_extents(inliers, fit.normal, fit.centroid)
    rotation = np.column_stack((u, v, fit.normal))
    orientation = _matrix_to_quaternion(rotation)
    lifetime_ns = 400000000

    plane = Marker()
    plane.header = header
    plane.ns = 'waterline_ransac'
    plane.id = id_base + 1
    plane.type = Marker.CUBE
    plane.action = Marker.ADD
    plane.pose = Pose(
        position=Point(
            x=float(fit.centroid[0]),
            y=float(fit.centroid[1]),
            z=float(fit.centroid[2]),
        ),
        orientation=orientation,
    )
    plane.scale = Vector3(
        x=float(2.0 * half_u),
        y=float(2.0 * half_v),
        z=float(thickness),
    )
    plane.color = plane_color
    plane.lifetime.nanosec = lifetime_ns

    outline = Marker()
    outline.header = header
    outline.ns = 'waterline_ransac'
    outline.id = id_base + 2
    outline.type = Marker.LINE_STRIP
    outline.action = Marker.ADD
    outline.scale.x = 0.04
    outline.color = ColorRGBA(
        r=min(plane_color.r + 0.2, 1.0),
        g=min(plane_color.g + 0.2, 1.0),
        b=min(plane_color.b + 0.2, 1.0),
        a=1.0,
    )
    outline.pose.orientation.w = 1.0
    outline.lifetime.nanosec = lifetime_ns
    corners = [
        fit.centroid + u * sx * half_u + v * sy * half_v
        for sx, sy in ((-1, -1), (1, -1), (1, 1), (-1, 1), (-1, -1))
    ]
    outline.points = [
        Point(x=float(p[0]), y=float(p[1]), z=float(p[2])) for p in corners
    ]

    normal_arrow = Marker()
    normal_arrow.header = header
    normal_arrow.ns = 'waterline_ransac'
    normal_arrow.id = id_base + 3
    normal_arrow.type = Marker.ARROW
    normal_arrow.action = Marker.ADD
    normal_arrow.scale = Vector3(x=0.08, y=0.14, z=0.18)
    normal_arrow.color = ColorRGBA(r=0.2, g=1.0, b=0.4, a=1.0)
    normal_arrow.lifetime.nanosec = lifetime_ns
    tip = fit.centroid + fit.normal * 1.0
    normal_arrow.points = [
        Point(
            x=float(fit.centroid[0]),
            y=float(fit.centroid[1]),
            z=float(fit.centroid[2]),
        ),
        Point(x=float(tip[0]), y=float(tip[1]), z=float(tip[2])),
    ]

    sample = inliers
    if len(sample) > max_points:
        step = int(np.ceil(len(sample) / float(max_points)))
        sample = sample[::step]
    points_marker = Marker()
    points_marker.header = header
    points_marker.ns = 'waterline_ransac'
    points_marker.id = id_base + 4
    points_marker.type = Marker.POINTS
    points_marker.action = Marker.ADD
    points_marker.scale = Vector3(x=0.06, y=0.06, z=0.06)
    points_marker.color = point_color
    points_marker.pose.orientation.w = 1.0
    points_marker.lifetime.nanosec = lifetime_ns
    points_marker.points = [
        Point(x=float(p[0]), y=float(p[1]), z=float(p[2])) for p in sample
    ]
    return [plane, outline, normal_arrow, points_marker]


def _rgb_uint32(rgb) -> np.uint32:
    red, green, blue = [int(v) for v in rgb]
    return np.uint32((red << 16) | (green << 8) | blue)


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


def _create_xyzrgb_cloud(
    header: Header, xyz: np.ndarray, rgb: np.ndarray,
) -> PointCloud2:
    count = int(len(xyz))
    dtype = np.dtype({
        'names': ['x', 'y', 'z', 'rgb'],
        'formats': ['<f4', '<f4', '<f4', '<u4'],
        'offsets': [0, 4, 8, 12],
        'itemsize': 16,
    })
    packed = np.zeros(count, dtype=dtype)
    if count:
        packed['x'] = xyz[:, 0]
        packed['y'] = xyz[:, 1]
        packed['z'] = xyz[:, 2]
        packed['rgb'] = rgb
    fields = [
        PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        PointField(name='rgb', offset=12, datatype=PointField.FLOAT32, count=1),
    ]
    return _numpy_cloud(header, packed, fields, 16)


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
        PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
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


def _matrix_to_quaternion(rotation: np.ndarray) -> Quaternion:
    from tf_transformations import quaternion_from_matrix

    matrix = np.eye(4)
    matrix[:3, :3] = rotation
    q = quaternion_from_matrix(matrix)
    return Quaternion(x=float(q[0]), y=float(q[1]), z=float(q[2]), w=float(q[3]))


def _delete_marker(header: Header, marker_id: int) -> Marker:
    marker = Marker()
    marker.header = header
    marker.ns = 'waterline_ransac'
    marker.id = marker_id
    marker.action = Marker.DELETEALL
    return marker


def _plane_params_from_config(ransac_cfg: dict, num_planes: int) -> list[RansacParams]:
    """Load ``first`` / ``second`` / ``third`` knobs; pad by repeating the last."""
    first = _ransac_params_from_config(ransac_cfg, 'first')
    params = [first]
    for key in ('second', 'third'):
        params.append(_ransac_params_from_config(ransac_cfg, key, defaults=params[-1]))
    if num_planes > len(params):
        params.extend([params[-1]] * (num_planes - len(params)))
    return params[:max(int(num_planes), 0)]


def _ransac_params_from_config(
    ransac_cfg: dict,
    key: str,
    defaults: RansacParams | None = None,
) -> RansacParams:
    nested = ransac_cfg.get(key, {}) or {}
    if not isinstance(nested, dict):
        nested = {}
    base = defaults or RansacParams()
    top_level = defaults is None

    def _value(name: str, cast, fallback):
        if name in nested:
            return cast(nested[name])
        if top_level and name in ransac_cfg:
            return cast(ransac_cfg[name])
        return fallback

    distance_fallback = base.distance_threshold
    if (
        key == 'second'
        and 'distance_threshold' not in nested
        and ransac_cfg.get('second_distance_threshold') is not None
    ):
        distance_fallback = float(ransac_cfg['second_distance_threshold'])

    return RansacParams(
        distance_threshold=_value('distance_threshold', float, distance_fallback),
        max_iterations=_value('max_iterations', int, base.max_iterations),
        max_tilt_deg=_value('max_tilt_deg', float, base.max_tilt_deg),
        min_inliers=_value('min_inliers', int, base.min_inliers),
        min_inlier_ratio=_value('min_inlier_ratio', float, base.min_inlier_ratio),
    )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = WaterlineRansacNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
