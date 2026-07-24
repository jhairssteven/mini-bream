"""Axis-aligned / oriented box volumes for boat self-filtering."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from tf_transformations import quaternion_from_euler, quaternion_matrix


@dataclass
class FilterBox:
    name: str
    center: np.ndarray
    half_extents: np.ndarray
    rotation: np.ndarray

    @classmethod
    def from_config(cls, entry: dict, margin: float) -> FilterBox:
        center = np.array(
            [entry['center']['x'], entry['center']['y'], entry['center']['z']],
            dtype=np.float64,
        )
        size = np.array(
            [entry['size']['x'], entry['size']['y'], entry['size']['z']],
            dtype=np.float64,
        )
        half_extents = 0.5 * size + margin
        roll, pitch, yaw = entry.get('rpy', [0.0, 0.0, 0.0])
        rotation = quaternion_matrix(
            quaternion_from_euler(float(roll), float(pitch), float(yaw))
        )[:3, :3]
        return cls(
            name=str(entry.get('name', 'box')),
            center=center,
            half_extents=half_extents,
            rotation=rotation,
        )

    def contains(self, points: np.ndarray) -> np.ndarray:
        if points.size == 0:
            return np.zeros(0, dtype=bool)
        local = (points - self.center) @ self.rotation
        return np.all(np.abs(local) <= self.half_extents, axis=1)


def load_filter_boxes(config: dict, margin: float) -> list[FilterBox]:
    boxes: list[FilterBox] = []

    hull = config.get('hull', {})
    if hull.get('enabled', True):
        boxes.append(FilterBox.from_config(
            {
                'name': 'hull',
                'center': hull['center'],
                'size': hull['size'],
                'rpy': hull.get('rpy', [0.0, 0.0, 0.0]),
            },
            margin,
        ))

    for entry in config.get('volumes', []):
        if entry.get('enabled', True):
            boxes.append(FilterBox.from_config(entry, margin))

    return boxes


def build_keep_mask(points: np.ndarray, boxes: list[FilterBox]) -> np.ndarray:
    if points.size == 0:
        return np.zeros(0, dtype=bool)
    keep = np.ones(len(points), dtype=bool)
    for box in boxes:
        keep &= ~box.contains(points)
    return keep


def read_xyz_grid(msg) -> np.ndarray:
    """Read xyz for every point slot in message order (includes NaN placeholders)."""
    from sensor_msgs_py import point_cloud2 as pc2

    try:
        structured = pc2.read_points_numpy(
            msg, field_names=('x', 'y', 'z'), skip_nans=False,
        )
        if isinstance(structured, np.ndarray):
            if structured.ndim == 2 and structured.shape[1] >= 3:
                return structured[:, :3].astype(np.float64)
            if structured.dtype.names and {'x', 'y', 'z'}.issubset(structured.dtype.names):
                return np.column_stack((
                    structured['x'], structured['y'], structured['z'],
                )).astype(np.float64)
    except (AttributeError, TypeError, ValueError):
        pass

    count = msg.height * msg.width if msg.height > 0 else msg.width
    points = np.array(
        list(pc2.read_points(msg, field_names=('x', 'y', 'z'), skip_nans=False)),
        dtype=np.float64,
    )
    if points.size == 0:
        return np.zeros((0, 3), dtype=np.float64)
    return points.reshape(count, 3)
