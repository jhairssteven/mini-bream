"""Sequential RANSAC waterline fitting for RoboSense point clouds.

See ``README.md`` in this directory for the leftover-cloud pipeline and YAML.
"""

from frontseat.lidar_filtering.ransac.plane import (
    PlaneFit,
    RansacParams,
    fit_sequential_planes,
    fit_waterline_plane,
    radial_mask,
)

__all__ = [
    'PlaneFit',
    'RansacParams',
    'fit_sequential_planes',
    'fit_waterline_plane',
    'radial_mask',
]
