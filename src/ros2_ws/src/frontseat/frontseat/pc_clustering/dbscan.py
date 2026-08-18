"""DBSCAN clustering for LiDAR noise rejection.

Points in clusters smaller than ``min_cluster_size`` (and optionally larger
than ``max_cluster_size``) are treated as noise and dropped from the filtered
cloud.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from scipy.sparse import coo_matrix
from scipy.sparse.csgraph import connected_components
from scipy.spatial import cKDTree

from frontseat.ransac.plane import radial_mask

_MIN_BOX = 0.05


@dataclass
class ClusterParams:
    """Knobs for one DBSCAN pass plus a post-cluster size gate.

    ``eps``
        Neighborhood radius in meters (DBSCAN ``eps``).
    ``min_samples``
        Core-point density, including the point itself.
    ``min_cluster_size``
        Drop clusters with fewer points than this after DBSCAN.
    ``max_cluster_size``
        Drop clusters larger than this. ``<=0`` disables the cap.
    ``voxel_size``
        Downsample before DBSCAN, then map labels back. ``<=0`` disables.
    ``max_radius``
        XY cylinder around the origin of the points passed in. Only those
        points are clustered; the rest pass through. ``<=0`` clusters
        the whole cloud.
    """

    eps: float = 0.45
    min_samples: int = 4
    min_cluster_size: int = 10
    max_cluster_size: int = 0
    voxel_size: float = 0.0
    max_radius: float = 0.0


@dataclass
class ClusterBox:
    """Axis-aligned bounding box of one kept cluster."""

    label: int
    count: int
    min_xyz: np.ndarray
    max_xyz: np.ndarray

    @property
    def center(self) -> np.ndarray:
        """Box center in the same frame as the points."""
        return 0.5 * (self.min_xyz + self.max_xyz)

    @property
    def size(self) -> np.ndarray:
        """Box edge lengths; each axis is at least ``_MIN_BOX`` meters."""
        return np.maximum(self.max_xyz - self.min_xyz, _MIN_BOX)


@dataclass
class ClusterResult:
    """Per-point DBSCAN labels after the size gate, plus kept AABBs.

    ``labels``
        ``-1`` is noise / rejected; ``>=0`` is a kept cluster id.
    ``keep_mask``
        True for points that stay in the filtered cloud.
    """

    labels: np.ndarray
    keep_mask: np.ndarray
    boxes: list[ClusterBox]
    candidate_mask: np.ndarray

    @property
    def candidate_count(self) -> int:
        """Number of points that entered DBSCAN."""
        return int(self.candidate_mask.sum())

    @property
    def kept_count(self) -> int:
        """Number of points that passed the size gate."""
        return int(self.keep_mask.sum())

    @property
    def removed_count(self) -> int:
        """Number of noise / undersized / oversized points."""
        return int((~self.keep_mask).sum())

    @property
    def cluster_count(self) -> int:
        """Number of kept clusters."""
        return len(self.boxes)


def dbscan_labels(points: np.ndarray, eps: float, min_samples: int) -> np.ndarray:
    """DBSCAN labels for ``(N, 3)`` points. ``-1`` is noise.

    Uses scikit-learn when available (Cython KD-tree). Falls back to a SciPy
    connected-component implementation otherwise.
    """
    n = int(points.shape[0]) if points.ndim == 2 else 0
    if n == 0:
        return np.zeros(0, dtype=np.int32)
    min_samples = max(int(min_samples), 1)
    if n < min_samples:
        return np.full(n, -1, dtype=np.int32)

    xyz = np.asarray(points, dtype=np.float64)
    try:
        from sklearn.cluster import DBSCAN
        labels = DBSCAN(
            eps=float(eps),
            min_samples=min_samples,
            algorithm='kd_tree',
            n_jobs=-1,
        ).fit_predict(xyz)
        return np.asarray(labels, dtype=np.int32)
    except ImportError:
        return _dbscan_labels_scipy(xyz, float(eps), min_samples)


def _dbscan_labels_scipy(
    points: np.ndarray, eps: float, min_samples: int,
) -> np.ndarray:
    """SciPy KD-tree DBSCAN used when scikit-learn is not installed."""
    n = int(points.shape[0])
    tree = cKDTree(points)
    pairs = tree.query_pairs(eps, output_type='ndarray')

    counts = np.ones(n, dtype=np.int32)
    if pairs.size:
        np.add.at(counts, pairs[:, 0], 1)
        np.add.at(counts, pairs[:, 1], 1)

    is_core = counts >= min_samples
    labels = np.full(n, -1, dtype=np.int32)
    if not np.any(is_core):
        return labels

    core_idx = np.flatnonzero(is_core)
    compact = np.full(n, -1, dtype=np.int32)
    compact[core_idx] = np.arange(len(core_idx), dtype=np.int32)
    n_core = int(len(core_idx))

    if pairs.size:
        core_edge = is_core[pairs[:, 0]] & is_core[pairs[:, 1]]
        core_pairs = pairs[core_edge]
    else:
        core_pairs = np.zeros((0, 2), dtype=np.int64)

    if core_pairs.size:
        row = compact[core_pairs[:, 0]]
        col = compact[core_pairs[:, 1]]
        data = np.ones(len(row) * 2, dtype=np.uint8)
        graph = coo_matrix(
            (data, (np.concatenate([row, col]), np.concatenate([col, row]))),
            shape=(n_core, n_core),
        )
        _, components = connected_components(graph, directed=False)
    else:
        components = np.arange(n_core, dtype=np.int32)

    labels[core_idx] = components.astype(np.int32)
    _assign_border_points(labels, pairs, is_core)
    return labels


def apply_size_filter(
    labels: np.ndarray,
    min_cluster_size: int,
    max_cluster_size: int = 0,
) -> np.ndarray:
    """Set undersized (and optionally oversized) clusters to noise (``-1``)."""
    out = np.asarray(labels, dtype=np.int32).copy()
    valid = out >= 0
    if not np.any(valid):
        return out

    unique, counts = np.unique(out[valid], return_counts=True)
    drop = unique[counts < int(min_cluster_size)]
    if int(max_cluster_size) > 0:
        drop = np.concatenate((drop, unique[counts > int(max_cluster_size)]))
    if drop.size:
        out[np.isin(out, drop)] = -1
    return out


def cluster_boxes(points: np.ndarray, labels: np.ndarray) -> list[ClusterBox]:
    """Axis-aligned boxes for every kept label (``>= 0``)."""
    boxes: list[ClusterBox] = []
    if points.size == 0:
        return boxes
    for lbl in np.unique(labels):
        if lbl < 0:
            continue
        pts = points[labels == lbl]
        boxes.append(ClusterBox(
            label=int(lbl),
            count=int(pts.shape[0]),
            min_xyz=pts.min(axis=0).astype(np.float64),
            max_xyz=pts.max(axis=0).astype(np.float64),
        ))
    return boxes


def voxel_centroids(
    points: np.ndarray, voxel_size: float,
) -> tuple[np.ndarray, np.ndarray]:
    """Mean point per occupied voxel and the inverse map to original points."""
    keys = np.floor(points / float(voxel_size)).astype(np.int64)
    _, inverse = np.unique(keys, axis=0, return_inverse=True)
    n_vox = int(inverse.max()) + 1 if inverse.size else 0
    centroids = np.zeros((n_vox, points.shape[1]), dtype=np.float64)
    counts = np.zeros(n_vox, dtype=np.int32)
    np.add.at(centroids, inverse, points)
    np.add.at(counts, inverse, 1)
    centroids /= np.maximum(counts[:, None], 1)
    return centroids, inverse


def cluster_points(
    points: np.ndarray,
    params: ClusterParams | None = None,
    candidate_mask: np.ndarray | None = None,
    **kwargs,
) -> ClusterResult:
    """Run DBSCAN then drop clusters outside the size gate.

    Extra ``kwargs`` override fields on ``params`` (or on ``ClusterParams``
    defaults when ``params`` is None). ``candidate_mask`` selects which
    points enter DBSCAN; if omitted, ``max_radius`` is applied in XY.
    Points outside the mask pass through unfiltered.
    """
    cfg = params or ClusterParams()
    if kwargs:
        cfg = ClusterParams(
            eps=float(kwargs.get('eps', cfg.eps)),
            min_samples=int(kwargs.get('min_samples', cfg.min_samples)),
            min_cluster_size=int(
                kwargs.get('min_cluster_size', cfg.min_cluster_size),
            ),
            max_cluster_size=int(
                kwargs.get('max_cluster_size', cfg.max_cluster_size),
            ),
            voxel_size=float(kwargs.get('voxel_size', cfg.voxel_size)),
            max_radius=float(kwargs.get('max_radius', cfg.max_radius)),
        )

    n = int(points.shape[0]) if points.ndim == 2 else 0
    if n == 0:
        empty = np.zeros(0, dtype=bool)
        return ClusterResult(
            labels=np.zeros(0, dtype=np.int32),
            keep_mask=empty,
            boxes=[],
            candidate_mask=empty,
        )

    if candidate_mask is None:
        candidate_mask = radial_mask(points, cfg.max_radius)
    else:
        candidate_mask = np.asarray(candidate_mask, dtype=bool).reshape(-1)
        if candidate_mask.shape[0] != n:
            raise ValueError('candidate_mask length must match points')

    keep = np.ones(n, dtype=bool)
    labels = np.full(n, -2, dtype=np.int32)
    if not np.any(candidate_mask):
        return ClusterResult(
            labels=labels,
            keep_mask=keep,
            boxes=[],
            candidate_mask=candidate_mask,
        )

    sub = points[candidate_mask]
    work = sub
    inverse = None
    if float(cfg.voxel_size) > 0.0 and sub.shape[0] > 0:
        work, inverse = voxel_centroids(sub, cfg.voxel_size)
    raw = dbscan_labels(work, cfg.eps, cfg.min_samples)
    if inverse is not None:
        raw = raw[inverse]
    sub_labels = apply_size_filter(
        raw, cfg.min_cluster_size, cfg.max_cluster_size,
    )
    labels[candidate_mask] = sub_labels
    keep[candidate_mask] = sub_labels >= 0
    return ClusterResult(
        labels=labels,
        keep_mask=keep,
        boxes=cluster_boxes(sub, sub_labels),
        candidate_mask=candidate_mask,
    )


def _assign_border_points(
    labels: np.ndarray,
    pairs: np.ndarray,
    is_core: np.ndarray,
) -> None:
    """Give each non-core neighbor of a core point that core's cluster id."""
    if not pairs.size:
        return
    left, right = pairs[:, 0], pairs[:, 1]
    core_left = is_core[left] & ~is_core[right]
    core_right = is_core[right] & ~is_core[left]
    if np.any(core_left):
        dst = right[core_left]
        unset = labels[dst] < 0
        labels[dst[unset]] = labels[left[core_left][unset]]
    if np.any(core_right):
        dst = left[core_right]
        unset = labels[dst] < 0
        labels[dst[unset]] = labels[right[core_right][unset]]
