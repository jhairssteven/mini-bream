"""Sequential RANSAC for near-horizontal planes in a LiDAR cloud.

Each pass fits one plane, then the next pass runs on the leftover points
(previous inliers are removed). Plane 1 is the water surface; later planes
are extra near-horizontal surfaces in the same candidate band.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

_MIN_SPAN = 1e-8
_HYPOTHESIS_BATCH = 64


@dataclass
class RansacParams:
    """Knobs for one sequential RANSAC pass.

    ``distance_threshold``
        Inlier band around the hypothesis, in meters.
    ``max_iterations``
        Random three-point hypotheses to try.
    ``max_tilt_deg``
        Reject planes whose normal is more than this from ``up_axis``
        (walls / boat sides).
    ``min_inliers``
        Absolute inlier floor; the pass fails if fewer points agree.
    ``min_inlier_ratio``
        Inlier floor as a fraction of the *current leftover* cloud.
    """

    distance_threshold: float = 0.12
    max_iterations: int = 250
    max_tilt_deg: float = 25.0
    min_inliers: int = 300
    min_inlier_ratio: float = 0.08


@dataclass
class PlaneFit:
    """Plane n·x + offset = 0 with unit normal pointing toward ``up_axis``."""

    normal: np.ndarray
    offset: float
    inlier_mask: np.ndarray
    centroid: np.ndarray

    @property
    def inlier_count(self) -> int:
        return int(self.inlier_mask.sum())

    def signed_distance(self, points: np.ndarray) -> np.ndarray:
        """Signed distance from each point to the plane."""
        return points @ self.normal + self.offset

    def height_at_origin(self) -> float | None:
        """Z intercept in the fit frame, or None if the plane is vertical."""
        nz = float(self.normal[2])
        if abs(nz) < 1e-6:
            return None
        return float(-self.offset / nz)


def radial_mask(
    points: np.ndarray,
    max_radius: float,
    center_xy: np.ndarray | None = None,
) -> np.ndarray:
    """True for points whose XY distance from ``center_xy`` is <= ``max_radius``.

    ``center_xy`` is ``(x, y)`` in the same frame as ``points``. When omitted,
    the center is the origin. ``max_radius <= 0`` disables the gate (all True).
    """
    if points.ndim != 2 or points.shape[1] < 2:
        return np.zeros(0, dtype=bool)
    if max_radius <= 0.0:
        return np.ones(len(points), dtype=bool)
    center = (
        np.zeros(2, dtype=np.float64)
        if center_xy is None
        else np.asarray(center_xy, dtype=np.float64).reshape(2)
    )
    delta = points[:, :2] - center
    return np.hypot(delta[:, 0], delta[:, 1]) <= float(max_radius)


def fit_waterline_plane(
    points: np.ndarray,
    distance_threshold: float = 0.12,
    max_iterations: int = 250,
    max_tilt_deg: float = 25.0,
    min_inliers: int = 300,
    min_inlier_ratio: float = 0.08,
    up_axis: np.ndarray | None = None,
    rng: np.random.Generator | None = None,
) -> PlaneFit | None:
    """Fit one near-horizontal plane with RANSAC, then SVD-refine inliers.

    ``points`` is (N, 3). Planes whose normal is more than ``max_tilt_deg``
    from ``up_axis`` are rejected so walls and boat sides are ignored.
    """
    fits = fit_sequential_planes(
        points,
        num_planes=1,
        first=RansacParams(
            distance_threshold=distance_threshold,
            max_iterations=max_iterations,
            max_tilt_deg=max_tilt_deg,
            min_inliers=min_inliers,
            min_inlier_ratio=min_inlier_ratio,
        ),
        up_axis=up_axis,
        rng=rng,
    )
    return fits[0] if fits else None


def fit_sequential_planes(
    points: np.ndarray,
    num_planes: int = 2,
    params: list[RansacParams] | None = None,
    first: RansacParams | None = None,
    second: RansacParams | None = None,
    third: RansacParams | None = None,
    distance_threshold: float = 0.12,
    max_iterations: int = 250,
    max_tilt_deg: float = 25.0,
    min_inliers: int = 300,
    min_inlier_ratio: float = 0.08,
    up_axis: np.ndarray | None = None,
    rng: np.random.Generator | None = None,
) -> list[PlaneFit]:
    """Fit planes one after another on leftover points.

    Pass *k* uses only points that were not inliers of passes ``0 .. k-1``.
    Inlier masks on the returned ``PlaneFit`` objects are mutually exclusive
    and index the original ``points`` array.

    ``params`` is the preferred per-pass list. Otherwise ``first`` / ``second``
    / ``third`` (or the shared kwargs) are used. Missing later passes reuse
    the last defined set of knobs.
    """
    if points.ndim != 2 or points.shape[1] != 3 or len(points) < 3:
        return []

    plane_params = _resolve_plane_params(
        max(int(num_planes), 0),
        params,
        first,
        second,
        third,
        RansacParams(
            distance_threshold=distance_threshold,
            max_iterations=max_iterations,
            max_tilt_deg=max_tilt_deg,
            min_inliers=min_inliers,
            min_inlier_ratio=min_inlier_ratio,
        ),
    )
    up = _unit(np.array([0.0, 0.0, 1.0], dtype=np.float64) if up_axis is None else up_axis)
    rng = np.random.default_rng() if rng is None else rng
    remaining = np.ones(len(points), dtype=bool)
    fits: list[PlaneFit] = []

    for params_k in plane_params:
        leftover = points[remaining]
        min_cos = float(np.cos(np.deg2rad(float(params_k.max_tilt_deg))))
        local = _fit_once(
            leftover,
            distance_threshold=float(params_k.distance_threshold),
            max_iterations=int(params_k.max_iterations),
            min_inliers=int(params_k.min_inliers),
            min_inlier_ratio=float(params_k.min_inlier_ratio),
            up=up,
            min_cos=min_cos,
            rng=rng,
        )
        if local is None:
            break
        full_mask = np.zeros(len(points), dtype=bool)
        full_mask[remaining] = local.inlier_mask
        fits.append(PlaneFit(
            normal=local.normal,
            offset=local.offset,
            inlier_mask=full_mask,
            centroid=local.centroid,
        ))
        remaining &= ~full_mask

    return fits


def _resolve_plane_params(
    num_planes: int,
    params: list[RansacParams] | None,
    first: RansacParams | None,
    second: RansacParams | None,
    third: RansacParams | None,
    fallback: RansacParams,
) -> list[RansacParams]:
    if num_planes <= 0:
        return []
    if params:
        listed = list(params)
    else:
        listed = [first or fallback]
        if second is not None:
            listed.append(second)
        if third is not None:
            listed.append(third)
    while len(listed) < num_planes:
        listed.append(listed[-1])
    return listed[:num_planes]


def _fit_once(
    points: np.ndarray,
    distance_threshold: float,
    max_iterations: int,
    min_inliers: int,
    min_inlier_ratio: float,
    up: np.ndarray,
    min_cos: float,
    rng: np.random.Generator,
) -> PlaneFit | None:
    n_points = len(points)
    if n_points < 3:
        return None

    normals, offsets = _sample_hypotheses(
        points, int(max_iterations), up, min_cos, rng,
    )
    if len(normals) == 0:
        return None

    best_count = -1
    best_normal = None
    best_offset = None
    for start in range(0, len(normals), _HYPOTHESIS_BATCH):
        batch_n = normals[start:start + _HYPOTHESIS_BATCH]
        batch_d = offsets[start:start + _HYPOTHESIS_BATCH]
        dist = np.abs(points @ batch_n.T + batch_d)
        counts = (dist < distance_threshold).sum(axis=0)
        local = int(np.argmax(counts))
        if int(counts[local]) > best_count:
            best_count = int(counts[local])
            best_normal = batch_n[local]
            best_offset = float(batch_d[local])

    if best_normal is None or best_count < int(min_inliers):
        return None
    if best_count / float(n_points) < float(min_inlier_ratio):
        return None

    inlier_mask = np.abs(points @ best_normal + best_offset) < distance_threshold
    refined = _svd_plane(points[inlier_mask], up, min_cos)
    if refined is not None:
        ref_n, ref_d = refined
        ref_mask = np.abs(points @ ref_n + ref_d) < distance_threshold
        if int(ref_mask.sum()) >= best_count:
            best_normal, best_offset, inlier_mask = ref_n, ref_d, ref_mask

    centroid = points[inlier_mask].mean(axis=0)
    return PlaneFit(
        normal=best_normal.astype(np.float64),
        offset=float(best_offset),
        inlier_mask=inlier_mask,
        centroid=centroid.astype(np.float64),
    )


def plane_basis(normal: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Return two unit axes spanning the plane (right-handed with ``normal``)."""
    n = _unit(normal)
    helper = np.array([1.0, 0.0, 0.0], dtype=np.float64)
    if abs(float(n[0])) > 0.9:
        helper = np.array([0.0, 1.0, 0.0], dtype=np.float64)
    u = _unit(np.cross(helper, n))
    v = np.cross(n, u)
    return u, v


def inlier_extents(
    points: np.ndarray,
    normal: np.ndarray,
    centroid: np.ndarray,
) -> tuple[np.ndarray, np.ndarray, float, float]:
    """Plane-aligned (u, v) axes and half-extents covering ``points``."""
    u, v = plane_basis(normal)
    local = points - centroid
    half_u = float(np.max(np.abs(local @ u))) if len(points) else 1.0
    half_v = float(np.max(np.abs(local @ v))) if len(points) else 1.0
    half_u = max(half_u, 1.0)
    half_v = max(half_v, 1.0)
    return u, v, half_u, half_v


def _sample_hypotheses(
    points: np.ndarray,
    max_iterations: int,
    up: np.ndarray,
    min_cos: float,
    rng: np.random.Generator,
) -> tuple[np.ndarray, np.ndarray]:
    n_points = len(points)
    idx = rng.integers(0, n_points, size=(max(max_iterations, 1), 3))
    unique = (idx[:, 0] != idx[:, 1]) & (idx[:, 1] != idx[:, 2]) & (idx[:, 0] != idx[:, 2])
    idx = idx[unique]
    if len(idx) == 0:
        return np.zeros((0, 3), dtype=np.float64), np.zeros(0, dtype=np.float64)

    p0 = points[idx[:, 0]]
    p1 = points[idx[:, 1]]
    p2 = points[idx[:, 2]]
    normals = np.cross(p1 - p0, p2 - p0)
    norms = np.linalg.norm(normals, axis=1)
    valid = norms > _MIN_SPAN
    if not np.any(valid):
        return np.zeros((0, 3), dtype=np.float64), np.zeros(0, dtype=np.float64)

    normals = normals[valid] / norms[valid, None]
    p0 = p0[valid]
    align = normals @ up
    flip = align < 0.0
    normals[flip] *= -1.0
    align = np.abs(align)
    tilt_ok = align >= min_cos
    normals = normals[tilt_ok]
    p0 = p0[tilt_ok]
    if len(normals) == 0:
        return np.zeros((0, 3), dtype=np.float64), np.zeros(0, dtype=np.float64)
    offsets = -np.einsum('ij,ij->i', normals, p0)
    return normals, offsets


def _svd_plane(
    inliers: np.ndarray,
    up: np.ndarray,
    min_cos: float,
) -> tuple[np.ndarray, float] | None:
    if len(inliers) < 3:
        return None
    centroid = inliers.mean(axis=0)
    centered = inliers - centroid
    try:
        _, _, vh = np.linalg.svd(centered, full_matrices=False)
    except np.linalg.LinAlgError:
        return None
    normal = _unit(vh[-1])
    if normal @ up < 0.0:
        normal = -normal
    if float(normal @ up) < min_cos:
        return None
    return normal, float(-normal @ centroid)


def _unit(vector: np.ndarray) -> np.ndarray:
    value = np.asarray(vector, dtype=np.float64).reshape(3)
    norm = float(np.linalg.norm(value))
    if norm < _MIN_SPAN:
        return np.array([0.0, 0.0, 1.0], dtype=np.float64)
    return value / norm
