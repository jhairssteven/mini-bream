"""Unit tests for DBSCAN clustering and the cluster size gate."""

import numpy as np
import pytest

from frontseat.lidar_filtering.pc_clustering.dbscan import (
    ClusterParams,
    apply_size_filter,
    cluster_points,
    dbscan_labels,
)


def _blob(center, n, rng, scale=0.05):
    offset = rng.normal(0.0, scale, size=(n, 3))
    return np.asarray(center, dtype=np.float64) + offset


def test_two_separated_blobs_are_two_clusters():
    rng = np.random.default_rng(0)
    left = _blob((-2.0, 0.0, 0.0), 40, rng)
    right = _blob((2.0, 0.0, 0.0), 40, rng)
    points = np.vstack((left, right))
    result = cluster_points(
        points,
        ClusterParams(eps=0.4, min_samples=4, min_cluster_size=10),
    )
    assert result.cluster_count == 2
    assert result.kept_count == 80
    assert result.removed_count == 0
    labels = result.labels
    assert len(np.unique(labels[labels >= 0])) == 2
    assert np.all(labels[:40] == labels[0])
    assert np.all(labels[40:] == labels[40])
    assert labels[0] != labels[40]


def test_sparse_noise_is_removed():
    rng = np.random.default_rng(1)
    blob = _blob((0.0, 0.0, 0.0), 50, rng, scale=0.04)
    noise = rng.uniform(-5.0, 5.0, size=(15, 3))
    points = np.vstack((blob, noise))
    result = cluster_points(
        points,
        ClusterParams(eps=0.25, min_samples=4, min_cluster_size=8),
    )
    assert result.cluster_count == 1
    assert result.kept_count >= 45
    assert int(np.sum(~result.keep_mask[-15:])) >= 10


def test_small_cluster_fails_size_gate():
    rng = np.random.default_rng(2)
    big = _blob((0.0, 0.0, 0.0), 40, rng)
    small = _blob((3.0, 0.0, 0.0), 6, rng, scale=0.03)
    points = np.vstack((big, small))
    result = cluster_points(
        points,
        ClusterParams(eps=0.3, min_samples=3, min_cluster_size=10),
    )
    assert result.cluster_count == 1
    assert result.kept_count == 40
    assert np.all(result.labels[40:] < 0)


def test_max_cluster_size_drops_huge_cluster():
    rng = np.random.default_rng(3)
    huge = _blob((0.0, 0.0, 0.0), 80, rng)
    other = _blob((4.0, 0.0, 0.0), 20, rng)
    points = np.vstack((huge, other))
    result = cluster_points(
        points,
        ClusterParams(
            eps=0.3, min_samples=4, min_cluster_size=5, max_cluster_size=40,
        ),
    )
    assert result.cluster_count == 1
    assert result.kept_count == 20
    assert np.all(result.labels[:80] < 0)


def test_empty_cloud():
    points = np.zeros((0, 3), dtype=np.float64)
    result = cluster_points(points)
    assert result.kept_count == 0
    assert result.cluster_count == 0
    assert result.labels.shape == (0,)


def test_too_few_points_are_noise():
    points = np.array([[0.0, 0.0, 0.0], [0.1, 0.0, 0.0]], dtype=np.float64)
    labels = dbscan_labels(points, eps=0.5, min_samples=4)
    assert np.all(labels < 0)


def test_apply_size_filter_vectorized():
    labels = np.array([0, 0, 0, 1, 1, -1, 2, 2, 2, 2], dtype=np.int32)
    out = apply_size_filter(labels, min_cluster_size=3, max_cluster_size=0)
    assert out.tolist() == [0, 0, 0, -1, -1, -1, 2, 2, 2, 2]


def test_cluster_box_extents():
    rng = np.random.default_rng(4)
    blob = _blob((1.0, -2.0, 0.5), 30, rng, scale=0.02)
    result = cluster_points(
        blob, ClusterParams(eps=0.2, min_samples=3, min_cluster_size=5),
    )
    assert result.cluster_count == 1
    box = result.boxes[0]
    assert box.count == 30
    center = box.center
    assert center[0] == pytest.approx(1.0, abs=0.05)
    assert center[1] == pytest.approx(-2.0, abs=0.05)
    assert center[2] == pytest.approx(0.5, abs=0.05)
    assert np.all(box.size > 0.0)


def test_voxel_downsample_keeps_separated_blobs():
    rng = np.random.default_rng(5)
    left = _blob((-2.0, 0.0, 0.0), 40, rng)
    right = _blob((2.0, 0.0, 0.0), 40, rng)
    points = np.vstack((left, right))
    result = cluster_points(
        points,
        ClusterParams(
            eps=0.4, min_samples=3, min_cluster_size=10, voxel_size=0.08,
        ),
    )
    assert result.cluster_count == 2
    assert result.kept_count == 80


def test_max_radius_skips_far_points():
    rng = np.random.default_rng(6)
    near = _blob((0.5, 0.0, 0.0), 40, rng)
    far_small = _blob((8.0, 0.0, 0.0), 6, rng, scale=0.03)
    points = np.vstack((near, far_small))
    gated = cluster_points(
        points,
        ClusterParams(
            eps=0.3, min_samples=3, min_cluster_size=10, max_radius=4.0,
        ),
    )
    assert gated.candidate_count == 40
    assert np.all(gated.keep_mask[40:])
    assert gated.kept_count == 46

    whole = cluster_points(
        points,
        ClusterParams(
            eps=0.3, min_samples=3, min_cluster_size=10, max_radius=0.0,
        ),
    )
    assert whole.candidate_count == 46
    assert np.all(whole.labels[40:] < 0)
    assert whole.kept_count == 40

