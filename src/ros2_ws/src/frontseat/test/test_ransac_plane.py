"""Unit tests for near-horizontal waterline RANSAC."""

import numpy as np
import pytest

from frontseat.lidar_filtering.ransac.plane import (
    RansacParams,
    fit_sequential_planes,
    fit_waterline_plane,
    plane_basis,
    radial_mask,
)


def test_recovers_horizontal_water_plane():
    rng = np.random.default_rng(0)
    n_inliers = 800
    xy = rng.uniform(-8.0, 8.0, size=(n_inliers, 2))
    water = np.column_stack((xy[:, 0], xy[:, 1], -0.05 + rng.normal(0.0, 0.02, n_inliers)))
    outliers = rng.uniform(-8.0, 8.0, size=(120, 3))
    outliers[:, 2] = rng.uniform(0.6, 2.5, size=120)
    points = np.vstack((water, outliers))

    fit = fit_waterline_plane(points, rng=rng, min_inliers=200, min_inlier_ratio=0.2)
    assert fit is not None
    assert abs(fit.normal[2]) > 0.97
    assert fit.inlier_count >= 700
    height = fit.height_at_origin()
    assert height == pytest.approx(-0.05, abs=0.03)


def test_rejects_vertical_wall():
    rng = np.random.default_rng(1)
    yz = rng.uniform(-5.0, 5.0, size=(600, 2))
    wall = np.column_stack((np.full(600, 4.0) + rng.normal(0.0, 0.02, 600), yz[:, 0], yz[:, 1]))
    fit = fit_waterline_plane(
        wall, rng=rng, max_tilt_deg=20.0, min_inliers=50, min_inlier_ratio=0.05,
    )
    assert fit is None


def test_prefers_water_over_wall():
    rng = np.random.default_rng(2)
    xy = rng.uniform(-6.0, 6.0, size=(700, 2))
    water = np.column_stack((xy[:, 0], xy[:, 1], np.zeros(700)))
    yz = rng.uniform(-4.0, 4.0, size=(400, 2))
    wall = np.column_stack((np.full(400, 5.0), yz[:, 0], yz[:, 1]))
    fit = fit_waterline_plane(
        np.vstack((water, wall)),
        rng=rng,
        min_inliers=200,
        min_inlier_ratio=0.1,
    )
    assert fit is not None
    assert abs(fit.normal[2]) > 0.95
    assert fit.inlier_count >= 650


def test_too_few_points_returns_none():
    points = np.array([[0.0, 0.0, 0.0], [1.0, 0.0, 0.0]], dtype=np.float64)
    assert fit_waterline_plane(points) is None


def test_plane_basis_is_orthonormal():
    normal = np.array([0.1, -0.05, 0.99], dtype=np.float64)
    normal = normal / np.linalg.norm(normal)
    u, v = plane_basis(normal)
    assert np.linalg.norm(u) == pytest.approx(1.0, abs=1e-6)
    assert np.linalg.norm(v) == pytest.approx(1.0, abs=1e-6)
    assert abs(float(u @ v)) < 1e-6
    assert abs(float(u @ normal)) < 1e-6
    assert abs(float(v @ normal)) < 1e-6
    assert np.linalg.norm(np.cross(u, v) - normal) < 1e-6


def test_sequential_planes_fit_remainder():
    rng = np.random.default_rng(3)
    xy_water = rng.uniform(-3.0, 3.0, size=(600, 2))
    water = np.column_stack((xy_water[:, 0], xy_water[:, 1], rng.normal(0.0, 0.02, 600)))
    xy_dock = rng.uniform(-3.0, 3.0, size=(400, 2))
    dock = np.column_stack((xy_dock[:, 0], xy_dock[:, 1], 0.45 + rng.normal(0.0, 0.02, 400)))
    points = np.vstack((water, dock))
    fits = fit_sequential_planes(
        points,
        num_planes=2,
        rng=rng,
        min_inliers=150,
        min_inlier_ratio=0.15,
        distance_threshold=0.08,
    )
    assert len(fits) == 2
    assert fits[0].inlier_count >= 500
    assert fits[1].inlier_count >= 300
    assert not np.any(fits[0].inlier_mask & fits[1].inlier_mask)
    heights = sorted(
        h for h in (fits[0].height_at_origin(), fits[1].height_at_origin()) if h is not None
    )
    assert heights[0] == pytest.approx(0.0, abs=0.05)
    assert heights[1] == pytest.approx(0.45, abs=0.05)


def test_second_plane_uses_lower_min_inliers():
    rng = np.random.default_rng(4)
    xy_water = rng.uniform(-3.0, 3.0, size=(500, 2))
    water = np.column_stack((xy_water[:, 0], xy_water[:, 1], rng.normal(0.0, 0.02, 500)))
    xy_small = rng.uniform(-1.0, 1.0, size=(20, 2))
    leftover = np.column_stack((
        xy_small[:, 0], xy_small[:, 1], 0.40 + rng.normal(0.0, 0.01, 20),
    ))
    points = np.vstack((water, leftover))
    fits = fit_sequential_planes(
        points,
        num_planes=2,
        first=RansacParams(
            distance_threshold=0.08,
            min_inliers=100,
            min_inlier_ratio=0.05,
        ),
        second=RansacParams(
            distance_threshold=0.08,
            min_inliers=10,
            min_inlier_ratio=0.01,
        ),
        rng=rng,
    )
    assert len(fits) == 2
    assert fits[0].inlier_count >= 400
    assert 10 <= fits[1].inlier_count < 100


def test_third_plane_fits_leftover_after_second():
    rng = np.random.default_rng(5)
    xy_water = rng.uniform(-3.0, 3.0, size=(500, 2))
    water = np.column_stack((xy_water[:, 0], xy_water[:, 1], rng.normal(0.0, 0.02, 500)))
    xy_mid = rng.uniform(-2.0, 2.0, size=(80, 2))
    mid = np.column_stack((xy_mid[:, 0], xy_mid[:, 1], 0.22 + rng.normal(0.0, 0.01, 80)))
    xy_hi = rng.uniform(-1.0, 1.0, size=(25, 2))
    high = np.column_stack((xy_hi[:, 0], xy_hi[:, 1], 0.40 + rng.normal(0.0, 0.01, 25)))
    points = np.vstack((water, mid, high))
    fits = fit_sequential_planes(
        points,
        num_planes=3,
        first=RansacParams(
            distance_threshold=0.06,
            min_inliers=100,
            min_inlier_ratio=0.05,
        ),
        second=RansacParams(
            distance_threshold=0.06,
            min_inliers=40,
            min_inlier_ratio=0.05,
        ),
        third=RansacParams(
            distance_threshold=0.06,
            min_inliers=10,
            min_inlier_ratio=0.02,
        ),
        rng=rng,
    )
    assert len(fits) == 3
    assert fits[0].inlier_count >= 400
    assert fits[1].inlier_count >= 60
    assert fits[2].inlier_count >= 15
    assert not np.any(fits[0].inlier_mask & fits[1].inlier_mask)
    assert not np.any(fits[0].inlier_mask & fits[2].inlier_mask)
    assert not np.any(fits[1].inlier_mask & fits[2].inlier_mask)
    heights = sorted(
        h for h in (
            fits[0].height_at_origin(),
            fits[1].height_at_origin(),
            fits[2].height_at_origin(),
        ) if h is not None
    )
    assert heights[0] == pytest.approx(0.0, abs=0.05)
    assert heights[1] == pytest.approx(0.22, abs=0.05)
    assert heights[2] == pytest.approx(0.40, abs=0.05)


def test_radial_mask_keeps_near_points():
    rng = np.random.default_rng(4)
    xy_water = rng.uniform(-3.0, 3.0, size=(500, 2))
    water = np.column_stack((xy_water[:, 0], xy_water[:, 1], rng.normal(0.0, 0.02, 500)))
    xy_small = rng.uniform(-1.0, 1.0, size=(20, 2))
    leftover = np.column_stack((
        xy_small[:, 0], xy_small[:, 1], 0.40 + rng.normal(0.0, 0.01, 20),
    ))
    points = np.vstack((water, leftover))
    fits = fit_sequential_planes(
        points,
        num_planes=2,
        first=RansacParams(
            distance_threshold=0.08,
            min_inliers=100,
            min_inlier_ratio=0.05,
        ),
        second=RansacParams(
            distance_threshold=0.08,
            min_inliers=10,
            min_inlier_ratio=0.01,
        ),
        rng=rng,
    )
    assert len(fits) == 2
    assert fits[0].inlier_count >= 400
    assert 10 <= fits[1].inlier_count < 100


def test_radial_mask_keeps_near_points():
    points = np.array([
        [1.0, 0.0, 0.0],
        [3.0, 4.0, 0.0],
        [10.0, 0.0, 0.0],
    ], dtype=np.float64)
    mask = radial_mask(points, 5.0)
    assert mask.tolist() == [True, True, False]
    assert radial_mask(points, 0.0).all()
