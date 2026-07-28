#!/usr/bin/env python3
"""Unit tests for path topic integration (no ROS runtime required)."""

from __future__ import annotations

import sys
import unittest
from pathlib import Path

MPC_DIR = Path(__file__).resolve().parents[1] / "mpc"
MOLO_DIR = MPC_DIR.parent
for path in (str(MPC_DIR), str(MOLO_DIR)):
    if path not in sys.path:
        sys.path.insert(0, path)

from path_reference import samples_from_nav_path  # noqa: E402


class _Pose:
    def __init__(self, x: float, y: float):
        self.position = type("P", (), {"x": x, "y": y})()
        self.orientation = type("Q", (), {"w": 1.0})()


class _PoseStamped:
    def __init__(self, frame_id: str, x: float, y: float):
        self.header = type("H", (), {"frame_id": frame_id})()
        self.pose = _Pose(x, y)


class _NavPath:
    def __init__(self, frame_id: str = "map"):
        self.header = type("H", (), {"frame_id": frame_id})()
        self.poses: list = []


def _line_path(frame_id: str = "map") -> _NavPath:
    msg = _NavPath(frame_id)
    for x in (0.0, 10.0, 20.0, 30.0):
        msg.poses.append(_PoseStamped(frame_id, x, 0.0))
    return msg


class PathFromRosTests(unittest.TestCase):
    def test_absolute_map_path(self):
        cfg = {
            "path": {
                "relative_to_start": False,
                "resample_step_m": 1.0,
                "cruise_speed_mps": 0.5,
                "smooth_dubins": False,
                "closed": False,
            }
        }
        samples, closed = samples_from_nav_path(_line_path(), cfg, (0.0, 0.0))
        self.assertFalse(closed)
        self.assertGreaterEqual(len(samples), 3)
        self.assertAlmostEqual(samples[0].x, 0.0, places=3)
        self.assertAlmostEqual(samples[-1].x, 30.0, places=1)

    def test_relative_path_offsets_by_pose(self):
        cfg = {
            "path": {
                "relative_to_start": True,
                "resample_step_m": 2.0,
                "cruise_speed_mps": 0.5,
                "smooth_dubins": False,
                "closed": False,
            }
        }
        samples, _ = samples_from_nav_path(_line_path(), cfg, (100.0, 5.0))
        self.assertAlmostEqual(samples[0].x, 100.0, places=3)
        self.assertAlmostEqual(samples[0].y, 5.0, places=3)


class PathFollowerConfigTests(unittest.TestCase):
    def test_build_ilos_external_path(self):
        sys.path.insert(0, str(MOLO_DIR / "path_follower"))
        from config import build_path_follower_config  # noqa: WPS433

        cfg = build_path_follower_config(controller="ilos", platform="sim", use_external_path=True)
        self.assertEqual(cfg["path"]["source"], "topic")
        self.assertEqual(cfg["path"]["reference_path_topic"], "/molo_mpc/reference_path")


if __name__ == "__main__":
    unittest.main()
