#!/usr/bin/env python3
"""Collect trajectory tracking metrics during a wpt_follower run."""

from __future__ import annotations

import argparse
import math
import sys
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import List, Optional, Tuple

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from rclpy.node import Node
from std_msgs.msg import Float32


def _dist_point_to_segment(
    px: float, py: float, ax: float, ay: float, bx: float, by: float
) -> float:
    dx, dy = bx - ax, by - ay
    len_sq = dx * dx + dy * dy
    if len_sq < 1e-9:
        return math.hypot(px - ax, py - ay)
    t = max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / len_sq))
    return math.hypot(px - (ax + t * dx), py - (ay + t * dy))


def cross_track_to_path(px: float, py: float, path_xy: List[Tuple[float, float]]) -> float:
    if len(path_xy) < 2:
        if path_xy:
            return math.hypot(px - path_xy[0][0], py - path_xy[0][1])
        return float("inf")
    return min(
        _dist_point_to_segment(px, py, path_xy[i][0], path_xy[i][1], path_xy[i + 1][0], path_xy[i + 1][1])
        for i in range(len(path_xy) - 1)
    )


@dataclass
class TrackingMetrics:
    xte_samples: List[float] = field(default_factory=list)
    head_err_samples: List[float] = field(default_factory=list)
    path_xte_samples: List[float] = field(default_factory=list)

    def add(self, xte: Optional[float], head_deg: Optional[float], path_xte: Optional[float]) -> None:
        if xte is not None and math.isfinite(xte):
            self.xte_samples.append(abs(xte))
        if head_deg is not None and math.isfinite(head_deg):
            self.head_err_samples.append(abs(head_deg))
        if path_xte is not None and math.isfinite(path_xte):
            self.path_xte_samples.append(path_xte)

    @staticmethod
    def _stats(vals: List[float]) -> dict:
        if not vals:
            return {"mean": float("inf"), "max": float("inf"), "rmse": float("inf"), "n": 0}
        mean = sum(vals) / len(vals)
        rmse = math.sqrt(sum(v * v for v in vals) / len(vals))
        return {"mean": mean, "max": max(vals), "rmse": rmse, "n": len(vals)}

    def summary(self) -> dict:
        xte = self._stats(self.xte_samples)
        path = self._stats(self.path_xte_samples)
        head = self._stats(self.head_err_samples)
        primary = path if path["n"] > 0 else xte
        return {
            "xte": xte,
            "path_xte": path,
            "heading_err_deg": head,
            "score": primary["rmse"],
        }


class TrackingEvaluator(Node):
    def __init__(self, duration_s: float):
        super().__init__("tracking_evaluator")
        self.duration_s = duration_s
        self.metrics = TrackingMetrics()
        self._path_xy: List[Tuple[float, float]] = []
        self._mission_xy: List[Tuple[float, float]] = []
        self._last_xte: Optional[float] = None
        self._last_head: Optional[float] = None
        self._pose: Optional[Tuple[float, float]] = None

        self.create_subscription(Float32, "/molo_wpt/cross_track_error", self._xte_cb, 10)
        self.create_subscription(Float32, "/molo_wpt/heading_error_deg", self._head_cb, 10)
        self.create_subscription(PoseStamped, "/molo_wpt/vehicle_pose", self._pose_cb, 10)
        self.create_subscription(Path, "/molo_wpt/path", self._path_cb, 10)
        self.create_subscription(Path, "/molo_wpt/mission_path", self._mission_cb, 10)
        self.create_timer(0.5, self._sample)

        self._start = time.monotonic()
        self._done = False

    def _xte_cb(self, msg: Float32) -> None:
        self._last_xte = float(msg.data)

    def _head_cb(self, msg: Float32) -> None:
        self._last_head = float(msg.data)

    def _pose_cb(self, msg: PoseStamped) -> None:
        self._pose = (msg.pose.position.x, msg.pose.position.y)

    def _path_cb(self, msg: Path) -> None:
        self._path_xy = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]

    def _mission_cb(self, msg: Path) -> None:
        self._mission_xy = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]

    def _reference_path(self) -> List[Tuple[float, float]]:
        if len(self._mission_xy) >= 2:
            return self._mission_xy
        return self._path_xy

    def _sample(self) -> None:
        path_xte = None
        ref = self._reference_path()
        if self._pose and ref:
            path_xte = cross_track_to_path(self._pose[0], self._pose[1], ref)
        self.metrics.add(self._last_xte, self._last_head, path_xte)

        if time.monotonic() - self._start >= self.duration_s and not self._done:
            self._done = True
            summary = self.metrics.summary()
            self.get_logger().info(
                f"EVAL score(rmse)={summary['score']:.4f} "
                f"path_xte_mean={summary['path_xte']['mean']:.4f} "
                f"path_xte_max={summary['path_xte']['max']:.4f} "
                f"xte_mean={summary['xte']['mean']:.4f} "
                f"head_mean={summary['heading_err_deg']['mean']:.2f}deg "
                f"n={summary['path_xte']['n']}"
            )
            print(f"SCORE {summary['score']:.6f}", flush=True)
            raise SystemExit(0)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=90.0, help="Evaluation window (seconds)")
    args = parser.parse_args()

    rclpy.init()
    node = TrackingEvaluator(args.duration)
    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
