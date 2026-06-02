#!/usr/bin/env python3
"""Evaluate MPC cross-track error from /molo_mpc/cross_track_error."""

from __future__ import annotations

import argparse
import math
import sys
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32


class MpcEvaluator(Node):
    def __init__(self, duration: float, skip_initial: float = 0.0):
        super().__init__("mpc_evaluator")
        self.samples: list[float] = []
        self.create_subscription(Float32, "/molo_mpc/cross_track_error", self._cb, 10)
        self._t0 = time.monotonic()
        self._duration = duration
        self._skip_until = self._t0 + skip_initial
        self.create_timer(0.5, self._done)

    def _cb(self, msg: Float32) -> None:
        if time.monotonic() < self._skip_until:
            return
        v = float(msg.data)
        if math.isfinite(v):
            self.samples.append(abs(v))

    def _done(self) -> None:
        if time.monotonic() - self._t0 < self._duration:
            return
        if not self.samples:
            print("SCORE inf", flush=True)
            raise SystemExit(0)
        rmse = math.sqrt(sum(s * s for s in self.samples) / len(self.samples))
        mean = sum(self.samples) / len(self.samples)
        mx = max(self.samples)
        self.get_logger().info(f"XTE rmse={rmse:.4f} mean={mean:.4f} max={mx:.4f} n={len(self.samples)}")
        print(f"SCORE {rmse:.6f}", flush=True)
        raise SystemExit(0)


def main() -> None:
    p = argparse.ArgumentParser()
    p.add_argument("--duration", type=float, default=90.0)
    p.add_argument("--skip-initial", type=float, default=0.0, help="Discard samples before this time (s)")
    args = p.parse_args()
    rclpy.init()
    node = MpcEvaluator(args.duration, skip_initial=args.skip_initial)
    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
