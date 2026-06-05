#!/usr/bin/env python3
"""Publish a uniform wind vector field as RViz arrow markers."""

from __future__ import annotations

import argparse
import math

import rclpy
from geometry_msgs.msg import Quaternion
from rclpy.node import Node
from std_msgs.msg import Float32
from visualization_msgs.msg import Marker, MarkerArray


def _yaw_q(yaw: float) -> Quaternion:
    q = Quaternion()
    q.z = math.sin(yaw * 0.5)
    q.w = math.cos(yaw * 0.5)
    return q


def _speed_color(speed: float, speed_min: float, speed_max: float) -> tuple[float, float, float]:
    """Map speed to RGB: blue (calm) -> green -> yellow -> red (strong)."""
    if speed_max <= speed_min:
        t = 1.0
    else:
        t = max(0.0, min(1.0, (speed - speed_min) / (speed_max - speed_min)))
    if t < 0.33:
        u = t / 0.33
        return 0.0, 0.4 + 0.6 * u, 1.0 - 0.6 * u
    if t < 0.66:
        u = (t - 0.33) / 0.33
        return u, 1.0, 0.0
    u = (t - 0.66) / 0.34
    return 1.0, 1.0 - u, 0.0


class WindFieldVisualizer(Node):
    def __init__(
        self,
        *,
        direction_topic: str,
        speed_topic: str,
        output_topic: str,
        frame_id: str,
        x_min: float,
        x_max: float,
        y_min: float,
        y_max: float,
        spacing: float,
        z: float,
        arrow_length: float,
        speed_min: float,
        speed_max: float,
        shaft_diameter: float,
        head_diameter: float,
    ) -> None:
        super().__init__("wind_field_viz")
        self._frame_id = frame_id
        self._x_min = x_min
        self._x_max = x_max
        self._y_min = y_min
        self._y_max = y_max
        self._spacing = spacing
        self._z = z
        self._arrow_length = arrow_length
        self._speed_min = speed_min
        self._speed_max = speed_max
        self._shaft_diameter = shaft_diameter
        self._head_diameter = head_diameter

        self._direction_deg: float | None = None
        self._speed_mps: float | None = None

        self._pub = self.create_publisher(MarkerArray, output_topic, 10)
        self.create_subscription(Float32, direction_topic, self._on_direction, 10)
        self.create_subscription(Float32, speed_topic, self._on_speed, 10)
        self.create_timer(0.2, self._publish)

        self.get_logger().info(
            f"Wind field viz: {direction_topic} + {speed_topic} -> {output_topic}"
        )

    def _on_direction(self, msg: Float32) -> None:
        self._direction_deg = float(msg.data)

    def _on_speed(self, msg: Float32) -> None:
        self._speed_mps = float(msg.data)

    def _grid_points(self) -> list[tuple[float, float]]:
        xs = []
        x = self._x_min
        while x <= self._x_max + 1e-6:
            xs.append(x)
            x += self._spacing

        ys = []
        y = self._y_min
        while y <= self._y_max + 1e-6:
            ys.append(y)
            y += self._spacing

        return [(x, y) for y in ys for x in xs]

    def _publish(self) -> None:
        if self._direction_deg is None or self._speed_mps is None:
            return

        yaw = math.radians(self._direction_deg)
        speed = max(0.0, self._speed_mps)
        r, g, b = _speed_color(speed, self._speed_min, self._speed_max)

        stamp = self.get_clock().now().to_msg()
        arr = MarkerArray()
        for idx, (x, y) in enumerate(self._grid_points()):
            m = Marker()
            m.header.stamp = stamp
            m.header.frame_id = self._frame_id
            m.ns = "wind_field"
            m.id = idx
            m.type = Marker.ARROW
            m.action = Marker.ADD
            m.pose.position.x = x
            m.pose.position.y = y
            m.pose.position.z = self._z
            m.pose.orientation = _yaw_q(yaw)
            m.scale.x = self._arrow_length
            m.scale.y = self._shaft_diameter
            m.scale.z = self._head_diameter
            m.color.r = r
            m.color.g = g
            m.color.b = b
            m.color.a = 0.85
            arr.markers.append(m)

        self._pub.publish(arr)


def _parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="Visualize VRX wind as an RViz arrow field.")
    p.add_argument("--direction-topic", default="/vrx/debug/wind/direction")
    p.add_argument("--speed-topic", default="/vrx/debug/wind/speed")
    p.add_argument("--output-topic", default="/molo_mpc/wind_field")
    p.add_argument("--frame-id", default="world")
    p.add_argument("--x-min", type=float, default=-50.0)
    p.add_argument("--x-max", type=float, default=50.0)
    p.add_argument("--y-min", type=float, default=-50.0)
    p.add_argument("--y-max", type=float, default=50.0)
    p.add_argument("--spacing", type=float, default=10.0)
    p.add_argument("--z", type=float, default=2.0)
    p.add_argument("--arrow-length", type=float, default=3.0)
    p.add_argument("--speed-min", type=float, default=0.0, help="Speed (m/s) mapped to blue")
    p.add_argument("--speed-max", type=float, default=15.0, help="Speed (m/s) mapped to red")
    p.add_argument("--shaft-diameter", type=float, default=0.15)
    p.add_argument("--head-diameter", type=float, default=0.35)
    return p.parse_args()


def main() -> None:
    args = _parse_args()
    rclpy.init()
    node = WindFieldVisualizer(
        direction_topic=args.direction_topic,
        speed_topic=args.speed_topic,
        output_topic=args.output_topic,
        frame_id=args.frame_id,
        x_min=args.x_min,
        x_max=args.x_max,
        y_min=args.y_min,
        y_max=args.y_max,
        spacing=args.spacing,
        z=args.z,
        arrow_length=args.arrow_length,
        speed_min=args.speed_min,
        speed_max=args.speed_max,
        shaft_diameter=args.shaft_diameter,
        head_diameter=args.head_diameter,
    )
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
