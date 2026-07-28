"""Shared path activation: YAML config or external nav_msgs/Path topic."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Callable, List, Optional, Tuple

from nav_msgs.msg import Path as NavPath
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from path_reference import PathSample, closest_index, rotate_path_to_index, samples_from_nav_path


def _wrap_angle(a: float) -> float:
    import math

    while a > math.pi:
        a -= 2.0 * math.pi
    while a <= -math.pi:
        a += 2.0 * math.pi
    return a


def _norm_frame(frame: str) -> str:
    return frame.strip().lstrip("/")


def _is_body_frame(frame: str, body_frames: set[str]) -> bool:
    norm = _norm_frame(frame)
    return norm in {_norm_frame(f) for f in body_frames}


def transform_body_path_to_map(
    samples: list[PathSample],
    origin_xy: tuple[float, float],
    yaw: float,
) -> list[PathSample]:
    """Rotate and translate a body-frame path into the map frame."""
    import math

    c, s = math.cos(yaw), math.sin(yaw)
    out: list[PathSample] = []
    for p in samples:
        out.append(
            PathSample(
                origin_xy[0] + c * p.x - s * p.y,
                origin_xy[1] + s * p.x + c * p.y,
                _wrap_angle(yaw + p.psi),
                p.u_ref,
                p.kappa,
            )
        )
    return out


OnPathReady = Callable[
    [List[PathSample], bool, Tuple[float, float], int, List[PathSample] | None],
    None,
]


@dataclass
class PathActivationResult:
    samples: List[PathSample]
    closed: bool
    origin_xy: Tuple[float, float]
    start_index: int


class PathActivationManager:
    """Load a mission path from config or an external ROS topic."""

    def __init__(
        self,
        node: Node,
        cfg: dict,
        on_ready: OnPathReady,
        build_config_path,
    ):
        self._node = node
        self._cfg = cfg
        self._on_ready = on_ready
        self._build_config_path = build_config_path

        path_cfg = cfg.get("path", {})
        wp_cfg = cfg.get("waypoints", {})
        self._source = str(path_cfg.get("source", "config")).lower()
        self._relative_to_start = bool(
            path_cfg.get("relative_to_start", wp_cfg.get("relative_to_start", True))
        )
        self._replan_on_update = bool(path_cfg.get("replan_on_update", False))
        self._expected_frame = str(cfg.get("frame_id", "world"))
        self._body_frames = set(path_cfg.get("body_frame_ids", []))
        self._transform_body_to_map = bool(path_cfg.get("transform_body_to_map", False))
        self._skip_rotate_to_closest = bool(path_cfg.get("skip_rotate_to_closest", False))
        self._trim_to_closest = bool(path_cfg.get("trim_to_closest", False))
        self._replan_min_interval_s = float(path_cfg.get("replan_min_interval_s", 0.0))
        self._replan_min_change_m = float(path_cfg.get("replan_min_change_m", 0.0))
        self._last_replan_ns = 0
        self._last_path_stamp = None
        self._viz_samples: List[PathSample] | None = None
        self._pending = True
        self._latched_path: Optional[NavPath] = None
        self._pose_xy: Optional[Tuple[float, float]] = None
        self._pose_yaw: Optional[float] = None
        self._path_sub = None

        if self._source == "topic":
            topic = str(path_cfg.get("reference_path_topic", "/plan"))
            qos = QoSProfile(
                depth=10,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.VOLATILE,
            )
            self._path_sub = node.create_subscription(NavPath, topic, self._on_path_msg, qos)
            node.get_logger().info(f"Path source: topic ({topic})")
        else:
            node.get_logger().info("Path source: config")

    @property
    def uses_topic_source(self) -> bool:
        return self._source == "topic"

    @property
    def pending(self) -> bool:
        return self._pending

    def notify_pose(self, x: float, y: float, yaw: float | None = None) -> None:
        self._pose_xy = (x, y)
        if yaw is not None:
            self._pose_yaw = yaw
        self._try_activate()

    def _path_endpoints(self, msg: NavPath) -> tuple[tuple[float, float], tuple[float, float]] | None:
        if len(msg.poses) < 2:
            return None
        a = msg.poses[0].pose.position
        b = msg.poses[-1].pose.position
        return (float(a.x), float(a.y)), (float(b.x), float(b.y))

    def _path_changed_enough(self, prev: NavPath | None, msg: NavPath) -> bool:
        if prev is None or self._replan_min_change_m <= 0.0:
            return True
        old_ep = self._path_endpoints(prev)
        new_ep = self._path_endpoints(msg)
        if old_ep is None or new_ep is None:
            return True
        (ox0, oy0), (ox1, oy1) = old_ep
        (nx0, ny0), (nx1, ny1) = new_ep
        delta = max(
            math.hypot(nx0 - ox0, ny0 - oy0),
            math.hypot(nx1 - ox1, ny1 - oy1),
        )
        return delta >= self._replan_min_change_m

    def _on_path_msg(self, msg: NavPath) -> None:
        if len(msg.poses) < 2:
            return
        frame = str(msg.header.frame_id)
        if (
            frame
            and _norm_frame(frame) != _norm_frame(self._expected_frame)
            and not _is_body_frame(frame, self._body_frames)
            and not self._transform_body_to_map
        ):
            self._node.get_logger().warning(
                f"Reference path frame '{frame}' != follower frame '{self._expected_frame}'"
            )

        # Always latch the latest planner path for visualization (ref_path).
        prev_path = self._latched_path
        self._latched_path = msg
        self._last_path_stamp = msg.header.stamp
        if self._pose_xy is not None:
            viz, _ = samples_from_nav_path(msg, self._cfg, self._pose_xy)
            if len(viz) >= 2:
                self._viz_samples = list(viz)

        if self._replan_on_update and not self._pending:
            now_ns = self._node.get_clock().now().nanoseconds
            if (
                self._replan_min_interval_s > 0.0
                and self._last_replan_ns > 0
                and (now_ns - self._last_replan_ns) < int(self._replan_min_interval_s * 1e9)
            ):
                return
            if not self._path_changed_enough(prev_path, msg):
                return
            self._pending = True
        self._try_activate()

    def _needs_body_transform(self, frame: str) -> bool:
        return self._transform_body_to_map and _is_body_frame(frame, self._body_frames)

    def _try_activate(self) -> None:
        if not self._pending:
            return

        if self._source == "topic":
            if self._pose_xy is None or self._latched_path is None:
                return
            frame = str(self._latched_path.header.frame_id)
            if self._needs_body_transform(frame) and self._pose_yaw is None:
                return
            samples, closed = samples_from_nav_path(self._latched_path, self._cfg, self._pose_xy)
            if self._needs_body_transform(frame):
                samples = transform_body_path_to_map(samples, self._pose_xy, self._pose_yaw)
                self._node.get_logger().info(
                    f"Transformed body-frame path ({_norm_frame(frame)}) to {self._expected_frame}"
                )
        else:
            if self._relative_to_start:
                if self._pose_xy is None:
                    return
                origin = self._pose_xy
            else:
                origin = (0.0, 0.0)
            samples, closed = self._build_config_path(self._cfg, origin)

        if len(samples) < 2:
            return

        origin_xy = self._pose_xy if self._pose_xy is not None else (0.0, 0.0)
        viz_samples = list(samples)
        if self._trim_to_closest:
            idx0 = closest_index(samples, origin_xy[0], origin_xy[1])
            samples = samples[idx0:]
        elif self._skip_rotate_to_closest:
            idx0 = 0
        else:
            idx0 = closest_index(samples, origin_xy[0], origin_xy[1])
            samples = rotate_path_to_index(samples, idx0)
        if len(samples) < 2:
            return
        self._pending = False
        self._last_replan_ns = self._node.get_clock().now().nanoseconds
        if self._latched_path is not None:
            self._last_path_stamp = self._latched_path.header.stamp
        self._viz_samples = viz_samples
        self._on_ready(samples, closed, origin_xy, idx0, viz_samples)
