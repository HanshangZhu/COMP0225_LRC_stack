#!/usr/bin/env python3
"""Single-robot exploration readiness gate for Isaac integration.

Exits with code 0 when required runtime criteria remain healthy for a
continuous stable window. Exits with code 1 on timeout.

Readiness means:
- required streams are present and not stale
- mapping coverage has crossed the configured threshold
- nav mode is active (not idle/waiting)
"""

from __future__ import annotations

from collections import deque
import json
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, PointCloud2
from std_msgs.msg import String


class ReadinessGate(Node):
    def __init__(self) -> None:
        super().__init__("readiness_gate")

        self.declare_parameter("odom_topic", "/go2_1/odom/nav")
        self.declare_parameter("imu_topic", "/go2_1/imu/data")
        self.declare_parameter("pointcloud_topic", "/go2_1/registered_scan")
        self.declare_parameter("cmd_topic", "/go2_1/isaac/cmd_vel")
        self.declare_parameter("map_topic", "/go2_1/map")
        self.declare_parameter("nav_status_topic", "/go2_1/nav_status")

        self.declare_parameter("startup_timeout_sec", 180.0)
        self.declare_parameter("stale_timeout_sec", 2.5)
        self.declare_parameter("stable_window_sec", 20.0)
        self.declare_parameter("required_coverage_ratio", 0.80)
        self.declare_parameter("wall_occupancy_threshold", 65)
        self.declare_parameter("min_wall_cells", 40)
        self.declare_parameter("coverage_mode", "flood_fill")
        self.declare_parameter("min_bounded_cells", 1200)

        self.odom_topic = str(self.get_parameter("odom_topic").value)
        self.imu_topic = str(self.get_parameter("imu_topic").value)
        self.pointcloud_topic = str(self.get_parameter("pointcloud_topic").value)
        self.cmd_topic = str(self.get_parameter("cmd_topic").value)
        self.map_topic = str(self.get_parameter("map_topic").value)
        self.nav_status_topic = str(self.get_parameter("nav_status_topic").value)

        self.startup_timeout_sec = float(self.get_parameter("startup_timeout_sec").value)
        self.stale_timeout_sec = float(self.get_parameter("stale_timeout_sec").value)
        self.stable_window_sec = float(self.get_parameter("stable_window_sec").value)
        self.required_coverage_ratio = float(self.get_parameter("required_coverage_ratio").value)
        self.wall_occupancy_threshold = int(self.get_parameter("wall_occupancy_threshold").value)
        self.min_wall_cells = int(self.get_parameter("min_wall_cells").value)
        self.coverage_mode = str(self.get_parameter("coverage_mode").value).strip().lower()
        self.min_bounded_cells = int(self.get_parameter("min_bounded_cells").value)

        self._start_time = time.monotonic()
        self._stable_start: float | None = None
        self.done = False
        self.exit_code = 1

        self._last_seen: dict[str, float | None] = {
            "odom": None,
            "imu": None,
            "pointcloud": None,
            "cmd": None,
            "map": None,
            "nav_status": None,
        }

        self._bounded_coverage_ratio: float | None = None
        self._bounded_total_cells = 0
        self._bounded_wall_cells = 0
        self._nav_mode = ""
        self._last_progress_bucket = -1

        # Subscriptions represent the minimum health contract for single-robot run.
        self.create_subscription(Odometry, self.odom_topic, self._odom_cb, qos_profile_sensor_data)
        self.create_subscription(Imu, self.imu_topic, self._imu_cb, qos_profile_sensor_data)
        self.create_subscription(PointCloud2, self.pointcloud_topic, self._pointcloud_cb, qos_profile_sensor_data)
        self.create_subscription(Twist, self.cmd_topic, self._cmd_cb, 10)
        self.create_subscription(OccupancyGrid, self.map_topic, self._map_cb, 1)
        self.create_subscription(String, self.nav_status_topic, self._nav_status_cb, 10)

        # Timer drives the readiness state machine and process exit decision.
        self.timer = self.create_timer(0.5, self._tick)

        self.get_logger().info(
            "Readiness gate started | "
            f"timeout={self.startup_timeout_sec:.1f}s stale={self.stale_timeout_sec:.1f}s "
            f"stable_window={self.stable_window_sec:.1f}s "
            f"required_coverage_ratio={self.required_coverage_ratio:.2f}"
        )

    def _mark(self, key: str) -> None:
        self._last_seen[key] = time.monotonic()

    def _odom_cb(self, _msg: Odometry) -> None:
        self._mark("odom")

    def _imu_cb(self, _msg: Imu) -> None:
        self._mark("imu")

    def _pointcloud_cb(self, _msg: PointCloud2) -> None:
        self._mark("pointcloud")

    def _cmd_cb(self, _msg: Twist) -> None:
        self._mark("cmd")

    def _map_cb(self, msg: OccupancyGrid) -> None:
        self._mark("map")
        width = int(msg.info.width)
        height = int(msg.info.height)
        data = msg.data

        if width <= 0 or height <= 0 or not data:
            self._bounded_coverage_ratio = None
            return

        wall_indices = [idx for idx, val in enumerate(data) if val >= self.wall_occupancy_threshold]
        self._bounded_wall_cells = len(wall_indices)
        if self._bounded_wall_cells < self.min_wall_cells:
            self._bounded_coverage_ratio = None
            self._bounded_total_cells = 0
            return

        # Two coverage estimators are supported:
        # - bbox: quick but coarse
        # - flood_fill: robust bounded-area estimate (default)
        mode = self.coverage_mode if self.coverage_mode in ("flood_fill", "bbox") else "flood_fill"
        if mode == "bbox":
            xs = [idx % width for idx in wall_indices]
            ys = [idx // width for idx in wall_indices]
            min_x = min(xs)
            max_x = max(xs)
            min_y = min(ys)
            max_y = max(ys)

            bbox_w = (max_x - min_x) + 1
            bbox_h = (max_y - min_y) + 1
            bbox_total = bbox_w * bbox_h
            if bbox_total <= 0:
                self._bounded_coverage_ratio = None
                self._bounded_total_cells = 0
                return

            known = 0
            for gy in range(min_y, max_y + 1):
                row = gy * width
                for gx in range(min_x, max_x + 1):
                    if data[row + gx] >= 0:
                        known += 1

            self._bounded_total_cells = bbox_total
            self._bounded_coverage_ratio = known / float(bbox_total)
            return

        wall_mask = [val >= self.wall_occupancy_threshold for val in data]
        exterior = [False] * (width * height)
        q: deque[int] = deque()

        def push_if_exterior(idx: int) -> None:
            if wall_mask[idx] or exterior[idx]:
                return
            exterior[idx] = True
            q.append(idx)

        # Seed flood fill from map border cells to classify the outside region.
        for x in range(width):
            push_if_exterior(x)
            push_if_exterior((height - 1) * width + x)
        for y in range(height):
            row = y * width
            push_if_exterior(row)
            push_if_exterior(row + (width - 1))

        while q:
            idx = q.popleft()
            x = idx % width
            y = idx // width
            if x > 0:
                push_if_exterior(idx - 1)
            if x < (width - 1):
                push_if_exterior(idx + 1)
            if y > 0:
                push_if_exterior(idx - width)
            if y < (height - 1):
                push_if_exterior(idx + width)

        bounded_total = 0
        known = 0
        for idx, is_wall in enumerate(wall_mask):
            if is_wall or exterior[idx]:
                continue
            bounded_total += 1
            if data[idx] >= 0:
                known += 1

        self._bounded_total_cells = bounded_total
        if bounded_total < self.min_bounded_cells:
            self._bounded_coverage_ratio = None
            return

        self._bounded_coverage_ratio = known / float(bounded_total)

    def _nav_status_cb(self, msg: String) -> None:
        self._mark("nav_status")
        try:
            payload = json.loads(msg.data)
            self._nav_mode = str(payload.get("mode", ""))
        except Exception:
            self._nav_mode = ""

    def _missing_or_stale(self, now: float) -> list[str]:
        bad = []
        for key, ts in self._last_seen.items():
            if ts is None:
                bad.append(f"{key}:missing")
                continue
            age = now - ts
            if age > self.stale_timeout_sec:
                bad.append(f"{key}:stale({age:.1f}s)")
        return bad

    def _tick(self) -> None:
        now = time.monotonic()
        elapsed = now - self._start_time

        bad_streams = self._missing_or_stale(now)
        coverage = self._bounded_coverage_ratio
        mode_ok = self._nav_mode not in ("", "idle", "waiting")
        coverage_ok = coverage is not None and coverage >= self.required_coverage_ratio
        healthy = (not bad_streams) and coverage_ok and mode_ok

        # A short healthy blip is not enough; we require a continuous stable window.
        if healthy:
            if self._stable_start is None:
                self._stable_start = now
            stable_for = now - self._stable_start
            if stable_for >= self.stable_window_sec:
                self.get_logger().info(
                    "READINESS PASS | "
                    f"stable_for={stable_for:.1f}s "
                    f"coverage={coverage:.3f}/{self.required_coverage_ratio:.3f} "
                    f"bbox_cells={self._bounded_total_cells} mode={self._nav_mode}"
                )
                self.exit_code = 0
                self.done = True
                return
        else:
            self._stable_start = None

        bucket = int(elapsed // 10)
        if bucket != self._last_progress_bucket:
            self._last_progress_bucket = bucket
            self.get_logger().info(
                "Readiness progress | "
                f"elapsed={elapsed:.1f}s bad={bad_streams or 'none'} "
                f"coverage={coverage if coverage is not None else 'n/a'}/{self.required_coverage_ratio:.3f} "
                f"bbox_cells={self._bounded_total_cells} wall_cells={self._bounded_wall_cells} "
                f"mode={self._nav_mode or 'unknown'}"
            )

        # Hard timeout gate for CI/automation; process exits with failure code.
        if elapsed > self.startup_timeout_sec:
            self.get_logger().error(
                "READINESS FAIL | "
                f"timeout={elapsed:.1f}s bad={bad_streams or 'none'} "
                f"coverage={coverage if coverage is not None else 'n/a'}/{self.required_coverage_ratio:.3f} "
                f"bbox_cells={self._bounded_total_cells} wall_cells={self._bounded_wall_cells} "
                f"mode={self._nav_mode or 'unknown'}"
            )
            self.exit_code = 1
            self.done = True


def main(args=None) -> int:
    rclpy.init(args=args)
    node = ReadinessGate()
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.25)
    finally:
        code = node.exit_code
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return code


if __name__ == "__main__":
    raise SystemExit(main())
