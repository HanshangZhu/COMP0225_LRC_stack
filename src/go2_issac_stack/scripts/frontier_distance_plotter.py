#!/usr/bin/env python3
"""
Plot robot distance-to-frontier over time for multiple namespaces.

Inputs per namespace:
- `/<ns>/odom/nav` (nav_msgs/Odometry)
- `/<ns>/frontier_goal_marker` (visualization_msgs/Marker)

Output:
- Continuously updated PNG plot at `output_path`.
"""

from __future__ import annotations

from dataclasses import dataclass, field
import math
import os
from typing import Optional

import rclpy
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from visualization_msgs.msg import Marker

_MATPLOTLIB_ERROR: Optional[Exception] = None
try:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    MATPLOTLIB_AVAILABLE = True
except Exception as exc:  # pragma: no cover - runtime environment dependent
    MATPLOTLIB_AVAILABLE = False
    _MATPLOTLIB_ERROR = exc


@dataclass
class RobotSeries:
    pose_x: Optional[float] = None
    pose_y: Optional[float] = None
    goal_x: Optional[float] = None
    goal_y: Optional[float] = None
    times_s: list[float] = field(default_factory=list)
    distances_m: list[float] = field(default_factory=list)


class FrontierDistancePlotter(Node):
    def __init__(self) -> None:
        super().__init__("frontier_distance_plotter")

        self.declare_parameter("namespaces", ["go2_1", "go2_2"])
        self.declare_parameter("odom_topic_suffix", "/odom/nav")
        self.declare_parameter("frontier_marker_topic_suffix", "/frontier_goal_marker")
        self.declare_parameter("sample_rate_hz", 2.0)
        self.declare_parameter("plot_write_rate_hz", 0.5)
        self.declare_parameter("max_points", 7200)
        self.declare_parameter("output_path", "/tmp/frontier_distance_over_time.png")
        self.declare_parameter("log_every_n_writes", 10)

        raw_namespaces = self.get_parameter("namespaces").value
        if isinstance(raw_namespaces, (list, tuple)):
            self.namespaces = [str(v) for v in raw_namespaces]
        else:
            self.namespaces = [str(raw_namespaces)]

        self.odom_topic_suffix = str(self.get_parameter("odom_topic_suffix").value)
        self.frontier_marker_topic_suffix = str(
            self.get_parameter("frontier_marker_topic_suffix").value
        )
        self.sample_rate_hz = max(0.1, float(self.get_parameter("sample_rate_hz").value))
        self.plot_write_rate_hz = max(
            0.05, float(self.get_parameter("plot_write_rate_hz").value)
        )
        self.max_points = max(100, int(self.get_parameter("max_points").value))
        self.output_path = os.path.expanduser(
            str(self.get_parameter("output_path").value)
        )
        self.log_every_n_writes = max(
            1, int(self.get_parameter("log_every_n_writes").value)
        )

        output_dir = os.path.dirname(self.output_path) or "."
        os.makedirs(output_dir, exist_ok=True)

        self.start_time_s: Optional[float] = None
        self.plot_write_count = 0
        self.series: dict[str, RobotSeries] = {
            ns: RobotSeries() for ns in self.namespaces
        }

        for ns in self.namespaces:
            odom_topic = self._join_topic(ns, self.odom_topic_suffix)
            frontier_topic = self._join_topic(ns, self.frontier_marker_topic_suffix)

            self.create_subscription(
                Odometry,
                odom_topic,
                lambda msg, namespace=ns: self._odom_cb(msg, namespace),
                20,
            )
            self.create_subscription(
                Marker,
                frontier_topic,
                lambda msg, namespace=ns: self._frontier_cb(msg, namespace),
                20,
            )

        self.sample_timer = self.create_timer(1.0 / self.sample_rate_hz, self._sample_once)
        self.plot_timer = None
        if MATPLOTLIB_AVAILABLE:
            self.plot_timer = self.create_timer(
                1.0 / self.plot_write_rate_hz, self._write_plot
            )
        else:
            self.get_logger().warning(
                f"matplotlib unavailable; disabling plot output. import_error={_MATPLOTLIB_ERROR}"
            )

        self.get_logger().info(
            "Frontier distance plotter started | "
            f"namespaces={self.namespaces} | output={self.output_path} | "
            f"sample_rate_hz={self.sample_rate_hz:.2f} | "
            f"plot_write_rate_hz={self.plot_write_rate_hz:.2f}"
        )

    @staticmethod
    def _join_topic(ns: str, suffix: str) -> str:
        clean_ns = ns.strip("/")
        if suffix.startswith("/"):
            return f"/{clean_ns}{suffix}"
        return f"/{clean_ns}/{suffix}"

    def _now_s(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _odom_cb(self, msg: Odometry, ns: str) -> None:
        state = self.series[ns]
        state.pose_x = msg.pose.pose.position.x
        state.pose_y = msg.pose.pose.position.y

    def _frontier_cb(self, msg: Marker, ns: str) -> None:
        # Ignore clear/delete messages; distance is only meaningful for an active goal.
        if msg.action in (Marker.DELETE, Marker.DELETEALL):
            return
        state = self.series[ns]
        state.goal_x = msg.pose.position.x
        state.goal_y = msg.pose.position.y

    def _sample_once(self) -> None:
        now_s = self._now_s()
        if self.start_time_s is None:
            self.start_time_s = now_s
        t_rel = now_s - self.start_time_s

        for ns in self.namespaces:
            state = self.series[ns]
            if (
                state.pose_x is None
                or state.pose_y is None
                or state.goal_x is None
                or state.goal_y is None
            ):
                continue
            d = math.hypot(state.goal_x - state.pose_x, state.goal_y - state.pose_y)
            state.times_s.append(t_rel)
            state.distances_m.append(d)

            overflow = len(state.times_s) - self.max_points
            if overflow > 0:
                del state.times_s[:overflow]
                del state.distances_m[:overflow]

    def _write_plot(self) -> None:
        if not MATPLOTLIB_AVAILABLE:
            return

        any_series = any(self.series[ns].times_s for ns in self.namespaces)
        if not any_series:
            return

        fig, ax = plt.subplots(figsize=(10.5, 4.8))
        palette = ["#1f77b4", "#d62728", "#2ca02c", "#9467bd"]

        for idx, ns in enumerate(self.namespaces):
            state = self.series[ns]
            if not state.times_s:
                continue
            color = palette[idx % len(palette)]
            ax.plot(
                state.times_s,
                state.distances_m,
                color=color,
                linewidth=2.0,
                label=ns,
            )
            ax.scatter(
                [state.times_s[-1]],
                [state.distances_m[-1]],
                color=color,
                s=18,
                zorder=3,
            )

        ax.set_title("Robot Distance To Frontier Over Time")
        ax.set_xlabel("Time [s]")
        ax.set_ylabel("Distance [m]")
        ax.grid(True, alpha=0.30)
        ax.legend(loc="upper right")
        fig.tight_layout()

        tmp_path = f"{self.output_path}.tmp"
        fig.savefig(tmp_path, dpi=130)
        os.replace(tmp_path, self.output_path)
        plt.close(fig)

        self.plot_write_count += 1
        if self.plot_write_count % self.log_every_n_writes == 0:
            self.get_logger().info(
                f"Wrote frontier distance plot ({self.plot_write_count}) -> {self.output_path}"
            )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = FrontierDistancePlotter()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == "__main__":
    main()
