#!/usr/bin/env python3
"""Synthetic source publisher for mtare_behavior_executive_cpp standalone testing."""

from __future__ import annotations

import math

import rclpy
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node


class ExactBackendMockSources(Node):
    def __init__(self) -> None:
        super().__init__("exact_backend_mock_sources")

        self.declare_parameter("namespaces", ["robot_a", "robot_b"])
        self.declare_parameter("publish_rate_hz", 8.0)
        self.declare_parameter("phase_period_sec", 12.0)
        self.declare_parameter("map_frame", "world")
        self.declare_parameter("tare_suffix", "/way_point_tare")
        self.declare_parameter("far_suffix", "/way_point_far")
        self.declare_parameter("goal_suffix", "/goal_point")
        self.declare_parameter("odom_suffix", "/odom/nav")

        self.namespaces = [str(x) for x in self.get_parameter("namespaces").value]
        self.publish_rate_hz = max(1.0, float(self.get_parameter("publish_rate_hz").value))
        self.phase_period_sec = max(1.0, float(self.get_parameter("phase_period_sec").value))
        self.map_frame = str(self.get_parameter("map_frame").value)
        self.tare_suffix = str(self.get_parameter("tare_suffix").value)
        self.far_suffix = str(self.get_parameter("far_suffix").value)
        self.goal_suffix = str(self.get_parameter("goal_suffix").value)
        self.odom_suffix = str(self.get_parameter("odom_suffix").value)

        self._start_time = self.get_clock().now()
        self._tick_count = 0
        self._phase_count = 4
        self._last_phase = -1

        self.tare_pubs = {}
        self.far_pubs = {}
        self.goal_pubs = {}
        self.odom_pubs = {}
        for idx, ns in enumerate(self.namespaces):
            self.tare_pubs[ns] = self.create_publisher(PointStamped, f"/{ns}{self.tare_suffix}", 10)
            self.far_pubs[ns] = self.create_publisher(PointStamped, f"/{ns}{self.far_suffix}", 10)
            self.goal_pubs[ns] = self.create_publisher(PointStamped, f"/{ns}{self.goal_suffix}", 10)
            self.odom_pubs[ns] = self.create_publisher(Odometry, f"/{ns}{self.odom_suffix}", 10)
            self._publish_odom(ns=ns, robot_idx=idx)

        self.timer = self.create_timer(1.0 / self.publish_rate_hz, self._tick)
        self.get_logger().info(
            "exact_backend_mock_sources started | "
            f"namespaces={self.namespaces} publish_rate_hz={self.publish_rate_hz:.1f} "
            f"phase_period_sec={self.phase_period_sec:.1f}"
        )

    def _phase(self) -> int:
        elapsed = (self.get_clock().now() - self._start_time).nanoseconds / 1e9
        return int(elapsed // self.phase_period_sec) % self._phase_count

    def _point_msg(self, x: float, y: float, frame_id: str) -> PointStamped:
        msg = PointStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = frame_id
        msg.point.x = float(x)
        msg.point.y = float(y)
        msg.point.z = 0.0
        return msg

    def _publish_odom(self, ns: str, robot_idx: int) -> None:
        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.map_frame
        msg.child_frame_id = "base_link"
        base_x = 1.0 + robot_idx * 10.0
        msg.pose.pose.position.x = base_x + 0.2 * math.sin(self._tick_count * 0.05)
        msg.pose.pose.position.y = 0.2 * math.cos(self._tick_count * 0.05)
        msg.pose.pose.orientation.w = 1.0
        self.odom_pubs[ns].publish(msg)

    def _tick(self) -> None:
        phase = self._phase()
        if phase != self._last_phase:
            self._last_phase = phase
            self.get_logger().info(
                f"phase={phase} "
                "(0=local_tare, 1=far_transit, 2=stale_inputs, 3=goal_fallback)"
            )

        for idx, ns in enumerate(self.namespaces):
            base_x = 2.0 + idx * 10.0
            self._publish_odom(ns=ns, robot_idx=idx)
            if phase == 0:
                # Local planner dominates.
                self.tare_pubs[ns].publish(self._point_msg(base_x + 2.0, 0.0, self.map_frame))
            elif phase == 1:
                # FAR planner dominates with relocation target present.
                self.far_pubs[ns].publish(self._point_msg(base_x + 8.0, 1.5, self.map_frame))
                self.goal_pubs[ns].publish(self._point_msg(base_x + 10.0, 0.0, self.map_frame))
            elif phase == 2:
                # Intentionally publish no waypoint sources to exercise stale/recovery behavior.
                pass
            else:
                # Goal fallback only.
                self.goal_pubs[ns].publish(self._point_msg(base_x + 6.0, -1.0, self.map_frame))

        self._tick_count += 1


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ExactBackendMockSources()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
