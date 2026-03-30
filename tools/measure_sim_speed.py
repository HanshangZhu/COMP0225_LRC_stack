#!/usr/bin/env python3
"""Sample /clock and report sim-time advance vs wall time (realtime factor).

Use while Gazebo + stack are running with use_sim_time. Subscribes with
BEST_EFFORT to match typical /clock publishers.
"""
from __future__ import annotations

import argparse
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rosgraph_msgs.msg import Clock


class ClockSampler(Node):
    def __init__(self) -> None:
        super().__init__("measure_sim_speed")
        qos = QoSProfile(
            depth=50,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            durability=DurabilityPolicy.VOLATILE,
        )
        self._sub = self.create_subscription(Clock, "/clock", self._cb, qos)
        self._first_wall: float | None = None
        self._first_sim_ns: int | None = None
        self._last_wall: float | None = None
        self._last_sim_ns: int | None = None
        self.samples = 0

    def _cb(self, msg: Clock) -> None:
        wall = time.time()
        sim_ns = int(msg.clock.sec) * 1_000_000_000 + int(msg.clock.nanosec)
        if self._first_wall is None:
            self._first_wall = wall
            self._first_sim_ns = sim_ns
        self._last_wall = wall
        self._last_sim_ns = sim_ns
        self.samples += 1


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--duration", type=float, default=30.0, help="Wall seconds to sample")
    args = p.parse_args()

    rclpy.init()
    node = ClockSampler()
    deadline = time.time() + args.duration
    while time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)

    if (
        node._first_wall is None
        or node._last_wall is None
        or node._first_sim_ns is None
        or node._last_sim_ns is None
        or node.samples < 2
    ):
        print("ERROR: insufficient /clock samples (is sim running?)", file=sys.stderr)
        node.destroy_node()
        rclpy.shutdown()
        return 1

    d_wall = node._last_wall - node._first_wall
    d_sim = (node._last_sim_ns - node._first_sim_ns) / 1e9
    rtf = d_sim / d_wall if d_wall > 1e-6 else 0.0
    print(
        f"samples={node.samples} wall_s={d_wall:.3f} sim_s={d_sim:.3f} "
        f"realtime_factor={rtf:.4f}"
    )
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
