#!/usr/bin/env python3
"""PointCloud2 contract probe for Isaac/ROS2 autonomy pipelines.

Reports whether the perception->mapping contract looks valid:
- frame id and publish rate
- width/height and fields
- valid XYZ ratio
- range and Z spread
- optional TF availability to a target frame
"""

from __future__ import annotations

import math
import struct
from dataclasses import dataclass
from typing import Optional

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import PointCloud2
from tf2_ros import Buffer, TransformException, TransformListener


@dataclass
class PointFieldOffsets:
    x: int
    y: int
    z: int


class PointcloudContractProbe(Node):
    def __init__(self) -> None:
        super().__init__("pointcloud_contract_probe")

        self.declare_parameter("topic", "/go2_1/lidar/points")
        self.declare_parameter("target_frame", "world")
        self.declare_parameter("check_tf", True)
        self.declare_parameter("report_hz", 1.0)
        self.declare_parameter("warn_min_valid_ratio", 0.50)
        self.declare_parameter("warn_min_z_spread_m", 0.50)
        self.declare_parameter("warn_min_points_per_sec", 10000.0)

        topic = str(self.get_parameter("topic").value)
        self.target_frame = str(self.get_parameter("target_frame").value)
        self.check_tf = bool(self.get_parameter("check_tf").value)
        report_hz = max(0.2, float(self.get_parameter("report_hz").value))
        self.warn_min_valid_ratio = max(0.0, min(1.0, float(self.get_parameter("warn_min_valid_ratio").value)))
        self.warn_min_z_spread_m = max(0.0, float(self.get_parameter("warn_min_z_spread_m").value))
        self.warn_min_points_per_sec = max(1.0, float(self.get_parameter("warn_min_points_per_sec").value))

        self.tf_buffer: Optional[Buffer] = None
        self.tf_listener: Optional[TransformListener] = None
        if self.check_tf:
            self.tf_buffer = Buffer(cache_time=Duration(seconds=5.0))
            self.tf_listener = TransformListener(self.tf_buffer, self)

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        self.create_subscription(PointCloud2, topic, self._pointcloud_cb, qos)

        self.last_frame_id = ""
        self.last_stamp_ns = 0
        self.last_width = 0
        self.last_height = 0
        self.last_fields = ""
        self.first_msg_stamp_ns = 0
        self.msg_count = 0
        self.point_count_total = 0
        self.valid_count_total = 0
        self.min_range = math.inf
        self.max_range = 0.0
        self.min_z = math.inf
        self.max_z = -math.inf

        self.create_timer(1.0 / report_hz, self._report)
        self.get_logger().info(f"Probing PointCloud2 contract on topic: {topic}")

    def _field_offsets(self, msg: PointCloud2) -> Optional[PointFieldOffsets]:
        x_off = y_off = z_off = None
        names = []
        for f in msg.fields:
            names.append(f.name)
            if f.name == "x":
                x_off = f.offset
            elif f.name == "y":
                y_off = f.offset
            elif f.name == "z":
                z_off = f.offset
        self.last_fields = ",".join(names)
        if x_off is None or y_off is None or z_off is None:
            return None
        return PointFieldOffsets(x=x_off, y=y_off, z=z_off)

    def _pointcloud_cb(self, msg: PointCloud2) -> None:
        offsets = self._field_offsets(msg)
        self.msg_count += 1
        self.last_frame_id = msg.header.frame_id
        self.last_stamp_ns = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        if self.first_msg_stamp_ns == 0 and self.last_stamp_ns > 0:
            self.first_msg_stamp_ns = self.last_stamp_ns
        self.last_width = msg.width
        self.last_height = msg.height

        if offsets is None or msg.point_step <= 0:
            return

        n_points = int(msg.width) * int(msg.height)
        self.point_count_total += n_points
        if n_points <= 0:
            return

        data = msg.data
        step = msg.point_step
        valid = 0
        for i in range(n_points):
            base = i * step
            try:
                x = struct.unpack_from("<f", data, base + offsets.x)[0]
                y = struct.unpack_from("<f", data, base + offsets.y)[0]
                z = struct.unpack_from("<f", data, base + offsets.z)[0]
            except struct.error:
                break
            if not (math.isfinite(x) and math.isfinite(y) and math.isfinite(z)):
                continue
            r = math.sqrt(x * x + y * y + z * z)
            if not math.isfinite(r) or r <= 0.0:
                continue
            valid += 1
            self.min_range = min(self.min_range, r)
            self.max_range = max(self.max_range, r)
            self.min_z = min(self.min_z, z)
            self.max_z = max(self.max_z, z)
        self.valid_count_total += valid

    def _report(self) -> None:
        if self.msg_count == 0:
            self.get_logger().warn("No pointcloud messages received yet")
            return

        elapsed_s = max((self.get_clock().now().nanoseconds - self.last_stamp_ns) / 1e9, 0.0)
        valid_ratio = (
            float(self.valid_count_total) / float(self.point_count_total)
            if self.point_count_total > 0
            else 0.0
        )
        points_per_sec = 0.0
        if self.first_msg_stamp_ns > 0 and self.last_stamp_ns > self.first_msg_stamp_ns:
            dt = (self.last_stamp_ns - self.first_msg_stamp_ns) / 1e9
            points_per_sec = float(self.point_count_total) / max(dt, 1e-3)
        z_spread = 0.0
        if math.isfinite(self.min_z) and math.isfinite(self.max_z):
            z_spread = self.max_z - self.min_z

        tf_status = "skip"
        if self.check_tf and self.tf_buffer is not None and self.last_frame_id:
            try:
                self.tf_buffer.lookup_transform(
                    self.target_frame, self.last_frame_id, rclpy.time.Time(), timeout=Duration(seconds=0.1)
                )
                tf_status = "ok"
            except TransformException:
                tf_status = "missing"

        warn_tokens = []
        if valid_ratio < self.warn_min_valid_ratio:
            warn_tokens.append(f"valid_ratio<{self.warn_min_valid_ratio:.2f}")
        if z_spread < self.warn_min_z_spread_m:
            warn_tokens.append(f"z_spread<{self.warn_min_z_spread_m:.2f}m")
        if points_per_sec < self.warn_min_points_per_sec:
            warn_tokens.append(f"points_ps<{int(self.warn_min_points_per_sec)}")
        if tf_status == "missing":
            warn_tokens.append("tf_missing")

        msg = (
            f"frame={self.last_frame_id or '<empty>'} size={self.last_width}x{self.last_height} "
            f"fields=[{self.last_fields}] valid_ratio={valid_ratio:.3f} "
            f"range=[{self.min_range if math.isfinite(self.min_range) else 0.0:.2f},"
            f"{self.max_range:.2f}] z_spread={z_spread:.2f}m points_ps={points_per_sec:.0f} tf={tf_status} "
            f"staleness={elapsed_s:.2f}s warnings={','.join(warn_tokens) if warn_tokens else 'none'}"
        )
        if warn_tokens:
            self.get_logger().warn(msg)
        else:
            self.get_logger().info(msg)


def main() -> None:
    rclpy.init()
    node = PointcloudContractProbe()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
