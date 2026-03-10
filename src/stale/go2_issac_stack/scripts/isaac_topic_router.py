#!/usr/bin/env python3
"""Topic compatibility layer between Isaac Sim topics and go2 autonomy contracts.

This node does a narrow job:
- remap simulator-facing topics to autonomy-facing topic names
- optionally override frame ids for odom/imu/pointcloud
- keep message payloads otherwise unchanged
"""

from __future__ import annotations

import copy
import struct

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import Imu, PointCloud2, PointField


class IsaacTopicRouter(Node):
    def __init__(self) -> None:
        super().__init__("isaac_topic_router")

        # IO topics
        self.declare_parameter("input_odom_topic", "/odom")
        self.declare_parameter("output_odom_topic", "/odom/ground_truth")
        self.declare_parameter("input_pointcloud_topic", "/lidar/points")
        self.declare_parameter("output_pointcloud_topic", "/registered_scan")
        self.declare_parameter("input_imu_topic", "/imu")
        self.declare_parameter("output_imu_topic", "/imu/data")
        self.declare_parameter("input_cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("output_cmd_vel_topic", "/isaac/cmd_vel")

        # Stream controls
        self.declare_parameter("relay_odom", True)
        self.declare_parameter("relay_pointcloud", True)
        self.declare_parameter("relay_imu", True)
        self.declare_parameter("relay_cmd_vel", True)

        # Optional frame overrides
        self.declare_parameter("odom_frame_id_override", "")
        self.declare_parameter("odom_child_frame_id_override", "")
        self.declare_parameter("pointcloud_frame_id_override", "")
        self.declare_parameter("imu_frame_id_override", "")
        self.declare_parameter("ensure_intensity_field", True)
        self.declare_parameter("default_intensity_value", 255.0)

        self.odom_in = str(self.get_parameter("input_odom_topic").value)
        self.odom_out = str(self.get_parameter("output_odom_topic").value)
        self.pc_in = str(self.get_parameter("input_pointcloud_topic").value)
        self.pc_out = str(self.get_parameter("output_pointcloud_topic").value)
        self.imu_in = str(self.get_parameter("input_imu_topic").value)
        self.imu_out = str(self.get_parameter("output_imu_topic").value)
        self.cmd_in = str(self.get_parameter("input_cmd_vel_topic").value)
        self.cmd_out = str(self.get_parameter("output_cmd_vel_topic").value)

        self.relay_odom = bool(self.get_parameter("relay_odom").value)
        self.relay_pc = bool(self.get_parameter("relay_pointcloud").value)
        self.relay_imu = bool(self.get_parameter("relay_imu").value)
        self.relay_cmd = bool(self.get_parameter("relay_cmd_vel").value)

        self.odom_frame_override = str(self.get_parameter("odom_frame_id_override").value)
        self.odom_child_override = str(self.get_parameter("odom_child_frame_id_override").value)
        self.pc_frame_override = str(self.get_parameter("pointcloud_frame_id_override").value)
        self.imu_frame_override = str(self.get_parameter("imu_frame_id_override").value)
        self.ensure_intensity_field = bool(self.get_parameter("ensure_intensity_field").value)
        self.default_intensity_value = float(self.get_parameter("default_intensity_value").value)
        self._warned_missing_intensity = False

        # Prevent accidental feedback loops when the same input/output topic is used.
        if self.relay_odom and self.odom_in == self.odom_out:
            self.get_logger().warn("relay_odom disabled: input_odom_topic == output_odom_topic")
            self.relay_odom = False
        if self.relay_pc and self.pc_in == self.pc_out:
            self.get_logger().warn("relay_pointcloud disabled: input_pointcloud_topic == output_pointcloud_topic")
            self.relay_pc = False
        if self.relay_imu and self.imu_in == self.imu_out:
            self.get_logger().warn("relay_imu disabled: input_imu_topic == output_imu_topic")
            self.relay_imu = False
        if self.relay_cmd and self.cmd_in == self.cmd_out:
            self.get_logger().warn("relay_cmd_vel disabled: input_cmd_vel_topic == output_cmd_vel_topic")
            self.relay_cmd = False

        self._counts = {
            "odom": 0,
            "pointcloud": 0,
            "imu": 0,
            "cmd_vel": 0,
        }
        self._stamp_samples = {
            "odom": 0,
            "pointcloud": 0,
            "imu": 0,
        }
        self._stamp_zero_counts = {
            "odom": 0,
            "pointcloud": 0,
            "imu": 0,
        }
        self._stamp_debug_topics = {
            "odom": (self.odom_in, self.odom_out),
            "pointcloud": (self.pc_in, self.pc_out),
            "imu": (self.imu_in, self.imu_out),
        }

        # QoS is chosen per stream semantics:
        # - cmd/odom/imu use reliable delivery for control/state
        # - high-rate pointcloud uses sensor-data QoS (best effort)
        cmd_qos = QoSProfile(depth=10)
        reliable_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)

        if self.relay_odom:
            self.odom_pub = self.create_publisher(Odometry, self.odom_out, reliable_qos)
            self.odom_sub = self.create_subscription(Odometry, self.odom_in, self._odom_cb, qos_profile_sensor_data)
        if self.relay_pc:
            self.pc_pub = self.create_publisher(PointCloud2, self.pc_out, qos_profile_sensor_data)
            self.pc_sub = self.create_subscription(PointCloud2, self.pc_in, self._pointcloud_cb, qos_profile_sensor_data)
        if self.relay_imu:
            self.imu_pub = self.create_publisher(Imu, self.imu_out, reliable_qos)
            self.imu_sub = self.create_subscription(Imu, self.imu_in, self._imu_cb, qos_profile_sensor_data)
        if self.relay_cmd:
            self.cmd_pub = self.create_publisher(Twist, self.cmd_out, cmd_qos)
            self.cmd_sub = self.create_subscription(Twist, self.cmd_in, self._cmd_vel_cb, cmd_qos)

        self.get_logger().info(
            "Isaac topic router started | "
            f"odom: {self.odom_in} -> {self.odom_out} ({self.relay_odom}) | "
            f"pc: {self.pc_in} -> {self.pc_out} ({self.relay_pc}) | "
            f"imu: {self.imu_in} -> {self.imu_out} ({self.relay_imu}) | "
            f"cmd: {self.cmd_in} -> {self.cmd_out} ({self.relay_cmd})"
        )

    def _maybe_log(self, key: str) -> None:
        # Periodic counters are useful during integration without flooding logs.
        self._counts[key] += 1
        if self._counts[key] % 200 == 1:
            self.get_logger().info(f"{key} relayed: {self._counts[key]}")

    def _debug_stamp(self, key: str, stamp) -> None:
        self._stamp_samples[key] += 1
        sec = int(stamp.sec)
        nanosec = int(stamp.nanosec)
        topic_in, topic_out = self._stamp_debug_topics[key]

        # Print early samples once so launch logs can confirm stamp source quickly.
        if self._stamp_samples[key] <= 3:
            self.get_logger().info(
                f"DEBUG_TS sample key={key} in={topic_in} out={topic_out} stamp={sec}.{nanosec:09d}"
            )

        if sec == 0 and nanosec == 0:
            self._stamp_zero_counts[key] += 1
            zero_count = self._stamp_zero_counts[key]
            # First few events + sparse periodic logs to avoid flooding.
            if zero_count <= 10 or zero_count % 200 == 0:
                self.get_logger().warn(
                    f"DEBUG_TS zero-stamp key={key} count={zero_count} in={topic_in} out={topic_out}"
                )

    def _odom_cb(self, msg: Odometry) -> None:
        self._debug_stamp("odom", msg.header.stamp)
        # Fast path: forward by reference when no frame rewrite is required.
        if not self.odom_frame_override and not self.odom_child_override:
            self.odom_pub.publish(msg)
            self._maybe_log("odom")
            return

        # Slow path: copy only when we must rewrite frame ids.
        out = copy.deepcopy(msg)
        if self.odom_frame_override:
            out.header.frame_id = self.odom_frame_override
        if self.odom_child_override:
            out.child_frame_id = self.odom_child_override
        self.odom_pub.publish(out)
        self._maybe_log("odom")

    def _pointcloud_cb(self, msg: PointCloud2) -> None:
        self._debug_stamp("pointcloud", msg.header.stamp)
        out = msg
        if self.ensure_intensity_field and not self._has_field(msg, "intensity"):
            out = self._append_default_intensity(msg)

        if self.pc_frame_override:
            if out is msg:
                out = copy.deepcopy(msg)
            out.header.frame_id = self.pc_frame_override

        self.pc_pub.publish(out)
        self._maybe_log("pointcloud")

    @staticmethod
    def _has_field(msg: PointCloud2, field_name: str) -> bool:
        for field in msg.fields:
            if field.name == field_name:
                return True
        return False

    @staticmethod
    def _get_field(msg: PointCloud2, field_name: str):
        for field in msg.fields:
            if field.name == field_name:
                return field
        return None

    def _append_default_intensity(self, msg: PointCloud2) -> PointCloud2:
        z_field = self._get_field(msg, "z")
        use_z_intensity = (
            z_field is not None
            and int(z_field.datatype) == int(PointField.FLOAT32)
            and int(z_field.count) == 1
            and int(z_field.offset) + 4 <= int(msg.point_step)
        )

        if not self._warned_missing_intensity:
            self._warned_missing_intensity = True
            if use_z_intensity:
                self.get_logger().warn(
                    "input pointcloud missing 'intensity'; synthesizing intensity from z field"
                )
            else:
                self.get_logger().warn(
                    "input pointcloud missing 'intensity'; appending default intensity field "
                    f"value={self.default_intensity_value:.1f}"
                )

        point_step_in = int(msg.point_step)
        if point_step_in <= 0:
            return msg

        point_step_out = point_step_in + 4
        width = int(msg.width)
        height = int(msg.height)
        total_points = width * height
        intensity_pack = struct.pack(">f" if msg.is_bigendian else "<f", self.default_intensity_value)
        z_offset = int(z_field.offset) if use_z_intensity and z_field is not None else -1

        src = memoryview(msg.data)
        dst = bytearray(total_points * point_step_out)

        # Fast-path for contiguous rows (common PointCloud2 layout).
        if total_points > 0:
            expected_row_step_in = width * point_step_in
            if int(msg.row_step) == expected_row_step_in:
                for i in range(total_points):
                    src_off = i * point_step_in
                    dst_off = i * point_step_out
                    dst[dst_off : dst_off + point_step_in] = src[src_off : src_off + point_step_in]
                    if use_z_intensity:
                        z_src = src_off + z_offset
                        dst[dst_off + point_step_in : dst_off + point_step_out] = src[z_src : z_src + 4]
                    else:
                        dst[dst_off + point_step_in : dst_off + point_step_out] = intensity_pack
            else:
                # Respect incoming row padding if present.
                row_step_in = int(msg.row_step)
                row_step_out = width * point_step_out
                for row in range(height):
                    row_src = row * row_step_in
                    row_dst = row * row_step_out
                    for col in range(width):
                        src_off = row_src + col * point_step_in
                        dst_off = row_dst + col * point_step_out
                        dst[dst_off : dst_off + point_step_in] = src[src_off : src_off + point_step_in]
                        if use_z_intensity:
                            z_src = src_off + z_offset
                            dst[dst_off + point_step_in : dst_off + point_step_out] = src[z_src : z_src + 4]
                        else:
                            dst[dst_off + point_step_in : dst_off + point_step_out] = intensity_pack

        out = PointCloud2()
        out.header = msg.header
        out.height = msg.height
        out.width = msg.width
        out.fields = list(msg.fields) + [
            PointField(name="intensity", offset=point_step_in, datatype=PointField.FLOAT32, count=1)
        ]
        out.is_bigendian = msg.is_bigendian
        out.point_step = point_step_out
        out.row_step = width * point_step_out
        out.data = bytes(dst)
        out.is_dense = msg.is_dense
        return out

    def _imu_cb(self, msg: Imu) -> None:
        self._debug_stamp("imu", msg.header.stamp)
        # Keep IMU payload untouched unless frame id override is requested.
        if not self.imu_frame_override:
            self.imu_pub.publish(msg)
            self._maybe_log("imu")
            return

        out = copy.deepcopy(msg)
        out.header.frame_id = self.imu_frame_override
        self.imu_pub.publish(out)
        self._maybe_log("imu")

    def _cmd_vel_cb(self, msg: Twist) -> None:
        self.cmd_pub.publish(msg)
        self._maybe_log("cmd_vel")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = IsaacTopicRouter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
