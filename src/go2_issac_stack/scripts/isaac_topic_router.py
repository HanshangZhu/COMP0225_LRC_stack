#!/usr/bin/env python3
"""Topic compatibility layer between Isaac Sim topics and go2 autonomy contracts.

This node does a narrow job:
- remap simulator-facing topics to autonomy-facing topic names
- optionally override frame ids for odom/imu/pointcloud
- keep message payloads otherwise unchanged
"""

from __future__ import annotations

import copy

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import Imu, PointCloud2


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
        # Keep point cloud payload untouched unless frame id override is requested.
        if not self.pc_frame_override:
            self.pc_pub.publish(msg)
            self._maybe_log("pointcloud")
            return

        out = copy.deepcopy(msg)
        out.header.frame_id = self.pc_frame_override
        self.pc_pub.publish(out)
        self._maybe_log("pointcloud")

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
