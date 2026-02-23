#!/usr/bin/env python3
"""Broadcast TF from Odometry for Isaac autonomy stacks.

Node role:
- subscribe to odometry topic
- publish dynamic transform odom -> base_link (configurable)

Why this exists:
- several Isaac launch paths publish Odometry but no TF
- RViz and TF-dependent nodes require explicit transforms
"""

from __future__ import annotations

import math

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import TransformBroadcaster


class OdomTfBroadcaster(Node):
    def __init__(self) -> None:
        super().__init__("odom_tf_broadcaster")

        self.declare_parameter("odom_topic", "/odom/nav")
        self.declare_parameter("parent_frame", "odom")
        self.declare_parameter("child_frame", "base_link")
        self.declare_parameter("max_publish_rate", 60.0)
        self.declare_parameter("min_motion_eps", 1e-6)
        # Re-publish static pose periodically because /tf is volatile (non-latched).
        self.declare_parameter("heartbeat_sec", 0.5)

        self.odom_topic = str(self.get_parameter("odom_topic").value)
        self.parent_frame = str(self.get_parameter("parent_frame").value)
        self.child_frame = str(self.get_parameter("child_frame").value)
        self.max_publish_rate = max(0.0, float(self.get_parameter("max_publish_rate").value))
        self.min_motion_eps = max(0.0, float(self.get_parameter("min_motion_eps").value))
        self.heartbeat_sec = max(0.05, float(self.get_parameter("heartbeat_sec").value))

        self._broadcaster = TransformBroadcaster(self)
        self._last_pub_ns = 0
        self._last_xyz: tuple[float, float, float] | None = None
        self._last_quat: tuple[float, float, float, float] | None = None
        self._last_send_wall_t = 0.0
        self._count = 0

        self.create_subscription(Odometry, self.odom_topic, self._odom_cb, 10)

        self.get_logger().info(
            "Odom TF broadcaster started | "
            f"odom={self.odom_topic} parent={self.parent_frame} child={self.child_frame} "
            f"max_rate={self.max_publish_rate:.1f}Hz"
        )

    def _odom_cb(self, msg: Odometry) -> None:
        now_ns = self.get_clock().now().nanoseconds
        if self.max_publish_rate > 0.0 and self._last_pub_ns != 0:
            min_period_ns = int(1e9 / self.max_publish_rate)
            if (now_ns - self._last_pub_ns) < min_period_ns:
                return

        # Use explicit frame override when provided, else inherit odom message frames.
        parent = self.parent_frame.strip() or msg.header.frame_id or "world"
        child = self.child_frame.strip() or msg.child_frame_id or "base_link"

        tx = float(msg.pose.pose.position.x)
        ty = float(msg.pose.pose.position.y)
        tz = float(msg.pose.pose.position.z)
        qx = float(msg.pose.pose.orientation.x)
        qy = float(msg.pose.pose.orientation.y)
        qz = float(msg.pose.pose.orientation.z)
        qw = float(msg.pose.pose.orientation.w)

        # Skip bit-identical pose repeats to reduce TF traffic noise.
        xyz = (tx, ty, tz)
        quat = (qx, qy, qz, qw)
        if self._last_xyz is not None and self._last_quat is not None:
            dpos = math.dist(xyz, self._last_xyz)
            dquat = math.dist(quat, self._last_quat)
            now_wall_t = self.get_clock().now().nanoseconds / 1e9
            if dpos <= self.min_motion_eps and dquat <= self.min_motion_eps and (now_wall_t - self._last_send_wall_t) < self.heartbeat_sec:
                return

        tf_msg = TransformStamped()
        tf_msg.header.stamp = msg.header.stamp
        if Time.from_msg(tf_msg.header.stamp).nanoseconds <= 0:
            tf_msg.header.stamp = self.get_clock().now().to_msg()
        tf_msg.header.frame_id = parent
        tf_msg.child_frame_id = child
        tf_msg.transform.translation.x = tx
        tf_msg.transform.translation.y = ty
        tf_msg.transform.translation.z = tz
        tf_msg.transform.rotation.x = qx
        tf_msg.transform.rotation.y = qy
        tf_msg.transform.rotation.z = qz
        tf_msg.transform.rotation.w = qw

        self._broadcaster.sendTransform(tf_msg)
        self._last_pub_ns = now_ns
        self._last_send_wall_t = self.get_clock().now().nanoseconds / 1e9
        self._last_xyz = xyz
        self._last_quat = quat

        self._count += 1
        if self._count % 200 == 1:
            self.get_logger().info(
                f"TF relayed {self._count} msgs | {parent}->{child} pos=({tx:.2f},{ty:.2f},{tz:.2f})"
            )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = OdomTfBroadcaster()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == "__main__":
    main()
