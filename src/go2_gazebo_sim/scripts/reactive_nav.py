#!/usr/bin/env python3
"""Thin ROS adapter for layered reactive navigation."""

import json
import math

import rclpy
from geometry_msgs.msg import PointStamped, TwistStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Empty, Int8, String

from reactive_nav_core import GoalState, NavRuntimeState, ReactiveNavConfig, ReactiveNavCoordinator, RobotState


class ReactiveNav(Node):
    def __init__(self) -> None:
        super().__init__("reactive_nav")

        self.cfg = ReactiveNavConfig.from_node(self)
        self.coordinator = ReactiveNavCoordinator(self.cfg)

        self.robot_state = RobotState()
        self.goal_state = GoalState()
        self.runtime_state = NavRuntimeState()
        self.last_scan: LaserScan | None = None
        self.last_scan_rx_sec: float | None = None
        self.external_stop = 0
        self.last_stop_rx_sec: float | None = None
        self.stop_msg_count = 0
        self.path_total_m = 0.0
        self.prev_odom_x: float | None = None
        self.prev_odom_y: float | None = None
        self.last_summary_sec: float | None = None
        self.summary_interval_sec = 10.0

        self.create_subscription(PointStamped, "/way_point", self.goal_cb, 10)
        self.create_subscription(Odometry, "/odom/ground_truth", self.odom_cb, 10)
        self.create_subscription(LaserScan, "/scan", self.scan_cb, qos_profile_sensor_data)
        self.create_subscription(Int8, self.cfg.stop_topic, self.stop_cb, 10)

        self.cmd_pub = self.create_publisher(TwistStamped, "/cmd_vel_stamped", 10)
        self.replan_pub = self.create_publisher(Empty, self.cfg.frontier_replan_topic, 10)
        self.status_pub = self.create_publisher(String, "/nav_status", 10)

        self.timer = self.create_timer(1.0 / self.cfg.control_rate, self.control_loop)
        self.get_logger().info("Reactive nav started")

    def goal_cb(self, msg: PointStamped) -> None:
        self.goal_state.x = msg.point.x
        self.goal_state.y = msg.point.y
        self.runtime_state.plan_waypoints_world = []
        self.runtime_state.plan_last_time_sec = None
        self.runtime_state.plan_last_goal = None
        self.get_logger().debug(f"New goal: ({self.goal_state.x:.2f}, {self.goal_state.y:.2f})")

    def odom_cb(self, msg: Odometry) -> None:
        self.robot_state.x = msg.pose.pose.position.x
        self.robot_state.y = msg.pose.pose.position.y
        if self.prev_odom_x is not None and self.prev_odom_y is not None:
            self.path_total_m += math.hypot(self.robot_state.x - self.prev_odom_x, self.robot_state.y - self.prev_odom_y)
        self.prev_odom_x = self.robot_state.x
        self.prev_odom_y = self.robot_state.y
        q = msg.pose.pose.orientation
        self.robot_state.yaw = self._yaw_from_quat(q.x, q.y, q.z, q.w)
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        self.robot_state.speed = math.hypot(vx, vy)

    def scan_cb(self, msg: LaserScan) -> None:
        self.last_scan = msg
        self.last_scan_rx_sec = self.get_clock().now().nanoseconds / 1e9

    def stop_cb(self, msg: Int8) -> None:
        self.external_stop = int(msg.data)
        self.last_stop_rx_sec = self.get_clock().now().nanoseconds / 1e9
        self.stop_msg_count += 1

    def control_loop(self) -> None:
        now = self.get_clock().now()
        now_sec = now.nanoseconds / 1e9

        result = self.coordinator.tick(
            now_sec=now_sec,
            runtime_state=self.runtime_state,
            robot_state=self.robot_state,
            goal_state=self.goal_state,
            scan=self.last_scan,
            external_stop=self.external_stop,
        )

        for level, message in result.events:
            if level == "warn":
                self.get_logger().warn(message)
            elif level == "error":
                self.get_logger().error(message)
            else:
                self.get_logger().debug(message)

        if result.request_replan:
            self.replan_pub.publish(Empty())

        msg = TwistStamped()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = "vehicle"
        msg.twist.linear.x = float(result.linear_x)
        msg.twist.angular.z = float(result.angular_z)
        self.cmd_pub.publish(msg)

        # Publish diagnostics for CLI monitoring
        diag = result.diagnostics.copy()
        diag["pos"] = [round(self.robot_state.x, 2), round(self.robot_state.y, 2)]
        diag["yaw"] = round(math.degrees(self.robot_state.yaw), 1)
        diag["speed"] = round(self.robot_state.speed, 3)
        diag["cmd"] = [round(result.linear_x, 3), round(result.angular_z, 3)]
        diag["stop_msgs"] = self.stop_msg_count
        diag["scan_age_sec"] = (
            None if self.last_scan_rx_sec is None else round(max(0.0, now_sec - self.last_scan_rx_sec), 2)
        )
        diag["stop_age_sec"] = (
            None if self.last_stop_rx_sec is None else round(max(0.0, now_sec - self.last_stop_rx_sec), 2)
        )

        status_msg = String()
        status_msg.data = json.dumps(diag, separators=(",", ":"))
        self.status_pub.publish(status_msg)
        self._maybe_log_local_summary(now_sec, diag, result.linear_x, result.angular_z)

    def _maybe_log_local_summary(self, now_sec: float, diag: dict, lin: float, ang: float) -> None:
        if self.last_summary_sec is None:
            self.last_summary_sec = now_sec
            return
        if (now_sec - self.last_summary_sec) < self.summary_interval_sec:
            return
        self.last_summary_sec = now_sec

        mode = diag.get("mode", "?")
        steer = diag.get("steer", "-")
        dist_goal = diag.get("dist_goal", "-")
        min_front = diag.get("min_front", "-")
        ext_stop = diag.get("ext_stop", self.external_stop)
        zero_reason = diag.get("zero_reason", "-")
        self.get_logger().info(
            f"LOCAL step: mode={mode} steer={steer} dist={dist_goal} "
            f"min_front={min_front} ext_stop={ext_stop} zero_reason={zero_reason} "
            f"cmd=({lin:.2f},{ang:.2f}) disp={self.path_total_m:.2f}m"
        )

    @staticmethod
    def _yaw_from_quat(x: float, y: float, z: float, w: float) -> float:
        siny = 2.0 * (w * z + x * y)
        cosy = 1.0 - 2.0 * (y * y + z * z)
        return math.atan2(siny, cosy)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ReactiveNav()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
