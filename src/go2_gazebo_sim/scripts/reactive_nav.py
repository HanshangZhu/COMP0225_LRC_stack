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
        self.external_stop = 0

        self.create_subscription(PointStamped, "/way_point", self.goal_cb, 10)
        self.create_subscription(Odometry, "/odom/ground_truth", self.odom_cb, 10)
        self.create_subscription(LaserScan, "/scan", self.scan_cb, qos_profile_sensor_data)
        self.create_subscription(Int8, self.cfg.stop_topic, self.stop_cb, 10)

        self.cmd_pub = self.create_publisher(TwistStamped, "/cmd_vel_stamped", 10)
        self.replan_pub = self.create_publisher(Empty, self.cfg.frontier_replan_topic, 10)
        self.status_pub = self.create_publisher(String, "/nav_status", 10)

        # --- CSV Logging Setup ---
        import time
        import os
        ns_sanitized = self.get_namespace().strip("/").replace("/", "_")
        self.csv_path = f"/tmp/reactive_nav_log_{ns_sanitized}_{int(time.time())}.csv"
        self.get_logger().info(f"Logging navigation data to: {self.csv_path}")
        with open(self.csv_path, "w") as f:
            f.write("timestamp,mode,x,y,yaw,goal_x,goal_y,dist_to_goal,min_obst_dist,cmd_vx,cmd_wz,replan\n")

        self.timer = self.create_timer(1.0 / self.cfg.control_rate, self.control_loop)
        self.get_logger().info("Reactive nav started")

    def goal_cb(self, msg: PointStamped) -> None:
        self.goal_state.x = msg.point.x
        self.goal_state.y = msg.point.y
        self.runtime_state.plan_waypoints_world = []
        self.runtime_state.plan_last_time_sec = None
        self.runtime_state.plan_last_goal = None
        self.get_logger().info(f"New goal: ({self.goal_state.x:.2f}, {self.goal_state.y:.2f})")

    def odom_cb(self, msg: Odometry) -> None:
        self.robot_state.x = msg.pose.pose.position.x
        self.robot_state.y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self.robot_state.yaw = self._yaw_from_quat(q.x, q.y, q.z, q.w)
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        self.robot_state.speed = math.hypot(vx, vy)

    def scan_cb(self, msg: LaserScan) -> None:
        self.last_scan = msg

    def stop_cb(self, msg: Int8) -> None:
        self.external_stop = int(msg.data)

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
                self.get_logger().info(message)

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
        status_msg = String()
        status_msg.data = json.dumps(diag, separators=(",", ":"))
        self.status_pub.publish(status_msg)

        # Append to CSV Log
        # Columns: timestamp,mode,x,y,yaw,goal_x,goal_y,dist_to_goal,min_obst_dist,cmd_vx,cmd_wz,replan
        try:
            dist_to_goal = math.hypot(self.goal_state.x - self.robot_state.x, self.goal_state.y - self.robot_state.y)
            mode = diag.get("mode", "unknown")
            min_obst = diag.get("mf", -1.0)  # min front distance
            replan = 1 if result.request_replan else 0
            
            log_line = (
                f"{now_sec:.3f},{mode},{self.robot_state.x:.3f},{self.robot_state.y:.3f},{self.robot_state.yaw:.3f},"
                f"{self.goal_state.x:.3f},{self.goal_state.y:.3f},{dist_to_goal:.3f},{min_obst:.3f},"
                f"{result.linear_x:.3f},{result.angular_z:.3f},{replan}\n"
            )
            with open(self.csv_path, "a") as f:
                f.write(log_line)
        except Exception as e:
            pass # Don't crash on logging

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

