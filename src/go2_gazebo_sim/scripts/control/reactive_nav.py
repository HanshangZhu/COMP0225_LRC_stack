#!/usr/bin/env python3
"""Thin ROS adapter for layered reactive navigation."""

import json
import math

import rclpy
from geometry_msgs.msg import PointStamped, PoseStamped, TwistStamped
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Empty, Int8, String
from visualization_msgs.msg import Marker

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
        self.goal_msg_seq = 0
        self.last_goal_rx_sec: float | None = None
        self.last_goal_xy: tuple[float, float] | None = None
        self.goal_frame_id = "world"
        self.stall_since_sec: float | None = None
        self.last_stall_warn_sec: float | None = None

        self.declare_parameter("debug_stall_warn_sec", 6.0)
        self.declare_parameter("debug_stall_warn_period_sec", 6.0)
        self.declare_parameter("debug_stall_speed_threshold", 0.03)
        self.declare_parameter("debug_stall_cmd_threshold", 0.02)
        self.debug_stall_warn_sec = max(0.0, float(self.get_parameter("debug_stall_warn_sec").value))
        self.debug_stall_warn_period_sec = max(
            0.5, float(self.get_parameter("debug_stall_warn_period_sec").value)
        )
        self.debug_stall_speed_threshold = max(0.0, float(self.get_parameter("debug_stall_speed_threshold").value))
        self.debug_stall_cmd_threshold = max(0.0, float(self.get_parameter("debug_stall_cmd_threshold").value))

        self.create_subscription(PointStamped, "/way_point", self.goal_cb, 10)
        self.create_subscription(Odometry, "/odom/ground_truth", self.odom_cb, 10)
        self.create_subscription(LaserScan, "/scan", self.scan_cb, qos_profile_sensor_data)
        self.create_subscription(Int8, self.cfg.stop_topic, self.stop_cb, 10)

        self.cmd_pub = self.create_publisher(TwistStamped, "/cmd_vel_stamped", 10)
        self.replan_pub = self.create_publisher(Empty, self.cfg.frontier_replan_topic, 10)
        self.status_pub = self.create_publisher(String, "/nav_status", 10)
        self.final_goal_marker_pub = self.create_publisher(Marker, "/final_goal_marker", 10)
        self.planned_path_pub = self.create_publisher(Path, "/planned_path", 10)

        self.timer = self.create_timer(1.0 / self.cfg.control_rate, self.control_loop)
        self.get_logger().info("Reactive nav started")

    def goal_cb(self, msg: PointStamped) -> None:
        self.goal_state.x = msg.point.x
        self.goal_state.y = msg.point.y
        self.goal_frame_id = msg.header.frame_id or "world"
        self.goal_msg_seq += 1
        self.last_goal_rx_sec = self.get_clock().now().nanoseconds / 1e9
        self.runtime_state.plan_waypoints_world = []
        self.runtime_state.plan_last_time_sec = None
        self.runtime_state.plan_last_goal = None
        goal_xy = (float(self.goal_state.x), float(self.goal_state.y))
        if self.last_goal_xy is None:
            self.get_logger().info(f"New goal[#{self.goal_msg_seq}]: ({goal_xy[0]:.2f}, {goal_xy[1]:.2f})")
        else:
            delta = math.hypot(goal_xy[0] - self.last_goal_xy[0], goal_xy[1] - self.last_goal_xy[1])
            if delta >= 0.05:
                self.get_logger().info(
                    f"Goal update[#{self.goal_msg_seq}]: ({goal_xy[0]:.2f}, {goal_xy[1]:.2f}) "
                    f"delta={delta:.2f}m"
                )
        self.last_goal_xy = goal_xy

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
        self._publish_navigation_visuals(now.to_msg())

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
        if self.goal_state.x is not None and self.goal_state.y is not None:
            dist_live = math.hypot(self.goal_state.x - self.robot_state.x, self.goal_state.y - self.robot_state.y)
            diag["dist_goal_live"] = round(dist_live, 2)
        if self.last_goal_rx_sec is not None:
            diag["goal_age_sec"] = round(max(0.0, now_sec - self.last_goal_rx_sec), 2)
        diag["goal_seq"] = self.goal_msg_seq

        status_msg = String()
        status_msg.data = json.dumps(diag, separators=(",", ":"))
        self.status_pub.publish(status_msg)
        self._maybe_log_stall_warning(now_sec, diag, result.linear_x, result.angular_z)
        self._maybe_log_local_summary(now_sec, diag, result.linear_x, result.angular_z)

    def _maybe_log_stall_warning(self, now_sec: float, diag: dict, lin: float, ang: float) -> None:
        mode = str(diag.get("mode", ""))
        speed = abs(float(self.robot_state.speed))
        cmd_small = abs(float(lin)) <= self.debug_stall_cmd_threshold and abs(float(ang)) <= self.debug_stall_cmd_threshold
        mode_monitored = mode in {"navigate", "goal_reached", "wall_scan", "unstick"}
        has_goal = self.goal_state.x is not None and self.goal_state.y is not None
        stalled_now = mode_monitored and has_goal and cmd_small and speed <= self.debug_stall_speed_threshold

        if not stalled_now:
            self.stall_since_sec = None
            return

        if self.stall_since_sec is None:
            self.stall_since_sec = now_sec
            return
        stalled_for = now_sec - self.stall_since_sec
        if stalled_for < self.debug_stall_warn_sec:
            return

        if self.last_stall_warn_sec is not None:
            if (now_sec - self.last_stall_warn_sec) < self.debug_stall_warn_period_sec:
                return
        self.last_stall_warn_sec = now_sec

        self.get_logger().warn(
            "STALL suspect: "
            f"mode={mode} stalled_for={stalled_for:.1f}s speed={speed:.3f} "
            f"cmd=({lin:.2f},{ang:.2f}) dist_goal={diag.get('dist_goal_live', diag.get('dist_goal', '-'))} "
            f"goal_age={diag.get('goal_age_sec', '-')}s min_front={diag.get('min_front', '-')} "
            f"blocked_sec={diag.get('blocked_sec', '-')} zero_reason={diag.get('zero_reason', '-')}"
        )

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

    def _publish_navigation_visuals(self, stamp_msg) -> None:
        frame_id = self.goal_frame_id or "world"
        self._publish_final_goal_marker(stamp_msg, frame_id)
        self._publish_planned_path(stamp_msg, frame_id)

    def _publish_final_goal_marker(self, stamp_msg, frame_id: str) -> None:
        marker = Marker()
        marker.header.stamp = stamp_msg
        marker.header.frame_id = frame_id
        marker.ns = "final_goal"
        marker.id = 1

        if self.goal_state.x is None or self.goal_state.y is None:
            marker.action = Marker.DELETE
            self.final_goal_marker_pub.publish(marker)
            return

        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = float(self.goal_state.x)
        marker.pose.position.y = float(self.goal_state.y)
        marker.pose.position.z = 0.10
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.32
        marker.scale.y = 0.32
        marker.scale.z = 0.32
        marker.color.a = 0.95
        # Dark blue selected frontier marker to distinguish it from raw frontier orange.
        marker.color.r = 0.05
        marker.color.g = 0.15
        marker.color.b = 0.70
        self.final_goal_marker_pub.publish(marker)

    def _publish_planned_path(self, stamp_msg, frame_id: str) -> None:
        path = Path()
        path.header.stamp = stamp_msg
        path.header.frame_id = frame_id

        # Always publish, even empty, so RViz clears stale path state.
        if self.goal_state.x is None or self.goal_state.y is None:
            self.planned_path_pub.publish(path)
            return

        points: list[tuple[float, float]] = []
        if math.isfinite(self.robot_state.x) and math.isfinite(self.robot_state.y):
            points.append((float(self.robot_state.x), float(self.robot_state.y)))

        for wx, wy in self.runtime_state.plan_waypoints_world:
            points.append((float(wx), float(wy)))

        goal_xy = (float(self.goal_state.x), float(self.goal_state.y))
        if not points:
            points.append(goal_xy)
        else:
            last = points[-1]
            if math.hypot(goal_xy[0] - last[0], goal_xy[1] - last[1]) > 0.05:
                points.append(goal_xy)

        for wx, wy in points:
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x = wx
            pose.pose.position.y = wy
            pose.pose.position.z = 0.04
            pose.pose.orientation.w = 1.0
            path.poses.append(pose)

        self.planned_path_pub.publish(path)

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
