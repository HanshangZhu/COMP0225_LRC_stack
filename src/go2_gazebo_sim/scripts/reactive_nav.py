#!/usr/bin/env python3
"""
Simple reactive navigation controller.

Drives toward a /way_point goal while avoiding obstacles detected by /scan.
Bypasses the CMU local_planner + pathFollower stack which is designed for
3D sensors and doesn't work well with a 2D lidar in narrow corridors.

Algorithm:
  1. Compute heading error to goal
  2. Use laser scan for reactive obstacle avoidance (VFH-lite)
  3. Slow down near obstacles
  4. Publish TwistStamped to /cmd_vel_stamped
"""
import math
from typing import Optional

import rclpy                                      # ROS 2 client library
from rclpy.node import Node                       # base node class
from geometry_msgs.msg import PointStamped, TwistStamped  # message types
from nav_msgs.msg import Odometry                 # odometry message
from sensor_msgs.msg import LaserScan             # laser scan message


class ReactiveNav(Node):
    def __init__(self):
        super().__init__("reactive_nav")

        # --- parameters ---------------------------------------------------
        self.declare_parameter("max_linear_speed", 0.35)    # m/s forward
        self.declare_parameter("max_angular_speed", 0.8)    # rad/s yaw
        self.declare_parameter("goal_tolerance", 0.8)       # m — stop when within this
        self.declare_parameter("obstacle_slow_dist", 0.6)   # m — start slowing
        self.declare_parameter("obstacle_stop_dist", 0.25)  # m — full stop
        self.declare_parameter("front_half_angle_deg", 40.0) # degrees — front cone
        self.declare_parameter("side_check_angle_deg", 70.0) # degrees — side bias zone
        self.declare_parameter("avoidance_gain", 1.5)       # lateral push strength
        self.declare_parameter("control_rate", 10.0)        # Hz
        self.declare_parameter("startup_delay", 15.0)       # seconds before driving
        self.declare_parameter("require_settle_before_motion", True)  # wait for stand-up drift to settle
        self.declare_parameter("settle_speed_threshold", 0.06)         # m/s threshold for "still"
        self.declare_parameter("settle_hold_sec", 2.0)                 # must stay still for this long
        self.declare_parameter("avoidance_deadband", 0.08)             # ignore tiny left/right imbalance
        self.declare_parameter("avoidance_max_ratio", 0.45)            # cap avoidance as fraction of max yaw rate
        self.declare_parameter("avoidance_conflict_scale", 0.30)       # down-weight avoidance if it fights goal turn
        self.declare_parameter("turn_in_place_on_block", True)         # when blocked, turn by goal heading only

        self.max_lin = float(self.get_parameter("max_linear_speed").value)
        self.max_ang = float(self.get_parameter("max_angular_speed").value)
        self.goal_tol = float(self.get_parameter("goal_tolerance").value)
        self.slow_dist = float(self.get_parameter("obstacle_slow_dist").value)
        self.stop_dist = float(self.get_parameter("obstacle_stop_dist").value)
        self.front_half = math.radians(float(self.get_parameter("front_half_angle_deg").value))
        self.side_half = math.radians(float(self.get_parameter("side_check_angle_deg").value))
        self.avoid_gain = float(self.get_parameter("avoidance_gain").value)
        self.rate = float(self.get_parameter("control_rate").value)
        self.startup_delay = float(self.get_parameter("startup_delay").value)
        self.require_settle = bool(self.get_parameter("require_settle_before_motion").value)
        self.settle_speed_threshold = float(self.get_parameter("settle_speed_threshold").value)
        self.settle_hold_sec = float(self.get_parameter("settle_hold_sec").value)
        self.avoidance_deadband = float(self.get_parameter("avoidance_deadband").value)
        self.avoidance_max_ratio = float(self.get_parameter("avoidance_max_ratio").value)
        self.avoidance_conflict_scale = float(self.get_parameter("avoidance_conflict_scale").value)
        self.turn_in_place_on_block = bool(self.get_parameter("turn_in_place_on_block").value)

        # --- state ---------------------------------------------------------
        self.goal_x: Optional[float] = None        # goal in odom frame
        self.goal_y: Optional[float] = None
        self.robot_x = 0.0                          # position in odom frame
        self.robot_y = 0.0
        self.robot_yaw = 0.0                        # heading in odom frame
        self.robot_speed = float("inf")             # planar speed (m/s)
        self.last_scan: Optional[LaserScan] = None
        self.start_time = None                       # set on first timer fire
        self.settle_start_time = None                # first timestamp below speed threshold
        self.settle_ready = False                    # one-shot startup settle gate

        # --- subscribers ---------------------------------------------------
        self.create_subscription(PointStamped, "/way_point", self.goal_cb, 10)
        self.create_subscription(Odometry, "/odom/ground_truth", self.odom_cb, 10)
        self.create_subscription(LaserScan, "/scan", self.scan_cb, 10)

        # --- publisher -----------------------------------------------------
        self.cmd_pub = self.create_publisher(TwistStamped, "/cmd_vel_stamped", 10)

        # --- timer ---------------------------------------------------------
        self.timer = self.create_timer(1.0 / self.rate, self.control_loop)

        self.get_logger().info("Reactive nav started")

    # === callbacks =========================================================
    def goal_cb(self, msg: PointStamped):
        self.goal_x = msg.point.x                   # store goal position
        self.goal_y = msg.point.y
        self.get_logger().info(
            f"New goal: ({self.goal_x:.2f}, {self.goal_y:.2f})"
        )

    def odom_cb(self, msg: Odometry):
        self.robot_x = msg.pose.pose.position.x     # update robot position
        self.robot_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation                # quaternion to yaw
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.robot_yaw = math.atan2(siny, cosy)
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        self.robot_speed = math.hypot(vx, vy)

    def scan_cb(self, msg: LaserScan):
        self.last_scan = msg                         # store latest scan

    # === control loop ======================================================
    def control_loop(self):
        # --- startup delay (sim-time safe) ---
        if self.start_time is None:
            self.start_time = self.get_clock().now()
        elapsed = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
        if elapsed < self.startup_delay:
            self._publish_zero()                     # hold still while waiting
            return

        # --- wait until stand-up transients settle ---
        if self.require_settle and not self.settle_ready:
            now = self.get_clock().now()
            if self.robot_speed > self.settle_speed_threshold:
                self.settle_start_time = None
                self._publish_zero()
                return

            if self.settle_start_time is None:
                self.settle_start_time = now
                self._publish_zero()
                return

            settle_elapsed = (now - self.settle_start_time).nanoseconds / 1e9
            if settle_elapsed < self.settle_hold_sec:
                self._publish_zero()
                return

            if not self.settle_ready:
                self.settle_ready = True
                self.get_logger().info(
                    f"Settle gate passed (speed<{self.settle_speed_threshold:.2f} m/s "
                    f"for {self.settle_hold_sec:.1f}s)."
                )

        # --- no goal yet → hold still ---
        if self.goal_x is None or self.goal_y is None:
            self._publish_zero()
            return

        # --- no scan yet → hold still ---
        if self.last_scan is None:
            self._publish_zero()
            return

        # --- compute heading to goal ---
        dx = self.goal_x - self.robot_x
        dy = self.goal_y - self.robot_y
        dist_to_goal = math.hypot(dx, dy)

        if dist_to_goal < self.goal_tol:            # close enough → stop
            self._publish_zero()
            return

        goal_angle = math.atan2(dy, dx)             # absolute angle to goal
        heading_err = self._wrap(goal_angle - self.robot_yaw)  # signed error

        # --- scan-based obstacle analysis ---
        scan = self.last_scan
        min_front = float("inf")                     # closest obstacle in front cone
        left_push_sum = 0.0                          # repulsive force sum from left
        right_push_sum = 0.0                         # repulsive force sum from right
        left_count = 0
        right_count = 0

        angle = scan.angle_min
        for r in scan.ranges:
            if not math.isfinite(r) or r < 0.05:    # skip invalid readings
                angle += scan.angle_increment
                continue
            abs_a = abs(angle)

            # front obstacle check
            if abs_a < self.front_half and r < min_front:
                min_front = r

            # side repulsion for avoidance
            if abs_a < self.side_half and r < self.slow_dist:
                force = (self.slow_dist - r) / self.slow_dist  # 0..1
                if angle > 0:                        # obstacle on the left
                    left_push_sum += force
                    left_count += 1
                else:                                # obstacle on the right
                    right_push_sum += force
                    right_count += 1

            angle += scan.angle_increment

        # Average per-side force is less jittery than raw sums when ray count changes.
        left_push = left_push_sum / left_count if left_count > 0 else 0.0
        right_push = right_push_sum / right_count if right_count > 0 else 0.0

        # --- compute linear speed ---
        lin = self.max_lin

        # slow down when heading is off (don't drive fast sideways)
        heading_factor = max(0.0, math.cos(heading_err))
        lin *= heading_factor

        # slow down near obstacles
        if min_front < self.slow_dist:
            speed_scale = max(0.0, (min_front - self.stop_dist) / (self.slow_dist - self.stop_dist))
            lin *= speed_scale

        if min_front < self.stop_dist:               # too close → stop forward
            lin = 0.0

        lin = max(0.0, min(lin, self.max_lin))

        # --- compute angular speed ---
        # base: turn toward goal
        goal_turn = self.max_ang * (2.0 / math.pi) * heading_err  # P-controller
        goal_turn = max(-self.max_ang, min(goal_turn, self.max_ang))

        # add obstacle avoidance bias with explicit conflict limiting.
        avoid_raw = right_push - left_push  # push away from closer side
        if abs(avoid_raw) < self.avoidance_deadband:
            avoid_raw = 0.0
        avoid_yaw = self.avoid_gain * avoid_raw

        if min_front < self.stop_dist and self.turn_in_place_on_block:
            # When blocked ahead, prioritize turning toward the goal direction.
            avoid_yaw = 0.0
        elif avoid_yaw * heading_err < 0.0:
            avoid_yaw *= self.avoidance_conflict_scale

        max_avoid = max(0.0, self.avoidance_max_ratio) * self.max_ang
        avoid_yaw = max(-max_avoid, min(avoid_yaw, max_avoid))

        ang = goal_turn + avoid_yaw
        ang = max(-self.max_ang, min(ang, self.max_ang))

        # --- publish ---
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "vehicle"
        msg.twist.linear.x = float(lin)
        msg.twist.angular.z = float(ang)
        self.cmd_pub.publish(msg)

    # === helpers ===========================================================
    def _publish_zero(self):
        """Publish zero velocity to actively stop the robot."""
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "vehicle"
        msg.twist.linear.x = 0.0
        msg.twist.angular.z = 0.0
        self.cmd_pub.publish(msg)

    @staticmethod
    def _wrap(a: float) -> float:
        """Wrap angle to [-pi, pi]."""
        while a > math.pi:
            a -= 2.0 * math.pi
        while a < -math.pi:
            a += 2.0 * math.pi
        return a


def main(args=None):
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
