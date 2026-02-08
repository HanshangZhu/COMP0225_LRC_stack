#!/usr/bin/env python3
"""
Recover-to-frontier logic when wall stop is triggered.

When /stop is active for a short duration, pick a nearby frontier from
/robot_vgraph and publish it as /goal_point to move away from walls.
"""
import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Int8
from visibility_graph_msg.msg import Graph


class FrontierRecovery(Node):
    def __init__(self):
        super().__init__("frontier_recovery")

        # Parameters
        self.declare_parameter("stop_topic", "/stop")
        self.declare_parameter("vgraph_topic", "/robot_vgraph")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("goal_topic", "/goal_point")
        self.declare_parameter("exploration_frame", "odom")
        self.declare_parameter("min_frontier_distance", 1.0)
        self.declare_parameter("trigger_stop_duration", 0.8)
        self.declare_parameter("cooldown_sec", 3.0)
        self.declare_parameter("publish_rate", 2.0)

        self.stop_topic = self.get_parameter("stop_topic").value
        self.vgraph_topic = self.get_parameter("vgraph_topic").value
        self.odom_topic = self.get_parameter("odom_topic").value
        self.goal_topic = self.get_parameter("goal_topic").value
        self.exploration_frame = self.get_parameter("exploration_frame").value
        self.min_frontier_dist = self.get_parameter("min_frontier_distance").value
        self.trigger_stop_duration = self.get_parameter("trigger_stop_duration").value
        self.cooldown_sec = self.get_parameter("cooldown_sec").value
        self.publish_rate = self.get_parameter("publish_rate").value

        # State
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.frontiers = []
        self.visited_frontiers = set()
        self.stop_active = False
        self.stop_start_time = None
        self.last_recovery_time = None

        # Subscribers
        self.stop_sub = self.create_subscription(
            Int8, self.stop_topic, self.stop_callback, 10
        )
        self.odom_sub = self.create_subscription(
            Odometry, self.odom_topic, self.odom_callback, 10
        )
        self.vgraph_sub = self.create_subscription(
            Graph, self.vgraph_topic, self.vgraph_callback, 10
        )

        # Publisher
        self.goal_pub = self.create_publisher(PointStamped, self.goal_topic, 10)

        # Timer
        self.timer = self.create_timer(1.0 / self.publish_rate, self.recovery_loop)

        self.get_logger().info("Frontier recovery started (wall-stop triggered)")

    def stop_callback(self, msg: Int8):
        is_stop = msg.data != 0
        if is_stop and not self.stop_active:
            self.stop_start_time = self.get_clock().now()
        if not is_stop:
            self.stop_start_time = None
        self.stop_active = is_stop

    def odom_callback(self, msg: Odometry):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y

    def vgraph_callback(self, msg: Graph):
        self.frontiers = []
        for node in msg.nodes:
            if node.is_frontier and not node.is_covered:
                self.frontiers.append(
                    {"x": node.position.x, "y": node.position.y, "z": node.position.z}
                )

    def get_distance(self, x, y):
        return math.sqrt((x - self.robot_x) ** 2 + (y - self.robot_y) ** 2)

    def select_frontier(self):
        if not self.frontiers:
            return None
        valid = []
        for f in self.frontiers:
            dist = self.get_distance(f["x"], f["y"])
            if dist < self.min_frontier_dist:
                continue
            key = (round(f["x"], 1), round(f["y"], 1))
            if key in self.visited_frontiers:
                continue
            f["distance"] = dist
            valid.append(f)
        if not valid and self.frontiers:
            self.visited_frontiers.clear()
            return self.select_frontier()
        if not valid:
            return None
        valid.sort(key=lambda f: f["distance"])
        return valid[0]

    def recovery_loop(self):
        if not self.stop_active:
            return
        if self.stop_start_time is None:
            return
        elapsed = (self.get_clock().now() - self.stop_start_time).nanoseconds / 1e9
        if elapsed < self.trigger_stop_duration:
            return
        if self.last_recovery_time is not None:
            since_last = (self.get_clock().now() - self.last_recovery_time).nanoseconds / 1e9
            if since_last < self.cooldown_sec:
                return
        frontier = self.select_frontier()
        if frontier is None:
            self.get_logger().warn("No frontiers available for recovery")
            self.last_recovery_time = self.get_clock().now()
            return
        key = (round(frontier["x"], 1), round(frontier["y"], 1))
        self.visited_frontiers.add(key)
        goal_msg = PointStamped()
        goal_msg.header.stamp = self.get_clock().now().to_msg()
        goal_msg.header.frame_id = self.exploration_frame
        goal_msg.point.x = frontier["x"]
        goal_msg.point.y = frontier["y"]
        goal_msg.point.z = frontier["z"]
        self.goal_pub.publish(goal_msg)
        self.last_recovery_time = self.get_clock().now()
        self.get_logger().info(
            f"Recovery goal published: ({frontier['x']:.2f}, {frontier['y']:.2f})"
        )


def main(args=None):
    rclpy.init(args=args)
    node = FrontierRecovery()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
