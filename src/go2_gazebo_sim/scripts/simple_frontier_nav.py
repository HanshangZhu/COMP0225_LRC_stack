#!/usr/bin/env python3
"""
Basic frontier-style navigation for Gazebo without FAR.

Uses the local terrain map to pick a far traversable point and publishes /way_point.
"""
import math
from typing import List, Tuple

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Header
from visualization_msgs.msg import Marker
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2


class SimpleFrontierNav(Node):
    def __init__(self):
        super().__init__("simple_frontier_nav")

        # Parameters
        self.declare_parameter("terrain_topic", "/terrain_map")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("waypoint_topic", "/way_point")
        self.declare_parameter("free_intensity_max", 0.15)
        self.declare_parameter("min_distance", 1.0)
        self.declare_parameter("max_distance", 4.0)
        self.declare_parameter("publish_rate", 1.0)
        self.declare_parameter("reselect_interval", 3.0)
        self.declare_parameter("max_points_sample", 2000)
        self.declare_parameter("fallback_forward_distance", 1.5)
        self.declare_parameter("frontier_cloud_topic", "/frontier_candidates")
        self.declare_parameter("frontier_goal_topic", "/frontier_goal")
        self.declare_parameter("frontier_marker_topic", "/frontier_goal_marker")

        self.terrain_topic = self.get_parameter("terrain_topic").value
        self.odom_topic = self.get_parameter("odom_topic").value
        self.waypoint_topic = self.get_parameter("waypoint_topic").value
        self.free_intensity_max = self.get_parameter("free_intensity_max").value
        self.min_distance = self.get_parameter("min_distance").value
        self.max_distance = self.get_parameter("max_distance").value
        self.publish_rate = self.get_parameter("publish_rate").value
        self.reselect_interval = self.get_parameter("reselect_interval").value
        self.max_points_sample = self.get_parameter("max_points_sample").value
        self.fallback_forward_distance = self.get_parameter("fallback_forward_distance").value
        self.frontier_cloud_topic = self.get_parameter("frontier_cloud_topic").value
        self.frontier_goal_topic = self.get_parameter("frontier_goal_topic").value
        self.frontier_marker_topic = self.get_parameter("frontier_marker_topic").value

        # State
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0
        self.last_goal: Tuple[float, float] | None = None
        self.last_goal_time = None
        self.terrain_points: List[Tuple[float, float, float]] = []
        self.terrain_frame = "odom"
        self.last_stats_time = None

        # Subscribers
        self.odom_sub = self.create_subscription(
            Odometry, self.odom_topic, self.odom_callback, 10
        )
        self.terrain_sub = self.create_subscription(
            PointCloud2, self.terrain_topic, self.terrain_callback, 5
        )

        # Publisher
        self.waypoint_pub = self.create_publisher(PointStamped, self.waypoint_topic, 10)
        self.frontier_cloud_pub = self.create_publisher(PointCloud2, self.frontier_cloud_topic, 10)
        self.frontier_goal_pub = self.create_publisher(PointStamped, self.frontier_goal_topic, 10)
        self.frontier_marker_pub = self.create_publisher(Marker, self.frontier_marker_topic, 10)

        # Timer
        self.timer = self.create_timer(1.0 / self.publish_rate, self.publish_goal)

        self.get_logger().info("Simple frontier nav started")

    def odom_callback(self, msg: Odometry):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.robot_yaw = math.atan2(siny_cosp, cosy_cosp)

    def terrain_callback(self, msg: PointCloud2):
        self.terrain_frame = msg.header.frame_id or "odom"
        points = []
        for i, p in enumerate(point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)):
            if i >= self.max_points_sample:
                break
            points.append((float(p[0]), float(p[1]), 0.0))
        self.terrain_points = points
        now = self.get_clock().now()
        if self.last_stats_time is None or (now - self.last_stats_time).nanoseconds / 1e9 > 5.0:
            if points:
                self.get_logger().info(f"terrain_map points={len(points)}")
            else:
                self.get_logger().info("terrain_map points=0")
            self.last_stats_time = now

    def get_candidates(self) -> List[Tuple[float, float]]:
        candidates: List[Tuple[float, float]] = []
        for x, y, intensity in self.terrain_points:
            if intensity > self.free_intensity_max:
                continue
            dx = x - self.robot_x
            dy = y - self.robot_y
            dist = math.hypot(dx, dy)
            if dist < self.min_distance or dist > self.max_distance:
                continue
            candidates.append((x, y))
        return candidates

    def select_goal(self) -> Tuple[float, float] | None:
        if not self.terrain_points:
            return None
        best = None
        best_dist = -1.0
        for x, y, intensity in self.terrain_points:
            if intensity > self.free_intensity_max:
                continue
            dx = x - self.robot_x
            dy = y - self.robot_y
            dist = math.hypot(dx, dy)
            if dist < self.min_distance or dist > self.max_distance:
                continue
            if dist > best_dist:
                best = (x, y)
                best_dist = dist
        if best is not None:
            return best
        # Fallback: ignore intensity if no candidates found.
        best_dist = -1.0
        for x, y, _intensity in self.terrain_points:
            dx = x - self.robot_x
            dy = y - self.robot_y
            dist = math.hypot(dx, dy)
            if dist < self.min_distance or dist > self.max_distance:
                continue
            if dist > best_dist:
                best = (x, y)
                best_dist = dist
        return best

    def publish_goal(self):
        now = self.get_clock().now()
        if self.last_goal_time is not None:
            if (now - self.last_goal_time).nanoseconds / 1e9 < self.reselect_interval:
                return
        if not self.terrain_points:
            self.publish_frontier_cloud([], now)
            self.publish_fallback_goal(now)
            return
        candidates = self.get_candidates()
        self.publish_frontier_cloud(candidates, now)
        goal = self.select_goal()
        if goal is None:
            if self.last_goal_time is None or (now - self.last_goal_time).nanoseconds / 1e9 > 2.0:
                self.get_logger().info("No frontier candidates within range")
            self.publish_fallback_goal(now)
            return
        if self.last_goal is not None:
            if math.hypot(goal[0] - self.last_goal[0], goal[1] - self.last_goal[1]) < 0.5:
                return
        self.publish_waypoint(goal[0], goal[1], now)
        self.get_logger().info(f"Publishing frontier goal: ({goal[0]:.2f}, {goal[1]:.2f})")

    def publish_fallback_goal(self, now):
        fx = float(self.fallback_forward_distance)
        fy = 0.0
        self.publish_waypoint(fx, fy, now, frame_id="vehicle")
        self.get_logger().info(f"Publishing fallback goal: ({fx:.2f}, {fy:.2f})")

    def publish_waypoint(self, x: float, y: float, now, frame_id: str | None = None):
        msg = PointStamped()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = frame_id or self.terrain_frame
        msg.point.x = float(x)
        msg.point.y = float(y)
        msg.point.z = 0.0
        self.waypoint_pub.publish(msg)
        self.frontier_goal_pub.publish(msg)
        marker = Marker()
        marker.header = msg.header
        marker.ns = "frontier_goal"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = msg.point.x
        marker.pose.position.y = msg.point.y
        marker.pose.position.z = msg.point.z
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.25
        marker.scale.y = 0.25
        marker.scale.z = 0.25
        marker.color.r = 1.0
        marker.color.g = 0.2
        marker.color.b = 0.2
        marker.color.a = 0.9
        self.frontier_marker_pub.publish(marker)
        self.last_goal = (float(x), float(y))
        self.last_goal_time = now

    def publish_frontier_cloud(self, points: List[Tuple[float, float]], now):
        header = Header()
        header.stamp = now.to_msg()
        header.frame_id = self.terrain_frame
        cloud = point_cloud2.create_cloud_xyz32(header, [(p[0], p[1], 0.0) for p in points])
        self.frontier_cloud_pub.publish(cloud)


def main(args=None):
    rclpy.init(args=args)
    node = SimpleFrontierNav()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
