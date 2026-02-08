#!/usr/bin/env python3
"""
Geometric frontier exploration using an occupancy grid.
Frontier cells are free cells adjacent to unknown cells.
"""
import math
from collections import deque
from typing import List, Tuple

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from geometry_msgs.msg import Point, PointStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Header
from visualization_msgs.msg import Marker, MarkerArray
from tf2_ros import Buffer, TransformListener, LookupException, ExtrapolationException


def bresenham(x0: int, y0: int, x1: int, y1: int) -> List[Tuple[int, int]]:
    points: List[Tuple[int, int]] = []
    dx = abs(x1 - x0)
    dy = abs(y1 - y0)
    x, y = x0, y0
    sx = 1 if x0 < x1 else -1
    sy = 1 if y0 < y1 else -1
    if dx > dy:
        err = dx / 2.0
        while x != x1:
            points.append((x, y))
            err -= dy
            if err < 0:
                y += sy
                err += dx
            x += sx
    else:
        err = dy / 2.0
        while y != y1:
            points.append((x, y))
            err -= dx
            if err < 0:
                x += sx
                err += dy
            y += sy
    points.append((x1, y1))
    return points


class GeometricFrontier(Node):
    def __init__(self):
        super().__init__("geometric_frontier")

        # Parameters
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("odom_topic", "/odom/ground_truth")
        self.declare_parameter("map_frame", "odom")
        self.declare_parameter("map_topic", "/map")
        self.declare_parameter("resolution", 0.1)
        self.declare_parameter("width", 400)
        self.declare_parameter("height", 400)
        self.declare_parameter("origin_x", -20.0)
        self.declare_parameter("origin_y", -20.0)
        self.declare_parameter("max_range", 6.0)
        self.declare_parameter("update_rate", 2.0)
        self.declare_parameter("frontier_min_size", 10)
        self.declare_parameter("selection_mode", "nearest")
        self.declare_parameter("goal_hysteresis_distance", 0.0)
        self.declare_parameter("goal_hold_sec", 0.0)
        self.declare_parameter("obstacle_clearance_cells", 1)
        self.declare_parameter("startup_delay", 10.0)        # seconds before publishing goals
        self.declare_parameter("frontier_goal_topic", "/way_point")
        self.declare_parameter("frontier_marker_topic", "/frontier_goal_marker")
        self.declare_parameter("frontier_regions_topic", "/frontier_markers")

        self.scan_topic = self.get_parameter("scan_topic").value
        self.odom_topic = self.get_parameter("odom_topic").value
        self.map_frame = self.get_parameter("map_frame").value
        self.map_topic = self.get_parameter("map_topic").value
        self.resolution = float(self.get_parameter("resolution").value)
        self.width = int(self.get_parameter("width").value)
        self.height = int(self.get_parameter("height").value)
        self.origin_x = float(self.get_parameter("origin_x").value)
        self.origin_y = float(self.get_parameter("origin_y").value)
        self.max_range = float(self.get_parameter("max_range").value)
        self.update_rate = float(self.get_parameter("update_rate").value)
        self.frontier_min_size = int(self.get_parameter("frontier_min_size").value)
        self.selection_mode = self.get_parameter("selection_mode").value
        self.goal_hysteresis_distance = float(self.get_parameter("goal_hysteresis_distance").value)
        self.goal_hold_sec = float(self.get_parameter("goal_hold_sec").value)
        self.obstacle_clearance_cells = int(self.get_parameter("obstacle_clearance_cells").value)
        self.startup_delay = float(self.get_parameter("startup_delay").value)
        self.frontier_goal_topic = self.get_parameter("frontier_goal_topic").value
        self.frontier_marker_topic = self.get_parameter("frontier_marker_topic").value
        self.frontier_regions_topic = self.get_parameter("frontier_regions_topic").value

        # State
        self.last_scan: LaserScan | None = None
        self.last_odom: Odometry | None = None
        self.laser_to_base = None
        self.last_goal: Tuple[float, float] | None = None
        self.last_goal_time: Time | None = None
        self.grid = [-1] * (self.width * self.height)
        self.start_time = None                              # set on first update
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Subscribers
        self.scan_sub = self.create_subscription(LaserScan, self.scan_topic, self.scan_cb, 10)
        self.odom_sub = self.create_subscription(Odometry, self.odom_topic, self.odom_cb, 10)

        # Publishers
        self.map_pub = self.create_publisher(OccupancyGrid, self.map_topic, 1)
        self.goal_pub = self.create_publisher(PointStamped, self.frontier_goal_topic, 10)
        self.goal_marker_pub = self.create_publisher(Marker, self.frontier_marker_topic, 10)
        self.regions_pub = self.create_publisher(MarkerArray, self.frontier_regions_topic, 10)

        # Timer
        self.timer = self.create_timer(1.0 / self.update_rate, self.update)

        self.get_logger().info("Geometric frontier node started")

    def scan_cb(self, msg: LaserScan):
        self.last_scan = msg

    def odom_cb(self, msg: Odometry):
        self.last_odom = msg

    def world_to_grid(self, x: float, y: float) -> Tuple[int, int] | None:
        gx = int((x - self.origin_x) / self.resolution)
        gy = int((y - self.origin_y) / self.resolution)
        if gx < 0 or gy < 0 or gx >= self.width or gy >= self.height:
            return None
        return gx, gy

    def grid_to_world(self, gx: int, gy: int) -> Tuple[float, float]:
        x = self.origin_x + (gx + 0.5) * self.resolution
        y = self.origin_y + (gy + 0.5) * self.resolution
        return x, y

    def set_cell(self, gx: int, gy: int, value: int):
        idx = gy * self.width + gx
        if self.grid[idx] == 100 and value == 0:
            return
        self.grid[idx] = value

    def update(self):
        if self.last_scan is None or self.last_odom is None:
            return

        # --- startup delay: let robot stand up first ---
        now = self.get_clock().now()
        if self.start_time is None:
            self.start_time = now
        elapsed = (now - self.start_time).nanoseconds / 1e9
        if elapsed < self.startup_delay:
            if int(elapsed) % 3 == 0:
                self.get_logger().info(
                    f"Startup delay: {self.startup_delay - elapsed:.0f}s remaining"
                )
            return

        scan = self.last_scan
        odom = self.last_odom

        if self.map_frame != "odom":
            self.get_logger().warn("map_frame is not 'odom'; expect map to be in odom frame.")

        if self.laser_to_base is None:
            try:
                self.laser_to_base = self.tf_buffer.lookup_transform(
                    "base_link",
                    scan.header.frame_id,
                    Time(),
                    timeout=rclpy.duration.Duration(seconds=0.5),
                )
            except (LookupException, ExtrapolationException):
                return

        odom_x = odom.pose.pose.position.x
        odom_y = odom.pose.pose.position.y
        odom_yaw = self.quat_to_yaw(odom.pose.pose.orientation)

        lb = self.laser_to_base.transform
        lb_yaw = self.quat_to_yaw(lb.rotation)
        lb_tx = lb.translation.x
        lb_ty = lb.translation.y

        laser_origin_x = odom_x + math.cos(odom_yaw) * lb_tx - math.sin(odom_yaw) * lb_ty
        laser_origin_y = odom_y + math.sin(odom_yaw) * lb_tx + math.cos(odom_yaw) * lb_ty

        origin_cell = self.world_to_grid(laser_origin_x, laser_origin_y)
        if origin_cell is None:
            return

        angle = scan.angle_min
        max_range = min(scan.range_max, self.max_range)
        for r in scan.ranges:
            if math.isfinite(r):
                dist = min(r, max_range)
            else:
                dist = max_range
            lx = dist * math.cos(angle)
            ly = dist * math.sin(angle)
            angle += scan.angle_increment

            bx = math.cos(lb_yaw) * lx - math.sin(lb_yaw) * ly + lb_tx
            by = math.sin(lb_yaw) * lx + math.cos(lb_yaw) * ly + lb_ty
            wx = odom_x + math.cos(odom_yaw) * bx - math.sin(odom_yaw) * by
            wy = odom_y + math.sin(odom_yaw) * bx + math.cos(odom_yaw) * by

            end_cell = self.world_to_grid(wx, wy)
            if end_cell is None:
                continue
            cells = bresenham(origin_cell[0], origin_cell[1], end_cell[0], end_cell[1])
            for cx, cy in cells[:-1]:
                self.set_cell(cx, cy, 0)
            if math.isfinite(r) and r < max_range * 0.99:
                self.set_cell(end_cell[0], end_cell[1], 100)
            else:
                self.set_cell(end_cell[0], end_cell[1], 0)

        self.publish_map(scan.header.stamp)
        frontiers = self.extract_frontiers()
        clusters = self.cluster_frontiers(frontiers)

        # --- debug logging (every update) ---
        free_count = sum(1 for c in self.grid if c == 0)
        occ_count = sum(1 for c in self.grid if c == 100)
        self.get_logger().info(
            f"grid: free={free_count} occ={occ_count} | "
            f"frontier_cells={len(frontiers)} clusters={len(clusters)} "
            f"(sizes={[len(c) for c in clusters[:5]]})"
        )

        self.publish_frontier_markers(clusters, scan.header.stamp)
        self.publish_goal(clusters, odom, scan.header.stamp)

    def publish_map(self, stamp):
        msg = OccupancyGrid()
        msg.header = Header()
        msg.header.stamp = stamp
        msg.header.frame_id = self.map_frame
        msg.info.resolution = self.resolution
        msg.info.width = self.width
        msg.info.height = self.height
        msg.info.origin.position.x = self.origin_x
        msg.info.origin.position.y = self.origin_y
        msg.info.origin.orientation.w = 1.0
        msg.data = self.grid
        self.map_pub.publish(msg)

    def extract_frontiers(self) -> List[Tuple[int, int]]:
        frontiers = []
        for gy in range(1, self.height - 1):
            for gx in range(1, self.width - 1):
                idx = gy * self.width + gx
                if self.grid[idx] != 0:
                    continue
                if self.is_near_obstacle(gx, gy):
                    continue
                neighbors = ((1, 0), (-1, 0), (0, 1), (0, -1))
                for nx, ny in neighbors:
                    nidx = (gy + ny) * self.width + (gx + nx)
                    if self.grid[nidx] == -1:
                        frontiers.append((gx, gy))
                        break
        return frontiers

    def is_near_obstacle(self, gx: int, gy: int) -> bool:
        if self.obstacle_clearance_cells <= 0:
            return False
        radius = self.obstacle_clearance_cells
        for dy in range(-radius, radius + 1):
            for dx in range(-radius, radius + 1):
                nx = gx + dx
                ny = gy + dy
                if nx < 0 or ny < 0 or nx >= self.width or ny >= self.height:
                    continue
                nidx = ny * self.width + nx
                if self.grid[nidx] == 100:
                    return True
        return False

    def cluster_frontiers(self, frontiers: List[Tuple[int, int]]) -> List[List[Tuple[int, int]]]:
        frontier_set = set(frontiers)
        clusters: List[List[Tuple[int, int]]] = []
        visited = set()
        for cell in frontiers:
            if cell in visited:
                continue
            cluster = []
            queue = deque([cell])
            visited.add(cell)
            while queue:
                cx, cy = queue.popleft()
                cluster.append((cx, cy))
                for dx in (-1, 0, 1):
                    for dy in (-1, 0, 1):
                        if dx == 0 and dy == 0:
                            continue
                        nx, ny = cx + dx, cy + dy
                        if (nx, ny) in frontier_set and (nx, ny) not in visited:
                            visited.add((nx, ny))
                            queue.append((nx, ny))
            if len(cluster) >= self.frontier_min_size:
                clusters.append(cluster)
        return clusters

    def publish_frontier_markers(self, clusters: List[List[Tuple[int, int]]], stamp):
        marker_array = MarkerArray()
        for i, cluster in enumerate(clusters):
            marker = Marker()
            marker.header.stamp = stamp
            marker.header.frame_id = self.map_frame
            marker.ns = "frontier_regions"
            marker.id = i
            marker.type = Marker.POINTS
            marker.action = Marker.ADD
            marker.scale.x = 0.08
            marker.scale.y = 0.08
            marker.color.r = 0.2
            marker.color.g = 0.8
            marker.color.b = 1.0
            marker.color.a = 0.8
            for gx, gy in cluster:
                x, y = self.grid_to_world(gx, gy)
                marker.points.append(self.point(x, y))
            marker_array.markers.append(marker)
        self.regions_pub.publish(marker_array)

    def publish_goal(self, clusters: List[List[Tuple[int, int]]], odom: Odometry, stamp):
        if not clusters:
            return
        robot_x = odom.pose.pose.position.x
        robot_y = odom.pose.pose.position.y
        best = None
        best_dist = 1e9
        best_size = -1
        best_score = -1.0
        for cluster in clusters:
            sx = 0.0
            sy = 0.0
            for gx, gy in cluster:
                x, y = self.grid_to_world(gx, gy)
                sx += x
                sy += y
            cx = sx / len(cluster)
            cy = sy / len(cluster)
            dist = math.hypot(cx - robot_x, cy - robot_y)
            size = len(cluster)
            score = size / max(dist, 0.1)
            if self.selection_mode == "largest":
                if size > best_size:
                    best_size = size
                    best_dist = dist
                    best = (cx, cy)
            elif self.selection_mode == "score":
                if score > best_score:
                    best_score = score
                    best_dist = dist
                    best = (cx, cy)
            else:
                if dist < best_dist:
                    best_dist = dist
                    best = (cx, cy)
        if best is None:
            return

        now = Time.from_msg(stamp)
        if self.last_goal is not None and self.last_goal_time is not None:
            if self.goal_hold_sec > 0.0:
                if (now - self.last_goal_time).nanoseconds / 1e9 < self.goal_hold_sec:
                    return
            if self.goal_hysteresis_distance > 0.0:
                if math.hypot(best[0] - self.last_goal[0], best[1] - self.last_goal[1]) < self.goal_hysteresis_distance:
                    return

        self.get_logger().info(
            f"GOAL -> ({best[0]:.2f}, {best[1]:.2f})  "
            f"dist={best_dist:.2f} mode={self.selection_mode}"
        )
        self.publish_goal_marker(best[0], best[1], stamp)
        msg = PointStamped()
        msg.header.stamp = stamp
        msg.header.frame_id = self.map_frame
        msg.point.x = best[0]
        msg.point.y = best[1]
        msg.point.z = 0.0
        self.goal_pub.publish(msg)
        self.last_goal = (best[0], best[1])
        self.last_goal_time = now

    def publish_goal_marker(self, x: float, y: float, stamp):
        marker = Marker()
        marker.header.stamp = stamp
        marker.header.frame_id = self.map_frame
        marker.ns = "frontier_goal"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = 0.0
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.3
        marker.scale.y = 0.3
        marker.scale.z = 0.3
        marker.color.r = 1.0
        marker.color.g = 0.4
        marker.color.b = 0.0
        marker.color.a = 0.9
        self.goal_marker_pub.publish(marker)

    @staticmethod
    def quat_to_yaw(q) -> float:
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    @staticmethod
    def point(x: float, y: float):
        p = Point()
        p.x = float(x)
        p.y = float(y)
        p.z = 0.0
        return p


def main(args=None):
    rclpy.init(args=args)
    node = GeometricFrontier()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
