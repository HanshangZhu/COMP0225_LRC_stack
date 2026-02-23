#!/usr/bin/env python3
"""Simple geometric frontier exploration for occupancy-grid edge following.

Design goals:
- Frontiers are free cells adjacent to unknown cells (edge of explored map).
- Select goals greedily toward frontier edge (farthest valid cluster-center from robot).
- On /frontier_replan, switch away from current frontier if possible.
"""

import math
from collections import deque
from typing import List, Optional, Tuple

import rclpy
from geometry_msgs.msg import Point, PointStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Empty, Header
from tf2_ros import Buffer, ExtrapolationException, LookupException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray


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
    def __init__(self) -> None:
        super().__init__("geometric_frontier")

        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("odom_topic", "/odom/ground_truth")
        self.declare_parameter("map_frame", "world")
        self.declare_parameter("map_topic", "/map")
        self.declare_parameter("resolution", 0.1)
        self.declare_parameter("width", 400)
        self.declare_parameter("height", 400)
        self.declare_parameter("origin_x", -20.0)
        self.declare_parameter("origin_y", -20.0)
        self.declare_parameter("max_range", 6.0)
        self.declare_parameter("max_clear_distance", 4.0)
        self.declare_parameter("update_rate", 2.0)
        self.declare_parameter("startup_delay", 24.0)
        self.declare_parameter("frontier_min_size", 6)
        self.declare_parameter("goal_min_separation", 0.8)
        self.declare_parameter("goal_reselect_distance", 0.9)
        self.declare_parameter("min_goal_distance", 1.0)
        self.declare_parameter("max_goal_distance", 0.0)
        self.declare_parameter("goal_selection_mode", "farthest")
        self.declare_parameter("frontier_extraction_mode", "wfd")
        self.declare_parameter("require_path_feasibility", True)
        self.declare_parameter("max_path_stretch", 3.0)
        self.declare_parameter("frontier_goal_topic", "/way_point")
        self.declare_parameter("frontier_marker_topic", "/frontier_goal_marker")
        self.declare_parameter("frontier_regions_topic", "/frontier_markers")
        self.declare_parameter("frontier_replan_topic", "/frontier_replan")
        self.declare_parameter("traversability_inflation_cells", 1)
        self.declare_parameter("denoise_isolated_obstacles", True)
        self.declare_parameter("denoise_occ_min_neighbors", 2)
        self.declare_parameter("max_scan_odom_dt", 0.10)

        self.scan_topic = str(self.get_parameter("scan_topic").value)
        self.odom_topic = str(self.get_parameter("odom_topic").value)
        self.map_frame = str(self.get_parameter("map_frame").value)
        self.map_topic = str(self.get_parameter("map_topic").value)
        self.resolution = float(self.get_parameter("resolution").value)
        self.width = int(self.get_parameter("width").value)
        self.height = int(self.get_parameter("height").value)
        self.origin_x = float(self.get_parameter("origin_x").value)
        self.origin_y = float(self.get_parameter("origin_y").value)
        self.max_range = float(self.get_parameter("max_range").value)
        self.max_clear_distance = max(0.0, float(self.get_parameter("max_clear_distance").value))
        self.update_rate = max(0.5, float(self.get_parameter("update_rate").value))
        self.startup_delay = max(0.0, float(self.get_parameter("startup_delay").value))
        self.frontier_min_size = max(1, int(self.get_parameter("frontier_min_size").value))
        self.goal_min_separation = max(0.1, float(self.get_parameter("goal_min_separation").value))
        self.goal_reselect_distance = max(0.1, float(self.get_parameter("goal_reselect_distance").value))
        self.min_goal_distance = max(0.0, float(self.get_parameter("min_goal_distance").value))
        self.max_goal_distance = float(self.get_parameter("max_goal_distance").value)
        self.goal_selection_mode = str(self.get_parameter("goal_selection_mode").value).strip().lower()
        if self.goal_selection_mode not in ("farthest", "nearest"):
            self.goal_selection_mode = "farthest"
        self.frontier_extraction_mode = str(
            self.get_parameter("frontier_extraction_mode").value
        ).strip().lower()
        if self.frontier_extraction_mode not in ("wfd", "grid_scan"):
            self.frontier_extraction_mode = "wfd"
        self.require_path_feasibility = bool(self.get_parameter("require_path_feasibility").value)
        self.max_path_stretch = max(1.0, float(self.get_parameter("max_path_stretch").value))
        self.frontier_goal_topic = str(self.get_parameter("frontier_goal_topic").value)
        self.frontier_marker_topic = str(self.get_parameter("frontier_marker_topic").value)
        self.frontier_regions_topic = str(self.get_parameter("frontier_regions_topic").value)
        self.frontier_replan_topic = str(self.get_parameter("frontier_replan_topic").value)
        self.traversability_inflation_cells = max(0, int(self.get_parameter("traversability_inflation_cells").value))
        self.denoise_isolated_obstacles = bool(self.get_parameter("denoise_isolated_obstacles").value)
        self.denoise_occ_min_neighbors = max(0, int(self.get_parameter("denoise_occ_min_neighbors").value))
        self.max_scan_odom_dt = max(0.0, float(self.get_parameter("max_scan_odom_dt").value))

        self.grid = [-1] * (self.width * self.height)
        self.last_scan: Optional[LaserScan] = None
        self.last_odom: Optional[Odometry] = None
        self.laser_to_base = None

        self.start_time: Optional[Time] = None
        self.last_goal: Optional[Tuple[float, float]] = None
        self.force_replan_requested = False
        self._last_sync_warn_ns = 0
        self._summary_interval_sec = 10.0
        self._last_summary_ns = 0
        self._last_frontier_count = 0
        self._last_cluster_count = 0
        self._last_free_count = 0
        self._last_occ_count = 0
        self._last_selected_dist = None

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.create_subscription(LaserScan, self.scan_topic, self.scan_cb, qos_profile_sensor_data)
        self.create_subscription(Odometry, self.odom_topic, self.odom_cb, 10)
        self.create_subscription(Empty, self.frontier_replan_topic, self.replan_cb, 10)

        self.map_pub = self.create_publisher(OccupancyGrid, self.map_topic, 1)
        self.goal_pub = self.create_publisher(PointStamped, self.frontier_goal_topic, 10)
        self.goal_marker_pub = self.create_publisher(Marker, self.frontier_marker_topic, 10)
        self.regions_pub = self.create_publisher(MarkerArray, self.frontier_regions_topic, 10)

        self.timer = self.create_timer(1.0 / self.update_rate, self.update)
        self.get_logger().info("Simple geometric frontier node started")

    def scan_cb(self, msg: LaserScan) -> None:
        self.last_scan = msg

    def odom_cb(self, msg: Odometry) -> None:
        self.last_odom = msg

    def replan_cb(self, _msg: Empty) -> None:
        self.force_replan_requested = True
        self.get_logger().debug("Frontier replan requested.")

    def world_to_grid(self, x: float, y: float) -> Optional[Tuple[int, int]]:
        gx = int((x - self.origin_x) / self.resolution)
        gy = int((y - self.origin_y) / self.resolution)
        if gx < 0 or gy < 0 or gx >= self.width or gy >= self.height:
            return None
        return gx, gy

    def grid_to_world(self, gx: int, gy: int) -> Tuple[float, float]:
        return (
            self.origin_x + (gx + 0.5) * self.resolution,
            self.origin_y + (gy + 0.5) * self.resolution,
        )

    def set_cell(self, gx: int, gy: int, value: int) -> None:
        idx = gy * self.width + gx
        self.grid[idx] = value

    def update(self) -> None:
        if self.last_scan is None or self.last_odom is None:
            return

        now = self.get_clock().now()
        if self.start_time is None:
            self.start_time = now
        if (now - self.start_time).nanoseconds / 1e9 < self.startup_delay:
            return

        scan = self.last_scan
        odom = self.last_odom

        if self.max_scan_odom_dt > 0.0:
            scan_t = Time.from_msg(scan.header.stamp).nanoseconds
            odom_t = Time.from_msg(odom.header.stamp).nanoseconds
            if scan_t > 0 and odom_t > 0:
                dt = abs(scan_t - odom_t) / 1e9
                if dt > self.max_scan_odom_dt:
                    if now.nanoseconds - self._last_sync_warn_ns > int(2e9):
                        self.get_logger().warn(
                            f"scan/odom desync: dt={dt:.3f}s > {self.max_scan_odom_dt:.3f}s; skipping update"
                        )
                        self._last_sync_warn_ns = now.nanoseconds
                    return

        if self.laser_to_base is None:
            try:
                self.laser_to_base = self.tf_buffer.lookup_transform(
                    "base_link",
                    scan.header.frame_id,
                    Time(),
                    timeout=Duration(seconds=0.5),
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
        for rng in scan.ranges:
            finite = math.isfinite(rng)
            dist = min(rng, max_range) if finite else max_range
            bearing = angle
            angle += scan.angle_increment

            lx = dist * math.cos(bearing)
            ly = dist * math.sin(bearing)

            bx = math.cos(lb_yaw) * lx - math.sin(lb_yaw) * ly + lb_tx
            by = math.sin(lb_yaw) * lx + math.cos(lb_yaw) * ly + lb_ty
            wx = odom_x + math.cos(odom_yaw) * bx - math.sin(odom_yaw) * by
            wy = odom_y + math.sin(odom_yaw) * bx + math.cos(odom_yaw) * by

            end_cell = self.world_to_grid(wx, wy)
            if end_cell is None:
                continue

            clear_dist = dist if self.max_clear_distance <= 0.0 else min(dist, self.max_clear_distance)
            clx = clear_dist * math.cos(bearing)
            cly = clear_dist * math.sin(bearing)
            cbx = math.cos(lb_yaw) * clx - math.sin(lb_yaw) * cly + lb_tx
            cby = math.sin(lb_yaw) * clx + math.cos(lb_yaw) * cly + lb_ty
            cwx = odom_x + math.cos(odom_yaw) * cbx - math.sin(odom_yaw) * cby
            cwy = odom_y + math.sin(odom_yaw) * cbx + math.cos(odom_yaw) * cby
            clear_end_cell = self.world_to_grid(cwx, cwy)
            if clear_end_cell is None:
                continue

            cells = bresenham(origin_cell[0], origin_cell[1], clear_end_cell[0], clear_end_cell[1])
            for cx, cy in cells[:-1]:
                self.set_cell(cx, cy, 0)

            if finite and rng < max_range * 0.99:
                self.set_cell(end_cell[0], end_cell[1], 100)
            elif finite and dist <= clear_dist + 1e-6:
                self.set_cell(end_cell[0], end_cell[1], 0)
            # Non-finite range (no return): keep endpoint unknown (do not force free).

        self.denoise_grid()
        self.publish_map(scan.header.stamp)
        if self.frontier_extraction_mode == "wfd":
            clusters = self.extract_frontier_clusters_wfd(odom_x, odom_y)
            frontier_count = sum(len(c) for c in clusters)
        else:
            frontiers = self.extract_frontiers()
            clusters = self.cluster_frontiers(frontiers)
            frontier_count = len(frontiers)
        self.publish_frontier_markers(clusters, scan.header.stamp)
        self.publish_goal(clusters, odom, scan.header.stamp)
        self._last_frontier_count = frontier_count
        self._last_cluster_count = len(clusters)
        self._last_free_count = sum(1 for c in self.grid if c == 0)
        self._last_occ_count = sum(1 for c in self.grid if c == 100)
        self._log_global_summary(now)

    def extract_frontiers(self) -> List[Tuple[int, int]]:
        frontiers: List[Tuple[int, int]] = []
        for gy in range(1, self.height - 1):
            for gx in range(1, self.width - 1):
                idx = gy * self.width + gx
                if self.grid[idx] != 0:
                    continue

                # Edge-of-exploration frontier: free cell touching unknown.
                for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1), (1, 1), (1, -1), (-1, 1), (-1, -1)):
                    nidx = (gy + dy) * self.width + (gx + dx)
                    if self.grid[nidx] == -1:
                        frontiers.append((gx, gy))
                        break
        return frontiers

    def cluster_frontiers(self, frontiers: List[Tuple[int, int]]) -> List[List[Tuple[int, int]]]:
        frontier_set = set(frontiers)
        visited = set()
        clusters: List[List[Tuple[int, int]]] = []

        for seed in frontiers:
            if seed in visited:
                continue
            q = deque([seed])
            visited.add(seed)
            cluster: List[Tuple[int, int]] = []

            while q:
                cx, cy = q.popleft()
                cluster.append((cx, cy))
                for dx in (-1, 0, 1):
                    for dy in (-1, 0, 1):
                        if dx == 0 and dy == 0:
                            continue
                        nb = (cx + dx, cy + dy)
                        if nb in frontier_set and nb not in visited:
                            visited.add(nb)
                            q.append(nb)

            if len(cluster) >= self.frontier_min_size:
                clusters.append(cluster)

        return clusters

    def cluster_center_cell(self, cluster: List[Tuple[int, int]]) -> Tuple[int, int]:
        cx = sum(c[0] for c in cluster) / float(len(cluster))
        cy = sum(c[1] for c in cluster) / float(len(cluster))
        return min(cluster, key=lambda c: (c[0] - cx) ** 2 + (c[1] - cy) ** 2)

    def _in_bounds(self, gx: int, gy: int) -> bool:
        return 0 <= gx < self.width and 0 <= gy < self.height

    def _idx(self, gx: int, gy: int) -> int:
        return gy * self.width + gx

    def _xy_from_idx(self, idx: int) -> Tuple[int, int]:
        return idx % self.width, idx // self.width

    def _is_free_idx(self, idx: int) -> bool:
        return self.grid[idx] == 0

    def _has_unknown_neighbor_idx(self, idx: int) -> bool:
        gx, gy = self._xy_from_idx(idx)
        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                if dx == 0 and dy == 0:
                    continue
                nx = gx + dx
                ny = gy + dy
                if not self._in_bounds(nx, ny):
                    continue
                if self.grid[self._idx(nx, ny)] == -1:
                    return True
        return False

    def _is_frontier_idx(self, idx: int) -> bool:
        return self._is_free_idx(idx) and self._has_unknown_neighbor_idx(idx)

    def _nearest_open_idx(self, sx: int, sy: int) -> Optional[int]:
        if not self._in_bounds(sx, sy):
            return None
        sidx = self._idx(sx, sy)
        if self._is_free_idx(sidx):
            return sidx

        q = deque([sidx])
        visited = {sidx}
        while q:
            cur = q.popleft()
            cx, cy = self._xy_from_idx(cur)
            for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                nx = cx + dx
                ny = cy + dy
                if not self._in_bounds(nx, ny):
                    continue
                nidx = self._idx(nx, ny)
                if nidx in visited:
                    continue
                visited.add(nidx)
                if self._is_free_idx(nidx):
                    return nidx
                q.append(nidx)
        return None

    def extract_frontier_clusters_wfd(self, robot_x: float, robot_y: float) -> List[List[Tuple[int, int]]]:
        """Wavefront Frontier Detection (WFD) from current robot pose."""
        start = self.world_to_grid(robot_x, robot_y)
        if start is None:
            return []
        start_idx = self._nearest_open_idx(start[0], start[1])
        if start_idx is None:
            return []

        map_queue = deque([start_idx])
        map_open = {start_idx}
        map_close: set[int] = set()
        frontier_close: set[int] = set()
        clusters: List[List[Tuple[int, int]]] = []

        while map_queue:
            p_idx = map_queue.popleft()
            map_open.discard(p_idx)
            if p_idx in map_close:
                continue

            if self._is_frontier_idx(p_idx):
                frontier_queue = deque([p_idx])
                frontier_open = {p_idx}
                cluster_idx: List[int] = []

                while frontier_queue:
                    q_idx = frontier_queue.popleft()
                    frontier_open.discard(q_idx)
                    if q_idx in frontier_close or q_idx in map_close:
                        continue
                    if self._is_frontier_idx(q_idx):
                        cluster_idx.append(q_idx)
                        qx, qy = self._xy_from_idx(q_idx)
                        for dx in (-1, 0, 1):
                            for dy in (-1, 0, 1):
                                if dx == 0 and dy == 0:
                                    continue
                                nx = qx + dx
                                ny = qy + dy
                                if not self._in_bounds(nx, ny):
                                    continue
                                nidx = self._idx(nx, ny)
                                if (
                                    nidx in frontier_open
                                    or nidx in frontier_close
                                    or nidx in map_close
                                ):
                                    continue
                                frontier_queue.append(nidx)
                                frontier_open.add(nidx)
                    frontier_close.add(q_idx)

                for fidx in cluster_idx:
                    map_close.add(fidx)
                if len(cluster_idx) >= self.frontier_min_size:
                    clusters.append([self._xy_from_idx(i) for i in cluster_idx])
                continue

            px, py = self._xy_from_idx(p_idx)
            for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                nx = px + dx
                ny = py + dy
                if not self._in_bounds(nx, ny):
                    continue
                nidx = self._idx(nx, ny)
                if nidx in map_open or nidx in map_close:
                    continue
                if not self._is_free_idx(nidx):
                    continue
                map_queue.append(nidx)
                map_open.add(nidx)

            map_close.add(p_idx)

        return clusters

    def denoise_grid(self) -> None:
        """Suppress isolated occupied speckles from noisy scans."""
        if not self.denoise_isolated_obstacles:
            return

        src = self.grid
        dst = list(src)
        changed = 0
        for gy in range(1, self.height - 1):
            row = gy * self.width
            for gx in range(1, self.width - 1):
                idx = row + gx
                if src[idx] != 100:
                    continue
                occ_n = 0
                for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1), (1, 1), (1, -1), (-1, 1), (-1, -1)):
                    if src[(gy + dy) * self.width + (gx + dx)] == 100:
                        occ_n += 1
                if occ_n < self.denoise_occ_min_neighbors:
                    dst[idx] = -1
                    changed += 1

        if changed > 0:
            self.grid = dst

    def build_traversable_mask(self) -> List[bool]:
        """Return a free-space mask with optional obstacle inflation."""
        traversable = [c == 0 for c in self.grid]
        infl = self.traversability_inflation_cells
        if infl <= 0:
            return traversable

        blocked = [False] * (self.width * self.height)
        for gy in range(self.height):
            row = gy * self.width
            for gx in range(self.width):
                if self.grid[row + gx] != 100:
                    continue
                for dy in range(-infl, infl + 1):
                    ny = gy + dy
                    if ny < 0 or ny >= self.height:
                        continue
                    for dx in range(-infl, infl + 1):
                        nx = gx + dx
                        if nx < 0 or nx >= self.width:
                            continue
                        blocked[ny * self.width + nx] = True

        for i, b in enumerate(blocked):
            if b:
                traversable[i] = False
        return traversable

    def reachable_distance_cells(self, robot_x: float, robot_y: float, traversable: List[bool]) -> dict[int, int]:
        start = self.world_to_grid(robot_x, robot_y)
        if start is None:
            return {}

        sx, sy = start
        start_idx = sy * self.width + sx
        if not traversable[start_idx]:
            # Best-effort: find nearest traversable cell around robot.
            found = None
            for r in range(1, 5):
                for dy in range(-r, r + 1):
                    ny = sy + dy
                    if ny < 0 or ny >= self.height:
                        continue
                    for dx in range(-r, r + 1):
                        nx = sx + dx
                        if nx < 0 or nx >= self.width:
                            continue
                        nidx = ny * self.width + nx
                        if traversable[nidx]:
                            found = (nx, ny, nidx)
                            break
                    if found is not None:
                        break
                if found is not None:
                    break
            if found is None:
                return {}
            sx, sy, start_idx = found

        q = deque([(sx, sy)])
        dist_steps = {start_idx: 0}
        while q:
            cx, cy = q.popleft()
            for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                nx = cx + dx
                ny = cy + dy
                if nx < 0 or ny < 0 or nx >= self.width or ny >= self.height:
                    continue
                nidx = ny * self.width + nx
                if nidx in dist_steps or not traversable[nidx]:
                    continue
                cidx = cy * self.width + cx
                dist_steps[nidx] = dist_steps[cidx] + 1
                q.append((nx, ny))
        return dist_steps

    def select_goal(
        self,
        clusters: List[List[Tuple[int, int]]],
        robot_x: float,
        robot_y: float,
        force_replan: bool,
        reachable_steps: dict[int, int],
    ) -> Optional[Tuple[float, float, float]]:
        candidates: List[Tuple[float, float, float, float]] = []
        for cluster in clusters:
            gx, gy = self.cluster_center_cell(cluster)
            gidx = gy * self.width + gx
            if gidx not in reachable_steps:
                continue
            wx, wy = self.grid_to_world(gx, gy)
            euclid_dist = math.hypot(wx - robot_x, wy - robot_y)
            if euclid_dist < self.min_goal_distance:
                continue
            if self.max_goal_distance > 0.0 and euclid_dist > self.max_goal_distance:
                continue
            path_dist = float(reachable_steps[gidx]) * self.resolution
            if self.require_path_feasibility:
                # Reject frontiers with highly circuitous paths.
                denom = max(euclid_dist, self.resolution)
                if (path_dist / denom) > self.max_path_stretch:
                    continue
            candidates.append((wx, wy, euclid_dist, path_dist))

        if not candidates:
            return None

        if self.goal_selection_mode == "nearest":
            # Conservative behavior: nearest feasible path target.
            candidates.sort(key=lambda c: (c[3], c[2]))
        else:
            # Aggressive exploration: prefer farther feasible frontiers.
            candidates.sort(key=lambda c: (-c[3], -c[2]))

        if force_replan and self.last_goal is not None:
            for c in candidates:
                if math.hypot(c[0] - self.last_goal[0], c[1] - self.last_goal[1]) >= self.goal_min_separation:
                    return (c[0], c[1], c[2])

        best = candidates[0]
        return (best[0], best[1], best[2])

    def publish_map(self, stamp) -> None:
        msg = OccupancyGrid()
        msg.header = Header(stamp=stamp, frame_id=self.map_frame)
        msg.info.resolution = self.resolution
        msg.info.width = self.width
        msg.info.height = self.height
        msg.info.origin.position.x = self.origin_x
        msg.info.origin.position.y = self.origin_y
        msg.info.origin.orientation.w = 1.0
        msg.data = self.grid
        self.map_pub.publish(msg)

    def publish_goal(self, clusters: List[List[Tuple[int, int]]], odom: Odometry, stamp) -> None:
        if not clusters:
            return

        robot_x = odom.pose.pose.position.x
        robot_y = odom.pose.pose.position.y
        traversable = self.build_traversable_mask()
        reachable_steps = self.reachable_distance_cells(robot_x, robot_y, traversable)
        if not reachable_steps:
            return

        # Keep pursuing the current goal to avoid A/B frontier oscillation.
        if self.last_goal is not None and not self.force_replan_requested:
            dist_to_last = math.hypot(self.last_goal[0] - robot_x, self.last_goal[1] - robot_y)
            if dist_to_last > self.goal_reselect_distance:
                return

        selected = self.select_goal(clusters, robot_x, robot_y, self.force_replan_requested, reachable_steps)
        if selected is None:
            self.force_replan_requested = False
            return

        gx, gy, dist = selected

        # Avoid noisy retargeting unless a replan was explicitly requested.
        if (
            self.last_goal is not None
            and not self.force_replan_requested
            and math.hypot(gx - self.last_goal[0], gy - self.last_goal[1]) < 0.35
        ):
            return

        self.publish_goal_marker(gx, gy, stamp)

        msg = PointStamped()
        msg.header.stamp = stamp
        msg.header.frame_id = self.map_frame
        msg.point.x = gx
        msg.point.y = gy
        msg.point.z = 0.0
        self.goal_pub.publish(msg)

        self.last_goal = (gx, gy)
        self.force_replan_requested = False
        self._last_selected_dist = dist
        self.get_logger().info(f"GOAL -> ({gx:.2f}, {gy:.2f}) dist={dist:.2f}")

    def _log_global_summary(self, now: Time) -> None:
        if self._last_summary_ns == 0:
            self._last_summary_ns = now.nanoseconds
            return
        if (now.nanoseconds - self._last_summary_ns) < int(self._summary_interval_sec * 1e9):
            return
        self._last_summary_ns = now.nanoseconds

        goal_txt = "None"
        if self.last_goal is not None:
            goal_txt = f"({self.last_goal[0]:.2f},{self.last_goal[1]:.2f})"
        dist_txt = "-" if self._last_selected_dist is None else f"{self._last_selected_dist:.2f}"
        self.get_logger().info(
            f"GLOBAL step: free={self._last_free_count} occ={self._last_occ_count} "
            f"frontiers={self._last_frontier_count} clusters={self._last_cluster_count} "
            f"goal={goal_txt} goal_dist={dist_txt}"
        )

    def publish_frontier_markers(self, clusters: List[List[Tuple[int, int]]], stamp) -> None:
        out = MarkerArray()

        clear = Marker()
        clear.header.stamp = stamp
        clear.header.frame_id = self.map_frame
        clear.action = Marker.DELETEALL
        out.markers.append(clear)

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
                wx, wy = self.grid_to_world(gx, gy)
                marker.points.append(self.point(wx, wy))

            out.markers.append(marker)

        self.regions_pub.publish(out)

    def publish_goal_marker(self, x: float, y: float, stamp) -> None:
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
        marker.scale.x = 0.30
        marker.scale.y = 0.30
        marker.scale.z = 0.30
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
    def point(x: float, y: float) -> Point:
        p = Point()
        p.x = float(x)
        p.y = float(y)
        p.z = 0.0
        return p


def main(args=None) -> None:
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
