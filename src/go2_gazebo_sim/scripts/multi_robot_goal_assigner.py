#!/usr/bin/env python3
"""Centralized multi-robot frontier goal assignment (Burgard-style heuristic).

Score for robot i and target t:
    score(i, t) = U_t - beta * V_t^i

Where:
- U_t starts at 1.0 and is reduced around assigned targets by P(d).
- V_t^i is path cost from robot i to target t on robot i's occupancy grid.
"""

from __future__ import annotations

import math
from collections import deque
from typing import Optional

import rclpy
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node


class MultiRobotGoalAssigner(Node):
    def __init__(self) -> None:
        super().__init__("multi_robot_goal_assigner")

        self.declare_parameter("namespaces", ["robot_a", "robot_b"])
        self.declare_parameter("publish_rate", 1.0)
        self.declare_parameter("beta", 0.18)
        self.declare_parameter("sensor_range", 3.5)
        self.declare_parameter("frontier_stride", 2)
        self.declare_parameter("max_targets", 800)
        self.declare_parameter("goal_topic_suffix", "/way_point_coord")
        self.declare_parameter("use_shared_map", False)
        self.declare_parameter("shared_map_topic", "/disco_slam/global_map")
        self.declare_parameter("shared_map_wait_sec", 8.0)
        self.declare_parameter("free_value", 0)
        self.declare_parameter("unknown_value", -1)
        self.declare_parameter("occupancy_block_threshold", 50)
        self.declare_parameter("switch_hysteresis", 0.05)
        self.declare_parameter("switch_min_dist", 0.35)

        self.namespaces = [str(x) for x in self.get_parameter("namespaces").value]
        self.publish_rate = max(0.2, float(self.get_parameter("publish_rate").value))
        self.beta = float(self.get_parameter("beta").value)
        self.sensor_range = max(0.1, float(self.get_parameter("sensor_range").value))
        self.frontier_stride = max(1, int(self.get_parameter("frontier_stride").value))
        self.max_targets = max(50, int(self.get_parameter("max_targets").value))
        self.goal_topic_suffix = str(self.get_parameter("goal_topic_suffix").value)
        self.use_shared_map = bool(self.get_parameter("use_shared_map").value)
        self.shared_map_topic = str(self.get_parameter("shared_map_topic").value)
        self.shared_map_wait_sec = max(0.0, float(self.get_parameter("shared_map_wait_sec").value))
        self.free_value = int(self.get_parameter("free_value").value)
        self.unknown_value = int(self.get_parameter("unknown_value").value)
        self.occ_thresh = int(self.get_parameter("occupancy_block_threshold").value)
        self.switch_hysteresis = max(0.0, float(self.get_parameter("switch_hysteresis").value))
        self.switch_min_dist = max(0.1, float(self.get_parameter("switch_min_dist").value))

        self.maps: dict[str, OccupancyGrid] = {}
        self.shared_map: Optional[OccupancyGrid] = None
        self.odoms: dict[str, Odometry] = {}
        self.last_goal: dict[str, tuple[float, float]] = {}
        self._warned_missing_shared_map = False
        self._shared_map_fallback_active = False
        self._start_ns = self.get_clock().now().nanoseconds
        self._summary_interval_sec = 10.0
        self._last_summary_ns = 0

        self.goal_pubs = {}
        for ns in self.namespaces:
            self.create_subscription(OccupancyGrid, f"/{ns}/map", lambda m, n=ns: self._map_cb(m, n), 1)
            self.create_subscription(Odometry, f"/{ns}/odom/nav", lambda m, n=ns: self._odom_cb(m, n), 10)
            self.goal_pubs[ns] = self.create_publisher(PointStamped, f"/{ns}{self.goal_topic_suffix}", 10)
        if self.use_shared_map:
            self.create_subscription(OccupancyGrid, self.shared_map_topic, self._shared_map_cb, 1)

        self.timer = self.create_timer(1.0 / self.publish_rate, self._tick)
        self.get_logger().info(
            f"Coordinator started for {self.namespaces} | beta={self.beta:.2f}, sensor_range={self.sensor_range:.2f}, "
            f"use_shared_map={self.use_shared_map} shared_map_topic={self.shared_map_topic} "
            f"shared_map_wait_sec={self.shared_map_wait_sec:.1f}"
        )

    def _map_cb(self, msg: OccupancyGrid, ns: str) -> None:
        self.maps[ns] = msg

    def _odom_cb(self, msg: Odometry, ns: str) -> None:
        self.odoms[ns] = msg

    def _shared_map_cb(self, msg: OccupancyGrid) -> None:
        self.shared_map = msg
        if self._shared_map_fallback_active:
            self.get_logger().info(f"Shared map received on {self.shared_map_topic}; switching to shared-map coordination.")
        self._warned_missing_shared_map = False
        self._shared_map_fallback_active = False

    @staticmethod
    def _grid_index(x: int, y: int, w: int) -> int:
        return y * w + x

    def _world_to_grid(self, msg: OccupancyGrid, wx: float, wy: float) -> Optional[tuple[int, int]]:
        gx = int((wx - msg.info.origin.position.x) / msg.info.resolution)
        gy = int((wy - msg.info.origin.position.y) / msg.info.resolution)
        if gx < 0 or gy < 0 or gx >= msg.info.width or gy >= msg.info.height:
            return None
        return (gx, gy)

    def _grid_to_world(self, msg: OccupancyGrid, gx: int, gy: int) -> tuple[float, float]:
        return (
            msg.info.origin.position.x + (gx + 0.5) * msg.info.resolution,
            msg.info.origin.position.y + (gy + 0.5) * msg.info.resolution,
        )

    def _is_free(self, data: list[int], idx: int) -> bool:
        return data[idx] == self.free_value

    def _is_unknown(self, data: list[int], idx: int) -> bool:
        return data[idx] == self.unknown_value

    def _extract_frontiers(self, msg: OccupancyGrid) -> list[tuple[float, float]]:
        w = int(msg.info.width)
        h = int(msg.info.height)
        data = msg.data
        out: list[tuple[float, float]] = []
        s = self.frontier_stride

        for gy in range(1, h - 1, s):
            row = gy * w
            for gx in range(1, w - 1, s):
                idx = row + gx
                if not self._is_free(data, idx):
                    continue
                found_unknown = False
                for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1), (1, 1), (-1, 1), (1, -1), (-1, -1)):
                    nidx = (gy + dy) * w + (gx + dx)
                    if self._is_unknown(data, nidx):
                        found_unknown = True
                        break
                if found_unknown:
                    out.append(self._grid_to_world(msg, gx, gy))
                    if len(out) >= self.max_targets:
                        return out
        return out

    def _distance_transform(self, msg: OccupancyGrid, start_w: tuple[float, float]) -> dict[int, int]:
        start = self._world_to_grid(msg, start_w[0], start_w[1])
        if start is None:
            return {}

        w = int(msg.info.width)
        h = int(msg.info.height)
        data = msg.data
        sx, sy = start
        sidx = self._grid_index(sx, sy, w)

        if not self._is_free(data, sidx):
            found = None
            for r in range(1, 13):
                for dy in range(-r, r + 1):
                    ny = sy + dy
                    if ny < 0 or ny >= h:
                        continue
                    for dx in range(-r, r + 1):
                        nx = sx + dx
                        if nx < 0 or nx >= w:
                            continue
                        nidx = self._grid_index(nx, ny, w)
                        if self._is_free(data, nidx):
                            found = (nx, ny, nidx)
                            break
                    if found is not None:
                        break
                if found is not None:
                    break
            if found is None:
                return {}
            sx, sy, sidx = found

        q = deque([(sx, sy)])
        dist = {sidx: 0}
        while q:
            cx, cy = q.popleft()
            cidx = self._grid_index(cx, cy, w)
            for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                nx = cx + dx
                ny = cy + dy
                if nx < 0 or ny < 0 or nx >= w or ny >= h:
                    continue
                nidx = self._grid_index(nx, ny, w)
                if nidx in dist:
                    continue
                if not self._is_free(data, nidx):
                    continue
                dist[nidx] = dist[cidx] + 1
                q.append((nx, ny))
        return dist

    def _merge_targets(self, target_lists: list[list[tuple[float, float]]], merge_resolution: float) -> list[tuple[float, float]]:
        out: list[tuple[float, float]] = []
        seen: set[tuple[int, int]] = set()
        q = max(0.05, float(merge_resolution))
        for targets in target_lists:
            for wx, wy in targets:
                key = (int(round(wx / q)), int(round(wy / q)))
                if key in seen:
                    continue
                seen.add(key)
                out.append((wx, wy))
                if len(out) >= self.max_targets:
                    return out
        return out

    def _apply_switch_hysteresis(self, ns: str, goal: tuple[float, float], assignment_score: float) -> tuple[float, float]:
        last = self.last_goal.get(ns)
        if last is None:
            return goal

        od = self.odoms[ns]
        rx = float(od.pose.pose.position.x)
        ry = float(od.pose.pose.position.y)
        dist_to_last = math.hypot(last[0] - rx, last[1] - ry)
        move = math.hypot(goal[0] - last[0], goal[1] - last[1])

        # Only apply hold logic while still traveling to the previous goal.
        if dist_to_last > self.switch_min_dist:
            if move < self.switch_min_dist:
                return last
            if assignment_score < self.switch_hysteresis:
                return last
        return goal

    def _maybe_log_summary(
        self,
        targets_total: int,
        per_ns_frontiers: dict[str, int],
        per_ns_reachable: dict[str, int],
        per_ns_assigned: dict[str, tuple[float, float]],
    ) -> None:
        now_ns = self.get_clock().now().nanoseconds
        if self._last_summary_ns == 0:
            self._last_summary_ns = now_ns
            return
        if (now_ns - self._last_summary_ns) < int(self._summary_interval_sec * 1e9):
            return
        self._last_summary_ns = now_ns

        parts = []
        for ns in self.namespaces:
            goal = per_ns_assigned.get(ns)
            goal_txt = "None" if goal is None else f"({goal[0]:.2f},{goal[1]:.2f})"
            parts.append(
                f"{ns}:frontiers={per_ns_frontiers.get(ns, 0)} "
                f"reachable={per_ns_reachable.get(ns, 0)} goal={goal_txt}"
            )
        self.get_logger().info(
            f"ASSIGN step: targets={targets_total} | " + " | ".join(parts)
        )

    def _publish_goal(self, ns: str, map_msg: OccupancyGrid, goal_w: tuple[float, float]) -> None:
        msg = PointStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = map_msg.header.frame_id or "world"
        msg.point.x = goal_w[0]
        msg.point.y = goal_w[1]
        msg.point.z = 0.0
        self.goal_pubs[ns].publish(msg)

    def _tick(self) -> None:
        if any(ns not in self.maps or ns not in self.odoms for ns in self.namespaces):
            return

        using_shared_map = self.use_shared_map and self.shared_map is not None
        target_map: OccupancyGrid
        if self.use_shared_map:
            if using_shared_map:
                target_map = self.shared_map  # type: ignore[assignment]
                self._warned_missing_shared_map = False
                self._shared_map_fallback_active = False
            else:
                waited_sec = (self.get_clock().now().nanoseconds - self._start_ns) / 1e9
                if waited_sec >= self.shared_map_wait_sec:
                    # Fail-open: continue coordinated assignment on local map inputs
                    # until shared map becomes available.
                    target_map = self.maps[self.namespaces[0]]
                    if not self._shared_map_fallback_active:
                        self.get_logger().warn(
                            f"Shared map not available on {self.shared_map_topic} after {waited_sec:.1f}s; "
                            "falling back to per-robot maps."
                        )
                        self._shared_map_fallback_active = True
                else:
                    if not self._warned_missing_shared_map:
                        self.get_logger().warn(
                            f"Waiting for shared map on {self.shared_map_topic}; "
                            f"fallback in {max(0.0, self.shared_map_wait_sec - waited_sec):.1f}s."
                        )
                        self._warned_missing_shared_map = True
                    return
        else:
            target_map = self.maps[self.namespaces[0]]

        per_ns_targets: dict[str, list[tuple[float, float]]] = {}
        for ns in self.namespaces:
            per_ns_targets[ns] = self._extract_frontiers(self.maps[ns])

        if using_shared_map:
            targets = self._extract_frontiers(target_map)
        else:
            merge_res = max(0.1, float(target_map.info.resolution) * 2.0)
            targets = self._merge_targets([per_ns_targets[ns] for ns in self.namespaces], merge_res)

        if not targets:
            return

        dist_maps = {}
        for ns in self.namespaces:
            od = self.odoms[ns]
            # When shared map is unavailable we fail-open to per-robot maps.
            cost_map = target_map if using_shared_map else self.maps[ns]
            dist_maps[ns] = self._distance_transform(cost_map, (od.pose.pose.position.x, od.pose.pose.position.y))

        utilities = [1.0 for _ in targets]
        unassigned = set(self.namespaces)
        assigned: dict[str, int] = {}
        assignment_scores: dict[str, float] = {}
        sigma = max(self.sensor_range * 0.5, 1e-3)

        while unassigned:
            best_pair = None
            best_score = -1e18

            for ns in list(unassigned):
                msg = target_map if using_shared_map else self.maps[ns]
                dist_map = dist_maps[ns]
                if not dist_map:
                    continue
                for ti, (wx, wy) in enumerate(targets):
                    g = self._world_to_grid(msg, wx, wy)
                    if g is None:
                        continue
                    idx = self._grid_index(g[0], g[1], int(msg.info.width))
                    if idx not in dist_map:
                        continue
                    cost_m = float(dist_map[idx]) * msg.info.resolution
                    score = utilities[ti] - self.beta * cost_m
                    if score > best_score:
                        best_score = score
                        best_pair = (ns, ti, score)

            if best_pair is None:
                break

            ns, ti, score = best_pair
            assigned[ns] = ti
            assignment_scores[ns] = float(score)
            unassigned.remove(ns)

            tx, ty = targets[ti]
            for j, (wx, wy) in enumerate(targets):
                d = math.hypot(wx - tx, wy - ty)
                if d > self.sensor_range:
                    continue
                p = math.exp(-0.5 * (d / sigma) * (d / sigma))
                utilities[j] = max(0.0, utilities[j] - p)

        # Fail-open: if collaborative assignment cannot find a reachable shared target
        # for a robot, use that robot's nearest local frontier.
        for ns in list(unassigned):
            local_targets = per_ns_targets.get(ns, [])
            if not local_targets:
                continue
            od = self.odoms[ns]
            rx = float(od.pose.pose.position.x)
            ry = float(od.pose.pose.position.y)
            nearest_idx = min(
                range(len(local_targets)),
                key=lambda i: math.hypot(local_targets[i][0] - rx, local_targets[i][1] - ry),
            )
            goal = local_targets[nearest_idx]
            goal = self._apply_switch_hysteresis(ns, goal, assignment_score=1.0)
            self.last_goal[ns] = goal
            self._publish_goal(ns, self.maps[ns], goal)
            unassigned.remove(ns)

        per_ns_assigned: dict[str, tuple[float, float]] = {}
        for ns, ti in assigned.items():
            goal = targets[ti]
            goal = self._apply_switch_hysteresis(ns, goal, assignment_scores.get(ns, 0.0))
            self.last_goal[ns] = goal
            self._publish_goal(ns, self.maps[ns], goal)
            per_ns_assigned[ns] = goal

        for ns in self.namespaces:
            if ns in self.last_goal and ns not in per_ns_assigned:
                per_ns_assigned[ns] = self.last_goal[ns]

        per_ns_reachable: dict[str, int] = {}
        for ns in self.namespaces:
            msg = target_map if using_shared_map else self.maps[ns]
            dist_map = dist_maps.get(ns, {})
            if not dist_map:
                per_ns_reachable[ns] = 0
                continue
            reachable = 0
            for wx, wy in targets:
                g = self._world_to_grid(msg, wx, wy)
                if g is None:
                    continue
                idx = self._grid_index(g[0], g[1], int(msg.info.width))
                if idx in dist_map:
                    reachable += 1
            per_ns_reachable[ns] = reachable

        per_ns_frontiers = {ns: len(per_ns_targets.get(ns, [])) for ns in self.namespaces}
        self._maybe_log_summary(
            targets_total=len(targets),
            per_ns_frontiers=per_ns_frontiers,
            per_ns_reachable=per_ns_reachable,
            per_ns_assigned=per_ns_assigned,
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MultiRobotGoalAssigner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
