#!/usr/bin/env python3
"""Centralized multi-robot frontier goal assignment.

Legacy mode score:
    score(i, t) = U_t - beta * V_t^i

Committed mode adds Level-1 anti-oscillation controls:
- goal lock timer
- progress-gated switching
- frontier blacklist after repeated failures

M-TARE mode adds:
- overlap penalty (mutual exclusion)
- pursuit utility when teammates become stale
"""

from __future__ import annotations

import math
import sys
import time
from collections import deque
from pathlib import Path
from typing import Any, Optional

import rclpy
from geometry_msgs.msg import Point, PointStamped
from map_merge_utils import build_fallback_map, build_shared_with_local_patches
from mdvrp_solver import first_goal_for_route, solve_mdvrp
from mtare_ros2.msg import GridWorldStatus
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from visualization_msgs.msg import Marker, MarkerArray


def _resolve_cfpa2_overlap_penalty_fn():
    candidates: list[Path] = []
    here = Path(__file__).resolve()
    # Source-tree path when running via symlink-install or directly.
    if len(here.parents) >= 3:
        candidates.append(here.parents[2] / "cfpa2-collaborative-exploration")
    # Installed script path fallback.
    if len(here.parents) >= 5:
        candidates.append(here.parents[4] / "src" / "cfpa2-collaborative-exploration")

    for root in candidates:
        if not (root / "cfpa2_demo").is_dir():
            continue
        root_str = str(root)
        if root_str not in sys.path:
            sys.path.insert(0, root_str)
        try:
            from cfpa2_demo.core.allocator import overlap_penalty as cfpa2_overlap_penalty
        except Exception:
            continue
        return cfpa2_overlap_penalty
    return None


_CFPA2_OVERLAP_PENALTY_FN = _resolve_cfpa2_overlap_penalty_fn()


def select_first_route_goals(
    *,
    namespaces: list[str],
    routes: dict[int, list[int]],
    exploring_cells: list[tuple[float, float, float]],
    robot_xy: dict[str, tuple[float, float]],
    min_assign_distance: float,
) -> dict[str, tuple[float, float]]:
    goals: dict[str, tuple[float, float]] = {}
    for idx, ns in enumerate(namespaces):
        goals_for_robot = routes.get(idx, [])
        goal = first_goal_for_route(
            goals_for_robot,
            exploring_cells,
            robot_xy=robot_xy.get(ns),
            min_assign_distance=min_assign_distance,
        )
        if goal is not None:
            goals[ns] = goal
    return goals


class MTareCoordinator(Node):
    def __init__(self) -> None:
        super().__init__("mtare_coordinator")

        self.declare_parameter("namespaces", ["robot_a", "robot_b"])
        self.declare_parameter("publish_rate", 1.0)
        self.declare_parameter("beta", 0.18)
        self.declare_parameter("sensor_range", 3.5)
        self.declare_parameter("frontier_stride", 2)
        self.declare_parameter("max_targets", 800)
        self.declare_parameter("goal_topic_suffix", "/way_point_coord")
        self.declare_parameter("output_mode", "waypoint_coord")
        self.declare_parameter("tare_goal_topic_suffix", "/way_point_tare")
        self.declare_parameter("relocation_goal_topic_suffix", "/goal_point")
        self.declare_parameter("grid_world_status_topic_suffix", "/grid_world_status")
        self.declare_parameter("use_shared_map", False)
        self.declare_parameter("shared_map_topic", "/disco_slam/global_map")
        self.declare_parameter("shared_map_wait_sec", 8.0)
        self.declare_parameter("shared_map_local_patch_radius_m", 2.5)
        self.declare_parameter("free_value", 0)
        self.declare_parameter("unknown_value", -1)
        self.declare_parameter("occupancy_block_threshold", 50)
        self.declare_parameter("switch_hysteresis", 0.05)
        self.declare_parameter("switch_min_dist", 0.35)
        self.declare_parameter("min_assign_distance", 0.30)

        # Algorithm-selection and Level-1 stabilization controls.
        self.declare_parameter("algorithm_mode", "mtare")
        self.declare_parameter("goal_lock_sec", 5.0)
        self.declare_parameter("progress_window_sec", 3.0)
        self.declare_parameter("progress_min_delta_m", 0.15)
        self.declare_parameter("blacklist_fail_count", 2)
        self.declare_parameter("blacklist_ttl_sec", 30.0)
        self.declare_parameter("blacklist_key_resolution", 0.5)
        self.declare_parameter("reached_blacklist_dist", 0.30)
        self.declare_parameter("reached_blacklist_repeat_count", 3)
        self.declare_parameter("reached_blacklist_ttl_sec", 12.0)
        self.declare_parameter("overlap_weight", 1.0)
        self.declare_parameter("cfpa2_w_ig", 1.0)
        self.declare_parameter("cfpa2_w_c", 0.6)
        self.declare_parameter("cfpa2_w_sw", 0.2)
        self.declare_parameter("cfpa2_lambda_overlap", 1.0)
        self.declare_parameter("cfpa2_sigma_overlap_m", 0.0)
        self.declare_parameter("communication_timeout_sec", 6.0)
        self.declare_parameter("prediction_horizon_sec", 4.0)
        self.declare_parameter("pursuit_weight", 2.0)
        self.declare_parameter("pursuit_switch_margin", 0.1)
        self.declare_parameter("exploration_gain_radius_cells", 4)
        self.declare_parameter("meeting_min_distance", 1.5)
        self.declare_parameter("teammate_stale_ttl_sec", 120.0)
        self.declare_parameter("mui_resolve_period_sec", 5.0)
        self.declare_parameter("mui_mdvrp_time_limit_sec", 1.0)
        self.declare_parameter("mui_max_exploring_cells", 120)
        self.declare_parameter("mui_cell_merge_resolution_m", 1.0)
        self.declare_parameter("mui_unreachable_penalty_m", 200.0)
        self.declare_parameter("marker_frame_override", "world")
        self.declare_parameter("coordinator_map_topic", "/mtare/coordinator_map")
        self.declare_parameter("robot_markers_topic", "/mtare/robot_markers")
        self.declare_parameter("trajectory_max_points", 600)
        self.declare_parameter("trajectory_min_point_distance", 0.08)
        self.declare_parameter("robot_marker_scale", 0.35)
        self.declare_parameter("perf_enable", True)
        self.declare_parameter("perf_tick_window_size", 240)
        self.declare_parameter("perf_min_samples", 20)
        self.declare_parameter("perf_tick_warn_p95_ms", 150.0)
        self.declare_parameter("perf_cpu_warn_pct", 15.0)

        self.namespaces = [str(x) for x in self.get_parameter("namespaces").value]
        self.publish_rate = max(0.2, float(self.get_parameter("publish_rate").value))
        self.beta = float(self.get_parameter("beta").value)
        self.sensor_range = max(0.1, float(self.get_parameter("sensor_range").value))
        self.frontier_stride = max(1, int(self.get_parameter("frontier_stride").value))
        self.max_targets = max(50, int(self.get_parameter("max_targets").value))
        self.goal_topic_suffix = str(self.get_parameter("goal_topic_suffix").value)
        self.output_mode = str(self.get_parameter("output_mode").value).strip().lower()
        if self.output_mode not in {"waypoint_coord", "exact_split"}:
            self.get_logger().warn(
                f"Unknown output_mode='{self.output_mode}', falling back to waypoint_coord"
            )
            self.output_mode = "waypoint_coord"
        self.tare_goal_topic_suffix = str(self.get_parameter("tare_goal_topic_suffix").value)
        self.relocation_goal_topic_suffix = str(self.get_parameter("relocation_goal_topic_suffix").value)
        self.grid_world_status_topic_suffix = str(
            self.get_parameter("grid_world_status_topic_suffix").value
        )
        self.use_shared_map = bool(self.get_parameter("use_shared_map").value)
        self.shared_map_topic = str(self.get_parameter("shared_map_topic").value)
        self.shared_map_wait_sec = max(0.0, float(self.get_parameter("shared_map_wait_sec").value))
        self.shared_map_local_patch_radius_m = max(
            0.0, float(self.get_parameter("shared_map_local_patch_radius_m").value)
        )
        self.free_value = int(self.get_parameter("free_value").value)
        self.unknown_value = int(self.get_parameter("unknown_value").value)
        self.occ_thresh = int(self.get_parameter("occupancy_block_threshold").value)
        self.switch_hysteresis = max(0.0, float(self.get_parameter("switch_hysteresis").value))
        self.switch_min_dist = max(0.1, float(self.get_parameter("switch_min_dist").value))
        self.min_assign_distance = max(0.0, float(self.get_parameter("min_assign_distance").value))

        self.algorithm_mode = str(self.get_parameter("algorithm_mode").value).strip().lower()
        if self.algorithm_mode not in {"legacy", "committed", "mtare", "cfpa2", "mui_tare"}:
            self.get_logger().warn(
                f"Unknown algorithm_mode='{self.algorithm_mode}', falling back to mtare"
            )
            self.algorithm_mode = "mtare"

        self.goal_lock_sec = max(0.0, float(self.get_parameter("goal_lock_sec").value))
        self.progress_window_sec = max(0.5, float(self.get_parameter("progress_window_sec").value))
        self.progress_min_delta_m = max(0.0, float(self.get_parameter("progress_min_delta_m").value))
        self.blacklist_fail_count = max(1, int(self.get_parameter("blacklist_fail_count").value))
        self.blacklist_ttl_sec = max(0.0, float(self.get_parameter("blacklist_ttl_sec").value))
        self.blacklist_key_resolution = max(
            0.05,
            float(self.get_parameter("blacklist_key_resolution").value),
        )
        self.reached_blacklist_dist = max(0.0, float(self.get_parameter("reached_blacklist_dist").value))
        self.reached_blacklist_repeat_count = max(
            1,
            int(self.get_parameter("reached_blacklist_repeat_count").value),
        )
        self.reached_blacklist_ttl_sec = max(
            0.0,
            float(self.get_parameter("reached_blacklist_ttl_sec").value),
        )
        self.overlap_weight = max(0.0, float(self.get_parameter("overlap_weight").value))
        self.cfpa2_w_ig = float(self.get_parameter("cfpa2_w_ig").value)
        self.cfpa2_w_c = max(0.0, float(self.get_parameter("cfpa2_w_c").value))
        self.cfpa2_w_sw = max(0.0, float(self.get_parameter("cfpa2_w_sw").value))
        self.cfpa2_lambda_overlap = max(0.0, float(self.get_parameter("cfpa2_lambda_overlap").value))
        self.cfpa2_sigma_overlap_m = max(0.0, float(self.get_parameter("cfpa2_sigma_overlap_m").value))
        self.communication_timeout_sec = max(0.0, float(self.get_parameter("communication_timeout_sec").value))
        self.prediction_horizon_sec = max(0.0, float(self.get_parameter("prediction_horizon_sec").value))
        self.pursuit_weight = max(0.0, float(self.get_parameter("pursuit_weight").value))
        self.pursuit_switch_margin = float(self.get_parameter("pursuit_switch_margin").value)
        self.exploration_gain_radius_cells = max(1, int(self.get_parameter("exploration_gain_radius_cells").value))
        self.meeting_min_distance = max(0.0, float(self.get_parameter("meeting_min_distance").value))
        self.teammate_stale_ttl_sec = max(0.0, float(self.get_parameter("teammate_stale_ttl_sec").value))
        self.mui_resolve_period_sec = max(0.2, float(self.get_parameter("mui_resolve_period_sec").value))
        self.mui_mdvrp_time_limit_sec = max(0.1, float(self.get_parameter("mui_mdvrp_time_limit_sec").value))
        self.mui_max_exploring_cells = max(10, int(self.get_parameter("mui_max_exploring_cells").value))
        self.mui_cell_merge_resolution_m = max(
            0.05,
            float(self.get_parameter("mui_cell_merge_resolution_m").value),
        )
        self.mui_unreachable_penalty_m = max(
            0.0,
            float(self.get_parameter("mui_unreachable_penalty_m").value),
        )
        self.marker_frame_override = str(self.get_parameter("marker_frame_override").value).strip()
        self.coordinator_map_topic = str(self.get_parameter("coordinator_map_topic").value).strip()
        self.robot_markers_topic = str(self.get_parameter("robot_markers_topic").value).strip()
        self.trajectory_max_points = max(10, int(self.get_parameter("trajectory_max_points").value))
        self.trajectory_min_point_distance = max(
            0.0, float(self.get_parameter("trajectory_min_point_distance").value)
        )
        self.robot_marker_scale = max(0.05, float(self.get_parameter("robot_marker_scale").value))
        self.perf_enable = bool(self.get_parameter("perf_enable").value)
        self.perf_tick_window_size = max(20, int(self.get_parameter("perf_tick_window_size").value))
        self.perf_min_samples = max(5, int(self.get_parameter("perf_min_samples").value))
        self.perf_tick_warn_p95_ms = max(0.0, float(self.get_parameter("perf_tick_warn_p95_ms").value))
        self.perf_cpu_warn_pct = max(0.0, float(self.get_parameter("perf_cpu_warn_pct").value))

        self.maps: dict[str, OccupancyGrid] = {}
        self.shared_map: Optional[OccupancyGrid] = None
        self.odoms: dict[str, Odometry] = {}
        self.grid_world_status: dict[str, GridWorldStatus] = {}
        self.grid_world_status_rx_time_ns: dict[str, int] = {}
        self.last_goal: dict[str, tuple[float, float]] = {}
        self.last_goal_set_time_ns: dict[str, int] = {}
        self.goal_progress_samples: dict[str, deque[tuple[int, float]]] = {
            ns: deque() for ns in self.namespaces
        }
        self.goal_fail_counts: dict[str, dict[tuple[int, int], int]] = {
            ns: {} for ns in self.namespaces
        }
        self.goal_blacklist_until_ns: dict[str, dict[tuple[int, int], int]] = {
            ns: {} for ns in self.namespaces
        }
        self.reached_goal_repeat_count: dict[str, int] = {ns: 0 for ns in self.namespaces}
        self.reached_goal_last_key: dict[str, Optional[tuple[int, int]]] = {
            ns: None for ns in self.namespaces
        }
        self.last_policy_reason: dict[str, str] = {ns: "init" for ns in self.namespaces}
        self.odom_rx_time_ns: dict[str, int] = {}
        self.odom_velocity_xy: dict[str, tuple[float, float]] = {ns: (0.0, 0.0) for ns in self.namespaces}
        self.trajectory_history: dict[str, deque[tuple[float, float]]] = {
            ns: deque(maxlen=self.trajectory_max_points) for ns in self.namespaces
        }

        self._warned_missing_shared_map = False
        self._shared_map_fallback_active = False
        self._warned_cfpa2_two_robot_only = False
        self._start_ns = self.get_clock().now().nanoseconds
        self._summary_interval_sec = 10.0
        self._last_summary_ns = 0
        self._last_prereq_warn_ns = 0
        self._tick_period_ms = 1000.0 / self.publish_rate
        self._perf_tick_durations_ms: deque[float] = deque(maxlen=self.perf_tick_window_size)
        self._last_perf_summary_ns = 0
        self._perf_last_cpu_process_sec = time.process_time()
        self._perf_last_cpu_wall_ns = time.perf_counter_ns()
        self._mui_last_solve_ns = 0
        self._mui_last_cell_keys: set[tuple[int, int, int]] = set()
        self._mui_routes: dict[str, list[int]] = {ns: [] for ns in self.namespaces}
        self._mui_cover_by_others: dict[str, set[int]] = {ns: set() for ns in self.namespaces}

        self.goal_pubs = {}
        self.tare_goal_pubs = {}
        self.relocation_goal_pubs = {}
        self.goal_marker_pubs = {}
        coordinator_map_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.coordinator_map_pub = self.create_publisher(
            OccupancyGrid,
            self.coordinator_map_topic,
            coordinator_map_qos,
        )
        self.robot_markers_pub = self.create_publisher(MarkerArray, self.robot_markers_topic, 10)
        for ns in self.namespaces:
            self.create_subscription(OccupancyGrid, f"/{ns}/map", lambda m, n=ns: self._map_cb(m, n), 1)
            self.create_subscription(Odometry, f"/{ns}/odom/nav", lambda m, n=ns: self._odom_cb(m, n), 10)
            self.create_subscription(
                GridWorldStatus,
                f"/{ns}{self.grid_world_status_topic_suffix}",
                lambda m, n=ns: self._grid_world_status_cb(m, n),
                10,
            )
            self.goal_pubs[ns] = self.create_publisher(PointStamped, f"/{ns}{self.goal_topic_suffix}", 10)
            self.tare_goal_pubs[ns] = self.create_publisher(PointStamped, f"/{ns}{self.tare_goal_topic_suffix}", 10)
            self.relocation_goal_pubs[ns] = self.create_publisher(
                PointStamped, f"/{ns}{self.relocation_goal_topic_suffix}", 10
            )
            self.goal_marker_pubs[ns] = self.create_publisher(Marker, f"/{ns}/mtare_goal_marker", 10)
        if self.use_shared_map:
            self.create_subscription(OccupancyGrid, self.shared_map_topic, self._shared_map_cb, 1)

        self.timer = self.create_timer(1.0 / self.publish_rate, self._tick)
        self.get_logger().info(
            f"Coordinator started for {self.namespaces} | mode={self.algorithm_mode} "
            f"beta={self.beta:.2f}, sensor_range={self.sensor_range:.2f}, "
            f"goal_lock_sec={self.goal_lock_sec:.1f}, progress_window_sec={self.progress_window_sec:.1f}, "
            f"progress_min_delta_m={self.progress_min_delta_m:.2f}, "
            f"blacklist_fail_count={self.blacklist_fail_count}, blacklist_ttl_sec={self.blacklist_ttl_sec:.1f}, "
            f"min_assign_distance={self.min_assign_distance:.2f}, "
            f"reached_blacklist_dist={self.reached_blacklist_dist:.2f}, "
            f"reached_blacklist_repeat_count={self.reached_blacklist_repeat_count}, "
            f"reached_blacklist_ttl_sec={self.reached_blacklist_ttl_sec:.1f}, "
            f"overlap_weight={self.overlap_weight:.2f}, "
            f"cfpa2_w_ig={self.cfpa2_w_ig:.2f}, "
            f"cfpa2_w_c={self.cfpa2_w_c:.2f}, "
            f"cfpa2_w_sw={self.cfpa2_w_sw:.2f}, "
            f"cfpa2_lambda_overlap={self.cfpa2_lambda_overlap:.2f}, "
            f"cfpa2_sigma_overlap_m={self.cfpa2_sigma_overlap_m:.2f}, "
            f"communication_timeout_sec={self.communication_timeout_sec:.2f}, "
            f"prediction_horizon_sec={self.prediction_horizon_sec:.2f}, "
            f"pursuit_weight={self.pursuit_weight:.2f}, "
            f"pursuit_switch_margin={self.pursuit_switch_margin:.2f}, "
            f"exploration_gain_radius_cells={self.exploration_gain_radius_cells}, "
            f"meeting_min_distance={self.meeting_min_distance:.2f}, "
            f"teammate_stale_ttl_sec={self.teammate_stale_ttl_sec:.2f}, "
            f"mui_resolve_period_sec={self.mui_resolve_period_sec:.2f}, "
            f"mui_mdvrp_time_limit_sec={self.mui_mdvrp_time_limit_sec:.2f}, "
            f"mui_max_exploring_cells={self.mui_max_exploring_cells}, "
            f"mui_cell_merge_resolution_m={self.mui_cell_merge_resolution_m:.2f}, "
            f"mui_unreachable_penalty_m={self.mui_unreachable_penalty_m:.1f}, "
            f"output_mode={self.output_mode} "
            f"goal_topic_suffix={self.goal_topic_suffix} "
            f"tare_goal_topic_suffix={self.tare_goal_topic_suffix} "
            f"relocation_goal_topic_suffix={self.relocation_goal_topic_suffix} "
            f"use_shared_map={self.use_shared_map} shared_map_topic={self.shared_map_topic} "
            f"shared_map_wait_sec={self.shared_map_wait_sec:.1f} "
            f"shared_map_local_patch_radius_m={self.shared_map_local_patch_radius_m:.2f} "
            f"perf_enable={self.perf_enable} "
            f"perf_tick_warn_p95_ms={self.perf_tick_warn_p95_ms:.1f} "
            f"perf_cpu_warn_pct={self.perf_cpu_warn_pct:.1f}"
        )

    def _map_cb(self, msg: OccupancyGrid, ns: str) -> None:
        self.maps[ns] = msg

    def _odom_cb(self, msg: Odometry, ns: str) -> None:
        self.odoms[ns] = msg
        self.odom_rx_time_ns[ns] = self.get_clock().now().nanoseconds
        self.odom_velocity_xy[ns] = (
            float(msg.twist.twist.linear.x),
            float(msg.twist.twist.linear.y),
        )
        self._append_trajectory(ns, msg)

    def _grid_world_status_cb(self, msg: GridWorldStatus, ns: str) -> None:
        self.grid_world_status[ns] = msg
        self.grid_world_status_rx_time_ns[ns] = self.get_clock().now().nanoseconds

    def _shared_map_cb(self, msg: OccupancyGrid) -> None:
        self.shared_map = msg
        if self._shared_map_fallback_active:
            self.get_logger().info(
                f"Shared map received on {self.shared_map_topic}; switching to shared-map coordination."
            )
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

    def _build_fallback_map(self) -> Optional[OccupancyGrid]:
        return build_fallback_map(
            namespaces=self.namespaces,
            maps=self.maps,
            unknown_value=self.unknown_value,
            free_value=self.free_value,
            occ_threshold=self.occ_thresh,
        )

    def _build_shared_with_local_patches(self, shared_msg: OccupancyGrid) -> OccupancyGrid:
        return build_shared_with_local_patches(
            shared_map=shared_msg,
            namespaces=self.namespaces,
            maps=self.maps,
            odoms=self.odoms,
            local_patch_radius_m=self.shared_map_local_patch_radius_m,
            unknown_value=self.unknown_value,
            free_value=self.free_value,
            occ_threshold=self.occ_thresh,
        )

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

    def _goal_key(self, goal: tuple[float, float]) -> tuple[int, int]:
        q = self.blacklist_key_resolution
        return (int(round(goal[0] / q)), int(round(goal[1] / q)))

    def _prune_blacklist(self, ns: str, now_ns: int) -> None:
        entries = self.goal_blacklist_until_ns[ns]
        expired = [k for k, until_ns in entries.items() if until_ns <= now_ns]
        for key in expired:
            entries.pop(key, None)

    def _is_blacklisted(self, ns: str, goal: tuple[float, float], now_ns: int) -> bool:
        self._prune_blacklist(ns, now_ns)
        key = self._goal_key(goal)
        until_ns = self.goal_blacklist_until_ns[ns].get(key, 0)
        return until_ns > now_ns

    def _register_goal_failure(self, ns: str, goal: tuple[float, float], now_ns: int, reason: str) -> None:
        key = self._goal_key(goal)
        counts = self.goal_fail_counts[ns]
        counts[key] = counts.get(key, 0) + 1
        if counts[key] < self.blacklist_fail_count:
            return

        counts[key] = 0
        if self.blacklist_ttl_sec <= 0.0:
            return

        until_ns = now_ns + int(self.blacklist_ttl_sec * 1e9)
        self.goal_blacklist_until_ns[ns][key] = until_ns
        self.get_logger().warn(
            f"{ns}: blacklisting goal ({goal[0]:.2f},{goal[1]:.2f}) for {self.blacklist_ttl_sec:.1f}s "
            f"after repeated {reason} failures."
        )

    def _distance_robot_to_goal(self, ns: str, goal: tuple[float, float]) -> float:
        od = self.odoms[ns]
        rx = float(od.pose.pose.position.x)
        ry = float(od.pose.pose.position.y)
        return math.hypot(goal[0] - rx, goal[1] - ry)

    def _goal_too_close(self, ns: str, goal: tuple[float, float]) -> bool:
        if self.min_assign_distance <= 0.0:
            return False
        return self._distance_robot_to_goal(ns, goal) <= self.min_assign_distance

    def _update_reached_goal_blacklist(self, ns: str, now_ns: int) -> None:
        if self.reached_blacklist_ttl_sec <= 0.0 or self.reached_blacklist_dist <= 0.0:
            return

        goal = self.last_goal.get(ns)
        if goal is None:
            self.reached_goal_last_key[ns] = None
            self.reached_goal_repeat_count[ns] = 0
            return

        key = self._goal_key(goal)
        dist = self._distance_robot_to_goal(ns, goal)
        if dist > self.reached_blacklist_dist:
            self.reached_goal_last_key[ns] = key
            self.reached_goal_repeat_count[ns] = 0
            return

        # Do not repeatedly extend an active blacklist entry for the same key.
        if self.goal_blacklist_until_ns[ns].get(key, 0) > now_ns:
            self.reached_goal_repeat_count[ns] = 0
            return

        if self.reached_goal_last_key[ns] == key:
            self.reached_goal_repeat_count[ns] += 1
        else:
            self.reached_goal_last_key[ns] = key
            self.reached_goal_repeat_count[ns] = 1

        if self.reached_goal_repeat_count[ns] < self.reached_blacklist_repeat_count:
            return

        self.reached_goal_repeat_count[ns] = 0
        until_ns = now_ns + int(self.reached_blacklist_ttl_sec * 1e9)
        self.goal_blacklist_until_ns[ns][key] = until_ns
        self.get_logger().warn(
            f"{ns}: blacklisting repeatedly reached goal ({goal[0]:.2f},{goal[1]:.2f}) "
            f"for {self.reached_blacklist_ttl_sec:.1f}s "
            f"after {self.reached_blacklist_repeat_count} near-goal repeats "
            f"(dist<={self.reached_blacklist_dist:.2f}m)."
        )

    def _goal_reachable(self, map_msg: OccupancyGrid, dist_map: dict[int, int], goal: tuple[float, float]) -> bool:
        g = self._world_to_grid(map_msg, goal[0], goal[1])
        if g is None:
            return False
        idx = self._grid_index(g[0], g[1], int(map_msg.info.width))
        return idx in dist_map

    def _update_progress_samples(self, ns: str, now_ns: int) -> None:
        goal = self.last_goal.get(ns)
        if goal is None:
            return

        samples = self.goal_progress_samples[ns]
        samples.append((now_ns, self._distance_robot_to_goal(ns, goal)))

        cutoff_ns = now_ns - int(self.progress_window_sec * 1e9)
        while len(samples) >= 2 and samples[0][0] < cutoff_ns:
            samples.popleft()

    def _progress_delta(self, ns: str) -> Optional[float]:
        samples = self.goal_progress_samples[ns]
        if len(samples) < 2:
            return None
        span_ns = samples[-1][0] - samples[0][0]
        if span_ns < int(0.5 * self.progress_window_sec * 1e9):
            return None
        return samples[0][1] - samples[-1][1]

    def _set_active_goal(self, ns: str, goal: tuple[float, float], now_ns: int) -> None:
        prev = self.last_goal.get(ns)
        if prev is None or math.hypot(prev[0] - goal[0], prev[1] - goal[1]) > 1e-6:
            self.last_goal_set_time_ns[ns] = now_ns
            self.goal_progress_samples[ns].clear()
        self.last_goal[ns] = goal

    def _set_policy_reason(self, ns: str, reason: str) -> None:
        self.last_policy_reason[ns] = reason

    def _apply_switch_hysteresis(self, ns: str, goal: tuple[float, float], assignment_score: float) -> tuple[float, float]:
        last = self.last_goal.get(ns)
        if last is None:
            self._set_policy_reason(ns, "switch/no_previous_goal")
            return goal

        od = self.odoms[ns]
        rx = float(od.pose.pose.position.x)
        ry = float(od.pose.pose.position.y)
        dist_to_last = math.hypot(last[0] - rx, last[1] - ry)
        move = math.hypot(goal[0] - last[0], goal[1] - last[1])

        # Only apply hold logic while still traveling to the previous goal.
        if dist_to_last > self.switch_min_dist:
            if move < self.switch_min_dist:
                self._set_policy_reason(ns, "hold/hysteresis_small_move")
                return last
            if assignment_score < self.switch_hysteresis:
                self._set_policy_reason(ns, "hold/hysteresis_low_score")
                return last
        self._set_policy_reason(ns, "switch/hysteresis_ok")
        return goal

    def _apply_goal_policy(
        self,
        ns: str,
        candidate_goal: tuple[float, float],
        assignment_score: float,
        map_msg: OccupancyGrid,
        dist_map: dict[int, int],
        now_ns: int,
    ) -> tuple[float, float]:
        if self.algorithm_mode != "committed":
            return self._apply_switch_hysteresis(ns, candidate_goal, assignment_score)

        last = self.last_goal.get(ns)
        if last is None:
            self._set_policy_reason(ns, "switch/no_previous_goal")
            return candidate_goal

        if self._is_blacklisted(ns, candidate_goal, now_ns):
            self._set_policy_reason(ns, "hold/candidate_blacklisted")
            return last

        dist_to_last = self._distance_robot_to_goal(ns, last)
        reached_last = dist_to_last <= self.switch_min_dist
        last_reachable = self._goal_reachable(map_msg, dist_map, last)
        hard_failure = not last_reachable

        last_set_ns = self.last_goal_set_time_ns.get(ns, 0)
        lock_active = (
            self.goal_lock_sec > 0.0
            and last_set_ns > 0
            and (now_ns - last_set_ns) < int(self.goal_lock_sec * 1e9)
        )

        if lock_active and not hard_failure and not reached_last:
            self._set_policy_reason(ns, "hold/goal_lock_active")
            return last

        delta = self._progress_delta(ns)
        stalled = delta is not None and delta < self.progress_min_delta_m

        candidate_move = math.hypot(candidate_goal[0] - last[0], candidate_goal[1] - last[1])
        if candidate_move < self.switch_min_dist:
            self._set_policy_reason(ns, "hold/small_candidate_move")
            return last

        if not reached_last and not hard_failure:
            # Keep commitment while making sufficient progress.
            if not stalled:
                self._set_policy_reason(ns, "hold/progressing")
                return last
            # Only switch on weak assignment scores when already stalled.
            if assignment_score < self.switch_hysteresis:
                self._set_policy_reason(ns, "hold/stalled_but_low_score")
                return last

        if not reached_last and (hard_failure or stalled):
            reason = "unreachable" if hard_failure else "stalled"
            self._register_goal_failure(ns, last, now_ns, reason)
            self._set_policy_reason(ns, f"switch/{reason}")
            return candidate_goal

        self._set_policy_reason(ns, "switch/reached_or_improved")
        return candidate_goal

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
            dist_txt = "-"
            age_txt = "-"
            if goal is not None and ns in self.odoms:
                dist_txt = f"{self._distance_robot_to_goal(ns, goal):.2f}"
            set_ns = self.last_goal_set_time_ns.get(ns, 0)
            if set_ns > 0:
                age_txt = f"{max(0.0, (now_ns - set_ns) / 1e9):.1f}"
            policy = self.last_policy_reason.get(ns, "-")
            parts.append(
                f"{ns}:frontiers={per_ns_frontiers.get(ns, 0)} "
                f"reachable={per_ns_reachable.get(ns, 0)} goal={goal_txt} "
                f"d={dist_txt} age={age_txt}s policy={policy}"
            )
        self.get_logger().info(
            f"ASSIGN step[{self.algorithm_mode}]: targets={targets_total} | " + " | ".join(parts)
        )

    @staticmethod
    def _percentile(sorted_values: list[float], quantile: float) -> float:
        if not sorted_values:
            return 0.0
        if len(sorted_values) == 1:
            return sorted_values[0]
        q = min(1.0, max(0.0, quantile))
        idx = q * float(len(sorted_values) - 1)
        lo = int(math.floor(idx))
        hi = int(math.ceil(idx))
        if lo == hi:
            return sorted_values[lo]
        frac = idx - lo
        return sorted_values[lo] * (1.0 - frac) + sorted_values[hi] * frac

    def _maybe_log_perf_summary(self) -> None:
        if not self.perf_enable:
            return

        now_ns = self.get_clock().now().nanoseconds
        if self._last_perf_summary_ns == 0:
            self._last_perf_summary_ns = now_ns
            self._perf_last_cpu_process_sec = time.process_time()
            self._perf_last_cpu_wall_ns = time.perf_counter_ns()
            return
        if (now_ns - self._last_perf_summary_ns) < int(self._summary_interval_sec * 1e9):
            return
        self._last_perf_summary_ns = now_ns

        sample_count = len(self._perf_tick_durations_ms)
        if sample_count < self.perf_min_samples:
            self.get_logger().info(
                f"PERF coordinator: collecting samples ({sample_count}/{self.perf_min_samples})"
            )
            return

        samples = list(self._perf_tick_durations_ms)
        sorted_samples = sorted(samples)
        p50_ms = self._percentile(sorted_samples, 0.50)
        p95_ms = self._percentile(sorted_samples, 0.95)
        mean_ms = sum(samples) / float(sample_count)
        max_ms = sorted_samples[-1]
        over_budget = sum(1 for val in samples if val > self._tick_period_ms)

        current_cpu_process_sec = time.process_time()
        current_cpu_wall_ns = time.perf_counter_ns()
        delta_cpu_sec = max(0.0, current_cpu_process_sec - self._perf_last_cpu_process_sec)
        delta_wall_sec = max(1e-6, (current_cpu_wall_ns - self._perf_last_cpu_wall_ns) / 1e9)
        cpu_pct = 100.0 * delta_cpu_sec / delta_wall_sec
        self._perf_last_cpu_process_sec = current_cpu_process_sec
        self._perf_last_cpu_wall_ns = current_cpu_wall_ns

        self.get_logger().info(
            "PERF coordinator: "
            f"tick_ms[p50={p50_ms:.1f} p95={p95_ms:.1f} mean={mean_ms:.1f} max={max_ms:.1f} "
            f"budget={self._tick_period_ms:.1f} over_budget={over_budget}/{sample_count}] "
            f"cpu={cpu_pct:.1f}%"
        )

        if self.perf_tick_warn_p95_ms > 0.0 and p95_ms > self.perf_tick_warn_p95_ms:
            self.get_logger().warn(
                "PERF threshold exceeded: "
                f"tick p95 {p95_ms:.1f}ms > {self.perf_tick_warn_p95_ms:.1f}ms"
            )
        if self.perf_cpu_warn_pct > 0.0 and cpu_pct > self.perf_cpu_warn_pct:
            self.get_logger().warn(
                "PERF threshold exceeded: "
                f"CPU {cpu_pct:.1f}% > {self.perf_cpu_warn_pct:.1f}% single-core budget"
            )

    def _record_tick_perf(self, tick_start_ns: int) -> None:
        if not self.perf_enable:
            return
        elapsed_ms = max(0.0, (time.perf_counter_ns() - tick_start_ns) / 1e6)
        self._perf_tick_durations_ms.append(elapsed_ms)
        self._maybe_log_perf_summary()

    def _tick(self) -> None:
        tick_start_ns = time.perf_counter_ns()
        try:
            self._tick_impl()
        finally:
            self._record_tick_perf(tick_start_ns)

    def _publish_goal(self, ns: str, map_msg: OccupancyGrid, goal_w: tuple[float, float]) -> None:
        if not self._goal_is_finite(goal_w):
            self.get_logger().warn(f"{ns}: dropping non-finite goal {goal_w}")
            return
        msg = PointStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = map_msg.header.frame_id or "world"
        msg.point.x = goal_w[0]
        msg.point.y = goal_w[1]
        msg.point.z = 0.0
        if self.output_mode == "exact_split":
            policy_reason = self.last_policy_reason.get(ns, "")
            # Relocation targets are currently tied to pursuit decisions.
            if "pursuit" in policy_reason:
                self.relocation_goal_pubs[ns].publish(msg)
            else:
                self.tare_goal_pubs[ns].publish(msg)
        else:
            self.goal_pubs[ns].publish(msg)
        self._publish_goal_marker(ns=ns, frame_id=msg.header.frame_id, goal_w=goal_w)

    def _publish_goal_marker(self, ns: str, frame_id: str, goal_w: tuple[float, float]) -> None:
        marker = Marker()
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.header.frame_id = self.marker_frame_override or frame_id or "world"
        marker.ns = "mtare_goal"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = float(goal_w[0])
        marker.pose.position.y = float(goal_w[1])
        marker.pose.position.z = 0.0
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.35
        marker.scale.y = 0.35
        marker.scale.z = 0.35
        color = self._ns_color(ns)
        marker.color.a = 0.95
        marker.color.r = color[0]
        marker.color.g = color[1]
        marker.color.b = color[2]
        self.goal_marker_pubs[ns].publish(marker)

    @staticmethod
    def _goal_is_finite(goal_w: tuple[float, float]) -> bool:
        return math.isfinite(float(goal_w[0])) and math.isfinite(float(goal_w[1]))

    def _ns_color(self, ns: str) -> tuple[float, float, float]:
        idx = self.namespaces.index(ns) if ns in self.namespaces else 0
        if idx == 0:
            return (1.0, 0.15, 0.15)
        if idx == 1:
            return (0.15, 1.0, 0.15)
        if idx % 2 == 0:
            return (0.2, 0.6, 1.0)
        return (1.0, 0.8, 0.2)

    def _append_trajectory(self, ns: str, odom_msg: Odometry) -> None:
        x = float(odom_msg.pose.pose.position.x)
        y = float(odom_msg.pose.pose.position.y)
        hist = self.trajectory_history[ns]
        if not hist:
            hist.append((x, y))
            return
        px, py = hist[-1]
        if math.hypot(x - px, y - py) >= self.trajectory_min_point_distance:
            hist.append((x, y))

    def _publish_coordinator_map(self, target_map: OccupancyGrid) -> None:
        self.coordinator_map_pub.publish(target_map)

    def _publish_robot_markers(self, target_map: OccupancyGrid) -> None:
        markers = MarkerArray()
        stamp = self.get_clock().now().to_msg()
        frame_id = self.marker_frame_override or target_map.header.frame_id or "world"

        for idx, ns in enumerate(self.namespaces):
            color = self._ns_color(ns)
            pose_id = 100 + idx
            traj_id = 200 + idx
            goal_id = 300 + idx
            label_id = 400 + idx

            od = self.odoms.get(ns)
            if od is not None:
                pose_marker = Marker()
                pose_marker.header.stamp = stamp
                pose_marker.header.frame_id = frame_id
                pose_marker.ns = "mtare_robot_pose"
                pose_marker.id = pose_id
                pose_marker.type = Marker.SPHERE
                pose_marker.action = Marker.ADD
                pose_marker.pose.position.x = float(od.pose.pose.position.x)
                pose_marker.pose.position.y = float(od.pose.pose.position.y)
                pose_marker.pose.position.z = float(od.pose.pose.position.z)
                pose_marker.pose.orientation.w = 1.0
                pose_marker.scale.x = self.robot_marker_scale
                pose_marker.scale.y = self.robot_marker_scale
                pose_marker.scale.z = self.robot_marker_scale
                pose_marker.color.a = 0.95
                pose_marker.color.r = color[0]
                pose_marker.color.g = color[1]
                pose_marker.color.b = color[2]
                markers.markers.append(pose_marker)

                label_marker = Marker()
                label_marker.header.stamp = stamp
                label_marker.header.frame_id = frame_id
                label_marker.ns = "mtare_robot_label"
                label_marker.id = label_id
                label_marker.type = Marker.TEXT_VIEW_FACING
                label_marker.action = Marker.ADD
                label_marker.pose.position.x = float(od.pose.pose.position.x)
                label_marker.pose.position.y = float(od.pose.pose.position.y)
                label_marker.pose.position.z = float(od.pose.pose.position.z) + 0.5
                label_marker.pose.orientation.w = 1.0
                label_marker.scale.z = 0.32
                label_marker.color.a = 1.0
                label_marker.color.r = color[0]
                label_marker.color.g = color[1]
                label_marker.color.b = color[2]
                label_marker.text = ns
                markers.markers.append(label_marker)

            traj_marker = Marker()
            traj_marker.header.stamp = stamp
            traj_marker.header.frame_id = frame_id
            traj_marker.ns = "mtare_robot_traj"
            traj_marker.id = traj_id
            traj_marker.type = Marker.LINE_STRIP
            traj_marker.action = Marker.ADD
            traj_marker.pose.orientation.w = 1.0
            traj_marker.scale.x = 0.08
            traj_marker.color.a = 0.95
            traj_marker.color.r = color[0]
            traj_marker.color.g = color[1]
            traj_marker.color.b = color[2]
            for x, y in self.trajectory_history[ns]:
                pt = Point()
                pt.x = float(x)
                pt.y = float(y)
                pt.z = 0.05
                traj_marker.points.append(pt)
            markers.markers.append(traj_marker)

            goal = self.last_goal.get(ns)
            if goal is not None:
                goal_marker = Marker()
                goal_marker.header.stamp = stamp
                goal_marker.header.frame_id = frame_id
                goal_marker.ns = "mtare_goal_points"
                goal_marker.id = goal_id
                goal_marker.type = Marker.SPHERE
                goal_marker.action = Marker.ADD
                goal_marker.pose.position.x = float(goal[0])
                goal_marker.pose.position.y = float(goal[1])
                goal_marker.pose.position.z = 0.08
                goal_marker.pose.orientation.w = 1.0
                goal_marker.scale.x = 0.24
                goal_marker.scale.y = 0.24
                goal_marker.scale.z = 0.24
                goal_marker.color.a = 0.95
                goal_marker.color.r = color[0]
                goal_marker.color.g = color[1]
                goal_marker.color.b = color[2]
                markers.markers.append(goal_marker)

        self.robot_markers_pub.publish(markers)

    def _robot_xy(self, ns: str) -> tuple[float, float]:
        od = self.odoms[ns]
        return (float(od.pose.pose.position.x), float(od.pose.pose.position.y))

    def _predict_teammate_xy(self, ns: str, now_ns: int) -> tuple[float, float]:
        rx, ry = self._robot_xy(ns)
        vx, vy = self.odom_velocity_xy.get(ns, (0.0, 0.0))
        dt = self.prediction_horizon_sec
        if ns in self.odom_rx_time_ns:
            age_sec = max(0.0, (now_ns - self.odom_rx_time_ns[ns]) / 1e9)
            dt += age_sec
        return (rx + vx * dt, ry + vy * dt)

    def _frontier_information_gain(self, msg: OccupancyGrid, goal: tuple[float, float]) -> float:
        g = self._world_to_grid(msg, goal[0], goal[1])
        if g is None:
            return 0.0
        gx, gy = g
        w = int(msg.info.width)
        h = int(msg.info.height)
        data = msg.data
        r = self.exploration_gain_radius_cells
        gain = 0.0
        for yy in range(max(0, gy - r), min(h, gy + r + 1)):
            row = yy * w
            for xx in range(max(0, gx - r), min(w, gx + r + 1)):
                idx = row + xx
                if data[idx] == self.unknown_value:
                    gain += 1.0
        return gain

    def _grid_path_cost_m(
        self,
        msg: OccupancyGrid,
        dist_map: dict[int, int],
        goal: tuple[float, float],
    ) -> Optional[float]:
        g = self._world_to_grid(msg, goal[0], goal[1])
        if g is None:
            return None
        idx = self._grid_index(g[0], g[1], int(msg.info.width))
        if idx not in dist_map:
            return None
        return float(dist_map[idx]) * msg.info.resolution

    def _cfpa2_switch_penalty(self, ns: str, goal: tuple[float, float]) -> float:
        last = self.last_goal.get(ns)
        if last is None:
            return 0.0
        return 0.0 if self._goal_key(last) == self._goal_key(goal) else 1.0

    def _cfpa2_single_utility(
        self,
        *,
        ns: str,
        goal: tuple[float, float],
        map_msg: OccupancyGrid,
        dist_map: dict[int, int],
    ) -> float:
        dist_m = self._grid_path_cost_m(map_msg, dist_map, goal)
        if dist_m is None or dist_m <= 0.0:
            return -1e18
        info_gain = self._frontier_information_gain(map_msg, goal)
        switch_penalty = self._cfpa2_switch_penalty(ns, goal)
        return (
            (self.cfpa2_w_ig * info_gain)
            - (self.cfpa2_w_c * dist_m)
            - (self.cfpa2_w_sw * switch_penalty)
        )

    def _cfpa2_overlap_penalty(self, goal_i: tuple[float, float], goal_j: tuple[float, float]) -> float:
        sigma = self.cfpa2_sigma_overlap_m if self.cfpa2_sigma_overlap_m > 0.0 else (2.0 * self.sensor_range)
        sigma = max(1e-3, sigma)
        if _CFPA2_OVERLAP_PENALTY_FN is not None:
            try:
                return float(_CFPA2_OVERLAP_PENALTY_FN(goal_i, goal_j, sigma))
            except Exception:
                pass

        dx = goal_i[0] - goal_j[0]
        dy = goal_i[1] - goal_j[1]
        d2 = (dx * dx) + (dy * dy)
        return math.exp(-d2 / (2.0 * sigma * sigma))

    def _exploration_utility(
        self,
        *,
        ns: str,
        goal: tuple[float, float],
        map_msg: OccupancyGrid,
        dist_maps: dict[str, dict[int, int]],
        assigned_goals: dict[str, tuple[float, float]],
    ) -> tuple[float, Optional[float]]:
        self_dist = self._grid_path_cost_m(map_msg, dist_maps.get(ns, {}), goal)
        if self_dist is None or self_dist <= 0.0:
            return (-1e18, None)

        info_gain = self._frontier_information_gain(map_msg, goal)
        base_score = info_gain / max(self_dist, 0.1)

        overlap_penalty = 0.0
        for other in self.namespaces:
            if other == ns:
                continue
            other_dist = self._grid_path_cost_m(map_msg, dist_maps.get(other, {}), goal)
            if other_dist is None or other_dist <= 0.0:
                continue
            if other_dist < self_dist:
                overlap_penalty += self.overlap_weight / max(other_dist, 0.25)

            assigned = assigned_goals.get(other)
            if assigned is not None:
                d_assigned = math.hypot(goal[0] - assigned[0], goal[1] - assigned[1])
                if d_assigned < self.sensor_range:
                    overlap_penalty += self.overlap_weight / max(d_assigned, 0.25)

        return (base_score - overlap_penalty, self_dist)

    def _best_pursuit_target(
        self,
        *,
        ns: str,
        now_ns: int,
    ) -> tuple[Optional[tuple[float, float]], float]:
        if self.communication_timeout_sec <= 0.0:
            return (None, -1e18)

        local_xy = self._robot_xy(ns)
        best_target: Optional[tuple[float, float]] = None
        best_utility = -1e18

        for other in self.namespaces:
            if other == ns:
                continue
            last_rx_ns = self.odom_rx_time_ns.get(other, 0)
            if last_rx_ns <= 0:
                continue

            stale_sec = max(0.0, (now_ns - last_rx_ns) / 1e9)
            if stale_sec < self.communication_timeout_sec:
                continue
            if self.teammate_stale_ttl_sec > 0.0 and stale_sec > self.teammate_stale_ttl_sec:
                continue

            predicted = self._predict_teammate_xy(other, now_ns)
            meeting_dist = math.hypot(predicted[0] - local_xy[0], predicted[1] - local_xy[1])
            if meeting_dist < self.meeting_min_distance:
                continue

            expected_overlap_reduction = max(1.0, stale_sec)
            utility = self.pursuit_weight * expected_overlap_reduction / max(meeting_dist, 0.25)
            if utility > best_utility:
                best_utility = utility
                best_target = predicted

        return (best_target, best_utility)

    def _mui_cell_key(self, point: tuple[float, float, float]) -> tuple[int, int, int]:
        q = self.mui_cell_merge_resolution_m
        qz = max(0.05, q)
        return (
            int(round(point[0] / q)),
            int(round(point[1] / q)),
            int(round(point[2] / qz)),
        )

    def _collect_mui_exploring_cells(
        self,
    ) -> tuple[list[tuple[float, float, float]], list[tuple[int, int, int]], dict[str, int]]:
        merged: dict[tuple[int, int, int], tuple[float, float, float]] = {}
        per_ns_frontiers: dict[str, int] = {}

        for ns in self.namespaces:
            status = self.grid_world_status.get(ns)
            if status is None:
                per_ns_frontiers[ns] = 0
                continue
            count = 0
            for pt in status.exploring_cell_positions:
                x = float(pt.x)
                y = float(pt.y)
                z = float(pt.z)
                if not (math.isfinite(x) and math.isfinite(y) and math.isfinite(z)):
                    continue
                key = self._mui_cell_key((x, y, z))
                merged.setdefault(key, (x, y, z))
                count += 1
            per_ns_frontiers[ns] = count

        keys = sorted(merged.keys())
        if len(keys) > self.mui_max_exploring_cells:
            robot_xy = [self._robot_xy(ns) for ns in self.namespaces]

            def _score(k: tuple[int, int, int]) -> float:
                x, y, _ = merged[k]
                return min(math.hypot(x - rx, y - ry) for rx, ry in robot_xy)

            keys = sorted(keys, key=_score)[: self.mui_max_exploring_cells]

        cells = [merged[k] for k in keys]
        return (cells, keys, per_ns_frontiers)

    def _build_mui_distance_matrix(
        self,
        map_msg: OccupancyGrid,
        exploring_cells: list[tuple[float, float, float]],
        robot_positions: list[tuple[float, float, float]],
    ) -> list[list[int]]:
        locations = list(exploring_cells) + list(robot_positions)
        if not locations:
            return []

        map_res = max(1e-3, float(map_msg.info.resolution))
        unreachable_cm = int(round(self.mui_unreachable_penalty_m * 100.0))
        xy_locations = [(loc[0], loc[1]) for loc in locations]
        dist_maps = [self._distance_transform(map_msg, (x, y)) for x, y in xy_locations]
        map_w = int(map_msg.info.width)

        matrix: list[list[int]] = []
        for i, (fx, fy) in enumerate(xy_locations):
            dist_map = dist_maps[i]
            row: list[int] = []
            for j, (tx, ty) in enumerate(xy_locations):
                if i == j:
                    row.append(0)
                    continue
                target_grid = self._world_to_grid(map_msg, tx, ty)
                if target_grid is not None:
                    tidx = self._grid_index(target_grid[0], target_grid[1], map_w)
                    if tidx in dist_map:
                        row.append(max(1, int(round(float(dist_map[tidx]) * map_res * 100.0))))
                        continue
                euclid_cm = int(round(math.hypot(tx - fx, ty - fy) * 100.0))
                row.append(max(1, euclid_cm + unreachable_cm))
            matrix.append(row)
        return matrix

    def _should_resolve_mui(self, now_ns: int, cell_keys: list[tuple[int, int, int]]) -> bool:
        if self._mui_last_solve_ns <= 0:
            return True
        elapsed_ns = now_ns - self._mui_last_solve_ns
        # Only re-solve on timer expiry or when a robot has reached its goal
        timer_expired = elapsed_ns >= int(self.mui_resolve_period_sec * 1e9)
        robot_reached_goal = False
        for ns in self.namespaces:
            goal = self.last_goal.get(ns)
            if goal is None:
                robot_reached_goal = True
                break
            if self._distance_robot_to_goal(ns, goal) <= self.switch_min_dist:
                robot_reached_goal = True
                break
        return timer_expired or robot_reached_goal

    def _tick_impl_mui_tare(
        self,
        *,
        now_ns: int,
        planning_map: OccupancyGrid,
    ) -> bool:
        missing_status = [ns for ns in self.namespaces if ns not in self.grid_world_status]
        if missing_status:
            if now_ns - self._last_prereq_warn_ns > int(2e9):
                self.get_logger().warn(
                    f"Waiting for grid status topics from: {missing_status}; "
                    "no MUI-TARE goals will be published yet."
                )
                self._last_prereq_warn_ns = now_ns
            return True

        exploring_cells, cell_keys, per_ns_frontiers = self._collect_mui_exploring_cells()

        if not exploring_cells:
            return True

        should_resolve = self._should_resolve_mui(now_ns, cell_keys)
        if should_resolve:
            robot_positions = []
            for ns in self.namespaces:
                rx, ry = self._robot_xy(ns)
                robot_positions.append((rx, ry, 0.0))
            distance_matrix = self._build_mui_distance_matrix(planning_map, exploring_cells, robot_positions)
            routes = solve_mdvrp(
                exploring_cell_positions=exploring_cells,
                robot_positions=robot_positions,
                distance_matrix=distance_matrix,
                time_limit_sec=self.mui_mdvrp_time_limit_sec,
            )
            if not routes:
                if now_ns - self._last_prereq_warn_ns > int(2e9):
                    self.get_logger().warn(
                        "MUI-TARE MDVRP solve failed; keeping previous MUI assignments."
                    )
                    self._last_prereq_warn_ns = now_ns
            else:
                self._mui_last_solve_ns = now_ns
                self._mui_last_cell_keys = set(cell_keys)
                all_cell_indices = set(range(len(exploring_cells)))
                for idx, ns in enumerate(self.namespaces):
                    own_route = routes.get(idx, [])
                    self._mui_routes[ns] = own_route
                    self._mui_cover_by_others[ns] = set(int(i) for i in (all_cell_indices - set(own_route)))

        robot_xy = {ns: self._robot_xy(ns) for ns in self.namespaces}
        indexed_routes = {i: self._mui_routes.get(ns, []) for i, ns in enumerate(self.namespaces)}
        candidate_goals = select_first_route_goals(
            namespaces=self.namespaces,
            routes=indexed_routes,
            exploring_cells=exploring_cells,
            robot_xy=robot_xy,
            min_assign_distance=self.min_assign_distance,
        )

        per_ns_assigned: dict[str, tuple[float, float]] = {}
        for ns in self.namespaces:
            candidate = candidate_goals.get(ns)
            if candidate is None:
                held = self.last_goal.get(ns)
                if held is None:
                    self._set_policy_reason(ns, "hold/mui_no_candidate")
                    continue
                self._set_policy_reason(ns, "hold/mui_keep_previous")
                goal = held
            else:
                self._set_policy_reason(ns, "switch/mui_tare_mdvrp")
                goal = self._apply_switch_hysteresis(ns, candidate, 1.0)

            self._set_active_goal(ns, goal, now_ns)
            publish_map = self.maps.get(ns, planning_map)
            self._publish_goal(ns, publish_map, goal)
            per_ns_assigned[ns] = goal

        per_ns_reachable = {ns: len(self._mui_routes.get(ns, [])) for ns in self.namespaces}
        self._maybe_log_summary(
            targets_total=len(exploring_cells),
            per_ns_frontiers=per_ns_frontiers,
            per_ns_reachable=per_ns_reachable,
            per_ns_assigned=per_ns_assigned,
        )
        return True

    def _tick_impl(self) -> None:
        now_ns = self.get_clock().now().nanoseconds
        using_shared_map = self.use_shared_map and self.shared_map is not None
        target_map: OccupancyGrid
        if self.use_shared_map:
            if using_shared_map:
                target_map = self.shared_map  # type: ignore[assignment]
                self._warned_missing_shared_map = False
                self._shared_map_fallback_active = False
            else:
                waited_sec = (now_ns - self._start_ns) / 1e9
                if waited_sec >= self.shared_map_wait_sec:
                    # Fail-open: continue coordinated assignment on local map inputs
                    # until shared map becomes available.
                    fallback_map = self._build_fallback_map()
                    if fallback_map is None:
                        if now_ns - self._last_prereq_warn_ns > int(2e9):
                            self.get_logger().warn(
                                "Shared-map fallback active but local maps are still unavailable; "
                                "waiting for /<ns>/map inputs."
                            )
                            self._last_prereq_warn_ns = now_ns
                        return
                    target_map = fallback_map
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
            if any(ns not in self.maps for ns in self.namespaces):
                if now_ns - self._last_prereq_warn_ns > int(2e9):
                    missing_maps = [ns for ns in self.namespaces if ns not in self.maps]
                    self.get_logger().warn(
                        f"Waiting for map topics from: {missing_maps}; no M-TARE goals will be published yet."
                    )
                    self._last_prereq_warn_ns = now_ns
                return
            fallback_map = self._build_fallback_map()
            if fallback_map is None:
                return
            target_map = fallback_map

        planning_map = target_map
        if using_shared_map:
            planning_map = self._build_shared_with_local_patches(target_map)

        self._publish_coordinator_map(planning_map)
        self._publish_robot_markers(planning_map)

        missing_odoms = [ns for ns in self.namespaces if ns not in self.odoms]
        if missing_odoms:
            if now_ns - self._last_prereq_warn_ns > int(2e9):
                self.get_logger().warn(
                    f"Waiting for odom/nav from: {missing_odoms}; no M-TARE goals will be published yet."
                )
                self._last_prereq_warn_ns = now_ns
            return

        for ns in self.namespaces:
            self._prune_blacklist(ns, now_ns)
            self._update_reached_goal_blacklist(ns, now_ns)

        per_ns_targets: dict[str, list[tuple[float, float]]] = {}
        for ns in self.namespaces:
            map_msg = self.maps.get(ns)
            per_ns_targets[ns] = self._extract_frontiers(map_msg) if map_msg is not None else []

        if using_shared_map:
            targets = self._extract_frontiers(planning_map)
        else:
            merge_res = max(0.1, float(planning_map.info.resolution) * 2.0)
            targets = self._merge_targets([per_ns_targets[ns] for ns in self.namespaces], merge_res)

        if self.algorithm_mode == "mui_tare":
            self._tick_impl_mui_tare(
                now_ns=now_ns,
                planning_map=planning_map,
            )
            return

        if not targets:
            return

        dist_maps = {}
        for ns in self.namespaces:
            od = self.odoms[ns]
            # When shared map is unavailable we fail-open to per-robot maps.
            cost_map = planning_map if using_shared_map else self.maps.get(ns)
            if cost_map is None:
                dist_maps[ns] = {}
                continue
            dist_maps[ns] = self._distance_transform(cost_map, (od.pose.pose.position.x, od.pose.pose.position.y))
            if self.algorithm_mode == "committed":
                self._update_progress_samples(ns, now_ns)

        if self.algorithm_mode == "cfpa2" and len(self.namespaces) == 2:
            ns_a, ns_b = self.namespaces[0], self.namespaces[1]
            map_a = planning_map if using_shared_map else self.maps.get(ns_a)
            map_b = planning_map if using_shared_map else self.maps.get(ns_b)

            utilities_a: dict[tuple[float, float], float] = {}
            utilities_b: dict[tuple[float, float], float] = {}

            if map_a is not None:
                for goal in targets:
                    if self._goal_too_close(ns_a, goal):
                        continue
                    if self._is_blacklisted(ns_a, goal, now_ns):
                        continue
                    score = self._cfpa2_single_utility(
                        ns=ns_a,
                        goal=goal,
                        map_msg=map_a,
                        dist_map=dist_maps.get(ns_a, {}),
                    )
                    if score > -1e17:
                        utilities_a[goal] = score

            if map_b is not None:
                for goal in targets:
                    if self._goal_too_close(ns_b, goal):
                        continue
                    if self._is_blacklisted(ns_b, goal, now_ns):
                        continue
                    score = self._cfpa2_single_utility(
                        ns=ns_b,
                        goal=goal,
                        map_msg=map_b,
                        dist_map=dist_maps.get(ns_b, {}),
                    )
                    if score > -1e17:
                        utilities_b[goal] = score

            candidate_goals: dict[str, tuple[float, float]] = {}
            assignment_scores: dict[str, float] = {}
            best_joint = -1e18
            best_pair: Optional[tuple[tuple[float, float], tuple[float, float], float, float]] = None

            for goal_a, score_a in utilities_a.items():
                for goal_b, score_b in utilities_b.items():
                    if goal_a == goal_b:
                        continue
                    overlap = self._cfpa2_overlap_penalty(goal_a, goal_b)
                    joint = score_a + score_b - (self.cfpa2_lambda_overlap * overlap)
                    if joint > best_joint:
                        best_joint = joint
                        best_pair = (goal_a, goal_b, score_a, score_b)

            if best_pair is not None:
                goal_a, goal_b, score_a, score_b = best_pair
                candidate_goals[ns_a] = goal_a
                candidate_goals[ns_b] = goal_b
                assignment_scores[ns_a] = score_a
                assignment_scores[ns_b] = score_b
                self._set_policy_reason(ns_a, "switch/cfpa2_joint")
                self._set_policy_reason(ns_b, "switch/cfpa2_joint")
            else:
                best_single_ns: Optional[str] = None
                best_single_goal: Optional[tuple[float, float]] = None
                best_single_score = -1e18
                if utilities_a:
                    goal_a, score_a = max(utilities_a.items(), key=lambda kv: kv[1])
                    if score_a > best_single_score:
                        best_single_ns = ns_a
                        best_single_goal = goal_a
                        best_single_score = score_a
                if utilities_b:
                    goal_b, score_b = max(utilities_b.items(), key=lambda kv: kv[1])
                    if score_b > best_single_score:
                        best_single_ns = ns_b
                        best_single_goal = goal_b
                        best_single_score = score_b
                if best_single_ns is not None and best_single_goal is not None:
                    candidate_goals[best_single_ns] = best_single_goal
                    assignment_scores[best_single_ns] = best_single_score
                    self._set_policy_reason(best_single_ns, "switch/cfpa2_fallback_single")

            per_ns_assigned: dict[str, tuple[float, float]] = {}
            for ns in self.namespaces:
                candidate = candidate_goals.get(ns)
                if candidate is None:
                    held = self.last_goal.get(ns)
                    if held is None:
                        self._set_policy_reason(ns, "hold/cfpa2_no_candidate")
                        continue
                    self._set_policy_reason(ns, "hold/cfpa2_keep_previous")
                    goal = held
                else:
                    goal = self._apply_switch_hysteresis(
                        ns,
                        candidate,
                        assignment_scores.get(ns, 0.0),
                    )

                self._set_active_goal(ns, goal, now_ns)
                publish_map = self.maps.get(ns, planning_map)
                self._publish_goal(ns, publish_map, goal)
                per_ns_assigned[ns] = goal

            per_ns_reachable: dict[str, int] = {}
            for ns in self.namespaces:
                msg = planning_map if using_shared_map else self.maps.get(ns)
                if msg is None:
                    per_ns_reachable[ns] = 0
                    continue
                dist_map = dist_maps.get(ns, {})
                reachable = 0
                for goal in targets:
                    if self._goal_too_close(ns, goal):
                        continue
                    if self._is_blacklisted(ns, goal, now_ns):
                        continue
                    if self._goal_reachable(msg, dist_map, goal):
                        reachable += 1
                per_ns_reachable[ns] = reachable

            per_ns_frontiers = {ns: len(per_ns_targets.get(ns, [])) for ns in self.namespaces}
            self._maybe_log_summary(
                targets_total=len(targets),
                per_ns_frontiers=per_ns_frontiers,
                per_ns_reachable=per_ns_reachable,
                per_ns_assigned=per_ns_assigned,
            )
            return

        if self.algorithm_mode == "cfpa2" and not self._warned_cfpa2_two_robot_only:
            self.get_logger().warn(
                "algorithm_mode=cfpa2 requires exactly two namespaces; falling back to collaborative mode."
            )
            self._warned_cfpa2_two_robot_only = True

        if self.algorithm_mode == "mtare":
            per_ns_assigned: dict[str, tuple[float, float]] = {}
            assigned_goals: dict[str, tuple[float, float]] = {}

            for ns in self.namespaces:
                map_msg = planning_map if using_shared_map else self.maps.get(ns)
                if map_msg is None:
                    self._set_policy_reason(ns, "hold/no_local_map")
                    continue
                best_explore_goal: Optional[tuple[float, float]] = None
                best_explore_utility = -1e18
                for goal in targets:
                    if self._goal_too_close(ns, goal):
                        continue
                    if self._is_blacklisted(ns, goal, now_ns):
                        continue
                    utility, _ = self._exploration_utility(
                        ns=ns,
                        goal=goal,
                        map_msg=map_msg,
                        dist_maps=dist_maps,
                        assigned_goals=assigned_goals,
                    )
                    if utility > best_explore_utility:
                        best_explore_utility = utility
                        best_explore_goal = goal

                pursuit_goal, pursuit_utility = self._best_pursuit_target(ns=ns, now_ns=now_ns)

                selected_goal: Optional[tuple[float, float]] = None
                if (
                    pursuit_goal is not None
                    and pursuit_utility > (best_explore_utility + self.pursuit_switch_margin)
                ):
                    selected_goal = pursuit_goal
                    self._set_policy_reason(ns, "switch/mtare_pursuit")
                elif best_explore_goal is not None:
                    selected_goal = best_explore_goal
                    self._set_policy_reason(ns, "switch/mtare_explore")

                if selected_goal is None:
                    held = self.last_goal.get(ns)
                    if held is None:
                        self._set_policy_reason(ns, "hold/mtare_no_candidate")
                        continue
                    selected_goal = held
                else:
                    selected_goal = self._apply_switch_hysteresis(
                        ns,
                        selected_goal,
                        max(best_explore_utility, pursuit_utility),
                    )

                self._set_active_goal(ns, selected_goal, now_ns)
                publish_map = self.maps.get(ns, planning_map)
                self._publish_goal(ns, publish_map, selected_goal)
                per_ns_assigned[ns] = selected_goal
                assigned_goals[ns] = selected_goal

            per_ns_reachable: dict[str, int] = {}
            for ns in self.namespaces:
                msg = planning_map if using_shared_map else self.maps.get(ns)
                if msg is None:
                    per_ns_reachable[ns] = 0
                    continue
                dist_map = dist_maps.get(ns, {})
                reachable = 0
                for goal in targets:
                    if self._goal_too_close(ns, goal):
                        continue
                    if self._is_blacklisted(ns, goal, now_ns):
                        continue
                    if self._goal_reachable(msg, dist_map, goal):
                        reachable += 1
                per_ns_reachable[ns] = reachable

            per_ns_frontiers = {ns: len(per_ns_targets.get(ns, [])) for ns in self.namespaces}
            self._maybe_log_summary(
                targets_total=len(targets),
                per_ns_frontiers=per_ns_frontiers,
                per_ns_reachable=per_ns_reachable,
                per_ns_assigned=per_ns_assigned,
            )
            return

        utilities = [1.0 for _ in targets]
        unassigned = set(self.namespaces)
        assigned: dict[str, int] = {}
        assignment_scores: dict[str, float] = {}
        sigma = max(self.sensor_range * 0.5, 1e-3)

        while unassigned:
            best_pair = None
            best_score = -1e18

            for ns in list(unassigned):
                msg = planning_map if using_shared_map else self.maps.get(ns)
                if msg is None:
                    continue
                dist_map = dist_maps[ns]
                if not dist_map:
                    continue
                for ti, (wx, wy) in enumerate(targets):
                    if self._goal_too_close(ns, (wx, wy)):
                        continue
                    if self._is_blacklisted(ns, (wx, wy), now_ns):
                        continue
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
        candidate_goals: dict[str, tuple[float, float]] = {}
        for ns in list(unassigned):
            local_targets = [
                goal
                for goal in per_ns_targets.get(ns, [])
                if not self._goal_too_close(ns, goal)
                if not self._is_blacklisted(ns, goal, now_ns)
            ]
            if not local_targets:
                continue
            od = self.odoms[ns]
            rx = float(od.pose.pose.position.x)
            ry = float(od.pose.pose.position.y)
            nearest_idx = min(
                range(len(local_targets)),
                key=lambda i: math.hypot(local_targets[i][0] - rx, local_targets[i][1] - ry),
            )
            candidate_goals[ns] = local_targets[nearest_idx]
            assignment_scores.setdefault(ns, 1.0)
            unassigned.remove(ns)

        for ns, ti in assigned.items():
            candidate_goals[ns] = targets[ti]

        per_ns_assigned: dict[str, tuple[float, float]] = {}
        for ns in self.namespaces:
            candidate = candidate_goals.get(ns)
            if candidate is None:
                # No new assignment candidate, hold current goal if any.
                held = self.last_goal.get(ns)
                if held is None:
                    self._set_policy_reason(ns, "hold/no_candidate_no_previous_goal")
                    continue
                self._set_policy_reason(ns, "hold/no_candidate")
                goal = held
            else:
                msg_for_ns = planning_map if using_shared_map else self.maps.get(ns)
                if msg_for_ns is None:
                    held = self.last_goal.get(ns)
                    if held is None:
                        self._set_policy_reason(ns, "hold/no_local_map")
                        continue
                    self._set_policy_reason(ns, "hold/no_local_map")
                    goal = held
                    self._set_active_goal(ns, goal, now_ns)
                    publish_map = self.maps.get(ns, planning_map)
                    self._publish_goal(ns, publish_map, goal)
                    per_ns_assigned[ns] = goal
                    continue
                goal = self._apply_goal_policy(
                    ns=ns,
                    candidate_goal=candidate,
                    assignment_score=assignment_scores.get(ns, 0.0),
                    map_msg=msg_for_ns,
                    dist_map=dist_maps.get(ns, {}),
                    now_ns=now_ns,
                )

            self._set_active_goal(ns, goal, now_ns)
            publish_map = self.maps.get(ns, planning_map)
            self._publish_goal(ns, publish_map, goal)
            per_ns_assigned[ns] = goal

        per_ns_reachable: dict[str, int] = {}
        for ns in self.namespaces:
            msg = planning_map if using_shared_map else self.maps.get(ns)
            if msg is None:
                per_ns_reachable[ns] = 0
                continue
            dist_map = dist_maps.get(ns, {})
            if not dist_map:
                per_ns_reachable[ns] = 0
                continue
            reachable = 0
            for wx, wy in targets:
                if self._goal_too_close(ns, (wx, wy)):
                    continue
                if self._is_blacklisted(ns, (wx, wy), now_ns):
                    continue
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
    node = MTareCoordinator()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
