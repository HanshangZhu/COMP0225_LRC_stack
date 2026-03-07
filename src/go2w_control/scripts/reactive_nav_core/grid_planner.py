"""A* grid planner on the global occupancy grid — C++ accelerated.

Plans in world coordinates using the OccupancyGrid published by
simple_scan_mapper_cpp.  Falls back to the existing scan-based local
planner when the map is unavailable.

Performance notes:
- A* inner loop runs in C++ via ctypes (~5ms for a 16m path on 400×400 grid).
- Inflation uses scipy.ndimage.binary_dilation (~9ms).
- Planning runs in a background thread so it never blocks the control loop.
- Falls back to pure-Python A* if the .so is missing.
"""

from __future__ import annotations

import ctypes
import heapq
import math
import os
import threading
from dataclasses import dataclass, field
from typing import Optional

import numpy as np

# Try scipy for fast dilation; fall back to manual numpy if unavailable
try:
    from scipy.ndimage import binary_dilation

    _HAS_SCIPY = True
except ImportError:
    _HAS_SCIPY = False

# Load C++ A* shared library
_astar_lib = None
try:
    _so_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "astar_grid.so")
    _astar_lib = ctypes.CDLL(_so_path)
    _astar_lib.astar_grid.restype = ctypes.c_int
    _astar_lib.astar_grid.argtypes = [
        ctypes.POINTER(ctypes.c_uint8),  # blocked
        ctypes.c_int, ctypes.c_int,       # W, H
        ctypes.c_int, ctypes.c_int,       # sx, sy
        ctypes.c_int, ctypes.c_int,       # gx, gy
        ctypes.POINTER(ctypes.c_int),     # path_x_out
        ctypes.POINTER(ctypes.c_int),     # path_y_out
        ctypes.c_int,                     # max_path_len
        ctypes.c_int,                     # max_cells
        ctypes.POINTER(ctypes.c_int),     # cells_explored_out
    ]
except Exception:
    _astar_lib = None


@dataclass
class OccGridInfo:
    """Lightweight mirror of nav_msgs/OccupancyGrid metadata."""

    resolution: float
    width: int
    height: int
    origin_x: float
    origin_y: float
    data: np.ndarray  # 2D bool array: True = blocked


@dataclass
class GridPlanResult:
    waypoints_world: list[tuple[float, float]] = field(default_factory=list)
    success: bool = False
    cells_explored: int = 0
    time_ms: float = 0.0


def _inflate_grid(occupied: np.ndarray, inflate_cells: int) -> np.ndarray:
    """Return a boolean grid with obstacles inflated by `inflate_cells`."""
    if inflate_cells <= 0:
        return occupied.copy()

    if _HAS_SCIPY:
        # Build circular structuring element
        d = 2 * inflate_cells + 1
        y, x = np.ogrid[-inflate_cells : inflate_cells + 1, -inflate_cells : inflate_cells + 1]
        kernel = (x * x + y * y) <= inflate_cells * inflate_cells
        return binary_dilation(occupied, structure=kernel).astype(bool)

    # Numpy fallback: shift-and-OR for each offset in the inflation disc
    result = occupied.copy()
    for dy in range(-inflate_cells, inflate_cells + 1):
        for dx in range(-inflate_cells, inflate_cells + 1):
            if dx * dx + dy * dy > inflate_cells * inflate_cells:
                continue
            shifted = np.roll(np.roll(occupied, dy, axis=0), dx, axis=1)
            if dy > 0:
                shifted[:dy, :] = False
            elif dy < 0:
                shifted[dy:, :] = False
            if dx > 0:
                shifted[:, :dx] = False
            elif dx < 0:
                shifted[:, dx:] = False
            result |= shifted
    return result


def occupancy_to_blocked(
    data: list[int] | np.ndarray,
    width: int,
    height: int,
    inflate_cells: int = 0,
) -> np.ndarray:
    """Convert ROS OccupancyGrid.data to inflated 2D boolean array."""
    arr = np.array(data, dtype=np.int8).reshape(height, width)
    occupied = arr == 100  # OccupancyGrid: 100 = occupied
    if inflate_cells > 0:
        occupied = _inflate_grid(occupied, inflate_cells)
    return occupied


def _astar_cpp(blocked: np.ndarray, sx: int, sy: int, gx: int, gy: int,
               max_cells: int) -> tuple[list[tuple[int, int]], int]:
    """Run A* via C++ shared library. Returns (path_cells, cells_explored)."""
    H, W = blocked.shape
    # Ensure contiguous uint8 row-major
    grid_flat = np.ascontiguousarray(blocked.astype(np.uint8)).ravel()

    max_path = W * H  # worst case
    if max_path > 200000:
        max_path = 200000
    path_x = (ctypes.c_int * max_path)()
    path_y = (ctypes.c_int * max_path)()
    explored = ctypes.c_int(0)

    path_len = _astar_lib.astar_grid(
        grid_flat.ctypes.data_as(ctypes.POINTER(ctypes.c_uint8)),
        W, H, sx, sy, gx, gy,
        path_x, path_y, max_path, max_cells,
        ctypes.byref(explored),
    )

    cells = [(path_x[i], path_y[i]) for i in range(path_len)]
    return cells, explored.value


def _astar_python(blocked: np.ndarray, sx: int, sy: int, gx: int, gy: int,
                  max_cells: int) -> tuple[list[tuple[int, int]], int]:
    """Pure-Python A* fallback."""
    H, W = blocked.shape
    SQRT2 = 1.4142135623730951
    neighbors = (
        (-1, 0, 1.0), (1, 0, 1.0), (0, -1, 1.0), (0, 1, 1.0),
        (-1, -1, SQRT2), (-1, 1, SQRT2), (1, -1, SQRT2), (1, 1, SQRT2),
    )

    open_heap: list[tuple[float, float, int, int]] = []
    heapq.heappush(open_heap, (math.hypot(sx - gx, sy - gy), 0.0, sx, sy))
    g_score = np.full((H, W), np.inf, dtype=np.float32)
    g_score[sy, sx] = 0.0
    closed = np.zeros((H, W), dtype=bool)
    came_from = {}

    found = False
    cells_explored = 0

    while open_heap and cells_explored < max_cells:
        _, cur_g, x, y = heapq.heappop(open_heap)
        if closed[y, x]:
            continue
        closed[y, x] = True
        cells_explored += 1

        if x == gx and y == gy:
            found = True
            break

        for dx, dy, cost in neighbors:
            nx, ny = x + dx, y + dy
            if not (0 <= nx < W and 0 <= ny < H):
                continue
            if blocked[ny, nx] or closed[ny, nx]:
                continue
            ng = cur_g + cost
            if ng < g_score[ny, nx]:
                g_score[ny, nx] = ng
                came_from[(nx, ny)] = (x, y)
                heapq.heappush(open_heap, (ng + math.hypot(nx - gx, ny - gy), ng, nx, ny))

    if not found:
        return [], cells_explored

    path_cells = []
    c = (gx, gy)
    while c in came_from:
        path_cells.append(c)
        c = came_from[c]
    path_cells.append((sx, sy))
    path_cells.reverse()
    return path_cells, cells_explored


def plan_on_grid(
    info: OccGridInfo,
    robot_x: float,
    robot_y: float,
    goal_x: float,
    goal_y: float,
    inflation_m: float = 0.25,
    waypoint_spacing_m: float = 0.5,
    max_cells: int = 80000,
) -> GridPlanResult:
    """Run A* from robot to goal on the occupancy grid."""
    import time

    t0 = time.monotonic()
    result = GridPlanResult()

    inflate_cells = max(0, int(math.ceil(inflation_m / info.resolution)))
    blocked = info.data

    if blocked.dtype != bool:
        blocked = occupancy_to_blocked(blocked.ravel(), info.width, info.height, inflate_cells)
    elif inflate_cells > 0 and not hasattr(info, "_inflated"):
        blocked = _inflate_grid(blocked, inflate_cells)

    H, W = blocked.shape

    def w2c(wx: float, wy: float) -> tuple[int, int] | None:
        cx = int(math.floor((wx - info.origin_x) / info.resolution))
        cy = int(math.floor((wy - info.origin_y) / info.resolution))
        if 0 <= cx < W and 0 <= cy < H:
            return (cx, cy)
        return None

    start_cell = w2c(robot_x, robot_y)
    goal_cell = w2c(goal_x, goal_y)
    if start_cell is None or goal_cell is None:
        result.time_ms = (time.monotonic() - t0) * 1000
        return result

    sx, sy = start_cell
    if blocked[sy, sx]:
        r = inflate_cells + 2
        y_lo, y_hi = max(0, sy - r), min(H, sy + r + 1)
        x_lo, x_hi = max(0, sx - r), min(W, sx + r + 1)
        blocked = blocked.copy()
        blocked[y_lo:y_hi, x_lo:x_hi] = False

    gx, gy = goal_cell
    if blocked[gy, gx]:
        goal_cell = _nearest_free(blocked, gx, gy, W, H, search_radius=15)
        if goal_cell is None:
            result.time_ms = (time.monotonic() - t0) * 1000
            return result
        gx, gy = goal_cell

    # Run A* — C++ if available, else Python
    if _astar_lib is not None:
        path_cells, cells_explored = _astar_cpp(blocked, sx, sy, gx, gy, max_cells)
    else:
        path_cells, cells_explored = _astar_python(blocked, sx, sy, gx, gy, max_cells)

    result.cells_explored = cells_explored

    if not path_cells:
        result.time_ms = (time.monotonic() - t0) * 1000
        return result

    if len(path_cells) < 2:
        result.success = True
        result.time_ms = (time.monotonic() - t0) * 1000
        return result

    # Convert to world, resample
    res = info.resolution
    ox, oy = info.origin_x, info.origin_y
    path_world = [(ox + (cx + 0.5) * res, oy + (cy + 0.5) * res) for cx, cy in path_cells]

    resampled = [path_world[0]]
    accum = 0.0
    for i in range(1, len(path_world)):
        dx = path_world[i][0] - path_world[i - 1][0]
        dy = path_world[i][1] - path_world[i - 1][1]
        accum += math.hypot(dx, dy)
        if accum >= waypoint_spacing_m:
            resampled.append(path_world[i])
            accum = 0.0
    if resampled[-1] != path_world[-1]:
        resampled.append(path_world[-1])

    result.waypoints_world = resampled[1:]  # skip start (robot pos)
    result.success = True
    result.time_ms = (time.monotonic() - t0) * 1000
    return result


def _nearest_free(
    blocked: np.ndarray,
    gx: int,
    gy: int,
    W: int,
    H: int,
    search_radius: int = 10,
) -> tuple[int, int] | None:
    best = None
    best_d = float("inf")
    for dy in range(-search_radius, search_radius + 1):
        for dx in range(-search_radius, search_radius + 1):
            nx, ny = gx + dx, gy + dy
            if 0 <= nx < W and 0 <= ny < H and not blocked[ny, nx]:
                d = dx * dx + dy * dy
                if d < best_d:
                    best_d = d
                    best = (nx, ny)
    return best


class AsyncGridPlanner:
    """Runs A* in a background thread; results are polled non-blocking."""

    def __init__(
        self,
        inflation_m: float = 0.25,
        waypoint_spacing_m: float = 0.5,
        replan_interval_sec: float = 2.0,
        goal_shift_threshold_m: float = 0.3,
    ):
        self.inflation_m = inflation_m
        self.waypoint_spacing_m = waypoint_spacing_m
        self.replan_interval_sec = replan_interval_sec
        self.goal_shift_threshold_m = goal_shift_threshold_m

        self._lock = threading.Lock()
        self._thread: threading.Thread | None = None
        self._last_result: GridPlanResult | None = None
        self._last_plan_time: float | None = None
        self._last_plan_goal: tuple[float, float] | None = None

        # Cache inflated grid to avoid recomputing every cycle
        self._cached_blocked: np.ndarray | None = None
        self._cached_map_stamp: float | None = None

    def request_plan(
        self,
        now_sec: float,
        info: OccGridInfo,
        robot_x: float,
        robot_y: float,
        goal_x: float,
        goal_y: float,
        map_stamp_sec: float = 0.0,
    ) -> GridPlanResult | None:
        """Non-blocking: start a plan if needed, return latest result if ready."""
        goal_xy = (goal_x, goal_y)

        # Check if we need to re-plan
        need_replan = False
        if self._last_plan_goal is None:
            need_replan = True
        elif self._last_plan_time is None or (now_sec - self._last_plan_time) >= self.replan_interval_sec:
            need_replan = True
        elif math.hypot(goal_xy[0] - self._last_plan_goal[0], goal_xy[1] - self._last_plan_goal[1]) > self.goal_shift_threshold_m:
            need_replan = True

        if need_replan and (self._thread is None or not self._thread.is_alive()):
            self._last_plan_time = now_sec
            self._last_plan_goal = goal_xy

            # Pre-compute inflated grid (cache if map hasn't changed)
            if self._cached_blocked is None or self._cached_map_stamp != map_stamp_sec:
                inflate_cells = max(0, int(math.ceil(self.inflation_m / info.resolution)))
                if info.data.dtype == bool:
                    blocked = _inflate_grid(info.data, inflate_cells) if inflate_cells > 0 else info.data.copy()
                else:
                    blocked = occupancy_to_blocked(info.data.ravel(), info.width, info.height, inflate_cells)
                self._cached_blocked = blocked
                self._cached_map_stamp = map_stamp_sec

            # Launch background thread
            blocked_snapshot = self._cached_blocked
            plan_info = OccGridInfo(
                resolution=info.resolution,
                width=info.width,
                height=info.height,
                origin_x=info.origin_x,
                origin_y=info.origin_y,
                data=blocked_snapshot,
            )

            def _run():
                r = plan_on_grid(
                    plan_info, robot_x, robot_y, goal_x, goal_y,
                    inflation_m=0.0,  # already inflated
                    waypoint_spacing_m=self.waypoint_spacing_m,
                )
                with self._lock:
                    self._last_result = r

            self._thread = threading.Thread(target=_run, daemon=True)
            self._thread.start()

        # Return latest completed result
        with self._lock:
            r = self._last_result
            self._last_result = None
            return r

    def force_replan(self):
        """Force replan on next request."""
        self._last_plan_goal = None
        self._last_plan_time = None
        self._cached_blocked = None
