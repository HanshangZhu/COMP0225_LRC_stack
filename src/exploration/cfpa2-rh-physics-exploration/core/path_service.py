from __future__ import annotations

import heapq
import math

from .map_manager import MapManager
from .types import Cell


def _heuristic(a: Cell, b: Cell, neighborhood: int) -> float:
    dx = abs(a[0] - b[0])
    dy = abs(a[1] - b[1])
    if neighborhood == 4:
        return float(dx + dy)
    return float((dx + dy) + (math.sqrt(2.0) - 2.0) * min(dx, dy))


def _neighbors(cell: Cell, neighborhood: int) -> list[tuple[Cell, float]]:
    x, y = cell
    if neighborhood == 4:
        return [((x + 1, y), 1.0), ((x - 1, y), 1.0), ((x, y + 1), 1.0), ((x, y - 1), 1.0)]
    c = math.sqrt(2.0)
    return [
        ((x + 1, y), 1.0),
        ((x - 1, y), 1.0),
        ((x, y + 1), 1.0),
        ((x, y - 1), 1.0),
        ((x + 1, y + 1), c),
        ((x + 1, y - 1), c),
        ((x - 1, y + 1), c),
        ((x - 1, y - 1), c),
    ]


def _is_valid_transition(
    map_mgr: MapManager,
    current: Cell,
    nxt: Cell,
    clearance_cells: int,
) -> bool:
    if not map_mgr.in_bounds(nxt):
        return False
    if not map_mgr.is_traversable(nxt, clearance_cells):
        return False

    dx = nxt[0] - current[0]
    dy = nxt[1] - current[1]
    if abs(dx) == 1 and abs(dy) == 1:
        side_a = (current[0] + dx, current[1])
        side_b = (current[0], current[1] + dy)
        if not map_mgr.is_traversable(side_a, clearance_cells):
            return False
        if not map_mgr.is_traversable(side_b, clearance_cells):
            return False
    return True


def _clearance_proxy(map_mgr: MapManager, cell: Cell, probe_radius: int) -> float:
    if probe_radius <= 0:
        return 0.0
    for radius in range(1, probe_radius + 1):
        if map_mgr.obstacle_count_around(cell, radius=radius) > 0:
            return float(radius - 1)
    return float(probe_radius)


def astar_path(
    map_mgr: MapManager,
    start: Cell,
    goal: Cell,
    neighborhood: int = 8,
    clearance_cells: int = 0,
) -> list[Cell] | None:
    if not map_mgr.in_bounds(start) or not map_mgr.in_bounds(goal):
        return None
    if not map_mgr.is_traversable(start, clearance_cells):
        return None
    if not map_mgr.is_traversable(goal, clearance_cells):
        return None
    if start == goal:
        return [start]

    open_heap: list[tuple[float, Cell]] = [(0.0, start)]
    came_from: dict[Cell, Cell] = {}
    g_score: dict[Cell, float] = {start: 0.0}
    closed: set[Cell] = set()

    while open_heap:
        _, current = heapq.heappop(open_heap)
        if current in closed:
            continue
        if current == goal:
            return _reconstruct(came_from, current)

        closed.add(current)

        for nxt, step_cost in _neighbors(current, neighborhood):
            if not _is_valid_transition(map_mgr, current, nxt, clearance_cells):
                continue

            tentative = g_score[current] + step_cost
            if tentative < g_score.get(nxt, float("inf")):
                came_from[nxt] = current
                g_score[nxt] = tentative
                f = tentative + _heuristic(nxt, goal, neighborhood)
                heapq.heappush(open_heap, (f, nxt))

    return None


def cost_aware_astar_path(
    map_mgr: MapManager,
    start: Cell,
    goal: Cell,
    neighborhood: int = 8,
    clearance_cells: int = 0,
    obstacle_cost_radius: int = 1,
    obstacle_cost_weight: float = 0.0,
    turn_cost_weight: float = 0.0,
    clearance_probe_radius: int = 0,
    clearance_bias_weight: float = 0.0,
) -> list[Cell] | None:
    if not map_mgr.in_bounds(start) or not map_mgr.in_bounds(goal):
        return None
    if not map_mgr.is_traversable(start, clearance_cells):
        return None
    if not map_mgr.is_traversable(goal, clearance_cells):
        return None
    if start == goal:
        return [start]

    open_heap: list[tuple[float, Cell]] = [(0.0, start)]
    came_from: dict[Cell, Cell] = {}
    g_score: dict[Cell, float] = {start: 0.0}
    closed: set[Cell] = set()

    while open_heap:
        _, current = heapq.heappop(open_heap)
        if current in closed:
            continue
        if current == goal:
            return _reconstruct(came_from, current)

        closed.add(current)
        prev = came_from.get(current)

        for nxt, step_cost in _neighbors(current, neighborhood):
            if not _is_valid_transition(map_mgr, current, nxt, clearance_cells):
                continue

            obstacle_penalty = obstacle_cost_weight * float(map_mgr.obstacle_count_around(nxt, radius=obstacle_cost_radius))
            clearance_penalty = 0.0
            if clearance_bias_weight > 0.0 and clearance_probe_radius > 0:
                clearance_penalty = clearance_bias_weight / (1.0 + _clearance_proxy(map_mgr, nxt, clearance_probe_radius))
            turn_penalty = 0.0
            if prev is not None and turn_cost_weight > 0.0:
                dx1 = current[0] - prev[0]
                dy1 = current[1] - prev[1]
                dx2 = nxt[0] - current[0]
                dy2 = nxt[1] - current[1]
                if (dx1, dy1) != (dx2, dy2):
                    turn_penalty = turn_cost_weight

            tentative = g_score[current] + step_cost + obstacle_penalty + clearance_penalty + turn_penalty
            if tentative < g_score.get(nxt, float("inf")):
                came_from[nxt] = current
                g_score[nxt] = tentative
                f = tentative + _heuristic(nxt, goal, neighborhood)
                heapq.heappush(open_heap, (f, nxt))

    return None


def maze_aware_astar_path(
    map_mgr: MapManager,
    start: Cell,
    goal: Cell,
    neighborhood: int = 8,
    clearance_cells: int = 0,
    obstacle_cost_radius: int = 1,
    obstacle_cost_weight: float = 0.08,
    turn_cost_weight: float = 0.03,
    clearance_probe_radius: int = 3,
    clearance_bias_weight: float = 0.30,
) -> list[Cell] | None:
    return cost_aware_astar_path(
        map_mgr=map_mgr,
        start=start,
        goal=goal,
        neighborhood=neighborhood,
        clearance_cells=clearance_cells,
        obstacle_cost_radius=obstacle_cost_radius,
        obstacle_cost_weight=obstacle_cost_weight,
        turn_cost_weight=turn_cost_weight,
        clearance_probe_radius=clearance_probe_radius,
        clearance_bias_weight=clearance_bias_weight,
    )


def plan_path(
    map_mgr: MapManager,
    start: Cell,
    goal: Cell,
    cfg: dict,
    neighborhood: int = 8,
    clearance_cells: int = 0,
) -> list[Cell] | None:
    local_cfg = cfg.get("planning", {}).get("local_planner", {})
    planner_type = str(local_cfg.get("type", "grid_astar")).strip().lower()
    if planner_type == "maze_aware_astar":
        return maze_aware_astar_path(
            map_mgr=map_mgr,
            start=start,
            goal=goal,
            neighborhood=neighborhood,
            clearance_cells=clearance_cells,
            obstacle_cost_radius=int(local_cfg.get("obstacle_cost_radius", 1)),
            obstacle_cost_weight=float(local_cfg.get("obstacle_cost_weight", 0.08)),
            turn_cost_weight=float(local_cfg.get("turn_cost_weight", 0.03)),
            clearance_probe_radius=int(local_cfg.get("clearance_probe_radius", 3)),
            clearance_bias_weight=float(local_cfg.get("clearance_bias_weight", 0.30)),
        )
    if planner_type == "cost_aware_astar":
        return cost_aware_astar_path(
            map_mgr=map_mgr,
            start=start,
            goal=goal,
            neighborhood=neighborhood,
            clearance_cells=clearance_cells,
            obstacle_cost_radius=int(local_cfg.get("obstacle_cost_radius", 1)),
            obstacle_cost_weight=float(local_cfg.get("obstacle_cost_weight", 0.15)),
            turn_cost_weight=float(local_cfg.get("turn_cost_weight", 0.05)),
            clearance_probe_radius=int(local_cfg.get("clearance_probe_radius", 0)),
            clearance_bias_weight=float(local_cfg.get("clearance_bias_weight", 0.0)),
        )
    return astar_path(
        map_mgr=map_mgr,
        start=start,
        goal=goal,
        neighborhood=neighborhood,
        clearance_cells=clearance_cells,
    )


def _reconstruct(came_from: dict[Cell, Cell], current: Cell) -> list[Cell]:
    path = [current]
    while current in came_from:
        current = came_from[current]
        path.append(current)
    path.reverse()
    return path


def path_cost(path: list[Cell] | None) -> float:
    if not path or len(path) <= 1:
        return 0.0 if path else float("inf")
    total = 0.0
    for a, b in zip(path[:-1], path[1:]):
        dx = abs(a[0] - b[0])
        dy = abs(a[1] - b[1])
        if dx == 1 and dy == 1:
            total += math.sqrt(2.0)
        else:
            total += 1.0
    return total


def heading_delta_cost(heading_deg: float, path: list[Cell]) -> float:
    if len(path) < 2:
        return 0.0
    x0, y0 = path[0]
    x1, y1 = path[1]
    target_heading = math.degrees(math.atan2(y1 - y0, x1 - x0))
    diff = abs((target_heading - heading_deg + 180.0) % 360.0 - 180.0)
    return diff / 180.0
