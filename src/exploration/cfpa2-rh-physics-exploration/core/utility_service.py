from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from .frontier_value_model import contextual_frontier_bonus
from .map_manager import OCCUPIED, UNKNOWN, MapManager
from .path_service import heading_delta_cost, path_cost, plan_path
from .types import Cell, RobotState


@dataclass
class CandidateEvaluation:
    utility: float
    information_gain: float
    travel_cost: float
    switch_penalty: float
    turn_penalty: float
    path: list[Cell]


def information_gain(
    map_mgr: MapManager,
    frontier_cell: Cell,
    radius: int,
    known_grid: np.ndarray | None = None,
) -> float:
    return float(map_mgr.count_unknown_in_radius(frontier_cell, radius, grid=known_grid))


def switch_penalty(robot: RobotState, frontier: Cell) -> float:
    if robot.current_target is None:
        return 0.0
    return 0.0 if robot.current_target == frontier else 1.0


def evaluate_candidate(
    robot: RobotState,
    frontier: Cell,
    map_mgr: MapManager,
    cfg: dict,
    neighborhood: int = 8,
    known_grid: np.ndarray | None = None,
    candidate=None,
    all_candidates: list | None = None,
) -> CandidateEvaluation | None:
    weights = cfg["planning"]["weights"]
    clearance = int(cfg["robots"].get("clearance_cells", 0))

    ig = information_gain(map_mgr, frontier, radius=int(cfg["frontier"]["ig_radius"]), known_grid=known_grid)
    path = plan_path(map_mgr, robot.pose, frontier, cfg=cfg, neighborhood=neighborhood, clearance_cells=clearance)
    if path is None:
        return None

    travel = path_cost(path)
    sw = switch_penalty(robot, frontier)
    turn = heading_delta_cost(robot.heading_deg, path)
    context_bonus = 0.0
    if candidate is not None and all_candidates is not None:
        context_bonus, _ = contextual_frontier_bonus(robot, candidate, all_candidates, map_mgr, cfg)

    score = (
        float(weights.get("w_ig", 1.0)) * ig
        - float(weights.get("w_cost", 1.0)) * travel
        - float(weights.get("w_switch", 0.0)) * sw
        - float(weights.get("w_turn", 0.0)) * turn
        + float(context_bonus)
    )

    return CandidateEvaluation(
        utility=score,
        information_gain=ig,
        travel_cost=travel,
        switch_penalty=sw,
        turn_penalty=turn,
        path=path,
    )


def overlap_penalty(a: Cell, b: Cell, sigma: float) -> float:
    if sigma <= 0:
        return 0.0
    dx = a[0] - b[0]
    dy = a[1] - b[1]
    d2 = dx * dx + dy * dy
    return math.exp(-d2 / (2.0 * sigma * sigma))


def path_interference_penalty(path1: list[Cell], path2: list[Cell], distance_threshold: float = 2.5) -> float:
    if not path1 or not path2:
        return 0.0
    thr2 = distance_threshold * distance_threshold
    penalty = 0.0
    n = min(len(path1), len(path2))
    for i in range(n):
        p = path1[i]
        q = path2[i]
        dx = p[0] - q[0]
        dy = p[1] - q[1]
        d2 = dx * dx + dy * dy
        if d2 <= thr2:
            penalty += (thr2 - d2) / max(thr2, 1e-6)
    return penalty / float(max(1, n))


def _grid_value(map_mgr: MapManager, cell: Cell, known_grid: np.ndarray | None = None) -> int:
    arr = map_mgr.known if known_grid is None else known_grid
    x, y = cell
    return int(arr[y, x])


def _is_open_cell(
    map_mgr: MapManager,
    cell: Cell,
    known_grid: np.ndarray | None = None,
    assume_unknown_open: bool = True,
) -> bool:
    if not map_mgr.in_bounds(cell):
        return False
    val = _grid_value(map_mgr, cell, known_grid=known_grid)
    if val == OCCUPIED:
        return False
    if val == UNKNOWN and not assume_unknown_open:
        return False
    return True


def cell_narrowness_score(
    map_mgr: MapManager,
    cell: Cell,
    known_grid: np.ndarray | None = None,
    assume_unknown_open: bool = True,
) -> float:
    """Return [0,1] narrowness proxy, where larger means narrower passage."""

    if not _is_open_cell(map_mgr, cell, known_grid=known_grid, assume_unknown_open=assume_unknown_open):
        return 1.0

    x, y = cell
    neighbors = [(x + 1, y), (x - 1, y), (x, y + 1), (x, y - 1)]
    open_deg = 0
    for n in neighbors:
        if _is_open_cell(map_mgr, n, known_grid=known_grid, assume_unknown_open=assume_unknown_open):
            open_deg += 1

    # 1.0 when degree<=1 (very narrow/dead-end), 0.0 when degree>=3.
    return max(0.0, min(1.0, (3.0 - float(open_deg)) / 2.0))


def path_crossing_penalty(path1: list[Cell], path2: list[Cell]) -> float:
    if len(path1) < 2 or len(path2) < 2:
        return 0.0
    n = min(len(path1), len(path2))
    crosses = 0
    for t in range(1, n):
        if path1[t - 1] == path2[t] and path2[t - 1] == path1[t]:
            crosses += 1
    return float(crosses) / float(max(1, n - 1))


def corridor_occupancy_penalty(
    path1: list[Cell],
    path2: list[Cell],
    map_mgr: MapManager,
    known_grid: np.ndarray | None = None,
    near_distance: float = 2.5,
) -> float:
    if not path1 or not path2:
        return 0.0

    n = min(len(path1), len(path2))
    penalty = 0.0
    near_d = max(0.5, float(near_distance))
    near_d2 = near_d * near_d

    for t in range(n):
        c1 = path1[t]
        c2 = path2[t]
        dx = c1[0] - c2[0]
        dy = c1[1] - c2[1]
        d2 = float(dx * dx + dy * dy)
        if d2 > near_d2:
            continue

        dist = math.sqrt(max(1e-9, d2))
        proximity = max(0.0, (near_d - dist) / near_d)
        narrow = max(
            cell_narrowness_score(map_mgr, c1, known_grid=known_grid),
            cell_narrowness_score(map_mgr, c2, known_grid=known_grid),
        )
        penalty += proximity * (0.35 + 0.65 * narrow)

    return penalty / float(max(1, n))


def narrow_passage_blocking_penalty(
    path1: list[Cell],
    path2: list[Cell],
    map_mgr: MapManager,
    known_grid: np.ndarray | None = None,
    window: int = 2,
) -> float:
    if not path1 or not path2:
        return 0.0

    n1 = len(path1)
    n2 = len(path2)
    w = max(0, int(window))
    penalty = 0.0
    cnt = 0

    idx_by_cell_p2: dict[Cell, list[int]] = {}
    for i, c in enumerate(path2):
        idx_by_cell_p2.setdefault(c, []).append(i)

    for t1, c in enumerate(path1):
        narrow = cell_narrowness_score(map_mgr, c, known_grid=known_grid)
        if narrow < 0.5:
            continue
        t2_list = idx_by_cell_p2.get(c, [])
        if not t2_list:
            continue
        dt = min(abs(t1 - t2) for t2 in t2_list)
        if dt > w:
            continue
        proximity_t = float(w - dt + 1) / float(w + 1)
        penalty += proximity_t * (0.4 + 0.6 * narrow)
        cnt += 1

    return penalty / float(max(1, cnt, max(n1, n2)))


def waiting_time_proxy(
    path1: list[Cell],
    path2: list[Cell],
    map_mgr: MapManager,
    known_grid: np.ndarray | None = None,
    window: int = 2,
) -> float:
    if not path1 or not path2:
        return 0.0

    # Proxy: repeated occupancy of same narrow cells within short time window.
    w = max(0, int(window))
    occ1: dict[Cell, list[int]] = {}
    occ2: dict[Cell, list[int]] = {}
    for t, c in enumerate(path1):
        occ1.setdefault(c, []).append(t)
    for t, c in enumerate(path2):
        occ2.setdefault(c, []).append(t)

    common = set(occ1.keys()) & set(occ2.keys())
    if not common:
        return 0.0

    penalty = 0.0
    norm = float(max(1, len(path1), len(path2)))
    for c in common:
        narrow = cell_narrowness_score(map_mgr, c, known_grid=known_grid)
        if narrow < 0.4:
            continue
        t1s = occ1[c]
        t2s = occ2[c]
        dt = min(abs(a - b) for a in t1s for b in t2s)
        if dt > w:
            continue
        penalty += (float(w - dt + 1) / float(w + 1)) * (0.3 + 0.7 * narrow)

    return penalty / norm
