from __future__ import annotations

from collections import deque
from dataclasses import dataclass

import numpy as np

from .map_manager import FREE, UNKNOWN, MapManager
from .types import Cell, FrontierCandidate


@dataclass
class FrontierCluster:
    cells: list[Cell]
    representative: Cell


def _neighbors(cell: Cell, neighborhood: int) -> list[Cell]:
    x, y = cell
    if neighborhood == 4:
        return [(x + 1, y), (x - 1, y), (x, y + 1), (x, y - 1)]
    out: list[Cell] = []
    for ny in range(y - 1, y + 2):
        for nx in range(x - 1, x + 2):
            if nx == x and ny == y:
                continue
            out.append((nx, ny))
    return out


def is_frontier_cell(map_mgr: MapManager, cell: Cell, neighborhood: int = 8, grid: np.ndarray | None = None) -> bool:
    arr = map_mgr.known if grid is None else grid
    if not map_mgr.in_bounds(cell):
        return False
    x, y = cell
    if int(arr[y, x]) != FREE:
        return False
    for n in _neighbors(cell, neighborhood):
        if map_mgr.in_bounds(n):
            nx, ny = n
            if int(arr[ny, nx]) == UNKNOWN:
                return True
    return False


def detect_frontiers(map_mgr: MapManager, neighborhood: int = 8, grid: np.ndarray | None = None) -> list[Cell]:
    arr = map_mgr.known if grid is None else grid
    ys, xs = np.where(arr == FREE)
    frontiers: list[Cell] = []
    for y, x in zip(ys.tolist(), xs.tolist()):
        cell = (x, y)
        if is_frontier_cell(map_mgr, cell, neighborhood=neighborhood, grid=arr):
            frontiers.append(cell)
    return frontiers


def cluster_frontiers(frontier_cells: list[Cell], neighborhood: int, min_cluster_size: int) -> list[list[Cell]]:
    frontier_set = set(frontier_cells)
    visited: set[Cell] = set()
    clusters: list[list[Cell]] = []

    for seed in frontier_cells:
        if seed in visited:
            continue
        q: deque[Cell] = deque([seed])
        visited.add(seed)
        comp: list[Cell] = []

        while q:
            cur = q.popleft()
            comp.append(cur)
            for nxt in _neighbors(cur, neighborhood):
                if nxt in frontier_set and nxt not in visited:
                    visited.add(nxt)
                    q.append(nxt)

        if len(comp) >= min_cluster_size:
            clusters.append(comp)

    return clusters


def representative(cluster: list[Cell], map_mgr: MapManager, neighborhood: int = 8, grid: np.ndarray | None = None) -> Cell | None:
    if not cluster:
        return None
    arr = map_mgr.known if grid is None else grid

    coords = np.array(cluster, dtype=float)
    centroid = coords.mean(axis=0)
    cx, cy = int(round(float(centroid[0]))), int(round(float(centroid[1])))

    if (cx, cy) in cluster and is_frontier_cell(map_mgr, (cx, cy), neighborhood=neighborhood, grid=arr):
        return (cx, cy)

    best_cell: Cell | None = None
    best_dist = float("inf")
    for c in cluster:
        if not is_frontier_cell(map_mgr, c, neighborhood=neighborhood, grid=arr):
            continue
        d = (c[0] - centroid[0]) ** 2 + (c[1] - centroid[1]) ** 2
        if d < best_dist:
            best_dist = d
            best_cell = c

    return best_cell


def reduce_frontier_candidates(
    clusters: list[FrontierCluster],
    target_min: int,
    target_max: int | None,
    min_rep_distance: float,
) -> list[FrontierCluster]:
    if not clusters:
        return []
    if target_max is None:
        return clusters

    max_count = max(1, int(target_max))
    min_count = min(max_count, max(0, int(target_min)))

    ranked = sorted(clusters, key=lambda c: (-len(c.cells), c.representative[1], c.representative[0]))

    selected: list[FrontierCluster] = []
    selected_reps: set[Cell] = set()
    min_dist_sq = min_rep_distance * min_rep_distance

    for c in ranked:
        if len(selected) >= max_count:
            break
        rep = c.representative
        if min_rep_distance > 0.0:
            too_close = any((rep[0] - s.representative[0]) ** 2 + (rep[1] - s.representative[1]) ** 2 < min_dist_sq for s in selected)
            if too_close:
                continue
        selected.append(c)
        selected_reps.add(rep)

    if len(selected) < min_count:
        for c in ranked:
            if c.representative in selected_reps:
                continue
            selected.append(c)
            selected_reps.add(c.representative)
            if len(selected) >= min_count or len(selected) >= max_count:
                break

    return selected[:max_count]


def build_frontier_candidates(map_mgr: MapManager, cfg: dict) -> tuple[list[Cell], list[FrontierCandidate]]:
    fcfg = cfg["frontier"]
    neighborhood = int(fcfg.get("neighborhood", 8))
    min_cluster_size = int(fcfg.get("min_cluster_size", 1))

    frontier_cells = detect_frontiers(map_mgr, neighborhood=neighborhood)
    comps = cluster_frontiers(frontier_cells, neighborhood=neighborhood, min_cluster_size=min_cluster_size)

    clusters: list[FrontierCluster] = []
    for comp in comps:
        rep = representative(comp, map_mgr, neighborhood=neighborhood)
        if rep is None:
            continue
        clusters.append(FrontierCluster(cells=comp, representative=rep))

    reduced = reduce_frontier_candidates(
        clusters,
        target_min=int(fcfg.get("target_frontier_count_min", 0)),
        target_max=(int(fcfg["target_frontier_count_max"]) if fcfg.get("target_frontier_count_max") is not None else None),
        min_rep_distance=float(fcfg.get("representative_min_distance", 0.0)),
    )

    candidates = [FrontierCandidate(representative=c.representative, cells=c.cells) for c in reduced]
    return frontier_cells, candidates
