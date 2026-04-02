from __future__ import annotations

import math
from typing import Iterable

import numpy as np

from .types import Cell

UNKNOWN = -1
FREE = 0
OCCUPIED = 1


def _normalize_angle_deg(angle_deg: float) -> float:
    out = angle_deg % 360.0
    if out < 0:
        out += 360.0
    return out


def _smallest_angle_diff_deg(a: float, b: float) -> float:
    d = abs(_normalize_angle_deg(a) - _normalize_angle_deg(b))
    return min(d, 360.0 - d)


class MapManager:
    """Shared occupancy map manager for planner-level simulation."""

    def __init__(self, truth_map: np.ndarray):
        if truth_map.ndim != 2:
            raise ValueError("truth_map must be 2D")
        self.truth = truth_map.astype(np.int8, copy=True)
        self.height, self.width = self.truth.shape
        self.known = np.full_like(self.truth, UNKNOWN)
        self._free_truth_count = int(np.count_nonzero(self.truth == FREE))

    def clone_known(self) -> np.ndarray:
        return self.known.copy()

    def in_bounds(self, cell: Cell) -> bool:
        x, y = cell
        return 0 <= x < self.width and 0 <= y < self.height

    def get_known(self, cell: Cell) -> int:
        x, y = cell
        return int(self.known[y, x])

    def get_truth(self, cell: Cell) -> int:
        x, y = cell
        return int(self.truth[y, x])

    def set_truth_free(self, cell: Cell) -> None:
        x, y = cell
        if self.truth[y, x] == OCCUPIED:
            self.truth[y, x] = FREE
            self._free_truth_count += 1

    def ensure_starts_free(self, starts: Iterable[Cell]) -> None:
        for c in starts:
            if not self.in_bounds(c):
                raise ValueError(f"Start out of bounds: {c}")
            self.set_truth_free(c)

    def neighbors(self, cell: Cell, neighborhood: int = 8) -> list[Cell]:
        x, y = cell
        if neighborhood == 4:
            out = []
            for nx, ny in ((x + 1, y), (x - 1, y), (x, y + 1), (x, y - 1)):
                if 0 <= nx < self.width and 0 <= ny < self.height:
                    out.append((nx, ny))
            return out
        out = []
        for ny in range(y - 1, y + 2):
            for nx in range(x - 1, x + 2):
                if nx == x and ny == y:
                    continue
                if 0 <= nx < self.width and 0 <= ny < self.height:
                    out.append((nx, ny))
        return out

    def _bresenham_line(self, start: Cell, end: Cell) -> list[Cell]:
        x0, y0 = start
        x1, y1 = end
        line: list[Cell] = []

        dx = abs(x1 - x0)
        sx = 1 if x0 < x1 else -1
        dy = -abs(y1 - y0)
        sy = 1 if y0 < y1 else -1
        err = dx + dy

        x, y = x0, y0
        while True:
            line.append((x, y))
            if x == x1 and y == y1:
                break
            e2 = 2 * err
            if e2 >= dy:
                err += dy
                x += sx
            if e2 <= dx:
                err += dx
                y += sy
        return line

    def _is_visible(self, start: Cell, end: Cell) -> bool:
        line = self._bresenham_line(start, end)
        for c in line[1:-1]:
            if self.get_truth(c) == OCCUPIED:
                return False
        return True

    def observe_from(
        self,
        center: Cell,
        heading_deg: float,
        sensor_range: int,
        fov_deg: float,
        use_line_of_sight: bool,
        miss_prob: float,
        rng: np.random.Generator,
    ) -> set[Cell]:
        cx, cy = center
        rr = sensor_range * sensor_range
        observed: set[Cell] = set()

        min_x = max(0, cx - sensor_range)
        max_x = min(self.width - 1, cx + sensor_range)
        min_y = max(0, cy - sensor_range)
        max_y = min(self.height - 1, cy + sensor_range)

        full_view = fov_deg >= 359.0

        for y in range(min_y, max_y + 1):
            dy = y - cy
            for x in range(min_x, max_x + 1):
                dx = x - cx
                if dx * dx + dy * dy > rr:
                    continue

                if not full_view and (dx != 0 or dy != 0):
                    ray_angle = math.degrees(math.atan2(dy, dx))
                    if _smallest_angle_diff_deg(ray_angle, heading_deg) > fov_deg * 0.5:
                        continue

                if use_line_of_sight and not self._is_visible(center, (x, y)):
                    continue

                if miss_prob > 0.0 and rng.random() < miss_prob:
                    continue

                self.known[y, x] = self.truth[y, x]
                observed.add((x, y))

        return observed

    def is_known_free(self, cell: Cell) -> bool:
        return self.in_bounds(cell) and self.get_known(cell) == FREE

    def is_known_occupied(self, cell: Cell) -> bool:
        return self.in_bounds(cell) and self.get_known(cell) == OCCUPIED

    def is_unknown(self, cell: Cell) -> bool:
        return self.in_bounds(cell) and self.get_known(cell) == UNKNOWN

    def clearance_ok(self, cell: Cell, clearance: int) -> bool:
        if clearance <= 0:
            return True
        cx, cy = cell
        for y in range(max(0, cy - clearance), min(self.height, cy + clearance + 1)):
            for x in range(max(0, cx - clearance), min(self.width, cx + clearance + 1)):
                if self.known[y, x] == OCCUPIED:
                    return False
        return True

    def is_traversable(self, cell: Cell, clearance: int) -> bool:
        if not self.is_known_free(cell):
            return False
        return self.clearance_ok(cell, clearance)

    def nearest_known_free(self, cell: Cell, max_radius: int = 20) -> Cell | None:
        if self.is_known_free(cell):
            return cell
        cx, cy = cell
        for radius in range(1, max_radius + 1):
            min_x = max(0, cx - radius)
            max_x = min(self.width - 1, cx + radius)
            min_y = max(0, cy - radius)
            max_y = min(self.height - 1, cy + radius)
            for y in range(min_y, max_y + 1):
                for x in range(min_x, max_x + 1):
                    if self.known[y, x] == FREE:
                        return (x, y)
        return None

    def obstacle_count_around(self, cell: Cell, radius: int = 1) -> int:
        cx, cy = cell
        cnt = 0
        for y in range(max(0, cy - radius), min(self.height, cy + radius + 1)):
            for x in range(max(0, cx - radius), min(self.width, cx + radius + 1)):
                if self.known[y, x] == OCCUPIED:
                    cnt += 1
        return cnt

    def known_free_count(self) -> int:
        return int(np.count_nonzero(self.known == FREE))

    def explored_free_ratio(self) -> float:
        if self._free_truth_count <= 0:
            return 1.0
        return self.known_free_count() / float(self._free_truth_count)

    def known_ratio(self) -> float:
        total = self.width * self.height
        known = int(np.count_nonzero(self.known != UNKNOWN))
        return known / float(total)

    def count_unknown_in_radius(self, center: Cell, radius: int, grid: np.ndarray | None = None) -> int:
        arr = self.known if grid is None else grid
        cx, cy = center
        rr = radius * radius
        gain = 0
        for y in range(max(0, cy - radius), min(self.height, cy + radius + 1)):
            dy = y - cy
            for x in range(max(0, cx - radius), min(self.width, cx + radius + 1)):
                dx = x - cx
                if dx * dx + dy * dy > rr:
                    continue
                if arr[y, x] == UNKNOWN:
                    gain += 1
        return gain

    def mark_virtual_revealed(self, center: Cell, radius: int, grid: np.ndarray) -> None:
        cx, cy = center
        rr = radius * radius
        for y in range(max(0, cy - radius), min(self.height, cy + radius + 1)):
            dy = y - cy
            for x in range(max(0, cx - radius), min(self.width, cx + radius + 1)):
                dx = x - cx
                if dx * dx + dy * dy > rr:
                    continue
                if grid[y, x] == UNKNOWN:
                    grid[y, x] = FREE
