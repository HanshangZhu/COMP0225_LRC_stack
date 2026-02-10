from __future__ import annotations

import math
from typing import Optional


def wrap_angle(a: float) -> float:
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


def bresenham(x0: int, y0: int, x1: int, y1: int) -> list[tuple[int, int]]:
    points: list[tuple[int, int]] = []
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


def world_delta_to_local(wx: float, wy: float, robot_yaw: float) -> tuple[float, float]:
    cos_yaw = math.cos(robot_yaw)
    sin_yaw = math.sin(robot_yaw)
    lx = cos_yaw * wx + sin_yaw * wy
    ly = -sin_yaw * wx + cos_yaw * wy
    return (lx, ly)


def local_to_world(
    robot_x: float,
    robot_y: float,
    robot_yaw: float,
    lx: float,
    ly: float,
) -> tuple[float, float]:
    cos_yaw = math.cos(robot_yaw)
    sin_yaw = math.sin(robot_yaw)
    wx = robot_x + cos_yaw * lx - sin_yaw * ly
    wy = robot_y + sin_yaw * lx + cos_yaw * ly
    return (wx, wy)


def local_to_grid(
    lx: float,
    ly: float,
    planner_resolution: float,
    planner_cells: int,
) -> Optional[tuple[int, int]]:
    c = planner_cells // 2
    gx = int(round(lx / planner_resolution)) + c
    gy = int(round(ly / planner_resolution)) + c
    if gx < 0 or gy < 0 or gx >= planner_cells or gy >= planner_cells:
        return None
    return (gx, gy)


def grid_to_local(
    gx: int,
    gy: int,
    planner_resolution: float,
    planner_cells: int,
) -> tuple[float, float]:
    c = planner_cells // 2
    lx = (gx - c) * planner_resolution
    ly = (gy - c) * planner_resolution
    return (lx, ly)
