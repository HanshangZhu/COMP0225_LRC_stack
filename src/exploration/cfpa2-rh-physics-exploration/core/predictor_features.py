from __future__ import annotations

import math
from typing import Sequence

import numpy as np

from .map_manager import FREE, OCCUPIED, UNKNOWN, MapManager
from .types import Cell, RobotState


def extract_occupancy_patch(map_mgr: MapManager, center: Cell, patch_radius: int) -> np.ndarray:
    """Extract local known-map patch around robot pose.

    Values are encoded as: UNKNOWN=-1, FREE=0, OCCUPIED=1.
    Out-of-bounds cells are treated as OCCUPIED to reflect boundary constraints.
    """

    r = max(0, int(patch_radius))
    size = 2 * r + 1
    out = np.full((size, size), OCCUPIED, dtype=np.float32)

    cx, cy = center
    for py in range(size):
        gy = cy - r + py
        if gy < 0 or gy >= map_mgr.height:
            continue
        for px in range(size):
            gx = cx - r + px
            if gx < 0 or gx >= map_mgr.width:
                continue
            out[py, px] = float(map_mgr.known[gy, gx])

    return out


def goal_direction_features(robot_pose: Cell, goal: Cell | None) -> tuple[float, float, float]:
    if goal is None:
        return 0.0, 0.0, 0.0
    dx = float(goal[0] - robot_pose[0])
    dy = float(goal[1] - robot_pose[1])
    dist = math.hypot(dx, dy)
    if dist <= 1e-6:
        return 0.0, 0.0, 0.0
    return dx / dist, dy / dist, dist


def build_physics_feature_vector(
    map_mgr: MapManager,
    robot: RobotState,
    teammate: RobotState | None,
    goal: Cell | None,
    patch_radius: int,
    max_speed: float,
) -> np.ndarray:
    cos_goal, sin_goal, goal_dist = goal_direction_features(robot.pose, goal)

    vx, vy = robot.velocity
    speed = robot.speed
    heading_rad = math.radians(float(robot.heading_deg))

    teammate_distance = 10.0
    teammate_vx = 0.0
    teammate_vy = 0.0
    if teammate is not None:
        teammate_distance = float(math.hypot(teammate.pose[0] - robot.pose[0], teammate.pose[1] - robot.pose[1]))
        teammate_vx, teammate_vy = teammate.velocity

    patch = extract_occupancy_patch(map_mgr, robot.pose, patch_radius)
    patch_flat = patch.reshape(-1)

    local_obstacle_density = float(np.count_nonzero(patch_flat == OCCUPIED)) / float(max(1, patch_flat.size))

    numeric = np.array(
        [
            math.cos(heading_rad),
            math.sin(heading_rad),
            cos_goal,
            sin_goal,
            min(1.0, goal_dist / 20.0),
            min(1.0, speed / max(1e-6, max_speed)),
            float(vx),
            float(vy),
            local_obstacle_density,
            min(1.0, teammate_distance / 10.0),
            float(teammate_vx),
            float(teammate_vy),
        ],
        dtype=np.float32,
    )

    return np.concatenate([numeric, patch_flat.astype(np.float32)], axis=0)


def feature_dimension(patch_radius: int) -> int:
    size = 2 * max(0, int(patch_radius)) + 1
    patch_dim = size * size
    numeric_dim = 12
    return numeric_dim + patch_dim


def local_context_for_predictor(
    map_mgr: MapManager,
    robot: RobotState,
    teammate: RobotState | None,
    cfg: dict,
) -> dict:
    pred_cfg = cfg.get("predictor", {})
    phy_cfg = pred_cfg.get("physics_residual", {})
    patch_radius = int(phy_cfg.get("occupancy_patch_radius", 4))

    patch = extract_occupancy_patch(map_mgr, robot.pose, patch_radius)
    teammate_distance = 10.0
    teammate_velocity = (0.0, 0.0)
    if teammate is not None:
        teammate_distance = float(math.hypot(teammate.pose[0] - robot.pose[0], teammate.pose[1] - robot.pose[1]))
        teammate_velocity = teammate.velocity

    local_obstacle_density = float(np.count_nonzero(patch == OCCUPIED)) / float(max(1, patch.size))

    return {
        "max_speed_cells_per_step": float(cfg["robots"].get("max_speed_cells_per_step", 1.0)),
        "local_obstacle_density": local_obstacle_density,
        "teammate_distance": teammate_distance,
        "robot_velocity": tuple(float(v) for v in robot.velocity),
        "teammate_velocity": tuple(float(v) for v in teammate_velocity),
        "occupancy_patch_flat": patch.reshape(-1).astype(np.float32).tolist(),
        "occupancy_patch_radius": patch_radius,
        "feature_dim": feature_dimension(patch_radius),
        "known_encoding": {"unknown": UNKNOWN, "free": FREE, "occupied": OCCUPIED},
    }
