from __future__ import annotations

import math

from .types import FrontierCandidate, RobotState


def frontier_policy_name(cfg: dict) -> str:
    return str(cfg.get("planning", {}).get("frontier_policy", {}).get("type", "classic")).strip().lower()


def contextual_frontier_bonus(
    robot: RobotState,
    candidate: FrontierCandidate,
    all_candidates: list[FrontierCandidate],
    map_mgr,
    cfg: dict,
) -> tuple[float, dict[str, float]]:
    policy = frontier_policy_name(cfg)
    if policy not in ("frontiernet_proxy", "hybrid_contextual"):
        return 0.0, {}

    policy_cfg = cfg.get("planning", {}).get("frontier_policy", {})
    rep = candidate.representative

    frontier_size = float(candidate.size)
    branch_radius = int(policy_cfg.get("branch_radius", max(3, int(cfg.get("frontier", {}).get("ig_radius", 4)))))
    branch_gain = float(map_mgr.count_unknown_in_radius(rep, branch_radius))

    clearance_radius = int(policy_cfg.get("clearance_radius", 1))
    local_obstacles = float(map_mgr.obstacle_count_around(rep, radius=clearance_radius))
    local_clearance = 1.0 / (1.0 + local_obstacles)

    density_radius = float(policy_cfg.get("density_radius", 8.0))
    density_radius_sq = density_radius * density_radius
    nearby_frontiers = 0.0
    for other in all_candidates:
        if other.representative == rep:
            continue
        dx = float(other.representative[0] - rep[0])
        dy = float(other.representative[1] - rep[1])
        if dx * dx + dy * dy <= density_radius_sq:
            nearby_frontiers += 1.0

    dx_goal = float(rep[0] - robot.pose[0])
    dy_goal = float(rep[1] - robot.pose[1])
    target_heading = math.degrees(math.atan2(dy_goal, dx_goal)) if (dx_goal or dy_goal) else robot.heading_deg
    heading_error = abs((target_heading - robot.heading_deg + 180.0) % 360.0 - 180.0) / 180.0
    heading_alignment = 1.0 - heading_error

    size_term = float(policy_cfg.get("w_cluster_size", 0.04)) * frontier_size
    branch_term = float(policy_cfg.get("w_branch_gain", 0.02)) * branch_gain
    density_term = float(policy_cfg.get("w_density", 0.18)) * nearby_frontiers
    clearance_term = float(policy_cfg.get("w_clearance", 0.35)) * local_clearance
    alignment_term = float(policy_cfg.get("w_heading_alignment", 0.12)) * heading_alignment

    bonus = size_term + branch_term + density_term + clearance_term + alignment_term
    breakdown = {
        "context_bonus": float(bonus),
        "cluster_size_bonus": float(size_term),
        "branch_gain_bonus": float(branch_term),
        "density_bonus": float(density_term),
        "clearance_bonus": float(clearance_term),
        "heading_alignment_bonus": float(alignment_term),
    }
    return float(bonus), breakdown
