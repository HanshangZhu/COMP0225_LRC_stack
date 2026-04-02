from __future__ import annotations

from dataclasses import dataclass

from .types import GoalAssignment, RobotState
from .utility_service import cell_narrowness_score


@dataclass
class FrontierScore:
    score: float
    breakdown: dict[str, float]


def _known_free_density(map_mgr, center: tuple[int, int], radius: int) -> float:
    cx, cy = center
    total = 0
    known_free = 0
    for y in range(max(0, cy - radius), min(map_mgr.height, cy + radius + 1)):
        for x in range(max(0, cx - radius), min(map_mgr.width, cx + radius + 1)):
            total += 1
            if map_mgr.is_known_free((x, y)):
                known_free += 1
    return float(known_free) / float(max(1, total))


def _loop_closure_proxy(map_mgr, frontier: tuple[int, int], radius: int) -> float:
    # Proxy for relocalization support: how much known free structure surrounds the frontier.
    density = _known_free_density(map_mgr, frontier, radius)
    narrowness = cell_narrowness_score(map_mgr, frontier)
    return max(0.0, density * (1.0 - 0.35 * narrowness))


def _uncertainty_reduction_proxy(map_mgr, frontier: tuple[int, int], inner_radius: int, outer_radius: int) -> float:
    inner = float(map_mgr.count_unknown_in_radius(frontier, inner_radius))
    outer = float(map_mgr.count_unknown_in_radius(frontier, outer_radius))
    return max(0.0, outer - 0.5 * inner)


def score_frontier(
    robot: RobotState,
    reachable_frontier,
    map_mgr,
    cfg: dict,
    current_assignment: GoalAssignment | None = None,
) -> FrontierScore:
    policy_cfg = cfg.get("planning", {}).get("active_slam_explorer", {})
    rep = reachable_frontier.candidate.representative

    branch_inner = int(policy_cfg.get("uncertainty_inner_radius", 3))
    branch_outer = int(policy_cfg.get("uncertainty_outer_radius", 7))
    loop_radius = int(policy_cfg.get("loop_closure_radius", 4))
    revisit_penalty = 1.0 if rep in robot.trajectory else 0.0
    commitment_bonus = 0.0
    if current_assignment is not None and current_assignment.valid and current_assignment.target == rep:
        commitment_bonus = 1.0

    uncertainty_reduction = _uncertainty_reduction_proxy(map_mgr, rep, branch_inner, branch_outer)
    loop_closure = _loop_closure_proxy(map_mgr, rep, loop_radius)
    frontier_size = float(reachable_frontier.candidate.size)
    obstacle_penalty = float(reachable_frontier.obstacle_count)
    travel_cost = float(reachable_frontier.travel_cost)
    information_gain = float(reachable_frontier.information_gain)

    w_gain = float(policy_cfg.get("w_information_gain", 1.0))
    w_distance = float(policy_cfg.get("w_travel_cost", 0.55))
    w_uncertainty = float(policy_cfg.get("w_uncertainty_reduction", 0.18))
    w_loop = float(policy_cfg.get("w_loop_closure", 0.12))
    w_frontier_size = float(policy_cfg.get("w_frontier_size", 0.05))
    w_revisit = float(policy_cfg.get("w_revisit_penalty", 0.30))
    w_obstacle = float(policy_cfg.get("w_target_obstacle_penalty", 0.03))
    w_commit = float(policy_cfg.get("w_commitment_bonus", 0.25))

    score = (
        w_gain * information_gain
        - w_distance * travel_cost
        + w_uncertainty * uncertainty_reduction
        + w_loop * loop_closure
        + w_frontier_size * frontier_size
        - w_revisit * revisit_penalty
        - w_obstacle * obstacle_penalty
        + w_commit * commitment_bonus
    )

    return FrontierScore(
        score=float(score),
        breakdown={
            "information_gain": information_gain,
            "travel_cost": travel_cost,
            "uncertainty_reduction": float(uncertainty_reduction),
            "loop_closure": float(loop_closure),
            "frontier_size": frontier_size,
            "revisit_penalty": float(revisit_penalty),
            "target_obstacle_penalty": obstacle_penalty,
            "commitment_bonus": float(commitment_bonus),
            "score": float(score),
        },
    )
