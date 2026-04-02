from __future__ import annotations

from dataclasses import dataclass

from .types import FrontierCandidate, GoalAssignment, RobotState


@dataclass
class MacroFrontierEvaluation:
    candidate: FrontierCandidate
    path: list[tuple[int, int]]
    score: float
    breakdown: dict[str, float]


def _branch_opening_potential(map_mgr, rep: tuple[int, int], inner_radius: int, outer_radius: int) -> float:
    inner = float(map_mgr.count_unknown_in_radius(rep, inner_radius))
    outer = float(map_mgr.count_unknown_in_radius(rep, outer_radius))
    return max(0.0, outer - 0.5 * inner)


def score_macro_frontier(
    robot: RobotState,
    candidate: FrontierCandidate,
    path: list[tuple[int, int]],
    map_mgr,
    cfg: dict,
    current_assignment: GoalAssignment | None = None,
) -> MacroFrontierEvaluation:
    policy_cfg = cfg.get("planning", {}).get("macro_frontier_explorer", {})
    rep = candidate.representative
    travel_cost = float(max(0, len(path) - 1))
    information_gain = float(
        map_mgr.count_unknown_in_radius(rep, int(policy_cfg.get("information_gain_radius", cfg.get("frontier", {}).get("ig_radius", 6))))
    )
    cluster_size = float(candidate.size)
    branch_opening = _branch_opening_potential(
        map_mgr,
        rep,
        inner_radius=int(policy_cfg.get("branch_inner_radius", 3)),
        outer_radius=int(policy_cfg.get("branch_outer_radius", 8)),
    )
    revisit_penalty = 1.0 if rep in robot.trajectory else 0.0
    commitment_bonus = 1.0 if current_assignment is not None and current_assignment.valid and current_assignment.target == rep else 0.0

    score = (
        float(policy_cfg.get("w_information_gain", 1.0)) * information_gain
        + float(policy_cfg.get("w_cluster_size", 0.08)) * cluster_size
        + float(policy_cfg.get("w_branch_opening", 0.16)) * branch_opening
        - float(policy_cfg.get("w_travel_cost", 0.45)) * travel_cost
        - float(policy_cfg.get("w_revisit_penalty", 0.28)) * revisit_penalty
        + float(policy_cfg.get("w_commitment_bonus", 0.40)) * commitment_bonus
    )
    breakdown = {
        "information_gain": information_gain,
        "cluster_size": cluster_size,
        "branch_opening": float(branch_opening),
        "travel_cost": travel_cost,
        "revisit_penalty": float(revisit_penalty),
        "commitment_bonus": float(commitment_bonus),
        "score": float(score),
    }
    return MacroFrontierEvaluation(candidate=candidate, path=path, score=float(score), breakdown=breakdown)
