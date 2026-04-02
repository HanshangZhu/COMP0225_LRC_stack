from __future__ import annotations

import math
from dataclasses import dataclass

from .path_service import heading_delta_cost, path_cost, plan_path
from .types import Cell, FrontierCandidate, GoalAssignment, RobotState
from .utility_service import information_gain, switch_penalty


@dataclass
class SingleRobotFrontierEvaluation:
    target: Cell
    path: list[Cell]
    utility: float
    information_gain: float
    travel_cost: float
    switch_penalty: float
    turn_penalty: float
    cluster_size_bonus: float
    goal_progress_bonus: float
    goal_steps: int


def _idle_assignment(robot_id: int) -> GoalAssignment:
    return GoalAssignment(robot_id=robot_id, target=None, path=[], utility=float("-inf"), valid=False, breakdown={})


def _cluster_centroid(candidate: FrontierCandidate) -> tuple[float, float]:
    if not candidate.cells:
        return float(candidate.representative[0]), float(candidate.representative[1])
    xs = [c[0] for c in candidate.cells]
    ys = [c[1] for c in candidate.cells]
    return float(sum(xs)) / float(len(xs)), float(sum(ys)) / float(len(ys))


def _rank_cluster_cells(robot: RobotState, candidate: FrontierCandidate, max_cells: int) -> list[Cell]:
    cx, cy = _cluster_centroid(candidate)

    def key(cell: Cell) -> tuple[float, float, int, int]:
        dx = float(cell[0] - robot.pose[0])
        dy = float(cell[1] - robot.pose[1])
        dist_robot = dx * dx + dy * dy
        ddx = float(cell[0] - cx)
        ddy = float(cell[1] - cy)
        dist_centroid = ddx * ddx + ddy * ddy
        return (dist_robot, dist_centroid, -cell[1], -cell[0])

    ranked = sorted(candidate.cells, key=key, reverse=True)
    if candidate.representative not in ranked:
        ranked.append(candidate.representative)
    if max_cells <= 0 or len(ranked) <= max_cells:
        return ranked
    return ranked[:max_cells]


def evaluate_frontier_cluster(
    robot: RobotState,
    candidate: FrontierCandidate,
    map_mgr,
    cfg: dict,
    reserved_targets: set[Cell] | None = None,
) -> SingleRobotFrontierEvaluation | None:
    reserved = reserved_targets or set()
    planning_cfg = cfg.get("planning", {})
    weights = planning_cfg.get("weights", {})
    solo_cfg = planning_cfg.get("single_robot_frontier", {})
    neighborhood = int(cfg.get("frontier", {}).get("neighborhood", 8))
    clearance = int(cfg.get("robots", {}).get("clearance_cells", 0))

    min_goal_steps = max(1, int(solo_cfg.get("min_goal_steps", 3)))
    preferred_goal_steps = max(min_goal_steps, int(solo_cfg.get("preferred_goal_steps", 8)))
    max_cells = max(1, int(solo_cfg.get("max_cells_per_cluster_eval", 32)))
    cluster_size_bonus = float(solo_cfg.get("w_cluster_size", 0.18)) * math.sqrt(float(max(1, candidate.size)))
    goal_progress_weight = float(solo_cfg.get("w_goal_progress", 0.55))

    valid: list[SingleRobotFrontierEvaluation] = []
    longer: list[SingleRobotFrontierEvaluation] = []

    for cell in _rank_cluster_cells(robot, candidate, max_cells):
        if cell in reserved or cell == robot.pose:
            continue

        path = plan_path(map_mgr, robot.pose, cell, cfg=cfg, neighborhood=neighborhood, clearance_cells=clearance)
        if path is None:
            continue

        goal_steps = max(0, len(path) - 1)
        if goal_steps <= 0:
            continue

        travel = path_cost(path)
        if not math.isfinite(travel) or travel <= 0.0:
            continue

        ig = information_gain(map_mgr, cell, radius=int(cfg["frontier"]["ig_radius"]))
        sw = switch_penalty(robot, cell)
        turn = heading_delta_cost(robot.heading_deg, path)
        progress_bonus = goal_progress_weight * min(float(goal_steps), float(preferred_goal_steps)) / float(preferred_goal_steps)

        utility = (
            float(weights.get("w_ig", 1.0)) * ig
            - float(weights.get("w_cost", 1.0)) * travel
            - float(weights.get("w_switch", 0.0)) * sw
            - float(weights.get("w_turn", 0.0)) * turn
            + cluster_size_bonus
            + progress_bonus
        )

        # VLM frontier boost — only active when VLMPlanner injects _vlm_boost_rep
        vlm_boost_rep = solo_cfg.get("_vlm_boost_rep")
        if vlm_boost_rep is not None and candidate.representative == tuple(vlm_boost_rep):
            utility += float(solo_cfg.get("_vlm_boost_amount", 0.0))

        ev = SingleRobotFrontierEvaluation(
            target=cell,
            path=path,
            utility=float(utility),
            information_gain=float(ig),
            travel_cost=float(travel),
            switch_penalty=float(sw),
            turn_penalty=float(turn),
            cluster_size_bonus=float(cluster_size_bonus),
            goal_progress_bonus=float(progress_bonus),
            goal_steps=int(goal_steps),
        )
        valid.append(ev)
        if goal_steps >= min_goal_steps:
            longer.append(ev)

    pool = longer if longer else valid
    if not pool:
        return None

    return max(pool, key=lambda ev: (ev.utility, ev.goal_steps, ev.information_gain))


def select_single_robot_assignment(
    robot: RobotState,
    candidates: list[FrontierCandidate],
    map_mgr,
    cfg: dict,
    reserved_targets: set[Cell] | None = None,
) -> GoalAssignment:
    best: SingleRobotFrontierEvaluation | None = None

    for candidate in candidates:
        ev = evaluate_frontier_cluster(robot, candidate, map_mgr, cfg, reserved_targets=reserved_targets)
        if ev is None:
            continue
        if best is None or (ev.utility, ev.goal_steps, ev.information_gain) > (best.utility, best.goal_steps, best.information_gain):
            best = ev

    if best is None:
        return _idle_assignment(robot.robot_id)

    return GoalAssignment(
        robot_id=robot.robot_id,
        target=best.target,
        path=best.path,
        utility=float(best.utility),
        valid=True,
        breakdown={
            "ig": float(best.information_gain),
            "travel_cost": float(best.travel_cost),
            "switch_penalty": float(best.switch_penalty),
            "turn_penalty": float(best.turn_penalty),
            "cluster_size_bonus": float(best.cluster_size_bonus),
            "goal_progress_bonus": float(best.goal_progress_bonus),
            "goal_steps": float(best.goal_steps),
        },
    )
