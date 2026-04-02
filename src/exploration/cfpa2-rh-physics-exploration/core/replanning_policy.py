from __future__ import annotations

from .frontier_manager import is_frontier_cell
from .types import Cell, GoalAssignment, RobotState


def should_replan(
    map_mgr,
    robots: list[RobotState],
    assignments: dict[int, GoalAssignment],
    frontier_reps: set[Cell],
    step_idx: int,
    prev_frontier_count: int,
    current_frontier_count: int,
    cfg: dict,
) -> tuple[bool, str]:
    repl_cfg = cfg["replanning"]
    interval = int(repl_cfg.get("periodic_replan_interval", 0))
    if interval > 0 and step_idx % interval == 0:
        return True, "periodic"

    if not bool(repl_cfg.get("enable_event_replan", True)):
        return False, "none"

    if prev_frontier_count >= 0:
        baseline = max(1, prev_frontier_count)
        ratio = abs(current_frontier_count - prev_frontier_count) / baseline
        if ratio > float(repl_cfg.get("frontier_change_threshold", 0.25)):
            return True, "frontier_change"

    stuck_thr = int(repl_cfg.get("stuck_threshold", 8))
    invalid_path_threshold = int(repl_cfg.get("invalidation_path_threshold", 3))
    invalid_dist = float(repl_cfg.get("invalidation_distance_threshold", 2.0))
    invalid_dist_sq = invalid_dist * invalid_dist
    neighborhood = int(cfg["frontier"].get("neighborhood", 8))

    for r in robots:
        a = assignments.get(r.robot_id)

        if a is not None and a.valid and r.at_target():
            return True, f"target_reached_r{r.robot_id}"

        if r.current_target is not None:
            if not is_frontier_cell(map_mgr, r.current_target, neighborhood=neighborhood):
                dx = r.current_target[0] - r.pose[0]
                dy = r.current_target[1] - r.pose[1]
                dist_sq = dx * dx + dy * dy
                near_completion = len(r.path) <= invalid_path_threshold
                if dist_sq > invalid_dist_sq and not near_completion:
                    return True, f"target_invalidated_r{r.robot_id}"

        if r.current_target is not None and not r.path:
            return True, f"path_empty_r{r.robot_id}"

        if r.current_target is not None and r.steps_since_progress > stuck_thr:
            return True, f"stuck_r{r.robot_id}"

    if not frontier_reps:
        return True, "frontier_empty"

    return False, "none"


def apply_hysteresis(
    old_assignments: dict[int, GoalAssignment],
    new_assignments: dict[int, GoalAssignment],
    old_score: float | None,
    new_score: float,
    cfg: dict,
) -> tuple[dict[int, GoalAssignment], bool]:
    margin = float(cfg["planning"].get("hysteresis_margin", 0.0))
    if old_score is None:
        return new_assignments, False
    if new_score > old_score + margin:
        return new_assignments, False
    return old_assignments, True
