from __future__ import annotations

from .path_service import plan_path
from .types import GoalAssignment, RobotState


def keep_current_goal(
    robot: RobotState,
    current_assignment: GoalAssignment | None,
    best_new_score: float,
    map_mgr,
    cfg: dict,
) -> GoalAssignment | None:
    if current_assignment is None or not current_assignment.valid or current_assignment.target is None:
        return None

    margin = float(cfg.get("planning", {}).get("active_slam_explorer", {}).get("commitment_margin", 0.75))
    neighborhood = int(cfg.get("frontier", {}).get("neighborhood", 8))
    clearance_cells = int(cfg.get("robots", {}).get("clearance_cells", 0))

    path = plan_path(
        map_mgr=map_mgr,
        start=robot.pose,
        goal=current_assignment.target,
        cfg=cfg,
        neighborhood=neighborhood,
        clearance_cells=clearance_cells,
    )
    if path is None:
        return None
    if best_new_score > float(current_assignment.utility) + margin:
        return None

    return GoalAssignment(
        robot_id=robot.robot_id,
        target=current_assignment.target,
        path=path,
        utility=float(current_assignment.utility),
        valid=True,
        breakdown=dict(current_assignment.breakdown),
    )
