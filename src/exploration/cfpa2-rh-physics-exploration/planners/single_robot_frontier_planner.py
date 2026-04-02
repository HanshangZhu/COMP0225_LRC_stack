from __future__ import annotations

from core.single_robot_frontier import select_single_robot_assignment
from core.types import PlannerInput, PlannerOutput

from .base_planner import BasePlanner


class SingleRobotFrontierPlanner(BasePlanner):
    name = "single_robot_frontier"

    def plan(self, planner_input: PlannerInput) -> PlannerOutput:
        robots = sorted(planner_input.robot_states, key=lambda robot: robot.robot_id)
        map_mgr = planner_input.shared_map
        cfg = planner_input.config
        candidates = planner_input.frontier_candidates
        reservation_state = planner_input.reservation_state

        if not robots:
            return PlannerOutput(planner_name=self.name, assignments={}, joint_score=float("-inf"), debug={"reason": "no_robot"})

        assignments = {}
        predicted_paths = {}
        reserved_targets = {
            cell
            for cell, entry in reservation_state.items()
            if int(entry.get("robot_id", -1)) not in {robot.robot_id for robot in robots}
        }
        total_score = 0.0

        for robot in robots:
            assignment = select_single_robot_assignment(
                robot=robot,
                candidates=candidates,
                map_mgr=map_mgr,
                cfg=cfg,
                reserved_targets=reserved_targets,
            )
            assignments[robot.robot_id] = assignment
            if assignment.valid and assignment.target is not None:
                reserved_targets.add(assignment.target)
                predicted_paths[robot.robot_id] = assignment.path
                total_score += float(assignment.utility)

        debug = {
            "candidate_count": len(candidates),
            "robot_count": len(robots),
            "mode": "single_robot_frontier",
        }
        return PlannerOutput(
            planner_name=self.name,
            assignments=assignments,
            joint_score=float(total_score if predicted_paths else float("-inf")),
            predicted_paths=predicted_paths,
            debug=debug,
        )
