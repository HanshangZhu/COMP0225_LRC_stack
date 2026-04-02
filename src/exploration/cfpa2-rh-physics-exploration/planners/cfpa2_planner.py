from __future__ import annotations

from core.assignment_solver import compute_candidate_utilities, solve_joint_cfpa2, solve_single_robot
from core.types import GoalAssignment, PlannerInput, PlannerOutput

from .base_planner import BasePlanner


class CFPA2Planner(BasePlanner):
    name = "cfpa2"

    def plan(self, planner_input: PlannerInput) -> PlannerOutput:
        robots = planner_input.robot_states
        candidates = planner_input.frontier_candidates
        map_mgr = planner_input.shared_map
        cfg = planner_input.config
        reservations = planner_input.reservation_state

        if not robots:
            return PlannerOutput(planner_name=self.name, assignments={}, joint_score=float("-inf"), debug={"reason": "no_robot"})

        if len(robots) == 1:
            u = compute_candidate_utilities(robots[0], candidates, cfg, map_mgr, reservations)
            a = solve_single_robot(robots[0], u)
            return PlannerOutput(
                planner_name=self.name,
                assignments={robots[0].robot_id: a},
                joint_score=float(a.utility if a.valid else float("-inf")),
                score_breakdown={"single_utility": float(a.utility) if a.valid else float("-inf")},
                predicted_paths={robots[0].robot_id: a.path},
                debug={"candidate_count": len(candidates)},
            )

        r1, r2 = robots[0], robots[1]
        u1 = compute_candidate_utilities(r1, candidates, cfg, map_mgr, reservations)
        u2 = compute_candidate_utilities(r2, candidates, cfg, map_mgr, reservations)

        assignments, joint_score, breakdown, debug = solve_joint_cfpa2(r1, r2, u1, u2, cfg)
        predicted_paths = {
            rid: a.path
            for rid, a in assignments.items()
            if isinstance(a, GoalAssignment) and a.valid
        }
        debug.update({"candidate_count": len(candidates), "u1_count": len(u1), "u2_count": len(u2)})

        return PlannerOutput(
            planner_name=self.name,
            assignments=assignments,
            joint_score=float(joint_score),
            score_breakdown=breakdown,
            predicted_paths=predicted_paths,
            debug=debug,
        )
