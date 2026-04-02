from __future__ import annotations

from core.active_slam_frontier_scorer import score_frontier
from core.frontier_filter import filter_reachable_frontiers
from core.goal_commitment import keep_current_goal
from core.types import GoalAssignment, PlannerInput, PlannerOutput

from .base_planner import BasePlanner


class ActiveSLAMExplorerPlanner(BasePlanner):
    name = "active_slam_explorer"

    def _idle(self, robot_id: int) -> GoalAssignment:
        return GoalAssignment(robot_id=robot_id, target=None, path=[], utility=float("-inf"), valid=False, breakdown={})

    def plan(self, planner_input: PlannerInput) -> PlannerOutput:
        robots = planner_input.robot_states
        map_mgr = planner_input.shared_map
        cfg = planner_input.config
        candidates = planner_input.frontier_candidates
        current_assignments = planner_input.current_assignments

        if not robots:
            return PlannerOutput(planner_name=self.name, assignments={}, joint_score=float("-inf"), debug={"reason": "no_robot"})

        assignments: dict[int, GoalAssignment] = {}
        predicted_paths: dict[int, list[tuple[int, int]]] = {}
        used_targets: set[tuple[int, int]] = set()
        score_breakdown: dict[str, float] = {}
        joint_score = 0.0

        # Single-robot-first architecture: multi-robot falls back to greedy non-overlapping assignment.
        robots_sorted = sorted(robots, key=lambda r: (r.robot_id, r.steps_since_progress))

        for robot in robots_sorted:
            reachable = filter_reachable_frontiers(robot, candidates, map_mgr, cfg)
            best_assignment = self._idle(robot.robot_id)
            best_score = float("-inf")
            best_breakdown: dict[str, float] = {}

            for item in reachable:
                rep = item.candidate.representative
                if rep in used_targets:
                    continue
                frontier_score = score_frontier(
                    robot=robot,
                    reachable_frontier=item,
                    map_mgr=map_mgr,
                    cfg=cfg,
                    current_assignment=current_assignments.get(robot.robot_id),
                )
                if frontier_score.score > best_score:
                    best_score = frontier_score.score
                    best_breakdown = dict(frontier_score.breakdown)
                    best_assignment = GoalAssignment(
                        robot_id=robot.robot_id,
                        target=rep,
                        path=item.path,
                        utility=float(frontier_score.score),
                        valid=True,
                        breakdown=dict(frontier_score.breakdown),
                    )

            kept = keep_current_goal(
                robot=robot,
                current_assignment=current_assignments.get(robot.robot_id),
                best_new_score=best_score,
                map_mgr=map_mgr,
                cfg=cfg,
            )
            if kept is not None and kept.target not in used_targets:
                best_assignment = kept
                best_score = float(kept.utility)
                best_breakdown = dict(kept.breakdown)

            assignments[robot.robot_id] = best_assignment
            if best_assignment.valid and best_assignment.target is not None:
                used_targets.add(best_assignment.target)
                predicted_paths[robot.robot_id] = list(best_assignment.path)
                joint_score += float(best_assignment.utility)
                for k, v in best_breakdown.items():
                    score_breakdown[f"r{robot.robot_id}_{k}"] = float(v)

        debug = {
            "candidate_count": len(candidates),
            "selected_targets": {rid: a.target for rid, a in assignments.items() if a.valid},
            "mode": "single_robot_first_greedy",
        }
        return PlannerOutput(
            planner_name=self.name,
            assignments=assignments,
            joint_score=float(joint_score if any(a.valid for a in assignments.values()) else float("-inf")),
            score_breakdown=score_breakdown,
            predicted_paths=predicted_paths,
            debug=debug,
        )
