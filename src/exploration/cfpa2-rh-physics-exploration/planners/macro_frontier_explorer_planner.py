from __future__ import annotations

from core.goal_commitment import keep_current_goal
from core.macro_frontier_scorer import MacroFrontierEvaluation, score_macro_frontier
from core.path_service import plan_path
from core.types import GoalAssignment, PlannerInput, PlannerOutput

from .base_planner import BasePlanner


class MacroFrontierExplorerPlanner(BasePlanner):
    name = "macro_frontier_explorer"

    def _idle(self, robot_id: int) -> GoalAssignment:
        return GoalAssignment(robot_id=robot_id, target=None, path=[], utility=float("-inf"), valid=False, breakdown={})

    def _reachable_macro_frontiers(self, robot, candidates, map_mgr, cfg, current_assignment) -> list[MacroFrontierEvaluation]:
        policy_cfg = cfg.get("planning", {}).get("macro_frontier_explorer", {})
        min_path_steps = int(policy_cfg.get("min_travel_steps", 4))
        topk = int(policy_cfg.get("candidate_topk", cfg.get("planning", {}).get("topk_candidate_limit", 8)))
        neighborhood = int(cfg.get("frontier", {}).get("neighborhood", 8))
        clearance_cells = int(cfg.get("robots", {}).get("clearance_cells", 0))

        long_range: list[MacroFrontierEvaluation] = []
        fallback: list[MacroFrontierEvaluation] = []

        for candidate in candidates:
            path = plan_path(
                map_mgr=map_mgr,
                start=robot.pose,
                goal=candidate.representative,
                cfg=cfg,
                neighborhood=neighborhood,
                clearance_cells=clearance_cells,
            )
            if path is None:
                continue
            ev = score_macro_frontier(
                robot=robot,
                candidate=candidate,
                path=path,
                map_mgr=map_mgr,
                cfg=cfg,
                current_assignment=current_assignment,
            )
            if len(path) - 1 >= min_path_steps:
                long_range.append(ev)
            else:
                fallback.append(ev)

        pool = long_range if long_range else fallback
        pool.sort(key=lambda item: item.score, reverse=True)
        return pool[: max(1, topk)] if pool else []

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

        for robot in sorted(robots, key=lambda r: r.robot_id):
            current_assignment = current_assignments.get(robot.robot_id)
            ranked = self._reachable_macro_frontiers(robot, candidates, map_mgr, cfg, current_assignment)

            best_assignment = self._idle(robot.robot_id)
            best_score = float("-inf")
            best_breakdown: dict[str, float] = {}

            for ev in ranked:
                rep = ev.candidate.representative
                if rep in used_targets:
                    continue
                if ev.score > best_score:
                    best_score = float(ev.score)
                    best_breakdown = dict(ev.breakdown)
                    best_assignment = GoalAssignment(
                        robot_id=robot.robot_id,
                        target=rep,
                        path=list(ev.path),
                        utility=float(ev.score),
                        valid=True,
                        breakdown=dict(ev.breakdown),
                    )

            kept = keep_current_goal(
                robot=robot,
                current_assignment=current_assignment,
                best_new_score=best_score,
                map_mgr=map_mgr,
                cfg={
                    **cfg,
                    "planning": {
                        **cfg.get("planning", {}),
                        "active_slam_explorer": {
                            "commitment_margin": float(
                                cfg.get("planning", {}).get("macro_frontier_explorer", {}).get("commitment_margin", 1.2)
                            )
                        },
                    },
                },
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

        return PlannerOutput(
            planner_name=self.name,
            assignments=assignments,
            joint_score=float(joint_score if any(a.valid for a in assignments.values()) else float("-inf")),
            score_breakdown=score_breakdown,
            predicted_paths=predicted_paths,
            debug={
                "candidate_count": len(candidates),
                "selected_targets": {rid: a.target for rid, a in assignments.items() if a.valid},
                "mode": "macro_frontier_cluster_scoring",
            },
        )
