from __future__ import annotations

from typing import Any

from core.assignment_solver import compute_candidate_utilities, solve_single_robot
from core.rollout_engine import rollout_pair_score
from core.types import GoalAssignment, PlannerInput, PlannerOutput
from core.utility_service import overlap_penalty, path_interference_penalty
from predictors import build_predictor

from .base_planner import BasePlanner


class RHCFPA2Planner(BasePlanner):
    name = "rh_cfpa2"

    def __init__(self, cfg: dict):
        self.predictor = build_predictor(cfg)

    def _topk(self, utilities: dict, k: int) -> list[tuple]:
        ranked = sorted(utilities.items(), key=lambda kv: kv[1].utility, reverse=True)
        return ranked[: max(1, k)]

    def _combine_score(self, immediate_score: float, future_score: float, rollout_cfg: dict) -> tuple[float, dict[str, float | str]]:
        mode = str(rollout_cfg.get("score_mode", "hybrid")).strip().lower()
        if mode == "immediate_only":
            return float(immediate_score), {
                "score_mode": mode,
                "immediate_weight": 1.0,
                "future_weight": 0.0,
            }
        if mode == "future_only":
            future_weight = float(rollout_cfg.get("future_only_weight", 1.0))
            return float(future_weight * future_score), {
                "score_mode": mode,
                "immediate_weight": 0.0,
                "future_weight": float(future_weight),
            }

        immediate_weight = float(rollout_cfg.get("immediate_weight", 1.0))
        future_weight = float(rollout_cfg.get("future_weight", rollout_cfg.get("rollout_weight", 0.05)))
        return float(immediate_weight * immediate_score + future_weight * future_score), {
            "score_mode": "hybrid",
            "immediate_weight": float(immediate_weight),
            "future_weight": float(future_weight),
        }

    def plan(self, planner_input: PlannerInput) -> PlannerOutput:
        robots = planner_input.robot_states
        map_mgr = planner_input.shared_map
        cfg = planner_input.config
        candidates = planner_input.frontier_candidates
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
                predicted_paths={robots[0].robot_id: a.path},
                debug={"predictor": self.predictor.name},
            )

        r1, r2 = robots[0], robots[1]
        u1 = compute_candidate_utilities(r1, candidates, cfg, map_mgr, reservations)
        u2 = compute_candidate_utilities(r2, candidates, cfg, map_mgr, reservations)

        if not u1 or not u2:
            a1 = solve_single_robot(r1, u1)
            a2 = solve_single_robot(r2, u2)
            if a1.valid and (not a2.valid or a1.utility >= a2.utility):
                assignments = {r1.robot_id: a1, r2.robot_id: GoalAssignment(r2.robot_id, None, [], float("-inf"), False, {})}
                return PlannerOutput(
                    planner_name=self.name,
                    assignments=assignments,
                    joint_score=float(a1.utility),
                    debug={"fallback": "robot1_only", "predictor": self.predictor.name},
                )
            assignments = {r1.robot_id: GoalAssignment(r1.robot_id, None, [], float("-inf"), False, {}), r2.robot_id: a2}
            return PlannerOutput(
                planner_name=self.name,
                assignments=assignments,
                joint_score=float(a2.utility if a2.valid else float("-inf")),
                debug={"fallback": "robot2_only", "predictor": self.predictor.name},
            )

        penalties = cfg["planning"]["penalties"]
        sensor_range = float(cfg["robots"]["sensor_range"])
        sigma = penalties.get("sigma_overlap")
        if sigma is None:
            sigma = 2.0 * sensor_range
        sigma = float(sigma)
        lambda_overlap = float(penalties.get("lambda_overlap", 0.0))
        mu_interference = float(penalties.get("mu_interference", 0.0))
        interference_distance = float(penalties.get("interference_distance", 2.5))

        topk = int(cfg["planning"].get("topk_candidate_limit", 8))
        ranked1 = self._topk(u1, topk)
        ranked2 = self._topk(u2, topk)
        rollout_cfg = cfg["planning"]["rollout"]
        score_mode = str(rollout_cfg.get("score_mode", "hybrid")).strip().lower()

        best_pair: tuple | None = None
        best_total = float("-inf")
        best_breakdown: dict[str, float] = {}
        best_predicted_paths: dict[int, list] = {}
        best_predictor_times: dict[int, float] = {}

        top_scored_pairs: list[dict[str, Any]] = []

        for fi, ev1 in ranked1:
            for fj, ev2 in ranked2:
                if fi == fj:
                    continue

                immediate_joint = float(ev1.utility + ev2.utility)
                overlap = overlap_penalty(fi, fj, sigma)
                interference = path_interference_penalty(ev1.path, ev2.path, distance_threshold=interference_distance)
                immediate_score = immediate_joint - lambda_overlap * overlap - mu_interference * interference

                rollout = rollout_pair_score(
                    map_mgr=map_mgr,
                    cfg=cfg,
                    robot1=r1,
                    robot2=r2,
                    goal1=fi,
                    goal2=fj,
                    path1=ev1.path,
                    path2=ev2.path,
                    candidates=candidates,
                    predictor=self.predictor,
                    reservation_state=reservations,
                )

                total_score, combine_breakdown = self._combine_score(
                    immediate_score=immediate_score,
                    future_score=rollout.future_score,
                    rollout_cfg=rollout_cfg,
                )
                top_scored_pairs.append(
                    {
                        "target_r1": tuple(fi),
                        "target_r2": tuple(fj),
                        "immediate_score": float(immediate_score),
                        "future_score": float(rollout.future_score),
                        "score_mode": combine_breakdown.get("score_mode", score_mode),
                        "immediate_weight": float(combine_breakdown.get("immediate_weight", 1.0)),
                        "future_weight": float(combine_breakdown.get("future_weight", 0.0)),
                        "total_score": float(total_score),
                    }
                )

                if total_score > best_total:
                    best_total = total_score
                    best_pair = (fi, fj, ev1, ev2)
                    best_breakdown = {
                        "immediate_joint": float(immediate_joint),
                        "overlap_penalty": float(overlap),
                        "interference_penalty": float(interference),
                        "immediate_score": float(immediate_score),
                        "future_score": float(rollout.future_score),
                        "immediate_weight": float(combine_breakdown.get("immediate_weight", 1.0)),
                        "future_weight": float(combine_breakdown.get("future_weight", 0.0)),
                        "total_score": float(total_score),
                    }
                    for k, v in rollout.breakdown.items():
                        best_breakdown[f"rollout_{k}"] = float(v)
                    best_predicted_paths = rollout.predicted_paths
                    best_predictor_times = rollout.predictor_inference_times

        if best_pair is None:
            a1 = solve_single_robot(r1, u1)
            a2 = solve_single_robot(r2, u2)
            return PlannerOutput(
                planner_name=self.name,
                assignments={r1.robot_id: a1, r2.robot_id: a2},
                joint_score=float((a1.utility if a1.valid else 0.0) + (a2.utility if a2.valid else 0.0)),
                debug={"fallback": "no_pair", "predictor": self.predictor.name},
            )

        fi, fj, ev1, ev2 = best_pair
        a1 = GoalAssignment(
            robot_id=r1.robot_id,
            target=fi,
            path=ev1.path,
            utility=float(ev1.utility),
            valid=True,
            breakdown={
                "ig": float(ev1.information_gain),
                "travel_cost": float(ev1.travel_cost),
                "switch_penalty": float(ev1.switch_penalty),
                "turn_penalty": float(ev1.turn_penalty),
            },
        )
        a2 = GoalAssignment(
            robot_id=r2.robot_id,
            target=fj,
            path=ev2.path,
            utility=float(ev2.utility),
            valid=True,
            breakdown={
                "ig": float(ev2.information_gain),
                "travel_cost": float(ev2.travel_cost),
                "switch_penalty": float(ev2.switch_penalty),
                "turn_penalty": float(ev2.turn_penalty),
            },
        )

        top_scored_pairs.sort(key=lambda x: x["total_score"], reverse=True)
        debug = {
            "predictor": self.predictor.name,
            "score_mode": score_mode,
            "predictor_inference_times": best_predictor_times,
            "top_pairs": top_scored_pairs[:5],
            "u1_count": len(u1),
            "u2_count": len(u2),
            "candidate_count": len(candidates),
        }

        return PlannerOutput(
            planner_name=self.name,
            assignments={r1.robot_id: a1, r2.robot_id: a2},
            joint_score=float(best_total),
            score_breakdown=best_breakdown,
            predicted_paths=best_predicted_paths,
            debug=debug,
        )
