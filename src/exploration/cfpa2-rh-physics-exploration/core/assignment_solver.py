from __future__ import annotations

from typing import Any

from .types import Cell, FrontierCandidate, GoalAssignment, RobotState
from .utility_service import CandidateEvaluation, evaluate_candidate, overlap_penalty, path_interference_penalty


def _idle_assignment(robot_id: int) -> GoalAssignment:
    return GoalAssignment(robot_id=robot_id, target=None, path=[], utility=float("-inf"), valid=False, breakdown={})


def _allowed_by_reservation(candidate: Cell, robot_id: int, reservation_state: dict[Cell, dict[str, int]]) -> bool:
    if not reservation_state:
        return True
    entry = reservation_state.get(candidate)
    if entry is None:
        return True
    return int(entry.get("robot_id", -1)) == robot_id


def compute_candidate_utilities(
    robot: RobotState,
    candidates: list[FrontierCandidate],
    cfg: dict,
    map_mgr,
    reservation_state: dict[Cell, dict[str, int]],
) -> dict[Cell, CandidateEvaluation]:
    neighborhood = int(cfg["frontier"].get("neighborhood", 8))
    out: dict[Cell, CandidateEvaluation] = {}
    for c in candidates:
        rep = c.representative
        if not _allowed_by_reservation(rep, robot.robot_id, reservation_state):
            continue
        ev = evaluate_candidate(robot, rep, map_mgr, cfg, neighborhood=neighborhood, candidate=c, all_candidates=candidates)
        if ev is None:
            continue
        out[rep] = ev
    return out


def solve_single_robot(robot: RobotState, utilities: dict[Cell, CandidateEvaluation]) -> GoalAssignment:
    if not utilities:
        return _idle_assignment(robot.robot_id)
    target, ev = max(utilities.items(), key=lambda kv: kv[1].utility)
    return GoalAssignment(
        robot_id=robot.robot_id,
        target=target,
        path=ev.path,
        utility=float(ev.utility),
        valid=True,
        breakdown={
            "ig": float(ev.information_gain),
            "travel_cost": float(ev.travel_cost),
            "switch_penalty": float(ev.switch_penalty),
            "turn_penalty": float(ev.turn_penalty),
        },
    )


def solve_joint_cfpa2(
    robot1: RobotState,
    robot2: RobotState,
    u1: dict[Cell, CandidateEvaluation],
    u2: dict[Cell, CandidateEvaluation],
    cfg: dict,
) -> tuple[dict[int, GoalAssignment], float, dict[str, float], dict[str, Any]]:
    if not u1 and not u2:
        return {robot1.robot_id: _idle_assignment(robot1.robot_id), robot2.robot_id: _idle_assignment(robot2.robot_id)}, float("-inf"), {}, {"fallback": "none"}

    penalties = cfg["planning"]["penalties"]
    sensor_range = float(cfg["robots"]["sensor_range"])
    sigma = penalties.get("sigma_overlap")
    if sigma is None:
        sigma = 2.0 * sensor_range
    sigma = float(sigma)

    lambda_overlap = float(penalties.get("lambda_overlap", 0.0))
    mu_interference = float(penalties.get("mu_interference", 0.0))
    interference_distance = float(penalties.get("interference_distance", 2.5))

    best_pair: tuple[Cell, Cell] | None = None
    best_score = float("-inf")
    best_breakdown: dict[str, float] = {}

    top_pairs: list[tuple[float, Cell, Cell]] = []

    for fi, ev1 in u1.items():
        for fj, ev2 in u2.items():
            if fi == fj:
                continue
            overlap = overlap_penalty(fi, fj, sigma)
            interference = path_interference_penalty(ev1.path, ev2.path, distance_threshold=interference_distance)
            joint_utility = float(ev1.utility + ev2.utility)
            score = joint_utility - lambda_overlap * overlap - mu_interference * interference
            top_pairs.append((score, fi, fj))
            if score > best_score:
                best_score = score
                best_pair = (fi, fj)
                best_breakdown = {
                    "joint_utility": joint_utility,
                    "overlap_penalty": overlap,
                    "interference_penalty": interference,
                    "score": score,
                }

    if best_pair is not None:
        fi, fj = best_pair
        a1 = GoalAssignment(
            robot_id=robot1.robot_id,
            target=fi,
            path=u1[fi].path,
            utility=float(u1[fi].utility),
            valid=True,
            breakdown={
                "ig": float(u1[fi].information_gain),
                "travel_cost": float(u1[fi].travel_cost),
                "switch_penalty": float(u1[fi].switch_penalty),
                "turn_penalty": float(u1[fi].turn_penalty),
            },
        )
        a2 = GoalAssignment(
            robot_id=robot2.robot_id,
            target=fj,
            path=u2[fj].path,
            utility=float(u2[fj].utility),
            valid=True,
            breakdown={
                "ig": float(u2[fj].information_gain),
                "travel_cost": float(u2[fj].travel_cost),
                "switch_penalty": float(u2[fj].switch_penalty),
                "turn_penalty": float(u2[fj].turn_penalty),
            },
        )
        top_pairs.sort(key=lambda x: x[0], reverse=True)
        debug = {
            "top_pairs": [
                {"score": float(s), "target_r1": tuple(fi_), "target_r2": tuple(fj_)}
                for s, fi_, fj_ in top_pairs[:5]
            ]
        }
        return {robot1.robot_id: a1, robot2.robot_id: a2}, float(best_score), best_breakdown, debug

    # fallback to one robot if only one side has feasible candidates
    a1 = solve_single_robot(robot1, u1)
    a2 = solve_single_robot(robot2, u2)
    if a1.valid and (not a2.valid or a1.utility >= a2.utility):
        return {robot1.robot_id: a1, robot2.robot_id: _idle_assignment(robot2.robot_id)}, float(a1.utility), {"fallback": 1.0}, {"fallback": "robot1_only"}
    if a2.valid:
        return {robot1.robot_id: _idle_assignment(robot1.robot_id), robot2.robot_id: a2}, float(a2.utility), {"fallback": 1.0}, {"fallback": "robot2_only"}

    return {robot1.robot_id: _idle_assignment(robot1.robot_id), robot2.robot_id: _idle_assignment(robot2.robot_id)}, float("-inf"), {}, {"fallback": "none"}


def tick_reservations(reservation_state: dict[Cell, dict[str, int]]) -> None:
    expired: list[Cell] = []
    for cell, entry in reservation_state.items():
        entry["ttl"] = int(entry.get("ttl", 0)) - 1
        if entry["ttl"] <= 0:
            expired.append(cell)
    for c in expired:
        reservation_state.pop(c, None)


def update_reservations(reservation_state: dict[Cell, dict[str, int]], assignments: dict[int, GoalAssignment], ttl: int) -> None:
    tick_reservations(reservation_state)
    for a in assignments.values():
        if not a.valid or a.target is None:
            continue
        reservation_state[a.target] = {"robot_id": int(a.robot_id), "ttl": int(ttl)}
