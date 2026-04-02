from __future__ import annotations

import numpy as np

from core.frontier_manager import build_frontier_candidates
from core.map_manager import MapManager, OCCUPIED
from core.types import GoalAssignment, PlannerInput, RobotState
from planners import build_planner


def _cfg() -> dict:
    return {
        "robots": {
            "num_robots": 1,
            "sensor_range": 6,
            "sensor_fov_deg": 360.0,
            "use_line_of_sight": True,
            "observation_miss_prob": 0.0,
            "clearance_cells": 0,
        },
        "frontier": {
            "neighborhood": 8,
            "min_cluster_size": 1,
            "target_frontier_count_min": 1,
            "target_frontier_count_max": 8,
            "representative_min_distance": 0.0,
            "ig_radius": 4,
        },
        "planning": {
            "planner_name": "active_slam_explorer",
            "topk_candidate_limit": 8,
            "local_planner": {
                "type": "maze_aware_astar",
                "obstacle_cost_radius": 1,
                "obstacle_cost_weight": 0.08,
                "turn_cost_weight": 0.03,
                "clearance_probe_radius": 3,
                "clearance_bias_weight": 0.30,
            },
            "active_slam_explorer": {
                "candidate_topk": 8,
                "information_gain_radius": 4,
                "min_information_gain": 1.0,
                "commitment_margin": 5.0,
            },
        },
    }


def test_active_slam_explorer_keeps_current_goal_when_margin_not_beaten() -> None:
    truth = np.zeros((20, 20), dtype=np.int8)
    truth[0, :] = OCCUPIED
    truth[-1, :] = OCCUPIED
    truth[:, 0] = OCCUPIED
    truth[:, -1] = OCCUPIED

    m = MapManager(truth)
    robot = RobotState(robot_id=1, pose=(4, 4), heading_deg=0.0)
    rng = np.random.default_rng(0)
    m.observe_from(robot.pose, robot.heading_deg, 6, 360.0, True, 0.0, rng)

    cfg = _cfg()
    _, candidates = build_frontier_candidates(m, cfg)
    assert candidates

    current = GoalAssignment(
        robot_id=1,
        target=candidates[0].representative,
        path=[robot.pose, candidates[0].representative],
        utility=100.0,
        valid=True,
        breakdown={"score": 100.0},
    )
    planner_input = PlannerInput(
        shared_map=m,
        robot_states=[robot],
        frontier_candidates=candidates,
        current_assignments={1: current},
        reservation_state={},
        step_idx=0,
        sim_time=0.0,
        config=cfg,
    )
    planner = build_planner(cfg)
    out = planner.plan(planner_input)
    assert out.assignments[1].target == current.target
