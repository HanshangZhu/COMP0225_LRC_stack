from __future__ import annotations

import numpy as np

from core.frontier_manager import build_frontier_candidates
from core.map_manager import FREE, OCCUPIED, MapManager
from core.types import GoalAssignment, PlannerInput, RobotState
from planners import build_planner


def _cfg() -> dict:
    return {
        "environment": {"map_name": "single_robot_unit"},
        "robots": {
            "num_robots": 1,
            "sensor_range": 5,
            "sensor_fov_deg": 360.0,
            "use_line_of_sight": True,
            "observation_miss_prob": 0.0,
            "clearance_cells": 0,
            "max_speed_cells_per_step": 1.0,
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
            "planner_name": "single_robot_frontier",
            "hysteresis_margin": 0.2,
            "weights": {"w_ig": 1.0, "w_cost": 0.45, "w_switch": 0.35, "w_turn": 0.05},
            "local_planner": {
                "type": "maze_aware_astar",
                "obstacle_cost_radius": 1,
                "obstacle_cost_weight": 0.08,
                "turn_cost_weight": 0.03,
                "clearance_probe_radius": 2,
                "clearance_bias_weight": 0.20,
            },
            "single_robot_frontier": {
                "min_goal_steps": 2,
                "preferred_goal_steps": 6,
                "max_cells_per_cluster_eval": 24,
                "w_cluster_size": 0.18,
                "w_goal_progress": 0.50,
            },
            "rollout": {"horizon": 1},
        },
        "predictor": {"type": "path_follow", "physics_residual": {"enabled": False}},
        "replanning": {
            "enable_event_replan": True,
            "periodic_replan_interval": 10,
            "frontier_change_threshold": 0.25,
            "stuck_threshold": 8,
            "invalidation_path_threshold": 3,
            "invalidation_distance_threshold": 2.0,
        },
        "termination": {"step_dt": 1.0},
    }


def _map_mgr() -> tuple[MapManager, RobotState]:
    truth = np.zeros((30, 30), dtype=np.int8)
    truth[0, :] = OCCUPIED
    truth[-1, :] = OCCUPIED
    truth[:, 0] = OCCUPIED
    truth[:, -1] = OCCUPIED
    truth[10:20, 15] = OCCUPIED
    truth[15, 15] = FREE

    m = MapManager(truth)
    robot = RobotState(robot_id=1, pose=(8, 20), heading_deg=90.0)
    rng = np.random.default_rng(0)
    m.observe_from(robot.pose, robot.heading_deg, 6, 360.0, True, 0.0, rng)
    return m, robot


def test_single_robot_frontier_planner_avoids_self_goal() -> None:
    cfg = _cfg()
    map_mgr, robot = _map_mgr()
    frontier_cells, candidates = build_frontier_candidates(map_mgr, cfg)

    assert frontier_cells
    assert candidates
    assert any(candidate.representative == robot.pose for candidate in candidates)

    planner = build_planner(cfg)
    out = planner.plan(
        PlannerInput(
            shared_map=map_mgr,
            robot_states=[robot],
            frontier_candidates=candidates,
            current_assignments={1: GoalAssignment(1, None, [], float("-inf"), False, {})},
            reservation_state={},
            step_idx=0,
            sim_time=0.0,
            config=cfg,
        )
    )

    assignment = out.assignments[1]
    assert assignment.valid
    assert assignment.target is not None
    assert assignment.target != robot.pose
    assert len(assignment.path) > 1
    assert assignment.breakdown["goal_steps"] >= 2.0
