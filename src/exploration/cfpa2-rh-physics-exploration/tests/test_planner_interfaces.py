from __future__ import annotations

import numpy as np

from core.frontier_manager import build_frontier_candidates
from core.map_manager import FREE, OCCUPIED, MapManager
from core.types import GoalAssignment, PlannerInput, RobotState
from planners import build_planner


def _cfg(planner_name: str) -> dict:
    return {
        "environment": {"map_name": "unit_test_map"},
        "robots": {
            "num_robots": 2,
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
            "target_frontier_count_max": 6,
            "representative_min_distance": 0.0,
            "ig_radius": 4,
        },
        "planning": {
            "planner_name": planner_name,
            "topk_candidate_limit": 6,
            "reassign_on_reach": True,
            "reservation_ttl": 8,
            "hysteresis_margin": 0.0,
            "frontier_policy": {"type": "frontiernet_proxy"},
            "local_planner": {
                "type": "cost_aware_astar",
                "obstacle_cost_radius": 1,
                "obstacle_cost_weight": 0.1,
                "turn_cost_weight": 0.05,
            },
            "weights": {"w_ig": 1.0, "w_cost": 0.4, "w_switch": 0.2, "w_turn": 0.0},
            "penalties": {"lambda_overlap": 0.5, "sigma_overlap": 8.0, "mu_interference": 0.1, "interference_distance": 2.5},
            "rollout": {
                "horizon": 4,
                "gamma": 0.9,
                "virtual_reveal_radius": 3,
                "virtual_gain_decay": 0.9,
                "rollout_weight": 0.05,
            },
        },
        "predictor": {
            "type": "path_follow",
            "horizon_steps": 6,
            "physics_residual": {"enabled": True, "weight_file": None, "residual_scale": 0.35},
            "constant_velocity": {"default_speed_cells_per_step": 0.8},
        },
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


def _map_mgr() -> MapManager:
    truth = np.zeros((30, 30), dtype=np.int8)
    truth[0, :] = OCCUPIED
    truth[-1, :] = OCCUPIED
    truth[:, 0] = OCCUPIED
    truth[:, -1] = OCCUPIED
    truth[14, 6:24] = OCCUPIED

    m = MapManager(truth)
    r1 = RobotState(robot_id=1, pose=(4, 4), heading_deg=0.0)
    r2 = RobotState(robot_id=2, pose=(8, 4), heading_deg=0.0)

    rng = np.random.default_rng(0)
    m.observe_from(r1.pose, r1.heading_deg, 6, 360.0, True, 0.0, rng)
    m.observe_from(r2.pose, r2.heading_deg, 6, 360.0, True, 0.0, rng)
    return m


def _planner_input(planner_name: str) -> PlannerInput:
    cfg = _cfg(planner_name)
    m = _map_mgr()
    robots = [RobotState(robot_id=1, pose=(4, 4), heading_deg=0.0), RobotState(robot_id=2, pose=(8, 4), heading_deg=0.0)]
    frontier_cells, candidates = build_frontier_candidates(m, cfg)
    assert frontier_cells
    assert candidates
    assignments = {1: GoalAssignment(1, None, [], float("-inf"), False, {}), 2: GoalAssignment(2, None, [], float("-inf"), False, {})}
    return PlannerInput(
        shared_map=m,
        robot_states=robots,
        frontier_candidates=candidates,
        current_assignments=assignments,
        reservation_state={},
        step_idx=0,
        sim_time=0.0,
        config=cfg,
    )


def test_cfpa2_planner_runs() -> None:
    planner_input = _planner_input("cfpa2")
    planner = build_planner(planner_input.config)
    out = planner.plan(planner_input)
    assert out.assignments
    assert out.planner_name == "cfpa2"


def test_active_slam_explorer_planner_runs() -> None:
    planner_input = _planner_input("active_slam_explorer")
    planner = build_planner(planner_input.config)
    out = planner.plan(planner_input)
    assert out.assignments
    assert out.planner_name == "active_slam_explorer"


def test_macro_frontier_explorer_planner_runs() -> None:
    planner_input = _planner_input("macro_frontier_explorer")
    planner = build_planner(planner_input.config)
    out = planner.plan(planner_input)
    assert out.assignments
    assert out.planner_name == "macro_frontier_explorer"


def test_rh_cfpa2_planner_runs() -> None:
    planner_input = _planner_input("rh_cfpa2")
    planner = build_planner(planner_input.config)
    out = planner.plan(planner_input)
    assert out.assignments
    assert out.planner_name == "rh_cfpa2"
    assert "predictor" in out.debug


def test_physics_rh_cfpa2_planner_runs() -> None:
    planner_input = _planner_input("physics_rh_cfpa2")
    planner = build_planner(planner_input.config)
    out = planner.plan(planner_input)
    assert out.assignments
    assert out.planner_name == "rh_cfpa2" or out.planner_name == "physics_rh_cfpa2"


def test_hybrid_explorer_planner_runs() -> None:
    planner_input = _planner_input("hybrid_explorer")
    planner = build_planner(planner_input.config)
    out = planner.plan(planner_input)
    assert out.assignments
    assert out.planner_name == "hybrid_explorer"
    assert "predictor" in out.debug
