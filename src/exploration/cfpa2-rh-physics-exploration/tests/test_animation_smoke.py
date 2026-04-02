from __future__ import annotations

import numpy as np

from core.animation_renderer import AnimationRenderer
from core.map_manager import MapManager
from core.types import FrontierCandidate, GoalAssignment, RobotState


def test_animation_renderer_smoke(tmp_path) -> None:
    truth = np.zeros((20, 20), dtype=np.int8)
    m = MapManager(truth)
    m.known[:, :] = truth

    cfg = {
        "experiment": {"enable_live_plot": False, "save_animation": True},
        "animation": {
            "fps": 4,
            "figsize": [8.0, 5.0],
            "show_frontier_cells": False,
            "show_sensor_fov": True,
            "save_gif": True,
            "save_mp4": False,
        },
    }

    renderer = AnimationRenderer(cfg)
    robots = [RobotState(robot_id=1, pose=(4, 4), heading_deg=0.0), RobotState(robot_id=2, pose=(8, 4), heading_deg=0.0)]
    candidates = [FrontierCandidate(representative=(10, 10), cells=[(10, 10), (11, 10)])]
    assignments = {
        1: GoalAssignment(robot_id=1, target=(10, 10), path=[(5, 5), (6, 6)], utility=1.0, valid=True),
        2: GoalAssignment(robot_id=2, target=(12, 10), path=[(9, 5), (10, 6)], utility=1.0, valid=True),
    }

    renderer.update(
        step_idx=0,
        map_mgr=m,
        robots=robots,
        frontier_cells=[(10, 10), (11, 10)],
        frontier_candidates=candidates,
        assignments=assignments,
        coverage=0.25,
        planner_name="cfpa2",
        seed=0,
        sim_time=0.0,
        replan_count=1,
        joint_score=1.23,
        last_replan_reason="initial",
        sensor_range=6,
        sensor_fov_deg=240.0,
    )

    gif_path, mp4_path = renderer.finalize(tmp_path / "anim_smoke")
    assert gif_path is not None
    assert mp4_path is None
