from __future__ import annotations

import numpy as np

from core.map_manager import MapManager
from core.rollout_engine import rollout_pair_score
from core.types import FrontierCandidate, RobotState
from predictors.path_follow_predictor import PathFollowPredictor


def _cfg() -> dict:
    return {
        "robots": {"sensor_range": 6, "max_speed_cells_per_step": 1.0},
        "planning": {
            "reassign_on_reach": True,
            "penalties": {"lambda_overlap": 0.5, "sigma_overlap": 8.0},
            "rollout": {
                "horizon": 5,
                "gamma": 0.9,
                "virtual_reveal_radius": 3,
                "virtual_gain_decay": 0.92,
            },
        },
        "predictor": {"horizon_steps": 6},
        "termination": {"step_dt": 1.0},
    }


def test_rollout_returns_future_score() -> None:
    truth = np.zeros((25, 25), dtype=np.int8)
    m = MapManager(truth)
    m.known[0:5, :] = 0
    r1 = RobotState(robot_id=1, pose=(5, 10), heading_deg=0.0)
    r2 = RobotState(robot_id=2, pose=(15, 10), heading_deg=180.0)

    candidates = [
        FrontierCandidate(representative=(5, 6), cells=[(5, 6), (6, 6)]),
        FrontierCandidate(representative=(15, 6), cells=[(15, 6), (14, 6)]),
    ]

    out = rollout_pair_score(
        map_mgr=m,
        cfg=_cfg(),
        robot1=r1,
        robot2=r2,
        goal1=(5, 6),
        goal2=(15, 6),
        path1=[(5, 10), (5, 9), (5, 8), (5, 7), (5, 6)],
        path2=[(15, 10), (15, 9), (15, 8), (15, 7), (15, 6)],
        candidates=candidates,
        predictor=PathFollowPredictor(),
    )

    assert isinstance(out.future_score, float)
    assert out.predicted_paths[1]
    assert out.predicted_paths[2]
