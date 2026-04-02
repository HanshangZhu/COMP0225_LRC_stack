from __future__ import annotations

import numpy as np

from core.map_manager import MapManager, OCCUPIED
from core.path_service import astar_path, plan_path


def test_astar_blocks_diagonal_corner_cutting() -> None:
    truth = np.zeros((6, 6), dtype=np.int8)
    truth[0, :] = OCCUPIED
    truth[-1, :] = OCCUPIED
    truth[:, 0] = OCCUPIED
    truth[:, -1] = OCCUPIED
    truth[1, 2] = OCCUPIED
    truth[2, 1] = OCCUPIED

    m = MapManager(truth)
    m.known[:, :] = truth

    path = astar_path(m, start=(1, 1), goal=(2, 2), neighborhood=8, clearance_cells=0)
    assert path is None


def test_maze_aware_planner_returns_valid_path() -> None:
    truth = np.zeros((12, 12), dtype=np.int8)
    truth[0, :] = OCCUPIED
    truth[-1, :] = OCCUPIED
    truth[:, 0] = OCCUPIED
    truth[:, -1] = OCCUPIED
    truth[5, 2:9] = OCCUPIED
    truth[5, 6] = 0

    m = MapManager(truth)
    m.known[:, :] = truth

    cfg = {
        "planning": {
            "local_planner": {
                "type": "maze_aware_astar",
                "obstacle_cost_radius": 1,
                "obstacle_cost_weight": 0.08,
                "turn_cost_weight": 0.03,
                "clearance_probe_radius": 3,
                "clearance_bias_weight": 0.30,
            }
        }
    }

    path = plan_path(m, start=(2, 2), goal=(9, 9), cfg=cfg, neighborhood=8, clearance_cells=0)
    assert path is not None
    assert path[0] == (2, 2)
    assert path[-1] == (9, 9)
