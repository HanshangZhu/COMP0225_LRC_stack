from __future__ import annotations

import numpy as np

from core.map_manager import FREE, OCCUPIED, MapManager
from core.predictor_features import build_physics_feature_vector, extract_occupancy_patch, feature_dimension
from core.types import RobotState


def test_extract_patch_shape() -> None:
    truth = np.zeros((15, 15), dtype=np.int8)
    truth[0, :] = OCCUPIED
    truth[:, 0] = OCCUPIED
    m = MapManager(truth)
    m.known[:, :] = truth

    p = extract_occupancy_patch(m, center=(5, 5), patch_radius=3)
    assert p.shape == (7, 7)


def test_physics_feature_dim_matches_config() -> None:
    truth = np.zeros((20, 20), dtype=np.int8)
    m = MapManager(truth)
    m.known[:, :] = truth

    r1 = RobotState(robot_id=1, pose=(5, 5), heading_deg=30.0, velocity=(0.5, 0.0))
    r2 = RobotState(robot_id=2, pose=(8, 5), heading_deg=0.0, velocity=(0.0, 0.0))

    dim = feature_dimension(4)
    feat = build_physics_feature_vector(
        map_mgr=m,
        robot=r1,
        teammate=r2,
        goal=(10, 10),
        patch_radius=4,
        max_speed=1.0,
    )
    assert feat.shape == (dim,)
