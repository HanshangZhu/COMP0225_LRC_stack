from __future__ import annotations

import numpy as np

from core.predictor_features import feature_dimension
from core.types import PredictorInput, RobotState
from predictors.physics_residual_predictor import PhysicsResidualPredictor


def test_physics_predictor_npz_backend(tmp_path) -> None:
    dim = feature_dimension(4)
    w = np.zeros((2, dim), dtype=np.float32)
    b = np.zeros((2,), dtype=np.float32)
    w_path = tmp_path / "weights.npz"
    np.savez(w_path, w=w, b=b)

    pred = PhysicsResidualPredictor(enabled=True, weight_file=str(w_path), residual_scale=0.35, occupancy_patch_radius=4)

    robot = RobotState(robot_id=1, pose=(3, 3), heading_deg=0.0, velocity=(0.0, 0.0))
    local_context = {
        "max_speed_cells_per_step": 1.0,
        "local_obstacle_density": 0.2,
        "teammate_distance": 5.0,
        "robot_velocity": (0.0, 0.0),
        "teammate_velocity": (0.0, 0.0),
        "occupancy_patch_flat": [0.0] * ((2 * 4 + 1) ** 2),
    }

    out = pred.predict(
        PredictorInput(
            robot_state=robot,
            goal=(8, 8),
            current_path=[(4, 4), (5, 5)],
            local_context=local_context,
            horizon_steps=5,
            step_dt=1.0,
        )
    )

    assert out.trajectory
    assert len(out.uncertainty) == len(out.trajectory)
    assert all(u >= 0.0 for u in out.uncertainty)
    assert out.debug.get("backend") in ("npz_linear", "analytic", "torch_mlp")
