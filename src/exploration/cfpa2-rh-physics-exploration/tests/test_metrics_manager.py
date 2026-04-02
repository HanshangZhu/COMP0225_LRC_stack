from __future__ import annotations

from core.metrics_manager import EpisodeMetrics
from core.types import RobotState


def test_metrics_summary_fields() -> None:
    m = EpisodeMetrics(planner_name="rh_cfpa2", map_name="maze", seed=0, rollout_horizon=5, predictor_type="path_follow")
    m.log_step(0, 0.0, 0.1, 20, 4, 1.2)
    m.log_replan("initial", 0.01)
    m.log_predictor_times({1: 0.001, 2: 0.002})
    m.log_decision_probe(
        base_predictor="path_follow",
        decision_signatures={
            "path_follow": ((1, (3, 3)), (2, (6, 6))),
            "physics_residual": ((1, (3, 3)), (2, (8, 6))),
        },
        predictor_scores={"path_follow": 12.0, "physics_residual": 11.2},
    )

    r1 = RobotState(robot_id=1, pose=(1, 1), heading_deg=0.0)
    r2 = RobotState(robot_id=2, pose=(2, 1), heading_deg=0.0)
    r1.total_move_steps = 10
    r2.total_move_steps = 11
    r1.revisited_move_steps = 2
    r2.revisited_move_steps = 3
    r1.path_length = 12.0
    r2.path_length = 13.5

    m.finalize([r1, r2], steps=21, sim_time=21.0, success=True, reason="coverage_reached")
    row = m.to_summary_row()

    assert row["success"] is True
    assert row["completion_steps"] == 21
    assert row["total_path_length"] > 0
    assert "prediction_error_by_horizon" in row
    assert "decision_divergence_rate" in row
    assert row["decision_probe_pair_count"] >= 1
