from __future__ import annotations

from core.config import load_experiment_config
from planners import build_planner


def test_single_robot_config_builds_cfpa2() -> None:
    cfg = load_experiment_config(
        "configs/base_single_robot.yaml",
        planner_cfg_path="configs/planner_cfpa2.yaml",
        env_cfg_path="configs/env_narrow_t_branches_single_robot.yaml",
    )
    assert int(cfg["robots"]["num_robots"]) == 1
    assert len(cfg["robots"]["start_positions"]) == 1
    planner = build_planner(cfg)
    assert planner is not None


def test_single_robot_config_builds_rh() -> None:
    cfg = load_experiment_config(
        "configs/base_single_robot.yaml",
        planner_cfg_path="configs/planner_rh_cfpa2.yaml",
        env_cfg_path="configs/env_narrow_t_dense_branches_single_robot.yaml",
    )
    assert int(cfg["robots"]["num_robots"]) == 1
    planner = build_planner(cfg)
    assert planner is not None


def test_single_robot_config_builds_single_robot_frontier() -> None:
    cfg = load_experiment_config(
        "configs/base_single_robot.yaml",
        planner_cfg_path="configs/planner_single_robot_frontier.yaml",
        env_cfg_path="configs/env_narrow_t_branches_single_robot.yaml",
    )
    assert int(cfg["robots"]["num_robots"]) == 1
    planner = build_planner(cfg)
    assert planner is not None
