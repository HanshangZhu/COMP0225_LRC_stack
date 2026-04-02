from __future__ import annotations

import argparse

import pandas as pd

from core.config import load_experiment_config, write_config_snapshot
from experiments.common import enforce_mp4_only, git_commit_hash, make_run_id, prepare_output_dirs, save_run_metadata
from simulators.grid_sim import GridSimulation

PLANNER_CFG = {
    "cfpa2": "configs/planner_cfpa2.yaml",
    "active_slam_explorer": "configs/planner_active_slam_explorer.yaml",
    "macro_frontier_explorer": "configs/planner_macro_frontier_explorer.yaml",
    "rh_cfpa2": "configs/planner_rh_cfpa2.yaml",
    "physics_rh_cfpa2": "configs/planner_physics_rh_cfpa2.yaml",
    "hybrid_explorer": "configs/planner_hybrid_explorer.yaml",
    "single_robot_frontier": "configs/planner_single_robot_frontier.yaml",
}

ENV_CFG = {
    "maze": "configs/env_maze.yaml",
    "narrow_t_branches": "configs/env_narrow_t_branches.yaml",
    "narrow_t_dense_branches": "configs/env_narrow_t_dense_branches.yaml",
    "narrow_t_asymmetric_branches": "configs/env_narrow_t_asymmetric_branches.yaml",
    "narrow_t_loop_branches": "configs/env_narrow_t_loop_branches.yaml",
}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Unified multi-robot exploration entrypoint")
    parser.add_argument("--base-config", type=str, default="configs/base.yaml")
    parser.add_argument(
        "--planner",
        type=str,
        default="cfpa2",
        choices=["cfpa2", "active_slam_explorer", "macro_frontier_explorer", "rh_cfpa2", "physics_rh_cfpa2", "hybrid_explorer", "single_robot_frontier"],
    )
    parser.add_argument("--planner-config", type=str, default=None)
    parser.add_argument(
        "--env",
        type=str,
        default="narrow_t_branches",
        choices=["maze", "narrow_t_branches", "narrow_t_dense_branches", "narrow_t_asymmetric_branches", "narrow_t_loop_branches"],
        help="Named environment preset; ignored when --env-config is provided.",
    )
    parser.add_argument("--env-config", type=str, default=None, help="Direct environment config path override")
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--run-id", type=str, default=None)
    parser.add_argument("--output-root", type=str, default="outputs")
    parser.add_argument("--max-steps", type=int, default=None)
    parser.add_argument("--disable-animation", action="store_true")
    parser.add_argument("--physics-weight-file", type=str, default=None)
    return parser.parse_args()


def main() -> None:
    args = parse_args()

    planner_cfg = args.planner_config or PLANNER_CFG[args.planner]
    env_cfg = args.env_config or ENV_CFG[args.env]

    cfg = load_experiment_config(args.base_config, planner_cfg_path=planner_cfg, env_cfg_path=env_cfg)
    cfg = enforce_mp4_only(cfg)
    cfg["planning"]["planner_name"] = args.planner

    if args.planner == "physics_rh_cfpa2" and args.physics_weight_file is not None:
        cfg["predictor"]["type"] = "physics_residual"
        cfg["predictor"]["physics_residual"]["enabled"] = True
        cfg["predictor"]["physics_residual"]["weight_file"] = args.physics_weight_file
    if args.max_steps is not None:
        cfg["termination"]["max_steps"] = int(args.max_steps)
    if args.disable_animation:
        cfg["experiment"]["save_animation"] = False

    run_id = args.run_id or make_run_id("single")
    dirs = prepare_output_dirs(args.output_root, run_id)

    write_config_snapshot(dirs["configs"] / "resolved_config.yaml", cfg)
    save_run_metadata(
        dirs["metadata"] / "run_metadata.json",
        {
            "run_id": run_id,
            "planner": args.planner,
            "env": args.env,
            "env_config": env_cfg,
            "planner_config": planner_cfg,
            "seed": args.seed,
            "git_commit": git_commit_hash(),
        },
    )

    sim = GridSimulation()
    map_name = cfg["environment"].get("map_name", cfg["environment"].get("map_type", "map"))
    stem = f"{args.planner}_{map_name}_seed{args.seed}"
    episode_dir = dirs["episode"] / stem

    result = sim.run_episode(
        cfg=cfg,
        planner_name=args.planner,
        seed=args.seed,
        output_dir=episode_dir,
        animation_stem=stem,
    )

    row = dict(result.summary)
    row.update(
        {
            "run_id": run_id,
            "coverage_csv": result.coverage_csv_path,
            "step_log_csv": result.step_log_csv_path,
            "animation_gif": result.animation_gif_path,
            "animation_mp4": result.animation_mp4_path,
        }
    )

    summary_csv = dirs["results_csv"] / "single_run_summary.csv"
    pd.DataFrame([row]).to_csv(summary_csv, index=False)

    print("=== Unified Main Summary ===")
    for k, v in row.items():
        print(f"{k}: {v}")
    print(f"summary_csv: {summary_csv}")


if __name__ == "__main__":
    main()
