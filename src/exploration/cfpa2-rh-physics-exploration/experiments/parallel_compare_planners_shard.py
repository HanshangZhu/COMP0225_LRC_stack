from __future__ import annotations

import argparse
import sys
from pathlib import Path

import pandas as pd

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from core.config import load_experiment_config, write_config_snapshot
from experiments.common import enforce_mp4_only, git_commit_hash, prepare_output_dirs, save_run_metadata
from simulators.grid_sim import GridSimulation

PLANNER_CFG = {
    "cfpa2": "configs/planner_cfpa2.yaml",
    "rh_cfpa2": "configs/planner_rh_cfpa2.yaml",
    "physics_rh_cfpa2": "configs/planner_physics_rh_cfpa2.yaml",
}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Run one shard of planner comparison cases")
    parser.add_argument("--base-config", type=str, default="configs/base.yaml")
    parser.add_argument("--planners", nargs="+", default=["cfpa2", "rh_cfpa2", "physics_rh_cfpa2"], choices=["cfpa2", "rh_cfpa2", "physics_rh_cfpa2"])
    parser.add_argument(
        "--env-configs",
        nargs="+",
        default=[
            "configs/env_maze.yaml",
            "configs/env_narrow_t_branches.yaml",
            "configs/env_narrow_t_asymmetric_branches.yaml",
            "configs/env_narrow_t_loop_branches.yaml",
        ],
    )
    parser.add_argument("--seed-start", type=int, default=0)
    parser.add_argument("--num-seeds", type=int, default=2)
    parser.add_argument("--max-steps", type=int, default=None)
    parser.add_argument("--run-id", type=str, required=True)
    parser.add_argument("--output-root", type=str, default="outputs")
    parser.add_argument("--animate-first-shard-case-only", action="store_true")
    parser.add_argument("--disable-animation", action="store_true")
    parser.add_argument("--physics-weight-file", type=str, default=None)
    parser.add_argument("--task-index", type=int, required=True)
    parser.add_argument("--num-tasks", type=int, required=True)
    return parser.parse_args()


def _build_cases(args: argparse.Namespace) -> list[tuple[int, str, str, int]]:
    cases: list[tuple[int, str, str, int]] = []
    combo_idx = 0
    for env_cfg_path in args.env_configs:
        for planner_name in args.planners:
            for seed in range(args.seed_start, args.seed_start + args.num_seeds):
                cases.append((combo_idx, env_cfg_path, planner_name, seed))
                combo_idx += 1
    return cases


def main() -> None:
    args = parse_args()
    if args.task_index < 0 or args.task_index >= args.num_tasks:
        raise ValueError(f"task-index {args.task_index} out of range for num-tasks={args.num_tasks}")

    dirs = prepare_output_dirs(args.output_root, args.run_id)
    shard_dir = dirs["results_csv"] / "shards"
    shard_dir.mkdir(parents=True, exist_ok=True)

    if args.task_index == 0:
        save_run_metadata(
            dirs["metadata"] / "run_metadata.json",
            {
                "run_id": args.run_id,
                "base_config": args.base_config,
                "planners": args.planners,
                "env_configs": args.env_configs,
                "seed_start": args.seed_start,
                "num_seeds": args.num_seeds,
                "num_tasks": args.num_tasks,
                "git_commit": git_commit_hash(),
            },
        )

    sim = GridSimulation()
    rows: list[dict] = []
    all_cases = _build_cases(args)
    selected = [case for case in all_cases if case[0] % args.num_tasks == args.task_index]

    print(f"[compare_planners_shard] task={args.task_index}/{args.num_tasks} selected_cases={len(selected)}", flush=True)

    for combo_idx, env_cfg_path, planner_name, seed in selected:
        env_label = Path(env_cfg_path).stem
        planner_cfg_path = PLANNER_CFG[planner_name]
        cfg = load_experiment_config(args.base_config, planner_cfg_path=planner_cfg_path, env_cfg_path=env_cfg_path)
        cfg = enforce_mp4_only(cfg)
        cfg["planning"]["planner_name"] = planner_name
        if planner_name == "physics_rh_cfpa2" and args.physics_weight_file is not None:
            cfg["predictor"]["type"] = "physics_residual"
            cfg["predictor"]["physics_residual"]["enabled"] = True
            cfg["predictor"]["physics_residual"]["weight_file"] = args.physics_weight_file
        if args.max_steps is not None:
            cfg["termination"]["max_steps"] = int(args.max_steps)

        if args.task_index == 0:
            snapshot_path = dirs["configs"] / f"resolved_{env_label}_{planner_name}.yaml"
            if not snapshot_path.exists():
                write_config_snapshot(snapshot_path, cfg)

        cfg_local = dict(cfg)
        cfg_local["experiment"] = dict(cfg.get("experiment", {}))
        if args.disable_animation:
            cfg_local["experiment"]["save_animation"] = False
        elif args.animate_first_shard_case_only:
            cfg_local["experiment"]["save_animation"] = bool(combo_idx == 0)

        map_name = cfg_local["environment"].get("map_name", cfg_local["environment"].get("map_type", env_label))
        stem = f"{planner_name}_{map_name}_seed{seed}"
        episode_dir = dirs["episode"] / map_name / planner_name / f"seed_{seed}"

        result = sim.run_episode(
            cfg=cfg_local,
            planner_name=planner_name,
            seed=seed,
            output_dir=episode_dir,
            animation_stem=stem,
        )

        row = dict(result.summary)
        row.update(
            {
                "run_id": args.run_id,
                "env_config": env_cfg_path,
                "planner_config": planner_cfg_path,
                "coverage_csv": result.coverage_csv_path,
                "step_log_csv": result.step_log_csv_path,
                "animation_gif": result.animation_gif_path,
                "animation_mp4": result.animation_mp4_path,
            }
        )
        rows.append(row)

        print(
            f"task={args.task_index} planner={planner_name} map={map_name} seed={seed} "
            f"success={row['success']} steps={row['completion_steps']} "
            f"coverage={row['final_coverage']:.3f}",
            flush=True,
        )

    shard_csv = shard_dir / f"compare_planners_results_task{args.task_index:03d}.csv"
    pd.DataFrame(rows).to_csv(shard_csv, index=False)
    print(f"shard_csv: {shard_csv}", flush=True)


if __name__ == "__main__":
    main()
