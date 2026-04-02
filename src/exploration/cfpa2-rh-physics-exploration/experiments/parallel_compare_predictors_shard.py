from __future__ import annotations

import argparse
import sys

import pandas as pd

from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from core.config import load_experiment_config, write_config_snapshot
from experiments.common import enforce_mp4_only, git_commit_hash, prepare_output_dirs, save_run_metadata
from simulators.grid_sim import GridSimulation

PLANNER_CFG = {
    "rh_cfpa2": "configs/planner_rh_cfpa2.yaml",
    "physics_rh_cfpa2": "configs/planner_physics_rh_cfpa2.yaml",
}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Run one shard of predictor comparison cases")
    parser.add_argument("--base-config", type=str, default="configs/base.yaml")
    parser.add_argument("--env-config", type=str, default="configs/env_maze.yaml")
    parser.add_argument("--planners", nargs="+", default=["rh_cfpa2", "physics_rh_cfpa2"], choices=["rh_cfpa2", "physics_rh_cfpa2"])
    parser.add_argument("--predictors", nargs="+", default=["path_follow", "constant_velocity", "physics_residual"], choices=["path_follow", "constant_velocity", "physics_residual"])
    parser.add_argument("--rollout-horizons", nargs="+", type=int, default=[3, 5, 7])
    parser.add_argument("--seed-start", type=int, default=0)
    parser.add_argument("--num-seeds", type=int, default=2)
    parser.add_argument("--max-steps", type=int, default=None)
    parser.add_argument("--run-id", type=str, required=True)
    parser.add_argument("--output-root", type=str, default="outputs")
    parser.add_argument("--disable-animation", action="store_true")
    parser.add_argument("--physics-weight-file", type=str, default=None)
    parser.add_argument("--disable-decision-probe", action="store_true")
    parser.add_argument("--decision-probe-max-replans", type=int, default=40)
    parser.add_argument("--task-index", type=int, required=True)
    parser.add_argument("--num-tasks", type=int, required=True)
    return parser.parse_args()


def _build_cases(args: argparse.Namespace) -> list[tuple[int, str, str, int, int]]:
    cases: list[tuple[int, str, str, int, int]] = []
    combo_idx = 0
    for planner_name in args.planners:
        for predictor_name in args.predictors:
            if planner_name == "physics_rh_cfpa2" and predictor_name != "physics_residual":
                continue
            for horizon in args.rollout_horizons:
                for seed in range(args.seed_start, args.seed_start + args.num_seeds):
                    cases.append((combo_idx, planner_name, predictor_name, int(horizon), seed))
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
                "env_config": args.env_config,
                "planners": args.planners,
                "predictors": args.predictors,
                "rollout_horizons": args.rollout_horizons,
                "seed_start": args.seed_start,
                "num_seeds": args.num_seeds,
                "num_tasks": args.num_tasks,
                "enable_decision_probe": not bool(args.disable_decision_probe),
                "decision_probe_max_replans": int(args.decision_probe_max_replans),
                "git_commit": git_commit_hash(),
            },
        )

    sim = GridSimulation()
    rows: list[dict] = []
    all_cases = _build_cases(args)
    selected = [case for case in all_cases if case[0] % args.num_tasks == args.task_index]

    print(f"[compare_predictors_shard] task={args.task_index}/{args.num_tasks} selected_cases={len(selected)}", flush=True)

    base_cfg_cache: dict[str, dict] = {}

    for combo_idx, planner_name, predictor_name, horizon, seed in selected:
        planner_cfg = PLANNER_CFG[planner_name]
        if planner_name not in base_cfg_cache:
            base_cfg = load_experiment_config(args.base_config, planner_cfg_path=planner_cfg, env_cfg_path=args.env_config)
            base_cfg = enforce_mp4_only(base_cfg)
            base_cfg["planning"]["planner_name"] = planner_name
            if args.max_steps is not None:
                base_cfg["termination"]["max_steps"] = int(args.max_steps)
            base_cfg_cache[planner_name] = base_cfg
        base_cfg = base_cfg_cache[planner_name]

        cfg = dict(base_cfg)
        cfg["predictor"] = dict(base_cfg.get("predictor", {}))
        cfg["predictor"]["type"] = predictor_name
        if predictor_name == "physics_residual":
            phy_cfg = dict(cfg["predictor"].get("physics_residual", {}))
            phy_cfg["enabled"] = True
            if args.physics_weight_file is not None:
                phy_cfg["weight_file"] = args.physics_weight_file
            cfg["predictor"]["physics_residual"] = phy_cfg
        cfg["planning"] = dict(base_cfg["planning"])
        cfg["planning"]["rollout"] = dict(base_cfg["planning"]["rollout"])
        cfg["planning"]["rollout"]["horizon"] = int(horizon)
        cfg["analysis"] = dict(base_cfg.get("analysis", {}))
        if not args.disable_decision_probe:
            probe_predictors = sorted(set([str(p) for p in args.predictors] + ["path_follow", "physics_residual"]))
            cfg["analysis"]["enable_predictor_decision_probe"] = True
            cfg["analysis"]["decision_probe_predictors"] = probe_predictors
            cfg["analysis"]["decision_probe_max_per_episode"] = int(args.decision_probe_max_replans)
        cfg["experiment"] = dict(base_cfg.get("experiment", {}))
        if args.disable_animation:
            cfg["experiment"]["save_animation"] = False

        if args.task_index == 0:
            tag = f"{planner_name}_{predictor_name}_h{horizon}"
            snapshot_path = dirs["configs"] / f"resolved_{tag}.yaml"
            if not snapshot_path.exists():
                write_config_snapshot(snapshot_path, cfg)

        map_name = cfg["environment"].get("map_name", cfg["environment"].get("map_type", "map"))
        stem = f"{planner_name}_{predictor_name}_h{horizon}_{map_name}_seed{seed}"
        episode_dir = dirs["episode"] / planner_name / predictor_name / f"h{horizon}" / f"seed_{seed}"

        result = sim.run_episode(
            cfg=cfg,
            planner_name=planner_name,
            seed=seed,
            output_dir=episode_dir,
            animation_stem=stem,
        )

        row = dict(result.summary)
        row.update(
            {
                "run_id": args.run_id,
                "planner_name": planner_name,
                "predictor_type": predictor_name,
                "rollout_horizon": horizon,
                "coverage_csv": result.coverage_csv_path,
                "step_log_csv": result.step_log_csv_path,
                "animation_gif": result.animation_gif_path,
                "animation_mp4": result.animation_mp4_path,
            }
        )
        rows.append(row)

        print(
            f"task={args.task_index} planner={planner_name} predictor={predictor_name} "
            f"h={horizon} seed={seed} success={row['success']} "
            f"steps={row['completion_steps']} coverage={row['final_coverage']:.3f}",
            flush=True,
        )

    shard_csv = shard_dir / f"compare_predictors_results_task{args.task_index:03d}.csv"
    pd.DataFrame(rows).to_csv(shard_csv, index=False)
    print(f"shard_csv: {shard_csv}", flush=True)


if __name__ == "__main__":
    main()
