from __future__ import annotations

import argparse
import copy
import itertools
import sys
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from core.config import load_experiment_config, write_config_snapshot
from experiments.common import enforce_mp4_only, git_commit_hash, make_run_id, prepare_output_dirs, save_run_metadata
from simulators.grid_sim import GridSimulation

PLANNER_CFG = {
    "rh_cfpa2": "configs/planner_rh_cfpa2.yaml",
    "physics_rh_cfpa2": "configs/planner_physics_rh_cfpa2.yaml",
}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Comprehensive RH-CFPA2 ablation across rollout settings")
    parser.add_argument("--base-config", type=str, default="configs/base.yaml")
    parser.add_argument(
        "--planners",
        nargs="+",
        default=["rh_cfpa2"],
        choices=["rh_cfpa2", "physics_rh_cfpa2"],
    )
    parser.add_argument(
        "--env-configs",
        nargs="+",
        default=[
            "configs/env_narrow_t_branches.yaml",
            "configs/env_narrow_t_dense_branches.yaml",
            "configs/env_narrow_t_asymmetric_branches.yaml",
            "configs/env_narrow_t_loop_branches.yaml",
        ],
    )
    parser.add_argument("--seed-start", type=int, default=0)
    parser.add_argument("--num-seeds", type=int, default=3)
    parser.add_argument("--max-steps", type=int, default=5000)
    parser.add_argument("--run-id", type=str, default=None)
    parser.add_argument("--output-root", type=str, default="outputs")
    parser.add_argument("--disable-animation", action="store_true")
    parser.add_argument("--physics-weight-file", type=str, default=None)
    parser.add_argument("--task-index", type=int, default=0)
    parser.add_argument("--num-tasks", type=int, default=1)

    parser.add_argument("--score-modes", nargs="+", default=["hybrid", "immediate_only"], choices=["hybrid", "immediate_only", "future_only"])
    parser.add_argument("--horizons", nargs="+", type=int, default=[3, 4, 5])
    parser.add_argument("--gammas", nargs="+", type=float, default=[0.88, 0.92])
    parser.add_argument("--immediate-weights", nargs="+", type=float, default=[0.85, 1.00, 1.15])
    parser.add_argument("--future-weights", nargs="+", type=float, default=[0.15, 0.25, 0.35])
    parser.add_argument("--frontier-consumption-weights", nargs="+", type=float, default=[0.10, 0.18, 0.28])
    parser.add_argument("--congestion-scales", nargs="+", type=float, default=[0.60, 1.00])
    parser.add_argument("--branch-scales", nargs="+", type=float, default=[0.70, 1.00])
    parser.add_argument("--topk-candidate-limits", nargs="+", type=int, default=[6])
    parser.add_argument("--max-combos", type=int, default=64)
    return parser.parse_args()


def _build_combos(args: argparse.Namespace) -> list[dict[str, float | int | str]]:
    combos = []
    for values in itertools.product(
        args.score_modes,
        args.horizons,
        args.gammas,
        args.immediate_weights,
        args.future_weights,
        args.frontier_consumption_weights,
        args.congestion_scales,
        args.branch_scales,
        args.topk_candidate_limits,
    ):
        combo = {
            "score_mode": str(values[0]),
            "horizon": int(values[1]),
            "gamma": float(values[2]),
            "immediate_weight": float(values[3]),
            "future_weight": float(values[4]),
            "frontier_consumption_weight": float(values[5]),
            "congestion_scale": float(values[6]),
            "branch_scale": float(values[7]),
            "topk_candidate_limit": int(values[8]),
        }
        combos.append(combo)

    max_combos = max(1, int(args.max_combos))
    if len(combos) <= max_combos:
        return combos

    idx = np.linspace(0, len(combos) - 1, max_combos, dtype=int)
    return [combos[i] for i in idx]


def _combo_name(combo_idx: int, combo: dict[str, float | int | str]) -> str:
    return (
        f"c{combo_idx:03d}_"
        f"{combo['score_mode']}_"
        f"h{combo['horizon']}_"
        f"g{combo['gamma']:.2f}_"
        f"iw{combo['immediate_weight']:.2f}_"
        f"fw{combo['future_weight']:.2f}_"
        f"fc{combo['frontier_consumption_weight']:.2f}_"
        f"cg{combo['congestion_scale']:.2f}_"
        f"bg{combo['branch_scale']:.2f}_"
        f"k{combo['topk_candidate_limit']}"
    )


def _apply_combo(cfg: dict, combo: dict[str, float | int | str]) -> dict:
    out = copy.deepcopy(cfg)
    rollout = out["planning"]["rollout"]

    rollout["score_mode"] = str(combo["score_mode"])
    rollout["horizon"] = int(combo["horizon"])
    rollout["gamma"] = float(combo["gamma"])
    rollout["immediate_weight"] = float(combo["immediate_weight"])
    rollout["future_weight"] = float(combo["future_weight"])
    rollout["frontier_consumption_weight"] = float(combo["frontier_consumption_weight"])
    out["planning"]["topk_candidate_limit"] = int(combo["topk_candidate_limit"])

    congestion_scale = float(combo["congestion_scale"])
    for key in [
        "lambda_corridor_occupancy",
        "lambda_narrow_blocking",
        "lambda_path_crossing",
        "lambda_waiting_time",
    ]:
        rollout[key] = float(rollout.get(key, 0.0)) * congestion_scale

    branch_scale = float(combo["branch_scale"])
    rollout["reassign_w_density"] = float(rollout.get("reassign_w_density", 0.0)) * branch_scale
    rollout["reassign_w_branch"] = float(rollout.get("reassign_w_branch", 0.0)) * branch_scale

    return out


def _objective_score(df: pd.DataFrame) -> pd.Series:
    return (
        10000.0 * df["success_rate"].astype(float)
        + 1000.0 * df["final_coverage"].astype(float)
        - df["completion_steps"].astype(float)
        - 0.05 * df["planner_compute_time_ms"].astype(float)
    )


def _plot_top_configs(summary_df: pd.DataFrame, out_dir: Path) -> None:
    if summary_df.empty:
        return
    out_dir.mkdir(parents=True, exist_ok=True)

    for planner_name, sub in summary_df.groupby("planner_name"):
        top = sub.sort_values("objective_score", ascending=False).head(12).copy()
        if top.empty:
            continue
        labels = [f"{row.combo_idx}:{row.score_mode}:h{int(row.horizon)}" for row in top.itertuples()]

        plt.figure(figsize=(11.5, 5.4))
        plt.bar(labels, top["objective_score"], color="#1565C0")
        plt.xticks(rotation=40, ha="right")
        plt.ylabel("objective_score")
        plt.title(f"Top RH Ablation Configs | {planner_name}")
        plt.grid(axis="y", alpha=0.25)
        plt.tight_layout()
        plt.savefig(out_dir / f"ablation_top_configs_{planner_name}.png", dpi=160)
        plt.close()


def main() -> None:
    args = parse_args()
    run_id = args.run_id or make_run_id("ablate_rh")
    dirs = prepare_output_dirs(args.output_root, run_id)

    combos = _build_combos(args)
    save_run_metadata(
        dirs["metadata"] / "run_metadata.json",
        {
            "run_id": run_id,
            "base_config": args.base_config,
            "planners": args.planners,
            "env_configs": args.env_configs,
            "seed_start": args.seed_start,
            "num_seeds": args.num_seeds,
            "max_steps": args.max_steps,
            "num_combos": len(combos),
            "task_index": int(args.task_index),
            "num_tasks": int(args.num_tasks),
            "git_commit": git_commit_hash(),
        },
    )

    sim = GridSimulation()
    rows: list[dict] = []
    task_index = max(0, int(args.task_index))
    num_tasks = max(1, int(args.num_tasks))
    combo_items = [(idx, combo) for idx, combo in enumerate(combos) if idx % num_tasks == task_index]

    for planner_name in args.planners:
        planner_cfg = PLANNER_CFG[planner_name]

        for env_cfg_path in args.env_configs:
            base_cfg = load_experiment_config(args.base_config, planner_cfg_path=planner_cfg, env_cfg_path=env_cfg_path)
            base_cfg = enforce_mp4_only(base_cfg)
            base_cfg["planning"]["planner_name"] = planner_name
            base_cfg["termination"]["max_steps"] = int(args.max_steps)
            if planner_name == "physics_rh_cfpa2" and args.physics_weight_file is not None:
                base_cfg["predictor"]["type"] = "physics_residual"
                base_cfg["predictor"]["physics_residual"]["enabled"] = True
                base_cfg["predictor"]["physics_residual"]["weight_file"] = args.physics_weight_file

            for combo_idx, combo in combo_items:
                cfg = _apply_combo(base_cfg, combo)
                if args.disable_animation:
                    cfg["experiment"]["save_animation"] = False

                combo_name = _combo_name(combo_idx, combo)
                env_label = Path(env_cfg_path).stem
                write_config_snapshot(dirs["configs"] / f"{planner_name}_{env_label}_{combo_name}.yaml", cfg)

                for seed in range(args.seed_start, args.seed_start + args.num_seeds):
                    map_name = cfg["environment"].get("map_name", cfg["environment"].get("map_type", env_label))
                    stem = f"{planner_name}_{combo_name}_{map_name}_seed{seed}"
                    episode_dir = dirs["episode"] / planner_name / combo_name / map_name / f"seed_{seed}"

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
                            "run_id": run_id,
                            "planner_name": planner_name,
                            "env_config": env_cfg_path,
                            "combo_idx": combo_idx,
                            "combo_name": combo_name,
                            **combo,
                            "coverage_csv": result.coverage_csv_path,
                            "step_log_csv": result.step_log_csv_path,
                            "animation_mp4": result.animation_mp4_path,
                        }
                    )
                    rows.append(row)

                    print(
                        f"planner={planner_name} combo={combo_name} map={map_name} seed={seed} "
                        f"success={row['success']} steps={row['completion_steps']} "
                        f"coverage={row['final_coverage']:.3f}",
                        flush=True,
                    )

    raw_df = pd.DataFrame(rows)
    shard_mode = num_tasks > 1
    shard_dir = dirs["results_csv"] / "shards"
    if shard_mode:
        shard_dir.mkdir(parents=True, exist_ok=True)
        raw_csv = shard_dir / f"ablate_rh_results_shard_{task_index:03d}.csv"
    else:
        raw_csv = dirs["results_csv"] / "ablate_rh_results.csv"
    raw_df.to_csv(raw_csv, index=False)

    if raw_df.empty:
        print(f"No ablation rows generated for task_index={task_index} num_tasks={num_tasks}")
        print(f"raw_csv: {raw_csv}")
        return

    summary_df = (
        raw_df.groupby(
            [
                "planner_name",
                "map_name",
                "combo_idx",
                "combo_name",
                "score_mode",
                "horizon",
                "gamma",
                "immediate_weight",
                "future_weight",
                "frontier_consumption_weight",
                "congestion_scale",
                "branch_scale",
                "topk_candidate_limit",
            ],
            as_index=False,
        )
        .agg(
            success_rate=("success", "mean"),
            completion_steps=("completion_steps", "mean"),
            completion_time=("completion_time", "mean"),
            final_coverage=("final_coverage", "mean"),
            total_path_length=("total_path_length", "mean"),
            planner_compute_time_ms=("planner_compute_time_ms_mean", "mean"),
        )
    )
    summary_df["objective_score"] = _objective_score(summary_df)
    if shard_mode:
        summary_csv = shard_dir / f"ablate_rh_summary_shard_{task_index:03d}.csv"
    else:
        summary_csv = dirs["results_csv"] / "ablate_rh_summary.csv"
    summary_df.to_csv(summary_csv, index=False)

    if shard_mode:
        print(f"task_index={task_index} num_tasks={num_tasks} combos={len(combo_items)}")
        print(f"raw_csv: {raw_csv}")
        print(f"summary_csv: {summary_csv}")
        return

    overall_df = (
        summary_df.groupby(
            [
                "planner_name",
                "combo_idx",
                "combo_name",
                "score_mode",
                "horizon",
                "gamma",
                "immediate_weight",
                "future_weight",
                "frontier_consumption_weight",
                "congestion_scale",
                "branch_scale",
                "topk_candidate_limit",
            ],
            as_index=False,
        )
        .agg(
            success_rate=("success_rate", "mean"),
            completion_steps=("completion_steps", "mean"),
            completion_time=("completion_time", "mean"),
            final_coverage=("final_coverage", "mean"),
            total_path_length=("total_path_length", "mean"),
            planner_compute_time_ms=("planner_compute_time_ms", "mean"),
        )
    )
    overall_df["objective_score"] = _objective_score(overall_df)
    overall_df = overall_df.sort_values(["planner_name", "objective_score"], ascending=[True, False])
    overall_csv = dirs["results_csv"] / "ablate_rh_overall_ranking.csv"
    overall_df.to_csv(overall_csv, index=False)

    best_per_map_df = (
        summary_df.sort_values(["planner_name", "map_name", "objective_score"], ascending=[True, True, False])
        .groupby(["planner_name", "map_name"], as_index=False)
        .head(1)
    )
    best_per_map_csv = dirs["results_csv"] / "ablate_rh_best_per_map.csv"
    best_per_map_df.to_csv(best_per_map_csv, index=False)

    _plot_top_configs(overall_df, dirs["plots"])

    print("\n=== RH Ablation Overall Top Configs ===")
    print(overall_df.groupby("planner_name", as_index=False).head(10).to_string(index=False))
    print(f"raw_csv: {raw_csv}")
    print(f"summary_csv: {summary_csv}")
    print(f"overall_csv: {overall_csv}")
    print(f"best_per_map_csv: {best_per_map_csv}")


if __name__ == "__main__":
    main()
