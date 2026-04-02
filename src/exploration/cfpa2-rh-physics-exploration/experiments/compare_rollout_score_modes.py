from __future__ import annotations

import argparse
import copy
import sys
from pathlib import Path

import matplotlib.pyplot as plt
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
    parser = argparse.ArgumentParser(description="Compare rollout score modes for RH planners")
    parser.add_argument("--base-config", type=str, default="configs/base.yaml")
    parser.add_argument("--env-config", type=str, default="configs/env_narrow_t_branches.yaml")
    parser.add_argument("--planners", nargs="+", default=["rh_cfpa2"], choices=["rh_cfpa2", "physics_rh_cfpa2"])
    parser.add_argument(
        "--score-modes",
        nargs="+",
        default=["immediate_only", "future_only", "hybrid"],
        choices=["immediate_only", "future_only", "hybrid"],
    )
    parser.add_argument("--seed-start", type=int, default=0)
    parser.add_argument("--num-seeds", type=int, default=3)
    parser.add_argument("--max-steps", type=int, default=None)
    parser.add_argument("--run-id", type=str, default=None)
    parser.add_argument("--output-root", type=str, default="outputs")
    parser.add_argument("--disable-animation", action="store_true")
    parser.add_argument("--physics-weight-file", type=str, default=None)
    return parser.parse_args()



def _plot(summary_df: pd.DataFrame, out_dir: Path) -> None:
    if summary_df.empty:
        return
    out_dir.mkdir(parents=True, exist_ok=True)

    for planner_name, sub in summary_df.groupby("planner_name"):
        sub = sub.copy()
        mode_order = ["immediate_only", "future_only", "hybrid"]
        sub["mode_order"] = sub["score_mode"].map({m: i for i, m in enumerate(mode_order)}).fillna(99)
        sub = sub.sort_values("mode_order")

        plt.figure(figsize=(8.6, 4.8))
        plt.bar(sub["score_mode"], sub["final_coverage"], color=["#b34d00", "#007a6c", "#1457ff"][: len(sub)])
        plt.ylim(0.0, 1.0)
        plt.ylabel("mean final coverage")
        plt.title(f"Score Mode Ablation | {planner_name}")
        plt.grid(axis="y", alpha=0.25)
        plt.tight_layout()
        plt.savefig(out_dir / f"score_mode_coverage_{planner_name}.png", dpi=160)
        plt.close()

        plt.figure(figsize=(8.6, 4.8))
        plt.bar(sub["score_mode"], sub["completion_steps"], color=["#b34d00", "#007a6c", "#1457ff"][: len(sub)])
        plt.ylabel("mean completion steps")
        plt.title(f"Score Mode Ablation (Steps) | {planner_name}")
        plt.grid(axis="y", alpha=0.25)
        plt.tight_layout()
        plt.savefig(out_dir / f"score_mode_steps_{planner_name}.png", dpi=160)
        plt.close()



def main() -> None:
    args = parse_args()
    run_id = args.run_id or make_run_id("compare_score_modes")
    dirs = prepare_output_dirs(args.output_root, run_id)

    save_run_metadata(
        dirs["metadata"] / "run_metadata.json",
        {
            "run_id": run_id,
            "base_config": args.base_config,
            "env_config": args.env_config,
            "planners": args.planners,
            "score_modes": args.score_modes,
            "git_commit": git_commit_hash(),
        },
    )

    sim = GridSimulation()
    rows: list[dict] = []

    for planner_name in args.planners:
        planner_cfg = PLANNER_CFG[planner_name]
        base_cfg = load_experiment_config(args.base_config, planner_cfg_path=planner_cfg, env_cfg_path=args.env_config)
        base_cfg = enforce_mp4_only(base_cfg)
        base_cfg["planning"]["planner_name"] = planner_name
        if args.max_steps is not None:
            base_cfg["termination"]["max_steps"] = int(args.max_steps)

        if planner_name == "physics_rh_cfpa2" and args.physics_weight_file is not None:
            base_cfg["predictor"]["type"] = "physics_residual"
            base_cfg["predictor"]["physics_residual"]["enabled"] = True
            base_cfg["predictor"]["physics_residual"]["weight_file"] = args.physics_weight_file

        for mode in args.score_modes:
            cfg = copy.deepcopy(base_cfg)
            cfg["planning"]["rollout"]["score_mode"] = mode
            if args.disable_animation:
                cfg["experiment"]["save_animation"] = False

            tag = f"{planner_name}_{mode}"
            write_config_snapshot(dirs["configs"] / f"resolved_{tag}.yaml", cfg)

            for seed in range(args.seed_start, args.seed_start + args.num_seeds):
                map_name = cfg["environment"].get("map_name", cfg["environment"].get("map_type", "map"))
                stem = f"{tag}_{map_name}_seed{seed}"
                episode_dir = dirs["episode"] / planner_name / mode / f"seed_{seed}"

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
                        "score_mode": mode,
                        "coverage_csv": result.coverage_csv_path,
                        "step_log_csv": result.step_log_csv_path,
                        "animation_gif": result.animation_gif_path,
                        "animation_mp4": result.animation_mp4_path,
                    }
                )
                rows.append(row)

                print(
                    f"planner={planner_name} mode={mode} seed={seed} success={row['success']} "
                    f"steps={row['completion_steps']} coverage={row['final_coverage']:.3f}",
                    flush=True,
                )

    raw_df = pd.DataFrame(rows)
    raw_csv = dirs["results_csv"] / "compare_score_modes_results.csv"
    raw_df.to_csv(raw_csv, index=False)

    summary_df = (
        raw_df.groupby(["planner_name", "score_mode"], as_index=False)
        .agg(
            success_rate=("success", "mean"),
            completion_steps=("completion_steps", "mean"),
            completion_time=("completion_time", "mean"),
            final_coverage=("final_coverage", "mean"),
            total_path_length=("total_path_length", "mean"),
            switches=("switching_count", "mean"),
            conflicts=("conflict_count", "mean"),
            congestion=("congestion_count", "mean"),
            planner_compute_time_ms=("planner_compute_time_ms_mean", "mean"),
            predictor_inference_time_ms=("predictor_inference_time_ms_mean", "mean"),
        )
        .sort_values(["planner_name", "score_mode"])
    )

    summary_csv = dirs["results_csv"] / "compare_score_modes_summary.csv"
    summary_df.to_csv(summary_csv, index=False)

    _plot(summary_df, dirs["plots"])

    print("\n=== Score Mode Summary ===")
    print(summary_df.to_string(index=False))
    print(f"raw_csv: {raw_csv}")
    print(f"summary_csv: {summary_csv}")


if __name__ == "__main__":
    main()
