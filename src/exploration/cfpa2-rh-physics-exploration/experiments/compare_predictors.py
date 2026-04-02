from __future__ import annotations

import argparse
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
    parser = argparse.ArgumentParser(description="Predictor and rollout-horizon ablation for RH planners")
    parser.add_argument("--base-config", type=str, default="configs/base.yaml")
    parser.add_argument("--env-config", type=str, default="configs/env_maze.yaml")
    parser.add_argument("--planners", nargs="+", default=["rh_cfpa2", "physics_rh_cfpa2"], choices=["rh_cfpa2", "physics_rh_cfpa2"])
    parser.add_argument("--predictors", nargs="+", default=["path_follow", "constant_velocity", "physics_residual"], choices=["path_follow", "constant_velocity", "physics_residual"])
    parser.add_argument("--rollout-horizons", nargs="+", type=int, default=[3, 5, 7])
    parser.add_argument("--seed-start", type=int, default=0)
    parser.add_argument("--num-seeds", type=int, default=2)
    parser.add_argument("--max-steps", type=int, default=None)
    parser.add_argument("--run-id", type=str, default=None)
    parser.add_argument("--output-root", type=str, default="outputs")
    parser.add_argument("--disable-animation", action="store_true")
    parser.add_argument("--physics-weight-file", type=str, default=None, help="Optional trained checkpoint for physics_residual predictor")
    parser.add_argument("--disable-decision-probe", action="store_true", help="Disable same-state predictor decision divergence probes")
    parser.add_argument("--decision-probe-max-replans", type=int, default=40, help="Max predictor probe calls per episode")
    return parser.parse_args()


def _plot(df: pd.DataFrame, out_dir: Path) -> None:
    if df.empty:
        return
    out_dir.mkdir(parents=True, exist_ok=True)

    for planner_name, sub in df.groupby("planner_name"):
        plt.figure(figsize=(8.2, 4.8))
        for predictor_name, sub2 in sub.groupby("predictor_type"):
            x = sub2["rollout_horizon"].to_list()
            y = sub2["completion_steps"].to_list()
            plt.plot(x, y, marker="o", linewidth=2, label=predictor_name)
        plt.title(f"Rollout Horizon Ablation | {planner_name}")
        plt.xlabel("rollout_horizon")
        plt.ylabel("mean completion steps")
        plt.grid(True, alpha=0.3)
        plt.legend()
        plt.tight_layout()
        plt.savefig(out_dir / f"rollout_ablation_{planner_name}.png", dpi=160)
        plt.close()

        plt.figure(figsize=(8.2, 4.8))
        for predictor_name, sub2 in sub.groupby("predictor_type"):
            x = sub2["rollout_horizon"].to_list()
            y = sub2["prediction_error_h3"].to_list()
            plt.plot(x, y, marker="s", linewidth=2, label=predictor_name)
        plt.title(f"Prediction Error@H3 Ablation | {planner_name}")
        plt.xlabel("rollout_horizon")
        plt.ylabel("mean prediction error (cells)")
        plt.grid(True, alpha=0.3)
        plt.legend()
        plt.tight_layout()
        plt.savefig(out_dir / f"prediction_error_h3_{planner_name}.png", dpi=160)
        plt.close()

        if "decision_divergence_rate" in sub.columns:
            plt.figure(figsize=(8.2, 4.8))
            for predictor_name, sub2 in sub.groupby("predictor_type"):
                x = sub2["rollout_horizon"].to_list()
                y = sub2["decision_divergence_rate"].to_list()
                plt.plot(x, y, marker="^", linewidth=2, label=predictor_name)
            plt.title(f"Decision Divergence Rate | {planner_name}")
            plt.xlabel("rollout_horizon")
            plt.ylabel("divergence rate")
            plt.ylim(0.0, 1.0)
            plt.grid(True, alpha=0.3)
            plt.legend()
            plt.tight_layout()
            plt.savefig(out_dir / f"decision_divergence_rate_{planner_name}.png", dpi=160)
            plt.close()

        if "predictor_rollout_score_variance_mean" in sub.columns:
            plt.figure(figsize=(8.2, 4.8))
            for predictor_name, sub2 in sub.groupby("predictor_type"):
                x = sub2["rollout_horizon"].to_list()
                y = sub2["predictor_rollout_score_variance_mean"].to_list()
                plt.plot(x, y, marker="d", linewidth=2, label=predictor_name)
            plt.title(f"Predictor-Aware Rollout Score Variance | {planner_name}")
            plt.xlabel("rollout_horizon")
            plt.ylabel("mean score variance")
            plt.grid(True, alpha=0.3)
            plt.legend()
            plt.tight_layout()
            plt.savefig(out_dir / f"rollout_score_variance_{planner_name}.png", dpi=160)
            plt.close()

        if "chosen_frontier_difference_mean" in sub.columns:
            plt.figure(figsize=(8.2, 4.8))
            for predictor_name, sub2 in sub.groupby("predictor_type"):
                x = sub2["rollout_horizon"].to_list()
                y = sub2["chosen_frontier_difference_mean"].to_list()
                plt.plot(x, y, marker="x", linewidth=2, label=predictor_name)
            plt.title(f"Chosen Frontier Difference Mean | {planner_name}")
            plt.xlabel("rollout_horizon")
            plt.ylabel("mean different goals per probe pair")
            plt.grid(True, alpha=0.3)
            plt.legend()
            plt.tight_layout()
            plt.savefig(out_dir / f"frontier_difference_mean_{planner_name}.png", dpi=160)
            plt.close()


def main() -> None:
    args = parse_args()
    run_id = args.run_id or make_run_id("compare_predictors")
    dirs = prepare_output_dirs(args.output_root, run_id)

    save_run_metadata(
        dirs["metadata"] / "run_metadata.json",
        {
            "run_id": run_id,
            "base_config": args.base_config,
            "env_config": args.env_config,
            "planners": args.planners,
            "predictors": args.predictors,
            "rollout_horizons": args.rollout_horizons,
            "enable_decision_probe": not bool(args.disable_decision_probe),
            "decision_probe_max_replans": int(args.decision_probe_max_replans),
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

        for predictor_name in args.predictors:
            if planner_name == "physics_rh_cfpa2" and predictor_name != "physics_residual":
                continue
            for horizon in args.rollout_horizons:
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

                tag = f"{planner_name}_{predictor_name}_h{horizon}"
                write_config_snapshot(dirs["configs"] / f"resolved_{tag}.yaml", cfg)

                for seed in range(args.seed_start, args.seed_start + args.num_seeds):
                    map_name = cfg["environment"].get("map_name", cfg["environment"].get("map_type", "map"))
                    stem = f"{tag}_{map_name}_seed{seed}"
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
                            "run_id": run_id,
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
                        f"planner={planner_name} predictor={predictor_name} h={horizon} seed={seed} "
                        f"success={row['success']} steps={row['completion_steps']} coverage={row['final_coverage']:.3f}",
                        flush=True,
                    )

    raw_df = pd.DataFrame(rows)
    raw_csv = dirs["results_csv"] / "compare_predictors_results.csv"
    raw_df.to_csv(raw_csv, index=False)

    summary_df = (
        raw_df.groupby(["planner_name", "predictor_type", "rollout_horizon"], as_index=False)
        .agg(
            success_rate=("success", "mean"),
            completion_steps=("completion_steps", "mean"),
            completion_time=("completion_time", "mean"),
            final_coverage=("final_coverage", "mean"),
            planner_compute_time_ms=("planner_compute_time_ms_mean", "mean"),
            predictor_inference_time_ms=("predictor_inference_time_ms_mean", "mean"),
            prediction_error_h1=("prediction_error_h1", "mean"),
            prediction_error_h3=("prediction_error_h3", "mean"),
            prediction_error_h5=("prediction_error_h5", "mean"),
            decision_probe_pair_count=("decision_probe_pair_count", "mean"),
            decision_divergence_rate=("decision_divergence_rate", "mean"),
            chosen_frontier_difference_mean=("chosen_frontier_difference_mean", "mean"),
            predictor_rollout_score_variance_mean=("predictor_rollout_score_variance_mean", "mean"),
        )
        .sort_values(["planner_name", "completion_steps"])
    )

    summary_csv = dirs["results_csv"] / "compare_predictors_summary.csv"
    summary_df.to_csv(summary_csv, index=False)

    _plot(summary_df, dirs["plots"])

    print("\n=== Predictor Summary ===")
    print(summary_df.to_string(index=False))
    print(f"raw_csv: {raw_csv}")
    print(f"summary_csv: {summary_csv}")


if __name__ == "__main__":
    main()
