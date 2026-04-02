from __future__ import annotations

import argparse
import sys
from pathlib import Path

import pandas as pd

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from experiments.ablate_rh_hyperparams import _objective_score, _plot_top_configs
from experiments.common import prepare_output_dirs


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Merge RH ablation shard CSVs")
    parser.add_argument("--run-id", type=str, required=True)
    parser.add_argument("--output-root", type=str, default="outputs")
    parser.add_argument("--expected-shards", type=int, default=None)
    parser.add_argument("--fail-missing", action="store_true")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    dirs = prepare_output_dirs(args.output_root, args.run_id)
    shard_dir = dirs["results_csv"] / "shards"
    shard_files = sorted(shard_dir.glob("ablate_rh_results_shard_*.csv"))

    if not shard_files:
        raise RuntimeError(f"No ablation shard CSVs found in {shard_dir}")

    if args.expected_shards is not None and len(shard_files) != int(args.expected_shards):
        msg = f"Expected {args.expected_shards} ablation shards, found {len(shard_files)}"
        if args.fail_missing:
            raise RuntimeError(msg)
        print(f"warning: {msg}")

    raw_df = pd.concat([pd.read_csv(path) for path in shard_files], ignore_index=True)
    raw_csv = dirs["results_csv"] / "ablate_rh_results.csv"
    raw_df.to_csv(raw_csv, index=False)

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
    summary_csv = dirs["results_csv"] / "ablate_rh_summary.csv"
    summary_df.to_csv(summary_csv, index=False)

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

    print("=== RH Ablation Merge Complete ===")
    print(f"raw_csv: {raw_csv}")
    print(f"summary_csv: {summary_csv}")
    print(f"overall_csv: {overall_csv}")
    print(f"best_per_map_csv: {best_per_map_csv}")


if __name__ == "__main__":
    main()
