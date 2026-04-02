from __future__ import annotations

import argparse
import sys
from pathlib import Path

import pandas as pd

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from experiments.common import prepare_output_dirs
from experiments.compare_planners_across_maps import _plot_metrics_tables, _plot_summary


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Merge planner comparison shard CSVs")
    parser.add_argument("--run-id", type=str, required=True)
    parser.add_argument("--output-root", type=str, default="outputs")
    parser.add_argument("--expected-shards", type=int, default=None)
    parser.add_argument("--fail-missing", action="store_true")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    dirs = prepare_output_dirs(args.output_root, args.run_id)
    shard_dir = dirs["results_csv"] / "shards"
    paths = sorted(shard_dir.glob("compare_planners_results_task*.csv"))

    if args.expected_shards is not None and len(paths) != int(args.expected_shards):
        msg = f"expected {int(args.expected_shards)} shard csvs, found {len(paths)} in {shard_dir}"
        if args.fail_missing:
            raise FileNotFoundError(msg)
        print(f"warning: {msg}", flush=True)

    if not paths:
        raise FileNotFoundError(f"No planner shard CSVs found in {shard_dir}")

    frames = []
    for p in paths:
        if not p.exists() or p.stat().st_size <= 0:
            continue
        try:
            frames.append(pd.read_csv(p))
        except pd.errors.EmptyDataError:
            continue
    if not frames:
        raise RuntimeError("All planner shard CSVs were empty.")

    raw_df = pd.concat(frames, ignore_index=True)
    raw_csv = dirs["results_csv"] / "compare_planners_results.csv"
    raw_df.to_csv(raw_csv, index=False)

    summary_df = (
        raw_df.groupby(["map_name", "planner_name"], as_index=False)
        .agg(
            success_rate=("success", "mean"),
            completion_steps=("completion_steps", "mean"),
            completion_time=("completion_time", "mean"),
            final_coverage=("final_coverage", "mean"),
            total_path_length=("total_path_length", "mean"),
            reassignments=("reassignment_count", "mean"),
            switches=("switching_count", "mean"),
            conflicts=("conflict_count", "mean"),
            congestion=("congestion_count", "mean"),
            planner_compute_time_ms=("planner_compute_time_ms_mean", "mean"),
            predictor_inference_time_ms=("predictor_inference_time_ms_mean", "mean"),
        )
        .sort_values(["map_name", "completion_steps"])
    )

    summary_csv = dirs["results_csv"] / "compare_planners_summary.csv"
    summary_df.to_csv(summary_csv, index=False)
    _plot_summary(summary_df, dirs["plots"])
    _plot_metrics_tables(summary_df, dirs["plots"])

    print(summary_df.to_string(index=False))
    print(f"raw_csv: {raw_csv}")
    print(f"summary_csv: {summary_csv}")
    print(f"plots_dir: {dirs['plots']}")


if __name__ == "__main__":
    main()
