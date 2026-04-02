from __future__ import annotations

import argparse
from pathlib import Path

import matplotlib.pyplot as plt
import pandas as pd


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Summarize unified planner benchmark CSV")
    parser.add_argument("--input", type=str, required=True, help="Path to compare_planners_results.csv or similar")
    parser.add_argument("--output", type=str, default=None, help="Optional output summary csv path")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    input_path = Path(args.input)
    if not input_path.exists():
        raise FileNotFoundError(f"Missing input CSV: {input_path}")

    df = pd.read_csv(input_path)

    required_cols = {"planner_name", "map_name", "success", "completion_steps", "final_coverage"}
    missing = required_cols - set(df.columns)
    if missing:
        raise ValueError(f"Input CSV missing required columns: {sorted(missing)}")

    summary = (
        df.groupby(["map_name", "planner_name"], as_index=False)
        .agg(
            success_rate=("success", "mean"),
            mean_steps=("completion_steps", "mean"),
            median_steps=("completion_steps", "median"),
            mean_final_coverage=("final_coverage", "mean"),
            mean_total_path_length=("total_path_length", "mean"),
            mean_conflict_count=("conflict_count", "mean"),
            mean_congestion_count=("congestion_count", "mean"),
            mean_planner_compute_time_ms=("planner_compute_time_ms_mean", "mean"),
            mean_predictor_inference_time_ms=("predictor_inference_time_ms_mean", "mean"),
        )
        .sort_values(["map_name", "mean_steps"])
    )

    if args.output is not None:
        out_csv = Path(args.output)
    else:
        out_csv = input_path.with_name(input_path.stem + "_summary.csv")

    out_csv.parent.mkdir(parents=True, exist_ok=True)
    summary.to_csv(out_csv, index=False)

    fig_dir = out_csv.parent.parent / "plots"
    fig_dir.mkdir(parents=True, exist_ok=True)

    for map_name, sub in summary.groupby("map_name"):
        plt.figure(figsize=(7.5, 4.3))
        plt.bar(sub["planner_name"], sub["mean_steps"], color=["#6D4C41", "#1E88E5", "#2E7D32"][: len(sub)])
        plt.title(f"Mean Completion Steps | {map_name}")
        plt.ylabel("steps")
        plt.grid(axis="y", alpha=0.3)
        plt.tight_layout()
        plt.savefig(fig_dir / f"summary_steps_{map_name}.png", dpi=160)
        plt.close()

    print(summary.to_string(index=False))
    print(f"summary_csv: {out_csv}")
    print(f"plots_dir: {fig_dir}")


if __name__ == "__main__":
    main()
