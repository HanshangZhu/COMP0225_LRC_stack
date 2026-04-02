from __future__ import annotations

import argparse
from pathlib import Path

import matplotlib.pyplot as plt
import pandas as pd


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Plot benchmark metrics from raw comparison CSV")
    parser.add_argument("--input", type=str, required=True, help="Path to raw results CSV")
    parser.add_argument("--output-dir", type=str, default=None)
    return parser.parse_args()


def _boxplot_completion(df: pd.DataFrame, out_dir: Path) -> None:
    for map_name, sub in df.groupby("map_name"):
        groups = []
        labels = []
        for planner_name, sub2 in sub.groupby("planner_name"):
            groups.append(sub2["completion_steps"].to_list())
            labels.append(planner_name)

        if not groups:
            continue
        plt.figure(figsize=(8.0, 4.5))
        plt.boxplot(groups, labels=labels)
        plt.title(f"Completion Steps Distribution | {map_name}")
        plt.ylabel("steps")
        plt.grid(axis="y", alpha=0.25)
        plt.tight_layout()
        plt.savefig(out_dir / f"box_completion_steps_{map_name}.png", dpi=160)
        plt.close()


def _plot_compute_vs_coverage(df: pd.DataFrame, out_dir: Path) -> None:
    plt.figure(figsize=(7.6, 4.8))
    for planner_name, sub in df.groupby("planner_name"):
        plt.scatter(
            sub["planner_compute_time_ms_mean"],
            sub["final_coverage"],
            label=planner_name,
            alpha=0.8,
        )
    plt.xlabel("planner_compute_time_ms_mean")
    plt.ylabel("final_coverage")
    plt.ylim(0.0, 1.01)
    plt.title("Planner Compute Time vs Coverage")
    plt.grid(True, alpha=0.3)
    plt.legend()
    plt.tight_layout()
    plt.savefig(out_dir / "scatter_compute_vs_coverage.png", dpi=160)
    plt.close()


def main() -> None:
    args = parse_args()
    input_path = Path(args.input)
    if not input_path.exists():
        raise FileNotFoundError(f"Missing input CSV: {input_path}")

    df = pd.read_csv(input_path)
    out_dir = Path(args.output_dir) if args.output_dir else input_path.parent.parent / "plots"
    out_dir.mkdir(parents=True, exist_ok=True)

    _boxplot_completion(df, out_dir)
    _plot_compute_vs_coverage(df, out_dir)

    print(f"plots_dir: {out_dir}")


if __name__ == "__main__":
    main()
