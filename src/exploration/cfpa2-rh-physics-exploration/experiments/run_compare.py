from __future__ import annotations

import argparse
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from experiments.compare_planners_across_maps import main as compare_planners_main

ENV_CFG = {
    "maze": "configs/env_maze.yaml",
    "narrow_t_branches": "configs/env_narrow_t_branches.yaml",
    "narrow_t_dense_branches": "configs/env_narrow_t_dense_branches.yaml",
    "narrow_t_asymmetric_branches": "configs/env_narrow_t_asymmetric_branches.yaml",
    "narrow_t_loop_branches": "configs/env_narrow_t_loop_branches.yaml",
}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Unified planner comparison wrapper")
    parser.add_argument("--base-config", type=str, default="configs/base.yaml")
    parser.add_argument(
        "--planners",
        nargs="+",
        default=["cfpa2", "active_slam_explorer", "macro_frontier_explorer", "rh_cfpa2", "physics_rh_cfpa2", "hybrid_explorer"],
        choices=["cfpa2", "active_slam_explorer", "macro_frontier_explorer", "rh_cfpa2", "physics_rh_cfpa2", "hybrid_explorer"],
    )
    parser.add_argument(
        "--envs",
        nargs="+",
        default=["maze", "narrow_t_branches", "narrow_t_asymmetric_branches", "narrow_t_loop_branches"],
        choices=["maze", "narrow_t_branches", "narrow_t_dense_branches", "narrow_t_asymmetric_branches", "narrow_t_loop_branches"],
        help="Named env presets; ignored when --env-configs is provided.",
    )
    parser.add_argument("--env-configs", nargs="+", default=None, help="Direct env config paths")
    parser.add_argument("--seed-start", type=int, default=0)
    parser.add_argument("--num-seeds", type=int, default=2)
    parser.add_argument("--max-steps", type=int, default=None)
    parser.add_argument("--run-id", type=str, default=None)
    parser.add_argument("--output-root", type=str, default="outputs")
    parser.add_argument("--animate-first-seed-only", action="store_true")
    parser.add_argument("--physics-weight-file", type=str, default=None)
    return parser.parse_args()


def main() -> None:
    args = parse_args()

    env_cfgs = list(args.env_configs) if args.env_configs else [ENV_CFG[e] for e in args.envs]

    forwarded_argv = [
        "compare_planners_across_maps.py",
        "--base-config",
        args.base_config,
        "--planners",
        *args.planners,
        "--env-configs",
        *env_cfgs,
        "--seed-start",
        str(args.seed_start),
        "--num-seeds",
        str(args.num_seeds),
        "--output-root",
        args.output_root,
    ]
    if args.run_id is not None:
        forwarded_argv.extend(["--run-id", args.run_id])
    if args.max_steps is not None:
        forwarded_argv.extend(["--max-steps", str(args.max_steps)])
    if args.animate_first_seed_only:
        forwarded_argv.append("--animate-first-seed-only")
    if args.physics_weight_file is not None:
        forwarded_argv.extend(["--physics-weight-file", args.physics_weight_file])

    original_argv = list(sys.argv)
    try:
        sys.argv = forwarded_argv
        compare_planners_main()
    finally:
        sys.argv = original_argv


if __name__ == "__main__":
    main()
