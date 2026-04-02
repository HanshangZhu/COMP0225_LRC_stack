from __future__ import annotations

import argparse
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from experiments.compare_planners_across_maps import main as compare_planners_main

ENV_CFG = {
    "narrow_t_branches": "configs/env_narrow_t_branches_single_robot.yaml",
    "narrow_t_dense_branches": "configs/env_narrow_t_dense_branches_single_robot.yaml",
}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Single-robot planner comparison wrapper")
    parser.add_argument("--base-config", type=str, default="configs/base_single_robot.yaml")
    parser.add_argument(
        "--planners",
        nargs="+",
        default=["single_robot_frontier", "cfpa2", "rh_cfpa2", "hybrid_explorer"],
        choices=["cfpa2", "active_slam_explorer", "macro_frontier_explorer", "rh_cfpa2", "physics_rh_cfpa2", "hybrid_explorer", "single_robot_frontier"],
    )
    parser.add_argument(
        "--envs",
        nargs="+",
        default=["narrow_t_branches", "narrow_t_dense_branches"],
        choices=["narrow_t_branches", "narrow_t_dense_branches"],
    )
    parser.add_argument("--seed-start", type=int, default=0)
    parser.add_argument("--num-seeds", type=int, default=3)
    parser.add_argument("--max-steps", type=int, default=None)
    parser.add_argument("--run-id", type=str, default=None)
    parser.add_argument("--output-root", type=str, default="outputs")
    parser.add_argument("--animate-first-seed-only", action="store_true")
    parser.add_argument("--physics-weight-file", type=str, default=None)
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    env_cfgs = [ENV_CFG[e] for e in args.envs]

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
