from __future__ import annotations

import argparse
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from core.config import load_experiment_config
from experiments.common import enforce_mp4_only, make_run_id, prepare_output_dirs
from simulators.grid_sim import GridSimulation

PLANNER_CFG = {
    "cfpa2": "configs/planner_cfpa2.yaml",
    "rh_cfpa2": "configs/planner_rh_cfpa2.yaml",
    "physics_rh_cfpa2": "configs/planner_physics_rh_cfpa2.yaml",
}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Export one animation for visual comparison")
    parser.add_argument("--base-config", type=str, default="configs/base.yaml")
    parser.add_argument("--env-config", type=str, default="configs/env_maze.yaml")
    parser.add_argument("--planner", type=str, default="cfpa2", choices=["cfpa2", "rh_cfpa2", "physics_rh_cfpa2"])
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--max-steps", type=int, default=450)
    parser.add_argument("--run-id", type=str, default=None)
    parser.add_argument("--output-root", type=str, default="outputs")
    parser.add_argument("--physics-weight-file", type=str, default=None)
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    planner_cfg = PLANNER_CFG[args.planner]

    cfg = load_experiment_config(args.base_config, planner_cfg_path=planner_cfg, env_cfg_path=args.env_config)
    cfg = enforce_mp4_only(cfg)
    cfg["planning"]["planner_name"] = args.planner
    if args.planner == "physics_rh_cfpa2" and args.physics_weight_file is not None:
        cfg["predictor"]["type"] = "physics_residual"
        cfg["predictor"]["physics_residual"]["enabled"] = True
        cfg["predictor"]["physics_residual"]["weight_file"] = args.physics_weight_file
    cfg["termination"]["max_steps"] = int(args.max_steps)
    cfg["experiment"]["save_animation"] = True
    cfg["animation"]["save_gif"] = False
    cfg["animation"]["save_mp4"] = True

    run_id = args.run_id or make_run_id("anim")
    dirs = prepare_output_dirs(args.output_root, run_id)

    map_name = cfg["environment"].get("map_name", cfg["environment"].get("map_type", "map"))
    stem = f"{args.planner}_{map_name}_seed{args.seed}"

    sim = GridSimulation()
    result = sim.run_episode(
        cfg=cfg,
        planner_name=args.planner,
        seed=args.seed,
        output_dir=dirs["episode"] / stem,
        animation_stem=stem,
    )

    print("animation_gif:", result.animation_gif_path)
    print("animation_mp4:", result.animation_mp4_path)
    print("coverage_csv:", result.coverage_csv_path)
    print("step_log_csv:", result.step_log_csv_path)


if __name__ == "__main__":
    main()
