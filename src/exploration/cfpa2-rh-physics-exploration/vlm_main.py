"""VLM-guided agentic exploration entry point.

Usage:
    python vlm_main.py --goal "find the red button" --env narrow_t_branches --seed 42
    python vlm_main.py --goal "find the red button" --vlm-backend claude --seed 0
"""
from __future__ import annotations

import os
import sys

# Load .env file (API keys etc.) — search current dir and parents
from dotenv import load_dotenv
load_dotenv()
load_dotenv(dotenv_path=os.path.join(os.path.dirname(__file__), "..", ".env"))
load_dotenv(dotenv_path=os.path.join(os.path.dirname(__file__), ".env.xai"))

# Must be set BEFORE any matplotlib import so the interactive backend is used
# for the live visualisation window.
if os.environ.get("MPLBACKEND") is None:
    import platform
    os.environ["MPLBACKEND"] = "macosx" if platform.system() == "Darwin" else "TkAgg"

import argparse
from pathlib import Path

import yaml
import pandas as pd

from core.config import load_experiment_config, write_config_snapshot
from simulators.grid_sim import GridSimulation
from simulators.grid_sim.map_generators import generate_map_with_artifacts


ENV_CFG = {
    "maze": "configs/env_maze.yaml",
    "narrow_t_branches": "configs/env_narrow_t_branches.yaml",
    "narrow_t_dense_branches": "configs/env_narrow_t_dense_branches.yaml",
    "narrow_t_asymmetric_branches": "configs/env_narrow_t_asymmetric_branches.yaml",
    "narrow_t_loop_branches": "configs/env_narrow_t_loop_branches.yaml",
}


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="VLM-guided artifact exploration")
    p.add_argument("--goal", type=str, required=True, help='Natural language goal, e.g. "find the red button"')
    p.add_argument(
        "--env",
        type=str,
        default="narrow_t_branches",
        choices=list(ENV_CFG.keys()),
        help="Named environment preset.",
    )
    p.add_argument("--env-config", type=str, default=None, help="Direct environment config path override")
    p.add_argument("--seed", type=int, default=0)
    p.add_argument("--vlm-backend", type=str, default="grok", choices=["grok", "groq", "gemini", "claude"])
    p.add_argument("--artifact-count", type=int, default=1)
    p.add_argument("--max-steps", type=int, default=3000)
    p.add_argument("--output-root", type=str, default="outputs")
    p.add_argument("--no-live", action="store_true", help="Disable live matplotlib window")
    p.add_argument("--no-animation", action="store_true", help="Disable animation saving")
    p.add_argument("--vlm-boost", type=float, default=5.0, help="Utility boost for VLM-preferred frontier")
    return p.parse_args()


def main() -> None:
    args = parse_args()

    # Load base VLM config + optional env override
    env_cfg_path = args.env_config or ENV_CFG.get(args.env)
    cfg = load_experiment_config(
        "configs/vlm_exploration.yaml",
        env_cfg_path=env_cfg_path,
    )

    # Override from CLI
    cfg["planning"]["planner_name"] = "vlm_frontier"
    cfg.setdefault("vlm", {})
    cfg["vlm"]["enabled"] = True
    cfg["vlm"]["goal_prompt"] = args.goal
    cfg["vlm"]["backend"] = args.vlm_backend
    cfg["vlm"]["artifact_count"] = args.artifact_count
    cfg["vlm"]["utility_boost"] = args.vlm_boost
    cfg["termination"]["max_steps"] = args.max_steps
    cfg["environment"]["random_seed"] = args.seed

    if args.no_live:
        cfg["experiment"]["enable_live_plot"] = False
    if args.no_animation:
        cfg["experiment"]["save_animation"] = False

    # Generate map + place artifacts at dead-ends
    env = cfg["environment"]
    start_positions = cfg["robots"]["start_positions"]
    exclusion = {(int(s[0]), int(s[1])) for s in start_positions}
    grid, artifact_positions = generate_map_with_artifacts(
        map_type=str(env["map_type"]),
        width=int(env["map_width"]),
        height=int(env["map_height"]),
        obstacle_density=float(env.get("obstacle_density", 0.0)),
        seed=int(args.seed),
        artifact_count=int(args.artifact_count),
        exclusion_zone=exclusion,
    )
    cfg["vlm"]["_artifact_positions"] = artifact_positions

    # Tell the simulator to use our pre-generated map
    cfg["environment"]["predefined_map_path"] = None  # prevent file-based map loading
    cfg["_prebuilt_truth_map"] = grid

    print(f"[VLM Exploration] goal: {args.goal!r}")
    print(f"[VLM Exploration] env: {args.env}  seed: {args.seed}  backend: {args.vlm_backend}")
    print(f"[VLM Exploration] artifacts placed: {len(artifact_positions)} at {artifact_positions}")
    print()

    # Prepare output dirs
    out_root = Path(args.output_root)
    run_dir = out_root / f"vlm_{args.env}_seed{args.seed}"
    run_dir.mkdir(parents=True, exist_ok=True)
    configs_dir = run_dir / "configs"
    configs_dir.mkdir(parents=True, exist_ok=True)
    # Write a serializable copy (exclude numpy array and internal keys)
    serializable_cfg = {k: v for k, v in cfg.items() if not k.startswith("_")}
    write_config_snapshot(configs_dir / "resolved_config.yaml", serializable_cfg)

    # Run episode
    sim = GridSimulation()
    map_name = env.get("map_name", env.get("map_type", "map"))
    stem = f"vlm_{map_name}_seed{args.seed}"

    result = sim.run_episode(
        cfg=cfg,
        planner_name="vlm_frontier",
        seed=args.seed,
        output_dir=run_dir,
        animation_stem=stem,
    )

    # Summary
    row = dict(result.summary)
    row["goal"] = args.goal
    row["vlm_backend"] = args.vlm_backend
    row["artifact_positions"] = str(artifact_positions)

    summary_csv = run_dir / "vlm_summary.csv"
    pd.DataFrame([row]).to_csv(summary_csv, index=False)

    print()
    print("=" * 60)
    print("VLM Exploration Summary")
    print("=" * 60)
    print(f"  Goal:              {args.goal}")
    print(f"  Success:           {result.summary.get('success', False)}")
    print(f"  Reason:            {result.summary.get('failure_reason', 'N/A')}")
    print(f"  Steps:             {result.summary.get('completion_steps', 'N/A')}")
    print(f"  Artifact step:     {result.summary.get('artifact_found_step', 'not found')}")
    print(f"  Final coverage:    {result.summary.get('final_coverage', 0):.1%}")
    print(f"  Animation:         {result.animation_mp4_path or 'none'}")
    print(f"  Summary CSV:       {summary_csv}")
    print("=" * 60)


if __name__ == "__main__":
    main()
