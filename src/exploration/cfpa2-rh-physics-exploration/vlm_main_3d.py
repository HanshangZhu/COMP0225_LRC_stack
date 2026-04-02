"""VLM-guided exploration with 3D Panda3D visualisation.

Keeps the 2D grid simulation as the planning brain; adds a 3D split-screen
window (3rd-person + 1st-person) rendered by Panda3D.

Usage:
    python vlm_main_3d.py --goal "find the red button" --env narrow_t_branches --seed 42
    python vlm_main_3d.py --goal "find the red button" --vlm-mode both --seed 0
"""
from __future__ import annotations

import os
import sys

# Load .env file (API keys etc.) — search current dir and parents
from dotenv import load_dotenv
load_dotenv()  # tries .env in cwd
load_dotenv(dotenv_path=os.path.join(os.path.dirname(__file__), "..", ".env"))  # parent
load_dotenv(dotenv_path=os.path.join(os.path.dirname(__file__), ".env.xai"))

# Interactive matplotlib for the 2D live plot (optional alongside 3D)
if os.environ.get("MPLBACKEND") is None:
    import platform
    os.environ["MPLBACKEND"] = "macosx" if platform.system() == "Darwin" else "TkAgg"

import argparse
from pathlib import Path

import numpy as np
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
    p = argparse.ArgumentParser(description="VLM exploration with 3D visualisation")
    p.add_argument("--goal", type=str, required=True,
                    help='Natural language goal, e.g. "find the red button"')
    p.add_argument("--env", type=str, default="narrow_t_branches",
                    choices=list(ENV_CFG.keys()))
    p.add_argument("--env-config", type=str, default=None)
    p.add_argument("--seed", type=int, default=0)
    p.add_argument("--vlm-backend", type=str, default="grok",
                    choices=["grok", "groq", "gemini", "claude"])
    p.add_argument("--vlm-mode", type=str, default="both",
                    choices=["topdown", "first_person", "both"],
                    help="What the VLM sees: topdown map, 1st-person 3D, or both")
    p.add_argument("--artifact-count", type=int, default=1)
    p.add_argument("--max-steps", type=int, default=3000)
    p.add_argument("--output-root", type=str, default="outputs")
    p.add_argument("--no-live", action="store_true",
                    help="Disable 2D matplotlib live window (3D always shown)")
    p.add_argument("--no-animation", action="store_true")
    p.add_argument("--vlm-boost", type=float, default=5.0)
    p.add_argument("--window-w", type=int, default=1400)
    p.add_argument("--window-h", type=int, default=700)
    return p.parse_args()


class Simulation3DRunner:
    """Orchestrates 2D sim + 3D Panda3D renderer via sample_callback."""

    def __init__(
        self,
        grid: np.ndarray,
        artifact_positions: list[tuple[int, int]],
        start_pos: tuple[int, int],
        vlm_mode: str,
        goal_text: str = "",
        window_w: int = 1400,
        window_h: int = 700,
    ) -> None:
        from visualization.panda3d_renderer import Exploration3DApp

        self._app = Exploration3DApp(
            grid=grid,
            start_pos=start_pos,
            window_w=window_w,
            window_h=window_h,
        )
        for ax, ay in artifact_positions:
            self._app.place_artifact(ax, ay)

        self._artifact_positions = artifact_positions
        self._vlm_mode = vlm_mode
        self._goal_text = goal_text
        self._planner_injected = False
        self._step_count = 0
        self._prev_explored: set[tuple[int, int]] = set()

    @property
    def app(self) -> "Exploration3DApp":
        return self._app

    def capture_first_person(self) -> str:
        return self._app.capture_first_person_b64()

    def callback(self, event: str, data: dict) -> None:
        """sample_callback passed to GridSimulation.run_episode."""
        if event == "step_begin":
            if not self._planner_injected:
                self._inject_planner(data.get("cfg", {}))

        elif event == "step_end":
            step_idx = data.get("step_idx", 0)
            robots = data.get("robots", [])
            map_mgr = data.get("map_mgr", None)
            assignments = data.get("assignments", {})
            cfg = data.get("cfg", {})

            # Update robot position
            if robots:
                r = robots[0]
                self._app.update_robot(
                    float(r.pose[0]), float(r.pose[1]), float(r.heading_deg)
                )

            # Update explored cells on floor (every 5 steps to avoid overhead)
            if map_mgr is not None and step_idx % 5 == 0:
                from core.map_manager import FREE
                free_yx = np.argwhere(map_mgr.known == FREE)
                explored = {(int(x), int(y)) for y, x in free_yx}
                new_cells = explored - self._prev_explored
                if new_cells:
                    self._app.mark_explored(new_cells)
                    self._prev_explored = explored

            # Update frontier markers (every 10 steps)
            if step_idx % 10 == 0 and map_mgr is not None:
                from core.frontier_manager import build_frontier_candidates
                _, fc = build_frontier_candidates(map_mgr, cfg)
                reps = [c.representative for c in fc]
                self._app.update_frontiers(reps)

            # Update HUD
            coverage = map_mgr.explored_free_ratio() if map_mgr else 0.0
            vlm_reasoning = ""
            from planners.vlm_planner import VLMPlanner
            if VLMPlanner._active_instance and VLMPlanner._active_instance._cached_response:
                vlm_reasoning = VLMPlanner._active_instance._cached_response.reasoning
            self._app.update_hud(
                step_idx=step_idx,
                coverage=coverage,
                goal=self._goal_text,
                vlm_reasoning=vlm_reasoning,
            )

            # Check artifact discovery
            if cfg.get("vlm", {}).get("enabled"):
                from core.artifact_manager import ArtifactManager
                # Check via planner's artifact manager
                if VLMPlanner._active_instance and VLMPlanner._active_instance._artifact_mgr:
                    mgr = VLMPlanner._active_instance._artifact_mgr
                    for i, pos in enumerate(self._artifact_positions):
                        if tuple(pos) in mgr.found_positions:
                            self._app.mark_artifact_found(i)

            # Advance 3D frame
            self._app.step()
            self._step_count += 1

    def _inject_planner(self, cfg: dict) -> None:
        from planners.vlm_planner import VLMPlanner
        if VLMPlanner._active_instance is not None:
            VLMPlanner._active_instance.set_image_provider(self.capture_first_person)
            VLMPlanner._active_instance.set_image_mode(self._vlm_mode)
            print(f"[3D] Injected image provider -> VLM mode: {self._vlm_mode}")
        self._planner_injected = True


def main() -> None:
    args = parse_args()

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

    # Generate map + artifacts
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
    # If configured start is inside a wall, relocate to a free corridor cell
    sx, sy = int(start_positions[0][0]), int(start_positions[0][1])
    if grid[sy, sx] != 0:
        free_cells = np.argwhere(grid == 0)  # (row, col) pairs
        if len(free_cells) > 0:
            # Pick the free cell with the most free neighbors (intersection)
            best_idx = 0
            best_n = 0
            for idx, (r, c) in enumerate(free_cells):
                n = sum(1 for dr, dc in [(-1,0),(1,0),(0,-1),(0,1)]
                        if 0 <= r+dr < grid.shape[0] and 0 <= c+dc < grid.shape[1]
                        and grid[r+dr, c+dc] == 0)
                if n > best_n:
                    best_n = n
                    best_idx = idx
            new_start = (int(free_cells[best_idx][1]), int(free_cells[best_idx][0]))
            print(f"[3D] Start ({sx},{sy}) is wall → relocated to {new_start}")
            cfg["robots"]["start_positions"] = [list(new_start)]
            start_positions = cfg["robots"]["start_positions"]
        else:
            # Carve start cell free as fallback
            grid[sy, sx] = 0

    cfg["vlm"]["_artifact_positions"] = artifact_positions
    cfg["environment"]["predefined_map_path"] = None
    cfg["_prebuilt_truth_map"] = grid

    print(f"[VLM 3D] goal: {args.goal!r}")
    print(f"[VLM 3D] env: {args.env}  seed: {args.seed}  vlm-mode: {args.vlm_mode}")
    print(f"[VLM 3D] artifacts: {len(artifact_positions)} at {artifact_positions}")
    print()

    # Output dirs
    out_root = Path(args.output_root)
    run_dir = out_root / f"vlm3d_{args.env}_seed{args.seed}"
    run_dir.mkdir(parents=True, exist_ok=True)
    configs_dir = run_dir / "configs"
    configs_dir.mkdir(parents=True, exist_ok=True)
    serializable_cfg = {k: v for k, v in cfg.items() if not k.startswith("_")}
    write_config_snapshot(configs_dir / "resolved_config.yaml", serializable_cfg)

    # Create the 3D visualizer
    start_pos = (int(start_positions[0][0]), int(start_positions[0][1]))
    runner = Simulation3DRunner(
        grid=grid,
        artifact_positions=artifact_positions,
        start_pos=start_pos,
        vlm_mode=args.vlm_mode,
        goal_text=args.goal,
        window_w=args.window_w,
        window_h=args.window_h,
    )

    # Run episode with 3D callback
    sim = GridSimulation()
    map_name = env.get("map_name", env.get("map_type", "map"))
    stem = f"vlm3d_{map_name}_seed{args.seed}"

    result = sim.run_episode(
        cfg=cfg,
        planner_name="vlm_frontier",
        seed=args.seed,
        output_dir=run_dir,
        animation_stem=stem,
        sample_callback=runner.callback,
    )

    # Summary
    row = dict(result.summary)
    row["goal"] = args.goal
    row["vlm_backend"] = args.vlm_backend
    row["vlm_mode"] = args.vlm_mode
    row["artifact_positions"] = str(artifact_positions)

    summary_csv = run_dir / "vlm_summary.csv"
    pd.DataFrame([row]).to_csv(summary_csv, index=False)

    print()
    print("=" * 60)
    print("VLM 3D Exploration Summary")
    print("=" * 60)
    print(f"  Goal:              {args.goal}")
    print(f"  VLM mode:          {args.vlm_mode}")
    print(f"  Success:           {result.summary.get('success', False)}")
    print(f"  Reason:            {result.summary.get('failure_reason', 'N/A')}")
    print(f"  Steps:             {result.summary.get('completion_steps', 'N/A')}")
    print(f"  Artifact step:     {result.summary.get('artifact_found_step', 'not found')}")
    print(f"  Final coverage:    {result.summary.get('final_coverage', 0):.1%}")
    print(f"  Summary CSV:       {summary_csv}")
    print("=" * 60)


if __name__ == "__main__":
    main()
