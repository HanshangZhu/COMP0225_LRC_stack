from __future__ import annotations

import argparse
import sys
from pathlib import Path

import imageio.v2 as imageio
import numpy as np

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
    parser = argparse.ArgumentParser(description="Export side-by-side planner comparison animation")
    parser.add_argument("--base-config", type=str, default="configs/base.yaml")
    parser.add_argument("--env-config", type=str, default="configs/env_maze.yaml")
    parser.add_argument("--planners", nargs="+", default=["cfpa2", "rh_cfpa2", "physics_rh_cfpa2"], choices=["cfpa2", "rh_cfpa2", "physics_rh_cfpa2"])
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--max-steps", type=int, default=240)
    parser.add_argument("--run-id", type=str, default=None)
    parser.add_argument("--output-root", type=str, default="outputs")
    parser.add_argument("--fps", type=int, default=8)
    parser.add_argument("--physics-weight-file", type=str, default=None)
    return parser.parse_args()


def _read_frames(video_path: str) -> list[np.ndarray]:
    frames: list[np.ndarray] = []
    reader = imageio.get_reader(video_path)
    for f in reader:
        frames.append(np.asarray(f))
    reader.close()
    return frames


def main() -> None:
    args = parse_args()
    run_id = args.run_id or make_run_id("side_by_side")
    dirs = prepare_output_dirs(args.output_root, run_id)

    sim = GridSimulation()
    planner_videos: list[str] = []

    for planner_name in args.planners:
        cfg = load_experiment_config(
            args.base_config,
            planner_cfg_path=PLANNER_CFG[planner_name],
            env_cfg_path=args.env_config,
        )
        cfg = enforce_mp4_only(cfg)
        cfg["planning"]["planner_name"] = planner_name
        if planner_name == "physics_rh_cfpa2" and args.physics_weight_file is not None:
            cfg["predictor"]["type"] = "physics_residual"
            cfg["predictor"]["physics_residual"]["enabled"] = True
            cfg["predictor"]["physics_residual"]["weight_file"] = args.physics_weight_file
        cfg["termination"]["max_steps"] = int(args.max_steps)
        cfg["experiment"]["save_animation"] = True
        cfg["animation"]["save_gif"] = False
        cfg["animation"]["save_mp4"] = True
        cfg["animation"]["fps"] = int(args.fps)

        map_name = cfg["environment"].get("map_name", cfg["environment"].get("map_type", "map"))
        stem = f"{planner_name}_{map_name}_seed{args.seed}"
        episode_dir = dirs["episode"] / "side_by_side" / planner_name

        result = sim.run_episode(
            cfg=cfg,
            planner_name=planner_name,
            seed=args.seed,
            output_dir=episode_dir,
            animation_stem=stem,
        )

        if result.animation_mp4_path is None:
            raise RuntimeError(f"Missing mp4 output for {planner_name}")
        planner_videos.append(result.animation_mp4_path)
        print(f"planner={planner_name} video={result.animation_mp4_path}", flush=True)

    all_frames = [_read_frames(p) for p in planner_videos]
    max_len = max(len(v) for v in all_frames)
    stitched: list[np.ndarray] = []

    for i in range(max_len):
        row_frames = []
        for seq in all_frames:
            idx = min(i, len(seq) - 1)
            row_frames.append(seq[idx])
        stitched.append(np.concatenate(row_frames, axis=1))

    map_name = load_experiment_config(args.base_config, env_cfg_path=args.env_config).get("environment", {}).get("map_name", "map")
    out_path = Path("outputs/animations") / f"side_by_side_{map_name}_seed{args.seed}.mp4"
    out_path.parent.mkdir(parents=True, exist_ok=True)

    with imageio.get_writer(out_path, fps=args.fps) as writer:
        for f in stitched:
            writer.append_data(f)

    print(f"side_by_side_mp4: {out_path}")


if __name__ == "__main__":
    main()
