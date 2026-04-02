# Unified Multi-Robot Exploration Framework (CFPA2 / RH-CFPA2 / Physics-RH-CFPA2)

This repository now includes a unified planner framework for high-level multi-robot exploration under one simulator, one metric protocol, and one animation pipeline.

## Key Points

- Shared simulator + observation + frontier + path + metrics + animation pipeline
- Three planners under one interface:
  - `cfpa2`
  - `rh_cfpa2`
  - `physics_rh_cfpa2`
- Predictor abstraction with pluggable backends:
  - `constant_velocity`
  - `path_follow`
  - `physics_residual`
- Go2W-like planner-level realism approximation:
  - footprint/clearance-aware traversability
  - finite FOV + LOS + occlusion
  - heading/turning constraints
  - slowdown near obstacles
  - motion uncertainty and conflict proxies

## Directory Layout

```text
configs/
core/
planners/
predictors/
simulators/grid_sim/
training/
experiments/
tests/
```

## Quick Start

Default unified entrypoint:

```bash
PYTHONPATH=. python main.py --planner cfpa2 --env narrow_t_branches
PYTHONPATH=. python main.py --planner rh_cfpa2 --env narrow_t_branches
PYTHONPATH=. python main.py --planner physics_rh_cfpa2 --env narrow_t_branches
```

Single run with animation:

```bash
PYTHONPATH=. python main.py --planner cfpa2 --env maze --seed 0
```

Planner comparison across maps:

```bash
PYTHONPATH=. python experiments/run_compare.py \
  --envs maze go2w_like narrow_t_branches \
  --seed-start 0 --num-seeds 2 --animate-first-seed-only
```

Predictor and rollout ablations:

```bash
PYTHONPATH=. python experiments/compare_predictors.py \
  --seed-start 0 --num-seeds 2 --disable-animation
```

The predictor comparison now includes planning sensitivity outputs:
- `decision_divergence_rate`
- `chosen_frontier_difference_mean`
- `predictor_rollout_score_variance_mean`

Rollout score-mode ablation (`immediate_only / future_only / hybrid`):

```bash
PYTHONPATH=. python experiments/compare_rollout_score_modes.py \
  --planners rh_cfpa2 physics_rh_cfpa2 \
  --score-modes immediate_only future_only hybrid \
  --seed-start 0 --num-seeds 2 --disable-animation
```

Plot metrics from raw CSV:

```bash
PYTHONPATH=. python experiments/plot_metrics.py \
  --input outputs/benchmarks/<run_id>/results_csv/compare_planners_results.csv
```

Export one animation directly:

```bash
PYTHONPATH=. python experiments/export_animation.py \
  --planner rh_cfpa2 --env-config configs/env_go2w_like.yaml --seed 0
```

Export side-by-side comparison animation:

```bash
PYTHONPATH=. python experiments/export_side_by_side_animation.py \
  --env-config configs/env_maze.yaml --seed 0 --max-steps 240
```

Large-scale residual predictor pipeline (cloud-friendly task split):

```bash
PYTHONPATH=. python training/collect_physics_residual_dataset.py \
  --env-configs configs/env_maze.yaml configs/env_go2w_like.yaml \
  --seed-start 0 --num-seeds 2000 \
  --hard-scenario-oversample-prob 0.70 \
  --hard-scenario-map-types sharp_turn_corridor narrow_t_branches bottleneck_rooms interaction_cross branching_deadend \
  --max-repeat-factor 3 \
  --task-index 0 --num-tasks 8 \
  --output-dir training/datasets/physics_residual_dataset

PYTHONPATH=. python training/merge_dataset_manifests.py \
  --inputs "training/datasets/physics_residual_dataset/task*/manifest.jsonl" \
  --output training/datasets/physics_residual_dataset/manifest_merged.jsonl

PYTHONPATH=. python training/train_physics_residual_torch.py \
  --manifest training/datasets/physics_residual_dataset/manifest_merged.jsonl \
  --output training/models/physics_residual_mlp.pt

PYTHONPATH=. python training/evaluate_physics_residual_torch.py \
  --manifest training/datasets/physics_residual_dataset/manifest_merged.jsonl \
  --checkpoint training/models/physics_residual_mlp.pt
```

## Cloud Execution Checklist

1. Environment
```bash
python -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
pip install -r training/requirements.txt
```

2. Run distributed collection with task split
- Worker `k` uses: `--task-index k --num-tasks N`

3. Merge + train
```bash
PYTHONPATH=. python training/merge_dataset_manifests.py \
  --inputs "training/datasets/physics_residual_dataset/task*/manifest.jsonl" \
  --output training/datasets/physics_residual_dataset/manifest_merged.jsonl

PYTHONPATH=. python training/train_physics_residual_torch.py \
  --manifest training/datasets/physics_residual_dataset/manifest_merged.jsonl \
  --output training/models/physics_residual_mlp.pt \
  --epochs 12 --batch-size 4096 --lr 1e-3 --hidden-dims 256,256
```

4. Benchmark with checkpoint
```bash
PYTHONPATH=. python experiments/run_compare.py \
  --planners cfpa2 rh_cfpa2 physics_rh_cfpa2 \
  --envs narrow_t_branches maze go2w_like \
  --seed-start 0 --num-seeds 5 --max-steps 5000 \
  --physics-weight-file training/models/physics_residual_mlp.pt \
  --run-id cloud_full_compare
```

5. Check predictor planning impact
```bash
PYTHONPATH=. python experiments/compare_predictors.py \
  --env-config configs/env_narrow_t_branches.yaml \
  --planners rh_cfpa2 \
  --predictors path_follow physics_residual \
  --rollout-horizons 3 5 7 \
  --seed-start 0 --num-seeds 5 --max-steps 5000 --disable-animation
```

Metrics to focus on:
- `decision_divergence_rate`
- `chosen_frontier_difference_mean`
- `predictor_rollout_score_variance_mean`

## Tests

```bash
PYTHONPATH=. pytest -q tests
```

## Notes

- This is still a planner-level approximation (not full rigid-body physics simulation).
- Adapter interfaces are provided in `core/adapters.py` for future Gazebo/Unitree integration.
- Legacy baseline scripts are retained for compatibility:
  - `python legacy_main.py`
  - `python experiments/legacy_run_compare.py`
