# Unified Multi-Robot Exploration Planner

This repository is the **upgraded framework version** of:
- https://github.com/ylhaichen/cfpa2-collaborative-exploration

It keeps CFPA2 baseline behavior, and extends the codebase into a unified planner framework supporting:
- `cfpa2` (centralized myopic joint frontier assignment baseline)
- `rh_cfpa2` (receding-horizon rollout CFPA2)
- `physics_rh_cfpa2` (RH-CFPA2 + pluggable physics residual trajectory predictor)

All planners share the same:
- map generation / occupancy representation
- frontier extraction and representative selection
- path planner and execution loop
- observation model (range + FOV + LOS + occlusion)
- metrics and logging
- animation and video export pipeline

## Default Entrypoints (Unified Framework)

The default execution path is now the unified framework:

```bash
python main.py --planner cfpa2 --env narrow_t_branches
python main.py --planner rh_cfpa2 --env narrow_t_branches
python main.py --planner physics_rh_cfpa2 --env narrow_t_branches
```

Unified comparison wrapper:

```bash
python experiments/run_compare.py --envs maze go2w_like narrow_t_branches --num-seeds 3
```

Legacy compatibility remains available, but it is no longer the default path:

- `python legacy_main.py`
- `python experiments/legacy_run_compare.py`

## Relationship To Original CFPA2 Repo

This repo is not an unrelated rewrite. It is a structured upgrade of the original CFPA2 project:
- CFPA2 core ideas are preserved as the baseline planner backend.
- `rh_cfpa2` and `physics_rh_cfpa2` are added under the same interfaces.
- Existing simulation concepts are retained and generalized into modular `core/`, `planners/`, `predictors/`, `simulators/`, and `experiments/` layers.

If you already used `cfpa2-collaborative-exploration`, this repo is the next-step framework for:
- consistent planner benchmarking
- predictor ablations
- large-scale data collection and training
- future Gazebo / real Go2W high-level integration via adapters

## Project Layout

```text
configs/
core/
planners/
predictors/
simulators/
experiments/
training/
tests/
```

Key files:
- planner interface: `planners/base_planner.py`
- predictor interface: `predictors/base_predictor.py`
- simulator interface: `simulators/base_simulator.py`
- Go2W/Gazebo adapter hooks: `core/adapters.py`

## Environment And Realism Approximation

This is a **planner-level realism approximation**, not a full rigid-body physics simulator.

Implemented approximations include:
- corridor/maze/rooms+bottleneck style maps
- configurable robot footprint/clearance in traversability
- finite sensing range + configurable FOV
- LOS and wall occlusion in observation
- heading update and turning delay behavior
- slowdown near obstacles + motion uncertainty
- reservation/conflict/congestion proxy metrics

## Quick Start

Create env and install dependencies:

```bash
python -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
```

Run one episode:

```bash
MPLCONFIGDIR=/tmp/matplotlib PYTHONPATH=. python main.py \
  --planner cfpa2 \
  --env maze \
  --seed 0
```

Run planner comparison:

```bash
MPLCONFIGDIR=/tmp/matplotlib PYTHONPATH=. python experiments/run_compare.py \
  --envs maze go2w_like narrow_t_branches \
  --seed-start 0 --num-seeds 3 --animate-first-seed-only
```

Run predictor + rollout ablation:

```bash
MPLCONFIGDIR=/tmp/matplotlib PYTHONPATH=. python experiments/compare_predictors.py \
  --planners rh_cfpa2 physics_rh_cfpa2 \
  --predictors path_follow constant_velocity physics_residual \
  --rollout-horizons 3 5 7 \
  --seed-start 0 --num-seeds 3 --disable-animation
```

`compare_predictors.py` now also reports planning-level sensitivity metrics:
- `decision_divergence_rate`
- `chosen_frontier_difference_mean`
- `predictor_rollout_score_variance_mean`

Run RH rollout score-mode ablation (`immediate_only / future_only / hybrid`):

```bash
MPLCONFIGDIR=/tmp/matplotlib PYTHONPATH=. python experiments/compare_rollout_score_modes.py \
  --planners rh_cfpa2 physics_rh_cfpa2 \
  --score-modes immediate_only future_only hybrid \
  --seed-start 0 --num-seeds 3 --disable-animation
```

Export side-by-side planner animation:

```bash
MPLCONFIGDIR=/tmp/matplotlib PYTHONPATH=. python experiments/export_side_by_side_animation.py \
  --env-config configs/env_maze.yaml --seed 0 --max-steps 240
```

## Physics-RH-CFPA2 Data Collection And Training Pipeline

The pipeline is implemented for large-scale cloud runs but **does not auto-run by default**.

### 1) Large-scale dataset collection (sharded)

Use distributed task splitting with `--task-index/--num-tasks`.

Task 0/4 example:

```bash
MPLCONFIGDIR=/tmp/matplotlib PYTHONPATH=. python training/collect_physics_residual_dataset.py \
  --base-config configs/base.yaml \
  --planner-config configs/planner_rh_cfpa2.yaml \
  --env-configs configs/env_maze.yaml configs/env_go2w_like.yaml \
  --planner-name rh_cfpa2 \
  --predictor-type path_follow \
  --seed-start 0 --num-seeds 2000 \
  --episodes-per-seed 1 \
  --max-steps 300 \
  --task-index 0 --num-tasks 4 \
  --shard-size 200000 \
  --hard-scenario-oversample-prob 0.70 \
  --hard-scenario-map-types sharp_turn_corridor narrow_t_branches bottleneck_rooms interaction_cross branching_deadend \
  --max-repeat-factor 3 \
  --output-dir training/datasets/physics_residual_dataset
```

Run task index `0..3` in parallel workers/machines.

### 2) Merge per-task manifests

```bash
PYTHONPATH=. python training/merge_dataset_manifests.py \
  --inputs "training/datasets/physics_residual_dataset/task*/manifest.jsonl" \
  --output training/datasets/physics_residual_dataset/manifest_merged.jsonl
```

### 3) Train torch MLP residual predictor

```bash
PYTHONPATH=. python training/train_physics_residual_torch.py \
  --manifest training/datasets/physics_residual_dataset/manifest_merged.jsonl \
  --output training/models/physics_residual_mlp.pt \
  --epochs 8 --batch-size 2048 --lr 1e-3 --hidden-dims 256,256
```

### 4) Evaluate trained predictor

```bash
PYTHONPATH=. python training/evaluate_physics_residual_torch.py \
  --manifest training/datasets/physics_residual_dataset/manifest_merged.jsonl \
  --checkpoint training/models/physics_residual_mlp.pt
```

### 5) Use trained model in benchmark

```bash
MPLCONFIGDIR=/tmp/matplotlib PYTHONPATH=. python experiments/compare_planners_across_maps.py \
  --env-configs configs/env_maze.yaml \
  --num-seeds 3 \
  --physics-weight-file training/models/physics_residual_mlp.pt
```

### Cloud Run Checklist (Recommended)

1. Prepare machine/env
```bash
git clone <your-upgrade-repo-url>
cd <repo>
python -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
pip install -r training/requirements.txt
```

2. Distributed collection (8 workers example)
- Worker 0 runs `--task-index 0 --num-tasks 8`
- Worker 1 runs `--task-index 1 --num-tasks 8`
- ...
- Worker 7 runs `--task-index 7 --num-tasks 8`

3. Merge manifests and train once all workers finish
```bash
PYTHONPATH=. python training/merge_dataset_manifests.py \
  --inputs "training/datasets/physics_residual_dataset/task*/manifest.jsonl" \
  --output training/datasets/physics_residual_dataset/manifest_merged.jsonl

PYTHONPATH=. python training/train_physics_residual_torch.py \
  --manifest training/datasets/physics_residual_dataset/manifest_merged.jsonl \
  --output training/models/physics_residual_mlp.pt \
  --epochs 12 --batch-size 4096 --lr 1e-3 --hidden-dims 256,256
```

4. Evaluate predictor and run planner benchmark with checkpoint
```bash
PYTHONPATH=. python training/evaluate_physics_residual_torch.py \
  --manifest training/datasets/physics_residual_dataset/manifest_merged.jsonl \
  --checkpoint training/models/physics_residual_mlp.pt

MPLCONFIGDIR=/tmp/matplotlib PYTHONPATH=. python experiments/run_compare.py \
  --planners cfpa2 rh_cfpa2 physics_rh_cfpa2 \
  --envs narrow_t_branches maze go2w_like \
  --seed-start 0 --num-seeds 5 --max-steps 5000 \
  --physics-weight-file training/models/physics_residual_mlp.pt \
  --run-id cloud_full_compare
```

5. Verify predictor impact (decision-level, not only RMSE)
```bash
MPLCONFIGDIR=/tmp/matplotlib PYTHONPATH=. python experiments/compare_predictors.py \
  --env-config configs/env_narrow_t_branches.yaml \
  --planners rh_cfpa2 \
  --predictors path_follow physics_residual \
  --rollout-horizons 3 5 7 \
  --seed-start 0 --num-seeds 5 --max-steps 5000 --disable-animation \
  --run-id cloud_predictor_sensitivity
```

### Myriad One-Command Submission

You can submit the full Myriad pipeline (collect array -> merge -> train+eval) with:

```bash
bash jobs/myriad_submit_full_pipeline.sh
```

Useful overrides:

```bash
NUM_TASKS=96 \
NUM_SEEDS=6000 \
RUN_TAG=myriad_big_run \
GPU_ALLOW_TAG=L \
bash jobs/myriad_submit_full_pipeline.sh
```

Notes:
- `GPU_ALLOW_TAG=L` targets A100-40G nodes; omit it to allow any GPU type.
- `VENV_PATH` defaults to `${REPO_DIR}/.venv`; override if your env lives elsewhere.

### Myriad Parallel Benchmark Submission

The benchmark workload is episode-parallel, so the recommended Myriad mode is a CPU array job rather than one long single-core job. The default submission script uses `36` array tasks and runs CPU-only unless you explicitly re-enable GPU requests:

```bash
NUM_TASKS=36 \
BENCHMARK_TAG=bench36_cpu \
PHYSICS_WEIGHT_FILE=/home/zcably0/physics_runs/full_20260312_172122/models/physics_residual_mlp.pt \
bash jobs/myriad_submit_parallel_benchmark.sh
```

This launches:
- planner comparison shards via `experiments/parallel_compare_planners_shard.py`
- predictor-sensitivity shards via `experiments/parallel_compare_predictors_shard.py`
- one merge job that builds final CSV summaries and plots

To run only the planner comparison stage:

```bash
RUN_COMPARE=1 RUN_PREDICTORS=0 bash jobs/myriad_submit_parallel_benchmark.sh
```

Key planning-sensitivity outputs:
- `decision_divergence_rate`
- `chosen_frontier_difference_mean`
- `predictor_rollout_score_variance_mean`

## Outputs

Each benchmark run writes a reproducible directory:

```text
outputs/benchmarks/<run_id>/
  configs/
  metadata/
  episode_logs/
  results_csv/
  plots/
```

Global animation copies are also written to:
- `outputs/animations/`

## Metrics

Unified metrics include:
- completion steps/time
- final coverage
- path length per robot / total path length
- replans / reassignment / switching
- overlap/duplicate exploration proxy
- conflict / congestion proxy
- planner compute time
- predictor inference time
- prediction error by horizon (`h1/h3/h5`)
- success rate across seeds

## Tests

```bash
MPLCONFIGDIR=/tmp/matplotlib PYTHONPATH=. pytest -q
```

## Notes

- For torch training scripts, install `torch` in your environment.
- Legacy scripts under `cfpa2_demo/` are preserved for reference/regression only; unified framework is the default path.
