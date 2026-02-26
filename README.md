# CMU Exploration Workspace (Gazebo-First)

This repository is a working fork/integration workspace for multi-robot exploration experiments.

## Status

- Gazebo stack is the active development target.
- Isaac stack (`src/go2_issac_stack`) is currently stale and not the primary path for new work.
- Default recommendation: run and develop on the Gazebo launch flow.

## Quick Start

1. Create/update `cmu_env`:
```bash
micromamba env create -f cmu_env.yml
# or, if env already exists:
micromamba env update -n cmu_env -f cmu_env.yml --prune
```
2. Build:
```bash
eval "$(micromamba shell hook -s bash)"
micromamba activate cmu_env
source /opt/ros/humble/setup.bash
colcon build --symlink-install --cmake-clean-cache --cmake-args -DPython3_EXECUTABLE=$CONDA_PREFIX/bin/python3
```
3. Load runtime env in each shell:
```bash
source setup_cmu_env.bash
```
4. Launch Gazebo stack:
```bash
./run_cfpa2_gazebo.sh
# or
./run_mtare_gazebo.sh
```

## `cmu_env` and Runtime Setup

- `cmu_env.yml` pins Python and package dependencies used in this workspace.
- `setup_cmu_env.bash` does three things:
  - activates `cmu_env` via micromamba,
  - sources ROS 2 Humble (`/opt/ros/humble/setup.bash`),
  - sources this workspace (`install/setup.bash`).
- If `install/setup.bash` is missing, build first with `colcon`.

## Launch Shell Scripts

- `run_cfpa2_gazebo.sh`
  - Activates `cmu_env` (conda or micromamba fallback).
  - Sources ROS/workspace setup.
  - Launches `two_go2_t_world_cfpa2.launch.py`.
  - Use this for CFPA2 two-robot Gazebo experiments.

- `run_mtare_gazebo.sh`
  - Activates `cmu_env`, sources ROS/workspace.
  - Optionally clears stale Gazebo processes.
  - Auto-selects a free `GAZEBO_MASTER_URI` port.
  - Sets `FASTRTPS_DEFAULT_PROFILES_FILE` when available.
  - Launches `dual_go2_modular.launch.py` with MTARE profile/backend.

- `send_goal.sh`
  - Publishes one `geometry_msgs/PointStamped` goal to `/way_point`.
  - Useful for manual sanity checks.

## Package Overview

| Path | Role | Source / Fork |
|---|---|---|
| `src/go2_gazebo_sim` | Main Gazebo simulation/launch stack (active development). | In-tree (this repo) |
| `src/go2_issac_stack` | Isaac-based stack, kept for reference; currently stale. | In-tree (this repo) |
| `src/go2_far_planner` | FAR planner integration code and configs. | In-tree (this repo) |
| `src/mtare_ros2` | ROS 2 side of MTARE integrations. | In-tree (this repo) |
| `src/mtare_ros1_ws` | ROS 1 workspace used by MTARE bridge flow. | In-tree (this repo) |
| `src/autonomy_stack_go2` | CMU base autonomy stack submodule. | https://github.com/jizhang-cmu/autonomy_stack_go2 |
| `src/fast_lio` | FAST-LIO ROS 2 fork submodule used for LiDAR-inertial odometry. | https://github.com/Ericsii/FAST_LIO_ROS2 |
| `src/unitree-go2-ros2` | Go2 CHAMP ROS 2 stack (workspace fork submodule). | https://github.com/HanshangZhu/unitree-go2-ros2 |
| `src/mtare_ros1_ws/src/mtare_planner` | TARE planner submodule (workspace fork). | https://github.com/HanshangZhu/tare_planner |
| `src/cfpa2-collaborative-exploration` | CFPA2 algorithm package snapshot/subrepo used by the Gazebo experiments. | https://github.com/ylhaichen/cfpa2-collaborative-exploration |

## Reproducibility Notes

- Use `REPRO.md` for the full clean-machine procedure.
- Before sharing results, record:
```bash
git rev-parse HEAD
git submodule status
```
