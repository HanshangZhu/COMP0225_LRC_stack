# CMU Exploration Workspace

Gazebo-first dual-Go2 exploration workspace with CFPA2, MTARE ROS2, and optional ROS1 bridge support.

## Snapshot Status (2026-02-26)

- Primary development path: Gazebo stack (`src/go2_gazebo_sim` + `src/go2_nav_algorithms` + `src/mtare_ros2`)
- Isaac stack (`src/go2_issac_stack`): present but stale/experimental in this snapshot
- ROS1 MTARE bridge path: supported through Docker (`run_mtare_ros1_bridge_container.sh`)

## Environment and Build

1. Create/update the Python env:

```bash
micromamba env create -f cmu_env.yml || true
micromamba env update -n cmu_env -f cmu_env.yml --prune
```

2. Build ROS2 workspace:

```bash
eval "$(micromamba shell hook -s bash)"
micromamba activate cmu_env
source /opt/ros/humble/setup.bash

# Prevent ROS2 colcon from trying to build ROS1 catkin workspace
touch src/mtare_ros1_ws/COLCON_IGNORE

colcon build --symlink-install --cmake-clean-cache \
  --cmake-args -DPython3_EXECUTABLE=$CONDA_PREFIX/bin/python3
```

3. Runtime shell setup:

```bash
source setup_cmu_env.bash
```

`setup_cmu_env.bash` activates `cmu_env`, sources ROS2 Humble, and sources this workspace.

## Run Entrypoints

| Script | Launch target | Notes |
|---|---|---|
| `run_cfpa2_gazebo.sh` | `go2_gazebo_sim/two_go2_t_world_cfpa2.launch.py` | CFPA2-style wrapper for dual robot Gazebo run |
| `run_mtare_gazebo.sh` | `go2_gazebo_sim/dual_go2_modular.launch.py profile:=mtare_ros2` | Script default is `planner_backend:=tare_ros2_exact`; override to `planner_backend:=mtare_ros2` for the most robust path |
| `run_mtare_ros1_bridge_container.sh` | Docker ROS1 + `ros1_bridge` stack | Bridges ROS1 `tare_planner` with ROS2 graph |
| `run_exploration_issac.sh` | `go2_issac_stack` profiles | Isaac path only; not the primary repro target |

Recommended launch commands:

```bash
./run_cfpa2_gazebo.sh
./run_mtare_gazebo.sh planner_backend:=mtare_ros2
```

## Repository Map

| Path | Role |
|---|---|
| `src/go2_gazebo_sim` | Canonical dual-robot Gazebo launch, spawn/control glue, RViz configs |
| `src/go2_nav_algorithms` | Mapper/frontier/goal-assignment nodes and pipeline builders |
| `src/mtare_ros2` | MTARE coordinator, exact-split bridge/executive, backend integration |
| `src/go2_tare_planner_ros2` | Exact-backend support scaffold |
| `src/autonomy_stack_go2` | FAR/graph/autonomy dependencies (submodule) |
| `src/fast_lio` | FAST-LIO ROS2 backend (submodule) |
| `src/unitree-go2-ros2` | Go2 CHAMP description/controller stack (submodule) |
| `src/mtare_ros1_ws/src/mtare_planner` | ROS1 TARE planner used by bridge mode (submodule) |
| `src/go2_issac_stack/assets/unitree_model` | Vendored Unitree USD assets (Git LFS tracked files) |

## Forked Submodule Remotes

Current submodule remotes are set to fork URLs:

| Submodule | Expected branch | Remote |
|---|---|---|
| `src/autonomy_stack_go2` | `foxy-humble` | `https://github.com/HanshangZhu/autonomy_stack_go2.git` |
| `src/fast_lio` | `ros2` | `https://github.com/HanshangZhu/FAST_LIO_ROS2.git` |
| `src/unitree-go2-ros2` | `humble` | `https://github.com/HanshangZhu/unitree-go2-ros2.git` |
| `src/mtare_ros1_ws/src/mtare_planner` | `melodic-noetic` | `https://github.com/HanshangZhu/tare_planner.git` |

## Known Caveats

- `src/cfpa2-collaborative-exploration` is a gitlink (`160000`) without `.gitmodules` entry.
  - If not populated after clone, manually clone it:

```bash
rm -rf src/cfpa2-collaborative-exploration
git clone https://github.com/ylhaichen/cfpa2-collaborative-exploration.git src/cfpa2-collaborative-exploration
```

- If ROS1 bridge/container previously created root-owned artifacts, fix permissions before normal local builds:

```bash
sudo chown -R "$USER":"$USER" src/mtare_ros1_ws/build src/mtare_ros1_ws/devel
```

- Exact backend (`tare_ros2_exact`) is stricter about FAR/graph dependencies than `mtare_ros2`.

- This repo uses Git LFS for unitree USD assets. After clone:

```bash
git lfs install
git lfs pull
```

## Additional Docs

- Repro procedure: [REPRO.md](REPRO.md)
- Runtime architecture and launch logic: [WALKTHROUGH.md](WALKTHROUGH.md)
