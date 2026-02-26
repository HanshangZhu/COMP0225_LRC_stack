# Reproducibility Guide

This guide is for reproducing the current Gazebo-first results from a clean machine.

## 1. Prerequisites

- Ubuntu + ROS2 Humble installed (`/opt/ros/humble`)
- `micromamba` available in shell
- `git lfs` installed
- Docker installed (only needed for ROS1 bridge mode)

## 2. Clone Workspace and Submodules

```bash
git clone --recurse-submodules <your_repo_url> cmu_exploration_ws
cd cmu_exploration_ws

git submodule sync --recursive
git submodule update --init --recursive
```

Optional sanity check for submodule remotes:

```bash
git -C src/autonomy_stack_go2 remote -v
git -C src/fast_lio remote -v
git -C src/unitree-go2-ros2 remote -v
git -C src/mtare_ros1_ws/src/mtare_planner remote -v
```

## 3. Pull LFS Assets

```bash
git lfs install
git lfs pull
```

## 4. CFPA2 Gitlink Caveat

`src/cfpa2-collaborative-exploration` is currently tracked as a gitlink without `.gitmodules`.
If the directory is missing/empty, clone it manually:

```bash
rm -rf src/cfpa2-collaborative-exploration
git clone https://github.com/ylhaichen/cfpa2-collaborative-exploration.git src/cfpa2-collaborative-exploration
```

## 5. Create/Update Environment

```bash
micromamba env create -f cmu_env.yml || true
micromamba env update -n cmu_env -f cmu_env.yml --prune

eval "$(micromamba shell hook -s bash)"
micromamba activate cmu_env
source /opt/ros/humble/setup.bash
```

## 6. Build ROS2 Workspace

Ignore ROS1 catkin workspace during colcon build:

```bash
touch src/mtare_ros1_ws/COLCON_IGNORE
```

If ROS1 bridge runs created root-owned artifacts, fix before build:

```bash
sudo chown -R "$USER":"$USER" src/mtare_ros1_ws/build src/mtare_ros1_ws/devel
```

Build:

```bash
colcon build --symlink-install --cmake-clean-cache \
  --cmake-args -DPython3_EXECUTABLE=$CONDA_PREFIX/bin/python3
```

Runtime setup:

```bash
source setup_cmu_env.bash
```

## 7. Run Experiments

CFPA2 wrapper flow:

```bash
./run_cfpa2_gazebo.sh
```

MTARE ROS2 flow (recommended default for this snapshot):

```bash
./run_mtare_gazebo.sh planner_backend:=mtare_ros2
```

Exact backend (optional, stricter dependencies):

```bash
./run_mtare_gazebo.sh planner_backend:=tare_ros2_exact
```

ROS1 bridge mode (optional):

```bash
./run_mtare_ros1_bridge_container.sh start
```

## 8. Smoke Checks

In another terminal:

```bash
source setup_cmu_env.bash
./tools/smoke_check.sh
```

Manual checks:

```bash
ros2 topic list | rg 'robot_a|robot_b|mtare|way_point|nav_status'
ros2 topic echo /robot_a/nav_status --once
ros2 topic echo /robot_b/nav_status --once
```

## 9. Record Provenance

Capture revision metadata with each run:

```bash
git rev-parse HEAD
git submodule status
git -C src/cfpa2-collaborative-exploration rev-parse HEAD || true
```

If ROS1 bridge used:

```bash
docker image inspect cmu-mtare-ros1-bridge:foxy-noetic --format '{{json .Config.Labels}}'
```

## 10. Cleanup Helpers

```bash
./tools/status_report.sh
./tools/clean_workspace.sh safe
```

For a fresh rebuild only:

```bash
./tools/clean_workspace.sh build
```

## 11. Important Git Note

Pushing parent repo `main` does not push submodule repos.
You must push each submodule branch separately, then commit updated submodule pointers in parent.
