# COMP0225 QRC Challange Repo

Gazebo-first dual-Go2 exploration workspace (ROS 2 Humble), centered on CFPA2 + M-TARE style coordination.

## Reproducibility First

Use the full step-by-step guide here:

- [reproducibility.md](reproducibility.md)

That guide includes:

- Linux system packages
- ROS 2 + tooling assumptions
- clone + submodule initialization
- `cmu_env` setup
- Git LFS assets
- build and smoke-check commands
- provenance capture for reproducible runs

## Quick Start

```bash
git clone --recurse-submodules https://github.com/HanshangZhu/COMP0225_LRC_stack.git
cd COMP0225_LRC_stack

git submodule sync --recursive
git submodule update --init --recursive

git lfs install
git lfs pull

micromamba env create -f cmu_env.yml || true
micromamba env update -n cmu_env -f cmu_env.yml --prune

eval "$(micromamba shell hook -s bash)"
micromamba activate cmu_env
source /opt/ros/humble/setup.bash

touch src/mtare_ros1_ws/COLCON_IGNORE

colcon build --symlink-install --cmake-clean-cache \
  --cmake-args -DPython3_EXECUTABLE=$CONDA_PREFIX/bin/python3
```

Runtime setup before running:

```bash
eval "$(micromamba shell hook -s bash)"
micromamba activate cmu_env
source /opt/ros/humble/setup.bash
source install/setup.bash
```

## Main Launch Entrypoints

| Script | Purpose | Notes |
|---|---|---|
| `run_cfpa2_gazebo.sh` | Main dual-robot CFPA2 Gazebo run | Canonical for current development |
| `run_mtare_gazebo.sh` | MTARE ROS2 runs | Alternative coordinator path |
| `run_mtare_ros1_bridge_container.sh` | ROS1 bridge mode | Docker-based legacy integration |
| `run_exploration_issac.sh` | Isaac path | Secondary/stale compared to Gazebo path |

Example:

```bash
./run_cfpa2_gazebo.sh
```

Override args:

```bash
./run_cfpa2_gazebo.sh use_fast_lio:=true switch_hysteresis:=0.08 goal_lock_sec:=6.0
```

## Repository Layout

| Path | Role |
|---|---|
| `src/go2_gazebo_sim` | Gazebo launch graph, assets, control/perception glue, RViz configs |
| `src/cfpa2_collaborative_autonomy` | CFPA2 ROS2 coordinator node |
| `src/mtare_ros2` | MTARE coordinator + shared-map related logic |
| `src/go2_nav_algorithms` | Frontier/navigation helper nodes |
| `src/go2_tare_planner_ros2` | TARE ROS2 backend integration |
| `src/autonomy_stack_go2` | FAR/graph dependencies (submodule) |
| `src/fast_lio` | FAST-LIO ROS2 backend (submodule) |
| `src/unitree-go2-ros2` | Unitree Go2 CHAMP/control stack (submodule) |
| `src/mtare_ros1_ws/src/mtare_planner` | ROS1 TARE planner (submodule) |
| `src/go2_issac_stack` | Isaac-oriented stack (secondary) |

## Submodule Remotes

Configured in `.gitmodules`:

- `src/autonomy_stack_go2` -> `https://github.com/HanshangZhu/autonomy_stack_go2.git`
- `src/fast_lio` -> `https://github.com/HanshangZhu/FAST_LIO_ROS2.git`
- `src/unitree-go2-ros2` -> `https://github.com/HanshangZhu/unitree-go2-ros2.git`
- `src/mtare_ros1_ws/src/mtare_planner` -> `https://github.com/HanshangZhu/tare_planner.git`

## Common Validation

After launch:

```bash
ros2 node list | grep -E "cfpa2_coordinator|robot_status_monitor|reactive_nav"
```

Planner startup logs:

```bash
latest=$(ls -1dt /tmp/ros_logs/20* | head -n1)
grep -n "\[planner_startup\]" "$latest/launch.log"
```

## Notes

- Do not commit generated ROS1 artifacts under `src/mtare_ros1_ws/build` and `src/mtare_ros1_ws/devel`.
- If ownership is wrong due to container builds:

```bash
sudo chown -R "$USER":"$USER" src/mtare_ros1_ws/build src/mtare_ros1_ws/devel
```

## Related Docs

- [reproducibility.md](reproducibility.md)
- [WALKTHROUGH.md](WALKTHROUGH.md)
- [open_problems.md](open_problems.md)
