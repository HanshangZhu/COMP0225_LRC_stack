# CMU Exploration Workspace

Gazebo-first dual-Go2 exploration workspace (ROS 2 Humble) with CFPA2 and M-TARE based coordination.

## Current Status

- Primary development target: Gazebo stack (`src/go2_gazebo_sim` + `src/mtare_ros2` + `src/cfpa2_collaborative_autonomy`).
- Isaac stack exists (`src/go2_issac_stack`) but is not the primary path.
- ROS1 bridge path is available for legacy TARE experiments.

## Quick Start

1. Create/update environment:

```bash
micromamba env create -f cmu_env.yml || true
micromamba env update -n cmu_env -f cmu_env.yml --prune
```

2. Build:

```bash
eval "$(micromamba shell hook -s bash)"
micromamba activate cmu_env
source /opt/ros/humble/setup.bash

# avoid building ROS1 catkin workspace via colcon
touch src/mtare_ros1_ws/COLCON_IGNORE

colcon build --symlink-install --cmake-clean-cache \
  --cmake-args -DPython3_EXECUTABLE=$CONDA_PREFIX/bin/python3
```

3. Runtime shell:

```bash
source setup_cmu_env.bash
```

## Canonical Run Scripts

| Script | Purpose | Notes |
|---|---|---|
| `run_cfpa2_gazebo.sh` | Main dual-robot CFPA2 run in Gazebo | Launches `go2_gazebo_sim/dual_go2_modular.launch.py` with `profile:=mtare_ros2 planner_backend:=cfpa2` |
| `run_mtare_gazebo.sh` | M-TARE ROS2 path | For MTARE-only runs |
| `run_mtare_ros1_bridge_container.sh` | ROS1 bridge mode | Docker-based ROS1 + `ros1_bridge` workflow |
| `run_exploration_issac.sh` | Isaac workflow | Not the primary repro path |

Override any launch arg by appending `arg:=value`, for example:

```bash
./run_cfpa2_gazebo.sh use_fast_lio:=true cfpa2_frontier_min_cluster_area_m2:=0.0
```

## What Is In `src/`

| Path | Role |
|---|---|
| `src/go2_gazebo_sim` | Gazebo launch graph, asset spawn/control, RViz configs, helper scripts |
| `src/cfpa2_collaborative_autonomy` | CFPA2 ROS2 coordinator node used by `planner_backend:=cfpa2` |
| `src/mtare_ros2` | MTARE ROS2 coordinator and shared map logic |
| `src/go2_nav_algorithms` | Mapping/frontier/nav helper nodes |
| `src/go2_tare_planner_ros2` | TARE ROS2 backend integration |
| `src/autonomy_stack_go2` | FAR/graph stack dependencies (submodule) |
| `src/fast_lio` | FAST-LIO ROS2 backend (submodule) |
| `src/unitree-go2-ros2` | Unitree Go2 CHAMP description/control stack (submodule) |
| `src/mtare_ros1_ws/src/mtare_planner` | Legacy ROS1 TARE planner (submodule) |
| `src/go2_issac_stack` | Isaac-oriented stack (secondary path) |

## Forked Submodules

Configured in `.gitmodules`:

- `src/autonomy_stack_go2` -> `https://github.com/HanshangZhu/autonomy_stack_go2.git` (`foxy-humble`)
- `src/fast_lio` -> `https://github.com/HanshangZhu/FAST_LIO_ROS2.git` (`ros2`)
- `src/unitree-go2-ros2` -> `https://github.com/HanshangZhu/unitree-go2-ros2.git` (`humble`)
- `src/mtare_ros1_ws/src/mtare_planner` -> `https://github.com/HanshangZhu/tare_planner.git` (`melodic-noetic`)

## CFPA2 Launch Notes

- Main wrapper script: `run_cfpa2_gazebo.sh`
- Wrapper launch file: `src/go2_gazebo_sim/launch/two_go2_t_world_cfpa2.launch.py`
- Canonical launch file: `src/go2_gazebo_sim/launch/dual_go2_modular.launch.py`

Coordinator startup is pose-guard gated, with fallback timeout start.  
If planner appears blocked, verify:

```bash
ros2 node list | grep -E "cfpa2_coordinator|robot_status_monitor|initial_pose_guard"
```

And inspect launch events:

```bash
latest=$(ls -1dt /tmp/ros_logs/20* | head -n1)
grep -n "\[planner_startup\]" "$latest/launch.log"
```

## Known Caveats

- `src/mtare_ros1_ws/build` and `src/mtare_ros1_ws/devel` are generated artifacts; do not commit.
- If those ROS1 artifacts are root-owned, fix before normal development:

```bash
sudo chown -R "$USER":"$USER" src/mtare_ros1_ws/build src/mtare_ros1_ws/devel
```

- Git LFS is required for large assets:

```bash
git lfs install
git lfs pull
```

## Related Docs

- [REPRO.md](REPRO.md)
- [WALKTHROUGH.md](WALKTHROUGH.md)
- [open_problems.md](open_problems.md)
