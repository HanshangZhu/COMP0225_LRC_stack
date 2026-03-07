# COMP0225 LRC Stack

Multi-robot autonomous exploration with Unitree Go2W (wheeled-legged quadrupeds) in Gazebo Classic, using CFPA2 coordinated frontier exploration. ROS 2 Humble.

## Architecture

```
Gazebo LiDAR (1024×16 gpu_ray @ 10Hz)
  → registered_scan → [qos_bridge] → registered_scan_reliable
                                            │
                          ┌─────────────────┤
                          ▼                 ▼
                  pointcloud_adapter    pointcloud_to_laserscan
                  (Fast-LIO, optional)  → scan_3d (LaserScan)
                                            │
                          ┌─────────────────┤
                          ▼                 ▼
                simple_scan_mapper_cpp   reactive_nav (12Hz)
                → /{ns}/map (OccGrid)    ← /{ns}/map (A* global planner)
                                         ← scan_3d (local obstacle avoidance)
                                         → cmd_vel_stamped

Gazebo p3d (50Hz) → odom/ground_truth → slam_odom_relay → odom/nav + TF
```

### Key Nodes

| Node | Language | Role |
|---|---|---|
| `simple_scan_mapper_cpp` | C++ | Builds 2D occupancy grid from LaserScan + TF |
| `reactive_nav` | Python | Layered navigation: A* on global map + scan-based local avoidance |
| `cfpa2_coordinator_node` | C++ | Joint frontier assignment with overlap penalties, space-time A* |
| `go2w_hybrid_cmd_router` | Python | Routes cmd_vel between leg controller and wheel velocity controller |
| `wall_collision_checker` | Python | Sensor-level obstacle gating (emergency stop) |
| `qos_bridge` | Python | Re-publishes sensor data with RELIABLE QoS |
| `slam_odom_relay` | Python | Odometry normalization + TF broadcast |

### Locomotion

The Go2W is a hybrid platform: 12 leg joints (effort-controlled via CHAMP) + 4 wheel joints (velocity-controlled via `forward_command_controller`). `go2w_hybrid_cmd_router` arbitrates between wheeled and legged modes at runtime.

## Quick Start

```bash
git clone --recurse-submodules https://github.com/HanshangZhu/COMP0225_LRC_stack.git
cd COMP0225_LRC_stack

git submodule sync --recursive && git submodule update --init --recursive
git lfs install && git lfs pull

micromamba env create -f cmu_env.yml || true
micromamba env update -n cmu_env -f cmu_env.yml --prune

eval "$(micromamba shell hook -s bash)"
micromamba activate cmu_env
source /opt/ros/humble/setup.bash

touch src/mtare_ros1_ws/COLCON_IGNORE

colcon build --symlink-install --cmake-clean-cache \
  --cmake-args -DPython3_EXECUTABLE=$CONDA_PREFIX/bin/python3
```

## Launch Entrypoints

| Script | What it does |
|---|---|
| `demo.sh` | **Single Go2W** with CFPA2 exploration (`gui:=true rviz:=true`) |
| `run_cfpa2_go2w_gazebo.sh` | **Dual Go2W** with CFPA2 coordination (canonical for development) |
| `run_cfpa2_gazebo.sh` | Dual Go2 (non-wheeled variant) with CFPA2 |
| `run_mtare_gazebo.sh` | Dual Go2 with M-TARE ROS2 coordinator |
| `start_go2w_qrc_demo.sh` | QRC demo entrypoint |

### Single robot (simplest)

```bash
micromamba activate cmu_env
./demo.sh
```

### Dual robot with CFPA2

```bash
micromamba activate cmu_env
./run_cfpa2_go2w_gazebo.sh
```

### Common overrides

```bash
# Headless (saves ~2 CPU cores)
./run_cfpa2_go2w_gazebo.sh gui:=false rviz:=false

# Enable Fast-LIO SLAM (real-robot or testing)
./demo.sh use_fast_lio:=true

# Change world
./demo.sh world:=$(ros2 pkg prefix go2_gazebo_sim)/share/go2_gazebo_sim/worlds/l_corridor.world
```

### Real robot

```bash
ros2 launch go2_gazebo_sim single_go2w_real_cfpa2.launch.py
```

Connects to Go2W via Ethernet (`192.168.123.x`). Uses Point-LIO for SLAM instead of ground-truth odometry. Includes joystick manual fallback with `cmd_vel_activity_mux`.

## Repository Layout

```
src/
├── go2_gazebo_sim/            Gazebo launch files, worlds, URDF/xacro, config
├── go2w_control/              reactive_nav, hybrid_cmd_router, wall_collision_checker
├── go2w_perception/           qos_bridge, slam_odom_relay, pointcloud_adapter, twist_bridge
├── go2w_spawn/                Spawn helpers, initial_pose_guard
├── go2w_observability/        Metrics logger, coverage visualizer, status monitor
├── go2_nav_algorithms/        simple_scan_mapper_cpp, frontier explorer
├── cfpa2_collaborative_autonomy/  CFPA2 coordinator + single-robot explorer
├── cfpa2-collaborative-exploration/  CFPA2 algorithm source / research code
├── mtare_ros2/                M-TARE ROS2 coordinator (alternative to CFPA2)
├── go2_far_planner/           FAR planner ROS2 port
├── go2_tare_planner_ros2/     TARE planner ROS2 port
├── fast_lio/                  Fast-LIO2 ROS2 (submodule)
├── autonomy_stack_go2/        CMU autonomy stack (submodule, provides terrain analysis)
├── unitree-go2-ros2/          CHAMP quadruped controller + Go2 description (submodule)
├── unitree_go2w_ros2/         Go2W description (wheels + legs)
├── go2_issac_stack/           Isaac Sim integration (secondary, experimental)
└── mtare_ros1_ws/             ROS1 TARE planner (COLCON_IGNORE'd, Docker bridge only)
```

## Gazebo Worlds

| World | Description |
|---|---|
| `3.world` | Multi-corridor environment with T-junction and L-branch (default) |
| `t_dual_corridor.world` | T-shaped dual corridor |
| `l_corridor.world` | L-shaped corridor |
| `empty_test.world` | Empty world for debugging |

## Configuration Files

| Config | Purpose |
|---|---|
| `go2_gazebo_sim/config/nav/simple_scan_mapper_single_go2w.yaml` | Mapper scoring (hit/miss/thresholds) |
| `go2_nav_algorithms/config/nav/geometric_frontier_single.yaml` | Frontier detection parameters |
| `go2w_control/config/reactive_nav_single_go2w.yaml` | Navigation speeds, planner params (sim) |
| `go2w_control/config/reactive_nav_real_go2w.yaml` | Navigation params (real robot — slower, wider margins) |
| `go2w_control/config/go2w_hybrid_motion.yaml` | Wheel/leg mode blending |
| `go2_gazebo_sim/config/ros_control/ros_control_go2w_robot.yaml` | ros2_control joints + controller_manager rate |
| `go2_gazebo_sim/urdf/go2w/go2w_description_3d_lidar.xacro` | Full robot URDF with sensors |

## SLAM Backends

| Backend | Status | When to use |
|---|---|---|
| Ground-truth odometry | **Default in sim** | `use_fast_lio:=false` — uses Gazebo `p3d` plugin |
| Fast-LIO | Available | `use_fast_lio:=true` — LiDAR-inertial fusion |
| Point-LIO | Real robot only | Launched via `go2w_bringup` on the real Go2W |

Fast-LIO is disabled by default in Gazebo because unrealistic IMU noise and LiDAR-IMU timestamp sync issues cause drift and "No Effective Points" errors. Ground-truth odometry with TF-based scan projection produces cleaner maps in simulation.

## Performance Notes

The simulation runs slower than real time on most hardware. The main bottlenecks are:

1. **ODE physics** at 500 Hz × 25 solver iterations (single-threaded `gzserver`)
2. **GPU ray LiDAR** — 16,384 rays per frame per robot
3. **Process count** — dual-robot pipeline spawns ~50 ROS 2 processes
4. **Rendering** — `gzclient` + `rviz` consume 1.5–2.5 CPU cores

Run headless (`gui:=false rviz:=false`) for experiments. See [AGENTS.md](AGENTS.md) for debugging methodology.

## Validation

After launch, verify nodes are running:

```bash
ros2 node list | grep -E "cfpa2|reactive_nav|simple_scan_mapper"
ros2 topic hz /robot/scan_3d        # expect ~10 Hz
ros2 topic hz /robot/odom/nav       # expect ~50 Hz
ros2 topic hz /robot/map            # expect ~4 Hz
```

## Submodule Remotes

| Path | Remote |
|---|---|
| `src/autonomy_stack_go2` | `https://github.com/HanshangZhu/autonomy_stack_go2.git` |
| `src/fast_lio` | `https://github.com/HanshangZhu/FAST_LIO_ROS2.git` |
| `src/unitree-go2-ros2` | `https://github.com/HanshangZhu/unitree-go2-ros2.git` |
| `src/mtare_ros1_ws/src/mtare_planner` | `https://github.com/HanshangZhu/tare_planner.git` |

## Related Docs

- [reproducibility.md](reproducibility.md) — Full reproducibility guide (system packages, env setup, provenance)
- [AGENTS.md](AGENTS.md) — AI assistant working instructions + debugging methodology
- [CONTEXT_HANDOFF.md](CONTEXT_HANDOFF.md) — Session-to-session development notes
- [open_problems.md](open_problems.md) — Known issues and TODOs
- [WALKTHROUGH.md](WALKTHROUGH.md) — Implementation walkthrough
