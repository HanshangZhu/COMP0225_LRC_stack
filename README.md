# COMP0225 LRC Stack

Multi-robot autonomous exploration with Unitree Go2W (wheeled-legged quadrupeds) in Gazebo Classic, using CFPA2 coordinated frontier exploration. ROS 2 Humble. Supports both simulation and real-robot deployment over Ethernet, WiFi (DDS), and WebRTC.

## Architecture

```
Gazebo LiDAR (1024×16 gpu_ray @ 10Hz)
  → registered_scan → [qos_bridge] → registered_scan_reliable
                                            │
                          ┌─────────────────┤
                          ▼                 ▼
                  pointcloud_adapter    (use_fast_lio=false only)
                  → velodyne_points     pointcloud_to_laserscan
                          ↓             → scan_3d (LaserScan)
                     Fast-LIO (SLAM)
                          │
            ┌─────────────┼─────────────────┐
            ▼             ▼                 ▼
      /Odometry    cloud_registered_body  (use_fast_lio=true)
      → slam_odom_relay   → pointcloud_to_laserscan
      → odom/nav + TF       → scan_3d (undistorted)
                                            │
                          ┌─────────────────┤
                          ▼                 ▼
                simple_scan_mapper_cpp   reactive_nav (12Hz)
                → /{ns}/map (OccGrid)    ← /{ns}/map (A* global planner)
                  (TRANSIENT_LOCAL QoS)  ← scan_3d (local obstacle avoidance)
                                         → cmd_vel_stamped

Gazebo p3d (50Hz) → odom/ground_truth → slam_odom_relay (GT bootstrap)
```

### Key Nodes

| Node | Language | Role |
|---|---|---|
| `simple_scan_mapper_cpp` | C++ | Builds 2D occupancy grid from LaserScan + TF (log-odds, stop-at-occupied raytrace) |
| `reactive_nav` | Python | Layered navigation: A* on global map + scan-based local avoidance |
| `cfpa2_coordinator_node` | C++ | Joint frontier assignment with overlap penalties, space-time A* |
| `go2w_hybrid_cmd_router` | Python | Routes cmd_vel between leg controller and wheel velocity controller |
| `wall_collision_checker` | Python | Sensor-level obstacle gating (emergency stop) |
| `qos_bridge` | Python | Re-publishes sensor data with RELIABLE QoS |
| `slam_odom_relay` | Python | Odometry normalization + TF broadcast |

### Locomotion

The Go2W is a hybrid platform: 12 leg joints (effort-controlled via CHAMP) + 4 wheel joints (velocity-controlled via `forward_command_controller`). `go2w_hybrid_cmd_router` arbitrates between wheeled and legged modes at runtime.

## Quick Start

Clone the repository:

```bash
git clone --recurse-submodules https://github.com/HanshangZhu/COMP0225_LRC_stack.git
cd COMP0225_LRC_stack
```

Pull submodules and LFS assets:

```bash
git submodule sync --recursive && git submodule update --init --recursive
git lfs install && git lfs pull
```

Create/update the conda environment:

```bash
micromamba env create -f cmu_env.yml || true
micromamba env update -n cmu_env -f cmu_env.yml --prune
```

Activate and build:

```bash
eval "$(micromamba shell hook -s bash)"
micromamba activate cmu_env
source /opt/ros/humble/setup.bash
```

```bash
touch src/mtare_ros1_ws/COLCON_IGNORE
```

```bash
colcon build --symlink-install --cmake-clean-cache \
  --cmake-args -DPython3_EXECUTABLE=$CONDA_PREFIX/bin/python3
```

## Launch Entrypoints

### Simulation

| Script | What it does |
|---|---|
| `demo.sh` | **Single Go2W** with CFPA2 exploration (`gui:=true rviz:=true`) |
| `run_cfpa2_go2w_gazebo.sh` | **Dual Go2W** with CFPA2 coordination (canonical for development) |
| `run_cfpa2_gazebo.sh` | Dual Go2 (non-wheeled variant) with CFPA2 |
| `run_mtare_gazebo.sh` | Dual Go2 with M-TARE ROS2 coordinator |

### Baselines

| Script | What it does |
|---|---|
| `baselines/run_mtare_baseline.sh` | M-TARE baseline benchmark |
| `baselines/run_mui_tare_baseline.sh` | MUI-TARE baseline benchmark |
| `baselines/run_gbplanner2_baseline.sh` | GBPlanner2 baseline benchmark |

### Real Robot

| Script | What it does |
|---|---|
| `go2w_ethernet_start.sh` | Connect to Go2W over **Ethernet** and launch go2w_driver via Docker |
| `go2w_webrtc_start.sh` | Connect to Go2W over **WiFi** and launch go2_ros2_sdk via WebRTC (no Docker) |
| `go2w_monitor.sh` | Topic monitor + optional cmd_vel publisher (works with all connectivity modes) |
| `go2_monitor.sh` | Connect to Go2 (legged) over Ethernet and monitor topics |
| `REAL_GO2W_Connect` | Source file — sets up ROS2 + CycloneDDS unicast for Go2W WiFi |

Real robot launch:

```bash
ros2 launch go2_real_bringup single_go2w_real_cfpa2.launch.py
```

Connects to Go2W via Ethernet (`192.168.123.x`) or WiFi (`192.168.12.x`). Uses Point-LIO for SLAM instead of ground-truth odometry. Includes joystick manual fallback with `cmd_vel_activity_mux`. See [WiFi deployment tutorial](DOCS/tutorials/tutorial_deploy_go2w_wifi.ipynb) for Docker-based WiFi setup.

### Quick examples

Single robot (simplest):

```bash
micromamba activate cmu_env
./demo.sh
```

Dual robot with CFPA2:

```bash
micromamba activate cmu_env
./run_cfpa2_go2w_gazebo.sh
```

### Common overrides

Headless (saves ~2 CPU cores):

```bash
./run_cfpa2_go2w_gazebo.sh gui:=false rviz:=false
```

Enable Fast-LIO SLAM (real-robot or testing):

```bash
./demo.sh use_fast_lio:=true
```

Change world:

```bash
./demo.sh world:=$(ros2 pkg prefix go2_gazebo_sim)/share/go2_gazebo_sim/worlds/l_corridor.world
```

## Repository Layout

```
├── DOCS/                      Documentation, tutorials, research notes
│   ├── tutorials/             Jupyter notebooks (WiFi deploy, cmd_vel)
│   └── research/              Open problems, reproducibility
├── baselines/                 Baseline benchmark scripts + SOTA survey
├── docker/                    Docker configs
│   ├── go2w_real/             Go2W real-robot WiFi deployment
│   └── mtare_ros1_bridge/     ROS1↔ROS2 bridge for TARE planner
├── test/                      Test infrastructure
├── papers/                    Reference papers
src/
├── go2_gazebo_sim/            Gazebo launch files, worlds, URDF/xacro, config
├── go2_real_bringup/          Real robot launch files (extracted from go2_gazebo_sim)
├── go2w_control/              reactive_nav, hybrid_cmd_router, wall_collision_checker
├── go2w_perception/           qos_bridge, slam_odom_relay, pointcloud_adapter, twist_bridge
├── go2w_spawn/                Spawn helpers, initial_pose_guard
├── go2w_observability/        Metrics logger, coverage visualizer, status monitor
├── go2_nav_algorithms/        simple_scan_mapper_cpp, frontier explorer, GBPlanner2
├── cfpa2_collaborative_autonomy/  CFPA2 coordinator + single-robot explorer
├── mtare_ros2/                M-TARE ROS2 coordinator (alternative to CFPA2)
├── go2_ros2_sdk/              Unitree Go2 ROS2 SDK (WebRTC connectivity)
├── go2_tare_planner_ros2/     TARE planner ROS2 port
├── fast_lio/                  Fast-LIO2 ROS2 (submodule)
├── autonomy_stack_go2/        CMU autonomy stack (submodule, provides terrain analysis)
├── unitree-go2-ros2/          CHAMP quadruped controller + Go2 description (submodule)
├── unitree_go2w_ros2/         Go2W driver + description (wheels + legs)
├── librealsense/              Intel RealSense SDK (for depth cameras)
├── stale/                     Deprecated packages (COLCON_IGNORE'd)
│   ├── cfpa2-collaborative-exploration/
│   ├── go2_far_planner/
│   └── go2_issac_stack/
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
| `go2_nav_algorithms/config/gbplanner2_single.yaml` | GBPlanner2 single-robot params |
| `go2_nav_algorithms/config/gbplanner2_dual.yaml` | GBPlanner2 dual-robot params |
| `go2w_control/config/reactive_nav_single_go2w.yaml` | Navigation speeds, planner params (sim) |
| `go2w_control/config/reactive_nav_real_go2w.yaml` | Navigation params (real robot — slower, wider margins) |
| `go2w_control/config/go2w_hybrid_motion.yaml` | Wheel/leg mode blending |
| `go2_gazebo_sim/config/ros_control/ros_control_go2w_robot.yaml` | ros2_control joints + controller_manager rate |
| `go2_gazebo_sim/urdf/go2w/go2w_description_3d_lidar.xacro` | Full robot URDF with sensors |

## SLAM Backends

| Backend | Status | When to use |
|---|---|---|
| **Fast-LIO** | **Default in sim** | `use_fast_lio:=true` (default) — LiDAR-inertial SLAM with motion undistortion |
| Ground-truth odometry | Fallback | `use_fast_lio:=false` — uses Gazebo `p3d` plugin (no motion compensation) |
| Point-LIO | Real robot only | Launched via `go2w_bringup` on the real Go2W |

Fast-LIO provides motion-undistorted point clouds (`cloud_registered_body`) that eliminate wall duplication during turns. The undistorted cloud feeds `pointcloud_to_laserscan` via a static TF `imu → body` (identity) that connects Fast-LIO's body frame to the URDF chain. Key config: `pointlio_gazebo.yaml` (voxel sizes, iterations, body cloud publishing).

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

- [AGENTS.md](AGENTS.md) — AI assistant working instructions + debugging methodology
- [DOCS/research/reproducibility.md](DOCS/research/reproducibility.md) — Full reproducibility guide (system packages, env setup, provenance)
- [DOCS/research/open_problems.md](DOCS/research/open_problems.md) — Known issues and TODOs
- [DOCS/CONTEXT_HANDOFF.md](DOCS/CONTEXT_HANDOFF.md) — Session-to-session development notes
- [DOCS/WALKTHROUGH.md](DOCS/WALKTHROUGH.md) — Implementation walkthrough
- [DOCS/tutorials/](DOCS/tutorials/) — Jupyter tutorials (WiFi deployment, cmd_vel control)
- [baselines/exploration_algorithms_survey.md](baselines/exploration_algorithms_survey.md) — SOTA exploration algorithm survey
