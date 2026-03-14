# COMP0225 LRC Stack

Multi-robot autonomous exploration with Unitree Go2W (wheeled-legged quadrupeds) in Gazebo Classic, using Burgard-style CFPA2 coordinated frontier exploration. ROS 2 Humble. Supports both simulation and real-robot deployment over Ethernet, WiFi (DDS), and WebRTC.

## Quick Start

```bash
# Clone
git clone --recurse-submodules https://github.com/HanshangZhu/COMP0225_LRC_stack.git
cd COMP0225_LRC_stack

# Submodules + LFS
git submodule sync --recursive && git submodule update --init --recursive
git lfs install && git lfs pull

# Environment
micromamba env create -f cmu_env.yml || true
micromamba env update -n cmu_env -f cmu_env.yml --prune
eval "$(micromamba shell hook -s bash)"
micromamba activate cmu_env
source /opt/ros/humble/setup.bash

# Build
touch src/mtare_ros1_ws/COLCON_IGNORE
colcon build --symlink-install --cmake-clean-cache \
  --cmake-args -DPython3_EXECUTABLE=$CONDA_PREFIX/bin/python3

# Run (single robot)
./demo.sh

# Run (dual robot with CFPA2)
./scripts/run_cfpa2_go2w_gazebo.sh
```

## Launch Entrypoints

### Simulation

| Script | What it does |
|---|---|
| `demo.sh` | **Single Go2W** with CFPA2 exploration (`gui:=true rviz:=true`) |
| `scripts/run_cfpa2_go2w_gazebo.sh` | **Dual Go2W** with CFPA2 coordination |
| `scripts/run_cfpa2_gazebo.sh` | Dual Go2 (non-wheeled) with CFPA2 |
| `scripts/run_mtare_gazebo.sh` | Dual Go2 with M-TARE ROS2 coordinator |
| `scripts/run_exploration_issac.sh` | Isaac Sim profiles (reference) |

### Real Robot

| Script | What it does |
|---|---|
| `go2w_ethernet_start.sh` | Connect to Go2W over **Ethernet** and launch driver via Docker |
| `hardware/scripts/go2w_webrtc_start.sh` | Connect via **WiFi** / WebRTC (no Docker) |
| `hardware/scripts/go2w_monitor.sh` | Topic monitor + optional cmd_vel publisher |

```bash
# Real robot autonomy launch
ros2 launch go2_real_bringup single_go2w_real_cfpa2.launch.py
```

### Baselines

| Script | What it does |
|---|---|
| `baselines/run_mtare_baseline.sh` | M-TARE baseline benchmark |
| `baselines/run_mui_tare_baseline.sh` | MUI-TARE baseline benchmark |
| `baselines/run_gbplanner2_baseline.sh` | GBPlanner2 baseline benchmark |

### Common Overrides

```bash
# Headless (saves ~2 CPU cores)
./scripts/run_cfpa2_go2w_gazebo.sh gui:=false rviz:=false

# Enable Fast-LIO SLAM
./demo.sh use_fast_lio:=true

# Change world
./demo.sh world:=$(ros2 pkg prefix go2_gazebo_sim)/share/go2_gazebo_sim/worlds/l_corridor.world
```

## Repository Layout

```
├── README.md                     This file
├── AGENTS.md                     AI assistant instructions + debugging methodology
├── demo.sh                       Single-robot launch (daily use)
├── go2w_ethernet_start.sh        Real-robot Ethernet connect (daily use)
├── cmu_env.yml                   Conda/micromamba environment
│
├── scripts/                      Simulation launch scripts
├── hardware/                     Real-robot stack
│   ├── scripts/                  Connectivity, monitoring, calibration scripts
│   ├── docker/                   Docker configs (WiFi deploy, ROS1 bridge)
│   └── calibration/              IMU calibration data + scripts
│
├── tools/                        Offline SLAM & analysis tools
│   ├── slam/                     Trajectory optimization, PGO, bundle adjustment
│   ├── replay/                   Bag replay scripts
│   ├── visualization/            Trajectory plotting
│   └── preprocessing/            Bag preprocessing
│
├── data/                         Generated data artifacts
│   ├── cartographer_tuning/      Tuning bags, results (PNGs, CSVs, meshes)
│   └── live_runs/                Live session recordings
│
├── docs/                         Documentation
│   ├── ARCHITECTURE.md           System architecture & data pipeline
│   ├── CONFIGURATION.md          Config files, worlds, SLAM backends
│   ├── WALKTHROUGH.md            Implementation walkthrough
│   ├── CONTEXT_HANDOFF.md        Development session notes
│   ├── research/                 Open problems, surveys, ideas
│   ├── tutorials/                Jupyter notebooks
│   └── papers/                   Reference PDFs
│
├── baselines/                    Baseline benchmark scripts
├── config/                       Root-level config (FastDDS, etc.)
│
├── src/                          ROS 2 packages
│   ├── go2_gazebo_sim/           Gazebo launch, worlds, URDF/xacro
│   ├── go2_real_bringup/         Real robot launch files
│   ├── go2_nav_algorithms/       Mapper, frontier explorer, GBPlanner2
│   ├── cfpa2_collaborative_autonomy/  CFPA2 coordinator
│   ├── go2w_control/             Reactive nav, hybrid cmd router
│   ├── go2w_perception/          QoS bridge, odom relay, pointcloud adapter
│   ├── go2w_spawn/               Spawn helpers
│   ├── go2w_observability/       Metrics, coverage viz, status monitor
│   ├── mtare_ros2/               M-TARE coordinator
│   ├── fast_lio/                 Fast-LIO2 (submodule)
│   ├── go2_ros2_sdk/             Unitree ROS2 SDK (WebRTC)
│   ├── go2_tare_planner_ros2/   TARE planner port
│   ├── autonomy_stack_go2/       CMU autonomy stack (submodule)
│   ├── unitree-go2-ros2/         CHAMP controller (submodule)
│   ├── unitree_go2w_ros2/        Go2W driver (wheels + legs)
│   ├── librealsense/             Intel RealSense SDK
│   ├── stale/                    Deprecated packages (COLCON_IGNORE'd)
│   └── mtare_ros1_ws/            ROS1 planner (COLCON_IGNORE'd)
│
├── build/                        (gitignored)
├── install/                      (gitignored)
└── log/                          (gitignored)
```

## Further Reading

- [ARCHITECTURE.md](docs/ARCHITECTURE.md) — system design, data pipeline, node graph
- [CONFIGURATION.md](docs/CONFIGURATION.md) — config files, Gazebo worlds, SLAM backends
- [WALKTHROUGH.md](docs/WALKTHROUGH.md) — full implementation walkthrough
- [CONTEXT_HANDOFF.md](docs/CONTEXT_HANDOFF.md) — session-to-session development notes
- [AGENTS.md](AGENTS.md) — AI assistant instructions + debugging methodology
- [Reproducibility guide](docs/research/reproducibility.md) — system packages, env setup
- [Open problems](docs/research/open_problems.md) — known issues and TODOs
- [Exploration survey](docs/research/exploration_algorithms_survey.md) — SOTA algorithm comparison
- [Tutorials](docs/tutorials/) — Jupyter notebooks (WiFi deployment, cmd_vel control)

## Submodule Remotes

| Path | Remote |
|---|---|
| `src/autonomy_stack_go2` | `https://github.com/HanshangZhu/autonomy_stack_go2.git` |
| `src/fast_lio` | `https://github.com/HanshangZhu/FAST_LIO_ROS2.git` |
| `src/unitree-go2-ros2` | `https://github.com/HanshangZhu/unitree-go2-ros2.git` |
| `src/mtare_ros1_ws/src/mtare_planner` | `https://github.com/HanshangZhu/tare_planner.git` |
