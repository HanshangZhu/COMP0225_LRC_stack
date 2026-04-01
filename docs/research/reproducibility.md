# Reproducibility Guide

This is the canonical reproducibility document for the COMP0225 LRC Stack.

## 1. Target Platform

- Ubuntu 22.04
- ROS 2 Humble installed at `/opt/ros/humble`
- Bash shell
- Internet access for apt/conda/git

## 2. System Packages

```bash
sudo apt update
sudo apt install -y \
  git git-lfs curl wget ca-certificates gnupg lsb-release \
  build-essential cmake ninja-build pkg-config \
  python3-pip python3-venv python3-rosdep \
  python3-colcon-common-extensions python3-vcstool python3-argcomplete \
  libeigen3-dev libyaml-cpp-dev libpcl-dev \
  docker.io
```

If `rosdep` was never initialized:

```bash
sudo rosdep init || true
rosdep update
```

## 3. Install micromamba

```bash
curl -Ls https://micro.mamba.pm/install.sh | bash
```

Open a new shell after install.

## 4. Clone

```bash
git clone --recurse-submodules https://github.com/HanshangZhu/COMP0225_LRC_stack.git
cd COMP0225_LRC_stack

git submodule sync --recursive
git submodule update --init --recursive
git lfs install && git lfs pull
```

### CFPA2 Gitlink Caveat

`src/cfpa2-collaborative-exploration` may exist as a gitlink without a `.gitmodules` entry. If missing:

```bash
rm -rf src/cfpa2-collaborative-exploration
git clone https://github.com/ylhaichen/cfpa2-collaborative-exploration.git src/cfpa2-collaborative-exploration
```

## 5. Environment Setup

```bash
micromamba env create -f cmu_env.yml || true
micromamba env update -n cmu_env -f cmu_env.yml --prune

eval "$(micromamba shell hook -s bash)"
micromamba activate cmu_env
source /opt/ros/humble/setup.bash
```

## 6. Install ROS Dependencies

```bash
rosdep install --from-paths \
  src/go2_gazebo_sim \
  src/go2_nav_algorithms \
  src/mtare_ros2 \
  src/cfpa2_collaborative_autonomy \
  src/go2_tare_planner_ros2 \
  src/autonomy_stack_go2 \
  src/fast_lio \
  src/unitree-go2-ros2 \
  --ignore-src -r -y --rosdistro humble
```

## 7. Build

```bash
touch src/mtare_ros1_ws/COLCON_IGNORE

colcon build --symlink-install --cmake-clean-cache \
  --cmake-args -DPython3_EXECUTABLE=$CONDA_PREFIX/bin/python3
```

Runtime shell setup (needed every new terminal):

```bash
eval "$(micromamba shell hook -s bash)"
micromamba activate cmu_env
source /opt/ros/humble/setup.bash
source install/setup.bash
```

---

## 8. Reproduce Gazebo Simulation

### Single Robot

```bash
./demo.sh
```

### Dual Robot (CFPA2 Coordination)

```bash
./scripts/run_cfpa2_go2w_gazebo.sh
```

### Common Overrides

```bash
# Headless
./scripts/run_cfpa2_go2w_gazebo.sh gui:=false rviz:=false

# Enable Fast-LIO SLAM
./demo.sh use_fast_lio:=true

# Alternative coordinators
./scripts/run_mtare_gazebo.sh planner_backend:=mtare_ros2
```

### Smoke Checks

```bash
ros2 node list | grep -E "cfpa2_coordinator|robot_status_monitor|default_nav"
ros2 topic echo /robot_a/nav_status --once
ros2 topic hz /robot/scan_3d        # expect ~10 Hz
ros2 topic hz /robot/odom/nav       # expect ~50 Hz
```

---

## 9. Reproduce Real Robot Deployment

### Prerequisites

- Unitree Go2W robot, powered on and fully booted
- USB-C Ethernet dongle (ASIX AX88179 or compatible)
- Ethernet cable to Go2W data port

### Step 1: Connect

```bash
./go2w_ethernet_start.sh
```

This will:
1. Configure the Ethernet interface (assign `192.168.123.100/24`)
2. Ping the robot at `192.168.123.161`
3. Set up CycloneDDS with unicast peer discovery
4. Verify sensor topics (`/utlidar/cloud`, `/utlidar/imu`)
5. Start a live topic monitor

### Step 2: Run SLAM

Choose one SLAM backend:

```bash
# Cartographer 3D (built-in loop closure + occupancy grid)
./go2w_ethernet_start.sh cartographer

# Point-LIO (LiDAR-only, fast convergence)
./go2w_ethernet_start.sh pointlio

# Fast-LIO (LiDAR-inertial, motion undistortion)
./go2w_ethernet_start.sh fastlio

# Point-LIO + SC-PGO loop closure
./go2w_ethernet_start.sh pointlio_sam
```

### Step 3: Full Autonomy

```bash
./go2w_ethernet_start.sh autonomy
```

This launches the complete stack:
- `transform_everything` → Cartographer 3D SLAM → `/map` + TF
- `carto_odom_bridge` → `/robot/odom/nav`
- CFPA2 frontier exploration → waypoints
- `default_nav` (A* + local avoidance) → `/cmd_vel`

Joystick manual override available during autonomous operation.

### Step 4: IMU Calibration (Optional)

If no calibration file exists at `hardware/calibration/imu_calib_data.yaml`, the `pointlio` mode will auto-calibrate. To calibrate manually:

```bash
micromamba activate cmu_env
python3 hardware/calibration/calibrate_imu.py
```

Keep the robot **completely still** during calibration (~10s).

### WiFi Alternative

For WiFi (DDS) deployment:

```bash
hardware/scripts/go2w_webrtc_start.sh
```

See also: [WiFi deployment tutorial](../tutorials/tutorial_deploy_go2w_wifi.ipynb)

---

## 10. Offline SLAM Tuning

### Replay Recorded Bags

```bash
# Record point cloud data
./tools/replay/record_pointcloud.sh

# Replay through Cartographer (see /cartographer-tuning workflow)
# Key config: src/go2_real_bringup/config/cartographer/go2w_3d_mapping_tuned.lua
```

### Trajectory Optimization

```bash
python3 tools/slam/optimize_yaw.py     # Yaw bias correction
python3 tools/slam/optimize_loop.py    # Loop closure optimization
python3 tools/slam/optimize_full.py    # Full pose graph optimization
python3 tools/slam/refine_trajectory.py  # ICP-based refinement
```

Results and intermediate data are stored in `data/cartographer_tuning/`.

---

## 11. Record Provenance

For each reproducibility run, capture exact revisions:

```bash
git rev-parse HEAD
git submodule status --recursive
micromamba env export -n cmu_env > artifacts/cmu_env_export.yml
```

## 12. Important Git Note

Pushing the parent repo `main` does **not** push submodule repos. Push submodule branches separately, then commit updated submodule pointers in parent.
