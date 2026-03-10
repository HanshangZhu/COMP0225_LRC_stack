# Reproducibility Guide (cmu_env + Submodules + System Setup)

This is the canonical reproducibility document for this workspace.

## 1. Target Platform

- Ubuntu 22.04 (recommended)
- ROS 2 Humble installed at `/opt/ros/humble`
- Bash shell
- Internet access for apt/conda/git

## 2. Linux System Packages

Install baseline build/runtime packages:

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

## 3. Install micromamba (if missing)

If `micromamba` is not already installed:

```bash
curl -Ls https://micro.mamba.pm/install.sh | bash
```

Open a new shell after install.

## 4. Clone the Workspace

```bash
git clone --recurse-submodules https://github.com/HanshangZhu/COMP0225_LRC_stack.git
cd COMP0225_LRC_stack
```

Sync/init submodules:

```bash
git submodule sync --recursive
git submodule update --init --recursive
```

Current `.gitmodules` remotes:

- `src/autonomy_stack_go2` -> `https://github.com/HanshangZhu/autonomy_stack_go2.git`
- `src/fast_lio` -> `https://github.com/HanshangZhu/FAST_LIO_ROS2.git`
- `src/unitree-go2-ros2` -> `https://github.com/HanshangZhu/unitree-go2-ros2.git`
- `src/mtare_ros1_ws/src/mtare_planner` -> `https://github.com/HanshangZhu/tare_planner.git`

Optional remote sanity check:

```bash
git -C src/autonomy_stack_go2 remote -v
git -C src/fast_lio remote -v
git -C src/unitree-go2-ros2 remote -v
git -C src/mtare_ros1_ws/src/mtare_planner remote -v
```

## 5. Git LFS Assets

```bash
git lfs install
git lfs pull
```

## 6. CFPA2 Gitlink Caveat

`src/cfpa2-collaborative-exploration` may exist as a gitlink without a `.gitmodules` entry.
If missing after clone, populate it manually:

```bash
rm -rf src/cfpa2-collaborative-exploration
git clone https://github.com/ylhaichen/cfpa2-collaborative-exploration.git src/cfpa2-collaborative-exploration
```

## 7. Create/Update `cmu_env`

```bash
micromamba env create -f cmu_env.yml || true
micromamba env update -n cmu_env -f cmu_env.yml --prune
```

Activate environment and ROS:

```bash
eval "$(micromamba shell hook -s bash)"
micromamba activate cmu_env
source /opt/ros/humble/setup.bash
```

## 8. Install ROS Dependencies (recommended)

Run `rosdep` on ROS2 stacks in this workspace:

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

## 9. Build

Ignore ROS1 catkin workspace during `colcon`:

```bash
touch src/mtare_ros1_ws/COLCON_IGNORE
```

If ROS1 container runs created root-owned files:

```bash
sudo chown -R "$USER":"$USER" src/mtare_ros1_ws/build src/mtare_ros1_ws/devel
```

Build command:

```bash
colcon build --symlink-install --cmake-clean-cache \
  --cmake-args -DPython3_EXECUTABLE=$CONDA_PREFIX/bin/python3
```

Runtime shell setup:

```bash
eval "$(micromamba shell hook -s bash)"
micromamba activate cmu_env
source /opt/ros/humble/setup.bash
source install/setup.bash
```

## 10. Reproduce Main Gazebo Run

Primary command:

```bash
./run_cfpa2_gazebo.sh
```

Optional override example:

```bash
./run_cfpa2_gazebo.sh use_fast_lio:=false switch_hysteresis:=0.05 goal_lock_sec:=5.0
```

Alternative runs:

```bash
./run_mtare_gazebo.sh planner_backend:=mtare_ros2
./run_mtare_ros1_bridge_container.sh start
```

## 11. Smoke Checks

```bash
./tools/smoke_check.sh
```

Manual checks:

```bash
ros2 node list | grep -E "cfpa2_coordinator|robot_status_monitor|reactive_nav"
ros2 topic echo /robot_a/nav_status --once
ros2 topic echo /robot_b/nav_status --once
```

Planner startup diagnostics:

```bash
latest=$(ls -1dt /tmp/ros_logs/20* | head -n1)
grep -n "\[planner_startup\]" "$latest/launch.log"
```

## 12. Record Provenance for Each Repro Run

Capture exact code revisions:

```bash
git rev-parse HEAD
git submodule status --recursive
git -C src/cfpa2-collaborative-exploration rev-parse HEAD || true
```

Capture environment:

```bash
micromamba env export -n cmu_env > artifacts/cmu_env_export.yml
```

## 13. Cleanup / Reset Helpers

Workspace status:

```bash
./tools/status_report.sh
```

Safe cleanup:

```bash
./tools/clean_workspace.sh safe
```

Fresh rebuild cleanup:

```bash
./tools/clean_workspace.sh build
```

## 14. Important Git Note

Pushing parent repo `main` does not push submodule repositories.
Push submodule branches separately, then commit updated submodule pointers in parent.
