# Reproducibility Guide

This document captures the exact steps to recreate the fallback Gazebo demo from a clean machine.

## 1. Clone with submodules

```bash
git clone --recurse-submodules https://github.com/HanshangZhu/COMP0225_LRC_stack.git
cd COMP0225_LRC_stack
```

If already cloned:

```bash
git submodule sync --recursive
git submodule update --init --recursive
```

## 2. Recreate `cmu_env`

```bash
micromamba env create -f cmu_env.yml
```

If `cmu_env` already exists:

```bash
micromamba env update -n cmu_env -f cmu_env.yml --prune
```

## 3. Build in `cmu_env`

```bash
eval "$(micromamba shell hook -s bash)"
micromamba activate cmu_env
source /opt/ros/humble/setup.bash

colcon build \
  --symlink-install \
  --cmake-clean-cache \
  --cmake-args -DPython3_EXECUTABLE=$CONDA_PREFIX/bin/python3
```

## 4. Run demo

```bash
./start_exploration.sh
```

## 5. Smoke test (headless checks)

In a second terminal:

```bash
./tools/smoke_check.sh
```

Expected pass conditions:
- `/scan` exists and has at least one publisher
- `/odom` exists
- `/way_point` receives at least one message
- `/cmd_vel_stamped` receives at least one message

## 6. Pinning and traceability

Capture exact versions before sharing:

```bash
git rev-parse HEAD
git submodule status
```

Include both outputs in demo notes/PR descriptions.
