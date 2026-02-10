#!/bin/bash
# Quick launch script for CMU Exploration with Go2 Robot

# 1. Kill any existing simulations to ensure a clean start
echo "Killing existing processes..."
pkill -9 -f gazebo
pkill -9 -f gzserver
pkill -9 -f gzclient
pkill -9 -f ros2
pkill -9 -f python3

# Wait for cleanup
sleep 2

# 2. Setup Environment (must use cmu_env for all runs)
echo "Setting up environment (cmu_env required)..."
if [ -z "$MAMBA_EXE" ]; then
    echo "ERROR: micromamba is not available in this shell."
    echo "  Please initialize micromamba or open a shell with MAMBA_EXE set."
    exit 1
fi

export MAMBA_ROOT_PREFIX="/home/hz/miniforge3"
eval "$($MAMBA_EXE shell hook --shell bash)"

# 3. Preflight + Launch inside cmu_env to enforce environment usage
echo "Launching Go2 Full Autonomy in L-Corridor (cmu_env)..."
micromamba run -n cmu_env /bin/bash -c "\
  source /opt/ros/humble/setup.bash && \
  source /home/hz/cmu_exploration_ws/install/setup.bash && \
  export RMW_FASTRTPS_USE_SHM=0 && \
  export FASTRTPS_DEFAULT_PROFILES_FILE=/home/hz/cmu_exploration_ws/fastdds_no_shm.xml && \
  export RMW_FASTRTPS_USE_QOS_FROM_XML=1 && \
  echo 'Checking Python/numpy (required for robot spawn and bridges)...' && \
  PYTHON_USED=\$(which python3) && \
  python3 -c 'import numpy' 2>/dev/null || { \
    echo 'ERROR: numpy not found for '\"\$(python3 --version)\"' at '\$PYTHON_USED; \
    echo '  Install in cmu_env: micromamba run -n cmu_env pip install numpy'; \
    exit 1; \
  } && \
  echo '  OK: numpy available' && \
  ros2 launch go2_gazebo_sim full_autonomy.launch.py gui:=true"
