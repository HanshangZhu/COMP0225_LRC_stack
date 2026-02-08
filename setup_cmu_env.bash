# Source this file to setup the environment for CMU Exploration
# usage: source ~/cmu_exploration_ws/setup_cmu_env.bash

if ! command -v micromamba >/dev/null 2>&1; then
  echo "ERROR: micromamba not found in PATH."
  return 1 2>/dev/null || exit 1
fi

eval "$(micromamba shell hook -s bash)"
micromamba activate cmu_env

# Source ROS 2 Humble
source /opt/ros/humble/setup.bash

# Source the workspace
source ~/cmu_exploration_ws/install/setup.bash

echo "CMU Exploration environment loaded (cmu_env active)."
