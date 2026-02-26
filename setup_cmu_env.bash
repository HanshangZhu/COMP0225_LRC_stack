# Source this file to setup the environment for CMU Exploration
# usage: source /path/to/cmu_exploration_ws/setup_cmu_env.bash

CMU_WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROS2_SETUP_BASH="${ROS2_SETUP_BASH:-/opt/ros/humble/setup.bash}"

if ! command -v micromamba >/dev/null 2>&1; then
  echo "ERROR: micromamba not found in PATH."
  return 1 2>/dev/null || exit 1
fi

eval "$(micromamba shell hook -s bash)"
micromamba activate cmu_env

# Source ROS 2 Humble
source "${ROS2_SETUP_BASH}"

# Source the workspace
source "${CMU_WS_DIR}/install/setup.bash"

echo "CMU Exploration environment loaded (cmu_env active)."
