#!/usr/bin/env bash
set -euo pipefail

WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROS2_SETUP_BASH="${ROS2_SETUP_BASH:-/opt/ros/humble/setup.bash}"

safe_source() {
  set +u
  # shellcheck disable=SC1090
  source "$1"
  set -u
}

if [[ -f "${HOME}/miniforge3/etc/profile.d/conda.sh" ]]; then
  safe_source "${HOME}/miniforge3/etc/profile.d/conda.sh"
  conda activate cmu_env
elif command -v micromamba >/dev/null 2>&1; then
  eval "$(micromamba shell hook -s bash)"
  micromamba activate cmu_env
else
  echo "Could not find conda/micromamba activation script for cmu_env." >&2
  exit 1
fi

if [[ ! -f "${ROS2_SETUP_BASH}" ]]; then
  echo "Missing ROS2 setup script: ${ROS2_SETUP_BASH}" >&2
  exit 1
fi
safe_source "${ROS2_SETUP_BASH}"

if [[ ! -f "${WS_DIR}/install/setup.bash" ]]; then
  echo "Missing ${WS_DIR}/install/setup.bash. Build first: colcon build --symlink-install" >&2
  exit 1
fi
safe_source "${WS_DIR}/install/setup.bash"

export ROS_LOG_DIR="${ROS_LOG_DIR:-/tmp/ros_logs}"
mkdir -p "${ROS_LOG_DIR}"

exec ros2 launch go2_gazebo_sim two_go2_t_world_cfpa2.launch.py "$@"
