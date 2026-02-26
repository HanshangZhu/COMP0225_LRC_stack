#!/bin/bash
# Send a goal to the end of the L-corridor (9, 5)

set -e

WORKSPACE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROS2_SETUP_BASH="${ROS2_SETUP_BASH:-/opt/ros/humble/setup.bash}"

safe_source() {
  local had_u=0
  if [[ $- == *u* ]]; then
    had_u=1
    set +u
  fi
  # shellcheck disable=SC1090
  source "$1"
  local rc=$?
  if [[ ${had_u} -eq 1 ]]; then
    set -u
  fi
  return ${rc}
}

if ! command -v micromamba >/dev/null 2>&1; then
  echo "ERROR: micromamba not found in PATH."
  exit 1
fi

eval "$(micromamba shell hook -s bash)"
micromamba activate cmu_env
safe_source "${ROS2_SETUP_BASH}"
safe_source "${WORKSPACE_DIR}/install/setup.bash"

echo "Sending goal to (9.0, 5.0)..."
ros2 topic pub /way_point geometry_msgs/msg/PointStamped "{header: {frame_id: 'odom'}, point: {x: 9.0, y: 5.0, z: 0.0}}" --once
