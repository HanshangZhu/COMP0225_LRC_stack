#!/bin/bash
# Send a goal to the end of the L-corridor (9, 5)

set -e

WORKSPACE_DIR="/home/hz/cmu_exploration_ws"

if ! command -v micromamba >/dev/null 2>&1; then
  echo "ERROR: micromamba not found in PATH."
  exit 1
fi

eval "$(micromamba shell hook -s bash)"
micromamba activate cmu_env
source /opt/ros/humble/setup.bash
source "${WORKSPACE_DIR}/install/setup.bash"

echo "Sending goal to (9.0, 5.0)..."
ros2 topic pub /way_point geometry_msgs/msg/PointStamped "{header: {frame_id: 'odom'}, point: {x: 9.0, y: 5.0, z: 0.0}}" --once
