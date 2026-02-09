#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT_DIR"

eval "$(micromamba shell hook -s bash)"
micromamba activate cmu_env
source /opt/ros/humble/setup.bash
source "$ROOT_DIR/install/setup.bash"

echo "[check] topic list"
ros2 topic list >/tmp/go2_topics.txt

if ! grep -qx '/scan' /tmp/go2_topics.txt; then
  echo "[fail] /scan topic missing"
  exit 1
fi
if ! grep -qx '/odom' /tmp/go2_topics.txt; then
  echo "[fail] /odom topic missing"
  exit 1
fi

echo "[check] /scan publisher count"
scan_info="$(ros2 topic info /scan)"
echo "$scan_info"
if ! echo "$scan_info" | grep -q 'Publisher count: [1-9]'; then
  echo "[fail] /scan has no publishers"
  exit 1
fi

echo "[check] /way_point message"
timeout 12s ros2 topic echo /way_point --once >/tmp/go2_waypoint.txt || {
  echo "[fail] no /way_point message within timeout"
  exit 1
}

echo "[check] /cmd_vel_stamped message"
timeout 12s ros2 topic echo /cmd_vel_stamped --once >/tmp/go2_cmd_vel.txt || {
  echo "[fail] no /cmd_vel_stamped message within timeout"
  exit 1
}

echo "[pass] smoke check passed"
