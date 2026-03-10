#!/bin/bash
# go2_monitor.sh — Connect to Go2 (legged) via Ethernet and monitor topics
#
# Usage:
#   ./go2_monitor.sh                 # monitor state + odom + joints
#   ./go2_monitor.sh launch          # launch driver first, then monitor
#   ./go2_monitor.sh 0.2             # monitor + publish vx=0.2
#   ./go2_monitor.sh 0.2 0.3         # monitor + publish vx=0.2 wz=0.3
#   ./go2_monitor.sh stop            # send near-zero to stop robot
#
# Ethernet Go2 default IP: 192.168.123.161

set -e
REPO_ROOT="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
ROBOT_IP="${GO2_IP:-192.168.123.161}"

# ── Source ROS2 + workspace ──────────────────────────────────────────
source /opt/ros/humble/setup.bash
[[ -f "$REPO_ROOT/install/setup.bash" ]] && source "$REPO_ROOT/install/setup.bash"

export ROBOT_IP="$ROBOT_IP"
export CONN_TYPE="webrtc"

ros2 daemon stop 2>/dev/null || true

# ── Connectivity check ───────────────────────────────────────────────
echo "Checking Go2 at $ROBOT_IP ..."
if ! ping -c 1 -W 2 "$ROBOT_IP" &>/dev/null; then
  echo "ERROR: Cannot reach robot at $ROBOT_IP" >&2
  echo "  Check Ethernet cable and that your interface has an IP in 192.168.123.x" >&2
  echo "  Hint: sudo ip addr add 192.168.123.100/24 dev <eth_iface>" >&2
  exit 1
fi
echo "  Robot reachable ✅"

# ── Handle arguments ─────────────────────────────────────────────────
ARG1="${1:-}"

# Stop command
if [[ "$ARG1" == "stop" ]]; then
  echo "Sending stop command..."
  ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
    "{linear: {x: 0.001, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
  echo "Stopped."
  exit 0
fi

# Launch driver
if [[ "$ARG1" == "launch" ]]; then
  echo ""
  echo "################################################"
  echo "  Launching go2_driver_node (WebRTC)"
  echo "  Robot IP: $ROBOT_IP"
  echo "  Ctrl+C to stop"
  echo "################################################"
  echo ""
  ros2 launch go2_robot_sdk webrtc_web.launch.py \
    robot_ip:="$ROBOT_IP" \
    enable_video:=false \
    enable_foxglove_bridge:=false
  exit 0
fi

# Publish cmd_vel in background if velocity given
if [[ -n "$ARG1" && "$ARG1" != "launch" ]]; then
  VX="$ARG1"
  WZ="${2:-0.0}"
  echo "Publishing cmd_vel: vx=$VX wz=$WZ (10 Hz)"
  ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
    "{linear: {x: $VX, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0, z: $WZ}}" --rate 10 &
  PUB_PID=$!
  trap "kill $PUB_PID 2>/dev/null; \
    ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
      '{linear: {x: 0.001}, angular: {z: 0.0}}'; exit" INT TERM
fi

# ── Monitor topics ───────────────────────────────────────────────────
echo ""
echo "=== Monitoring Go2 topics (Ctrl+C to stop) ==="
echo ""

# State
echo "--- /go2_states ---"
ros2 topic echo /go2_states --once 2>/dev/null &

# Odom
echo "--- /odom (streaming) ---"
ros2 topic echo /odom --field pose.pose.position --field twist.twist.linear &

# Joint states
echo "--- /joint_states ---"
ros2 topic echo /joint_states --field name --field position &

wait
