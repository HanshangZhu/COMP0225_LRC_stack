#!/bin/bash
# Go2W topic monitor + optional cmd_vel publisher
# Works with WebRTC (go2w_webrtc_start.sh), DDS/WiFi (Docker), and Ethernet modes.
#
# Usage:
#   ./go2w_monitor.sh                     # watch topics (auto-detect mode)
#   ./go2w_monitor.sh 0.3                 # watch + publish vx=0.3
#   ./go2w_monitor.sh 0.3 0.5            # watch + publish vx=0.3 wz=0.5
#   ./go2w_monitor.sh stop               # send zero velocity
#   GO2W_DDS=1 ./go2w_monitor.sh         # force ethernet/DDS mode
#   GO2W_DDS=1 ./go2w_monitor.sh topics  # list all robot topics (DDS only)
#   GO2W_DDS=1 ./go2w_monitor.sh pc      # stream point cloud info (DDS only)

source /opt/ros/humble/setup.bash
REPO_ROOT="$( cd "$( dirname "${BASH_SOURCE[0]}" )/../.." &> /dev/null && pwd )"
[[ -f "$REPO_ROOT/install/setup.bash" ]] && source "$REPO_ROOT/install/setup.bash"

# ── Ethernet/DDS mode setup ──────────────────────────────────────────
ETH_IFACE="${GO2W_ETH_IFACE:-enxc8a36240a4c7}"
ROBOT_IP="${GO2W_ETH_IP:-192.168.123.161}"

if [[ "${CONN_TYPE:-}" == "cyclonedds" ]] || [[ "${GO2W_DDS:-}" == "1" ]]; then
  export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
  export CYCLONEDDS_URI="<CycloneDDS><Domain>
    <General>
      <Interfaces>
        <NetworkInterface name=\"${ETH_IFACE}\" priority=\"default\" multicast=\"true\" />
      </Interfaces>
    </General>
    <Discovery>
      <Peers><Peer address=\"${ROBOT_IP}\"/></Peers>
      <ParticipantIndex>auto</ParticipantIndex>
    </Discovery>
  </Domain></CycloneDDS>"
  export ROS_DOMAIN_ID=0
  DDS_MODE=1
  echo "📡 Ethernet/DDS mode → $ETH_IFACE → $ROBOT_IP"
fi
ros2 daemon stop 2>/dev/null || true

ARG1="${1:-}"
WZ="${2:-0.0}"

# ── Stop ──────────────────────────────────────────────────────────────
if [[ "$ARG1" == "stop" ]]; then
  ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{}"
  echo "Stopped."
  exit 0
fi

# ── Topic list (DDS only) ────────────────────────────────────────────
if [[ "$ARG1" == "topics" ]]; then
  echo "=== All robot topics ==="
  timeout 8 ros2 topic list 2>/dev/null
  exit 0
fi

# ── Point cloud monitor (DDS only) ───────────────────────────────────
if [[ "$ARG1" == "pc" ]]; then
  echo "=== Point Cloud Monitor ==="
  echo "── /utlidar/cloud rate ──"
  timeout 10 ros2 topic hz /utlidar/cloud 2>/dev/null &
  echo "── /utlidar/cloud_deskewed rate ──"
  timeout 10 ros2 topic hz /utlidar/cloud_deskewed 2>/dev/null &
  wait
  exit 0
fi

# ── Publish cmd_vel in background if vx given ────────────────────────
if [[ -n "$ARG1" && "$ARG1" != "launch" ]]; then
  VX="$ARG1"
  echo "Publishing cmd_vel: vx=$VX wz=$WZ (10 Hz)"
  ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
    "{linear: {x: $VX}, angular: {z: $WZ}}" --rate 10 &
  PUB_PID=$!
  trap "kill $PUB_PID 2>/dev/null; ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist '{}'; exit" INT TERM
fi

# ── Monitor topics ───────────────────────────────────────────────────
if [[ "${DDS_MODE:-}" == "1" ]]; then
  echo ""
  echo "=== Monitoring Go2W topics (Ethernet/DDS) — Ctrl+C to stop ==="
  echo ""

  echo "── /utlidar/robot_pose ──"
  ros2 topic echo /utlidar/robot_pose --field pose.position --field pose.orientation &

  echo "── /utlidar/cloud rate ──"
  ros2 topic hz /utlidar/cloud &

  echo "── /cmd_vel ──"
  ros2 topic echo /cmd_vel 2>/dev/null &

  wait
else
  echo ""
  echo "=== Monitoring Go2W topics (WebRTC) — Ctrl+C to stop ==="
  echo ""

  echo "── /cmd_vel ──"
  ros2 topic echo /cmd_vel &

  echo "── /joint_states ──"
  ros2 topic echo /joint_states --field name --field position &

  wait
fi
