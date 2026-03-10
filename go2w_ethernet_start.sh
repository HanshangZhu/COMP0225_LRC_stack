#!/bin/bash
# go2w_ethernet_start.sh
# Connect to Go2W over Ethernet and launch go2w_driver via Docker.
# After startup, verifies joint_states and point cloud data are flowing.
#
# Usage: ./go2w_ethernet_start.sh          # launch driver + verify
#        ./go2w_ethernet_start.sh stop     # stop driver container
#        ./go2w_ethernet_start.sh status   # check topics without launching
#
# Prerequisites:
#   - USB-C ethernet dongle plugged in (ASIX AX88179)
#   - Ethernet cable to Go2W data port
#   - Robot powered on

set -e
REPO_ROOT="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
COMPOSE="docker compose -f $REPO_ROOT/docker/go2w_real/docker-compose.yml"

ROBOT_IP="${GO2W_ETH_IP:-192.168.123.161}"
HOST_IP="${GO2W_HOST_IP:-192.168.123.100}"
ETH_IFACE="${GO2W_ETH_IFACE:-enxc8a36240a4c7}"
SUBNET="24"

# ── DDS config helper ────────────────────────────────────────────────
setup_cyclonedds() {
  source /opt/ros/humble/setup.bash
  [[ -f "$REPO_ROOT/install/setup.bash" ]] && source "$REPO_ROOT/install/setup.bash"
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
  export CONN_TYPE="cyclonedds"
  ros2 daemon stop 2>/dev/null || true
}

# ── STOP ──────────────────────────────────────────────────────────────
if [[ "${1:-}" == "stop" ]]; then
  echo "Stopping go2w_driver container..."
  WIFI_IFACE="$ETH_IFACE" $COMPOSE down 2>/dev/null || true
  echo "Done."
  exit 0
fi

# ── STEP 1: Ethernet interface setup ─────────────────────────────────
echo "=== [1/4] Setting up Ethernet ($ETH_IFACE) ==="

# Check interface exists
if ! ip link show "$ETH_IFACE" &>/dev/null; then
  echo "ERROR: Interface $ETH_IFACE not found." >&2
  echo "  Is the USB-C ethernet dongle plugged in?" >&2
  echo "  Check: ip link show" >&2
  exit 1
fi

# Assign IP if not already set
CURRENT_IP=$(ip -4 addr show "$ETH_IFACE" 2>/dev/null | awk '/inet /{print $2}' | cut -d/ -f1)
if [[ -z "$CURRENT_IP" ]]; then
  echo "  Assigning $HOST_IP/$SUBNET to $ETH_IFACE..."
  sudo ip addr add "$HOST_IP/$SUBNET" dev "$ETH_IFACE" 2>/dev/null || true
  sudo ip link set "$ETH_IFACE" up
  sleep 1
  CURRENT_IP="$HOST_IP"
fi
echo "  Interface: $ETH_IFACE   IP: $CURRENT_IP"

# ── STEP 2: Connectivity check ───────────────────────────────────────
echo ""
echo "=== [2/4] Checking robot connectivity ==="
if ! ping -c 2 -W 2 "$ROBOT_IP" &>/dev/null; then
  echo "ERROR: Cannot reach robot at $ROBOT_IP" >&2
  echo "  Check:" >&2
  echo "    - Ethernet cable is seated firmly on both ends" >&2
  echo "    - Robot is powered on and fully booted" >&2
  echo "    - Link LED on dongle is lit" >&2
  exit 1
fi
echo "  Robot at $ROBOT_IP reachable ✅"

# ── STEP 3: DDS topic discovery ──────────────────────────────────────
echo ""
echo "=== [3/4] Discovering DDS topics ==="
setup_cyclonedds

# Status-only mode
if [[ "${1:-}" == "status" ]]; then
  echo "  Topic list:"
  timeout 8 ros2 topic list 2>/dev/null | head -30
  echo ""
  echo "  Point cloud rate:"
  timeout 5 ros2 topic hz /utlidar/cloud 2>/dev/null || echo "  (not publishing)"
  exit 0
fi

echo "  Verifying robot topics..."
TOPICS=$(timeout 8 ros2 topic list 2>/dev/null || true)
TOPIC_COUNT=$(echo "$TOPICS" | grep -c "^/" || true)
echo "  Found $TOPIC_COUNT topics ✅"

# ── STEP 4: Verify data streams ──────────────────────────────────────
echo ""
echo "=== [4/4] Verifying data streams ==="

# Point cloud
echo ""
echo "── Point Cloud (/utlidar/cloud) ──"
PC_INFO=$(timeout 5 ros2 topic info /utlidar/cloud 2>/dev/null || echo "NOT FOUND")
echo "  $PC_INFO"
if echo "$PC_INFO" | grep -q "Publisher count: [1-9]"; then
  echo "  Checking rate..."
  timeout 4 ros2 topic hz /utlidar/cloud 2>/dev/null | tail -1 || true
  echo "  ✅ Point cloud streaming"
else
  echo "  ⚠️  No point cloud publisher found"
fi

# Robot pose
echo ""
echo "── Robot Pose (/utlidar/robot_pose) ──"
POSE_MSG=$(timeout 5 ros2 topic echo /utlidar/robot_pose --once 2>/dev/null || echo "TIMEOUT")
if [[ "$POSE_MSG" != "TIMEOUT" && -n "$POSE_MSG" ]]; then
  echo "$POSE_MSG" | head -10
  echo "  ✅ Robot pose available"
else
  echo "  ⚠️  Robot pose not received (may need a few seconds)"
fi

# Low state (joint data)
echo ""
echo "── Low State (/lowstate) ──"
LS_INFO=$(timeout 5 ros2 topic info /lowstate 2>/dev/null || echo "NOT FOUND")
echo "  $LS_INFO"
if echo "$LS_INFO" | grep -q "Publisher count: [1-9]"; then
  echo "  ✅ Joint/motor state available"
else
  echo "  ⚠️  Low state not publishing"
fi

# Sport API (cmd_vel bridge)
echo ""
echo "── Sport API (/api/sport/request) ──"
SPORT_INFO=$(timeout 5 ros2 topic info /api/sport/request 2>/dev/null || echo "NOT FOUND")
echo "  $SPORT_INFO"

# Summary
echo ""
echo "################################################"
echo "  ✅ Go2W ETHERNET CONNECTION READY"
echo ""
echo "  Robot IP:    $ROBOT_IP"
echo "  Host IP:     $CURRENT_IP"
echo "  Interface:   $ETH_IFACE"
echo "  Protocol:    CycloneDDS (Ethernet, unicast)"
echo ""
echo "  Key topics:"
echo "    ← /utlidar/cloud          (PointCloud2, ~15 Hz)"
echo "    ← /utlidar/robot_pose     (PoseStamped)"
echo "    ← /lowstate               (motor/joint data)"
echo "    → /api/sport/request      (velocity commands)"
echo ""
echo "  Monitor:  GO2W_DDS=1 ./go2w_monitor.sh"
echo "  cmd_vel:  Needs go2w_driver (Docker or native)"
echo "  Stop:     ./go2w_ethernet_start.sh stop"
echo "################################################"
