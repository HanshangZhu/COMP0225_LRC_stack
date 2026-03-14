#!/bin/bash
# go2w_webrtc_start.sh
# Connect to Go2W WiFi and launch go2_ros2_sdk via WebRTC.
# No Docker, no Ethernet — works purely over WiFi.
#
# Usage: ./go2w_webrtc_start.sh
# Stop:  Ctrl+C
#
# IMPORTANT: Close the Unitree phone app before running this!
#            WebRTC and the app cannot share the connection.

set -e
REPO_ROOT="$( cd "$( dirname "${BASH_SOURCE[0]}" )/../.." &> /dev/null && pwd )"
SSID="Go2_21585"
PASSWORD="00000000"

# ── STEP 1: WiFi ──────────────────────────────────────────────────────
echo "=== [1/2] Connecting to $SSID ==="
ros2 daemon stop 2>/dev/null || true
CURRENT=$(nmcli -t -f ACTIVE,SSID dev wifi | grep '^yes' | cut -d: -f2)
if [[ "$CURRENT" == "$SSID" ]]; then
  echo "Already connected to $SSID"
else
  nmcli device wifi rescan 2>/dev/null; sleep 2
  nmcli device wifi connect "$SSID" password "$PASSWORD"
fi
sleep 2

# Get WiFi IP
WIFI_IFACE=$(iw dev 2>/dev/null | awk '/Interface/{print $2; exit}')
WIFI_IFACE=${WIFI_IFACE:-wlp0s20f3}
MY_IP=$(ip -4 addr show "$WIFI_IFACE" 2>/dev/null | awk '/inet /{print $2}' | cut -d/ -f1)
echo "  Interface: $WIFI_IFACE   IP: $MY_IP"

# Quick connectivity check
if ! ping -c 1 -W 2 192.168.12.1 &>/dev/null; then
  echo "ERROR: Cannot reach robot at 192.168.12.1 — check robot is powered on." >&2
  exit 1
fi
echo "  Robot reachable ✅"

# ── STEP 2: Launch go2_ros2_sdk (WebRTC) ──────────────────────────────
echo ""
echo "################################################"
echo "=== [2/2] Starting go2_ros2_sdk (WebRTC) ==="
echo "  Robot IP: 192.168.12.1"
echo "  Protocol: WebRTC (WiFi)"
echo "  In another terminal: ./go2w_monitor.sh"
echo "  Press Ctrl+C to stop"
echo "################################################"
echo ""

source /opt/ros/humble/setup.bash
source "$REPO_ROOT/install/setup.bash"
export ROBOT_IP="192.168.12.1"
export CONN_TYPE="webrtc"

# Launch driver-only by default (no Nav2/SLAM/RViz)
# Override with: ./go2w_webrtc_start.sh nav2:=true slam:=true rviz2:=true
ros2 launch go2_robot_sdk robot.launch.py \
  nav2:=false slam:=false rviz2:=false foxglove:=false joystick:=false teleop:=false \
  "$@"
