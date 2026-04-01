#!/bin/bash
# go2w_start_autonomy_wifi.sh
# Same pipeline as go2w_start_autonomy.sh but over robot WiFi AP (CycloneDDS unicast).
#
# Usage: ./go2w_start_autonomy_wifi.sh                    # default (scan mapper, OA disabled)
#        ./go2w_start_autonomy_wifi.sh scan false         # scan mapper, OA disabled (api_id=1008)
#        ./go2w_start_autonomy_wifi.sh octomap true      # octomap mapper, OA enabled
#        ./go2w_start_autonomy_wifi.sh elevation false  # elevation mapper, OA disabled
#        ./go2w_start_autonomy_wifi.sh stop              # kill all autonomy processes
#
# Env:
#   GO2W_WIFI_SSID       — robot AP SSID (default Go2_21585; some units use Unitree_Go2W*)
#   GO2W_WIFI_PASSWORD   — AP password (default 00000000; try go2-w if needed)
#   GO2W_WIFI_IFACE      — optional; else first interface from `iw dev`
#   GO2W_WIFI_ROBOT_IP   — default 192.168.12.1
#
# Prerequisites:
#   - NetworkManager (nmcli) available
#   - Laptop WiFi connected to the Go2W access point
#   - Close Unitree phone app if using WebRTC elsewhere on the same robot

set -e
REPO_ROOT="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"

ROBOT_IP="${GO2W_WIFI_ROBOT_IP:-192.168.12.1}"  # Robot gateway on WiFi subnet
WIFI_SSID="${GO2W_WIFI_SSID:-Go2_21585}"         # Default SSID pattern (override per serial)
WIFI_PASSWORD="${GO2W_WIFI_PASSWORD:-00000000}"  # Many units ship with 00000000; try go2-w via env
MAPPER_TYPE="${1:-scan}"
OBSTACLE_AVOIDANCE="${2:-false}"
NAV_LAUNCH_FILE="${GO2W_NAV_LAUNCH_FILE:-single_go2w_real_cfpa2.launch.py}"
NAV_LAUNCH_FILE="${NAV_LAUNCH_FILE#@}"
DEFAULT_NAV_LAUNCH_FILE="single_go2w_real_cfpa2.launch.py"

# ── CycloneDDS for WiFi (unicast peer = robot gateway) ─────────────
setup_cyclonedds_wifi() {
  source /opt/ros/humble/setup.bash
  [[ -f "$REPO_ROOT/install/setup.bash" ]] && source "$REPO_ROOT/install/setup.bash"
  export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
  # lo: local ROS nodes; WIFI_IFACE: traffic to/from robot AP
  export CYCLONEDDS_URI="<CycloneDDS><Domain>
    <General>
      <AllowMulticast>false</AllowMulticast>
      <MaxMessageSize>65500 B</MaxMessageSize>
      <FragmentSize>1280 B</FragmentSize>
      <Interfaces>
        <NetworkInterface name=\"${WIFI_IFACE}\" priority=\"default\" multicast=\"false\" />
      </Interfaces>
    </General>
    <Internal>
      <SocketReceiveBufferSize min=\"1MB\" max=\"8MB\" />
      <Watermarks>
        <WhcHigh>500kB</WhcHigh>
      </Watermarks>
    </Internal>
    <Discovery>
      <LeaseDuration>20s</LeaseDuration>
      <Peers><Peer address=\"${ROBOT_IP}\"/></Peers>
      <ParticipantIndex>auto</ParticipantIndex>
      <MaxAutoParticipantIndex>30</MaxAutoParticipantIndex>
    </Discovery>
  </Domain></CycloneDDS>"
  export ROS_DOMAIN_ID=0
  (ros2 daemon stop &>/dev/null &) ; sleep 1 ; pkill -9 -f _ros2_daemon 2>/dev/null || true
}

# ── STOP ───────────────────────────────────────────────────────────
if [[ "$MAPPER_TYPE" == "stop" ]]; then
  echo "Stopping all autonomy processes..."
  pkill -9 -f cartographer_node 2>/dev/null || true
  pkill -9 -f cartographer_occupancy 2>/dev/null || true
  pkill -9 -f transform_everything 2>/dev/null || true
  pkill -9 -f default_nav 2>/dev/null || true
  pkill -9 -f cfpa2 2>/dev/null || true
  pkill -9 -f carto_odom_bridge 2>/dev/null || true
  pkill -9 -f twist_bridge 2>/dev/null || true
  pkill -9 -f cmd_vel_activity_mux 2>/dev/null || true
  pkill -9 -f cmd_vel_to_sport 2>/dev/null || true
  pkill -9 -f octomap_server 2>/dev/null || true
  pkill -9 -f elevation_to_occupancy 2>/dev/null || true
  pkill -9 -f simple_scan_mapper 2>/dev/null || true
  pkill -9 -f frontier_3d_markers 2>/dev/null || true
  pkill -9 -f pointcloud_to_laserscan 2>/dev/null || true
  killall -9 rviz2 2>/dev/null || true
  (ros2 daemon stop &>/dev/null &) ; sleep 1 ; pkill -9 -f _ros2_daemon 2>/dev/null || true
  echo "Done."
  exit 0
fi

case "$MAPPER_TYPE" in
  scan|octomap|elevation) ;;
  *)
    echo "ERROR: Unknown mapper type '$MAPPER_TYPE'"
    echo "Valid options: scan (default), octomap, elevation"
    exit 1
    ;;
esac

case "$OBSTACLE_AVOIDANCE" in
  true|false) ;;
  *)
    echo "ERROR: Invalid obstacle avoidance flag '$OBSTACLE_AVOIDANCE'"
    echo "Use: true or false"
    exit 1
    ;;
esac

# ── WiFi: nmcli connect + interface ───────────────────────────────
echo "=== [1/4] WiFi — connect to $WIFI_SSID ==="
if ! command -v nmcli &>/dev/null; then
  echo "ERROR: nmcli not found. Install NetworkManager or connect manually." >&2
  exit 1
fi

ros2 daemon stop 2>/dev/null || true
CURRENT=$(nmcli -t -f ACTIVE,SSID dev wifi 2>/dev/null | grep '^yes' | cut -d: -f2 || true)
if [[ "$CURRENT" == "$WIFI_SSID" ]]; then
  echo "  Already connected to $WIFI_SSID"
else
  nmcli device wifi rescan 2>/dev/null || true
  sleep 2
  nmcli device wifi connect "$WIFI_SSID" password "$WIFI_PASSWORD"
fi
sleep 2

WIFI_IFACE="${GO2W_WIFI_IFACE:-}"
if [[ -z "$WIFI_IFACE" ]]; then
  WIFI_IFACE=$(iw dev 2>/dev/null | awk '/Interface/{print $2; exit}')
fi
WIFI_IFACE=${WIFI_IFACE:-wlp0s20f3}
if ! ip link show "$WIFI_IFACE" &>/dev/null; then
  echo "ERROR: WiFi interface $WIFI_IFACE not found. Set GO2W_WIFI_IFACE." >&2
  exit 1
fi
MY_IP=$(ip -4 addr show "$WIFI_IFACE" 2>/dev/null | awk '/inet /{print $2}' | cut -d/ -f1)
echo "  Interface: $WIFI_IFACE   laptop IP: ${MY_IP:-dhcp}"

echo ""
echo "=== [2/4] Checking robot connectivity (ping $ROBOT_IP) ==="
if ! ping -c 2 -W 2 "$ROBOT_IP" &>/dev/null; then
  echo "ERROR: Cannot reach robot at $ROBOT_IP" >&2
  echo "  Check robot power, SSID/password (try GO2W_WIFI_PASSWORD=go2-w), and WiFi link." >&2
  exit 1
fi
echo "  Robot at $ROBOT_IP reachable ✅"

echo ""
echo "=== [3/4] Setting up CycloneDDS (WiFi unicast) ==="
setup_cyclonedds_wifi

PKG_PREFIX="$(ros2 pkg prefix go2_real_bringup 2>/dev/null || true)"
if [[ -z "$PKG_PREFIX" || ! -f "$PKG_PREFIX/share/go2_real_bringup/launch/$NAV_LAUNCH_FILE" ]]; then
  echo "WARN: nav launch '$NAV_LAUNCH_FILE' not found in go2_real_bringup install." >&2
  echo "      Falling back to '$DEFAULT_NAV_LAUNCH_FILE'." >&2
  NAV_LAUNCH_FILE="$DEFAULT_NAV_LAUNCH_FILE"
fi

echo ""
echo "=== [4/4] Verifying robot topics ==="
PC_PUB=$(timeout 5 ros2 topic info /utlidar/cloud 2>/dev/null | grep -o 'Publisher count: [0-9]*' || echo "Publisher count: 0")
IMU_PUB=$(timeout 5 ros2 topic info /utlidar/imu 2>/dev/null | grep -o 'Publisher count: [0-9]*' || echo "Publisher count: 0")
echo "  /utlidar/cloud: $PC_PUB"
echo "  /utlidar/imu:   $IMU_PUB"

echo ""
echo "################################################"
echo "  FULL AUTONOMY — Go2W (WiFi)"
echo "  Mapper: $MAPPER_TYPE"
echo "  SSID: $WIFI_SSID  Robot: $ROBOT_IP"
echo "################################################"
echo ""

echo "  Cleaning up stale processes..."
pkill -9 -f cartographer_node 2>/dev/null || true
pkill -9 -f cartographer_occupancy 2>/dev/null || true
pkill -9 -f transform_everything 2>/dev/null || true
pkill -9 -f default_nav 2>/dev/null || true
pkill -9 -f cfpa2 2>/dev/null || true
pkill -9 -f carto_odom_bridge 2>/dev/null || true
pkill -9 -f twist_bridge 2>/dev/null || true
pkill -9 -f cmd_vel_activity_mux 2>/dev/null || true
pkill -9 -f cmd_vel_to_sport 2>/dev/null || true
pkill -9 -f octomap_server 2>/dev/null || true
pkill -9 -f elevation_to_occupancy 2>/dev/null || true
pkill -9 -f simple_scan_mapper 2>/dev/null || true
pkill -9 -f frontier_3d_markers 2>/dev/null || true
pkill -9 -f pointcloud_to_laserscan 2>/dev/null || true
killall -9 rviz2 2>/dev/null || true
timeout 5 ros2 daemon stop 2>/dev/null || true
sleep 1

set +e
ALL_PIDS=""

cleanup() {
  echo ""
  echo "  Shutting down autonomy stack..."
  kill $ALL_PIDS 2>/dev/null
  sleep 1
  kill -9 $ALL_PIDS 2>/dev/null || true
  echo "  Stopped."
  exit 0
}
trap cleanup INT TERM

echo "  [1/7] Starting transform_everything..."
ros2 run transform_sensors transform_everything &
TE_PID=$!
ALL_PIDS="$TE_PID"
sleep 2

CARTO_CFG="$REPO_ROOT/src/go2_real_bringup/config/cartographer"
echo "  [2/7] Starting Cartographer 3D..."
ros2 run cartographer_ros cartographer_node \
  -configuration_directory "$CARTO_CFG" \
  -configuration_basename go2w_3d_mapping.lua \
  --ros-args \
  -r points2:=/utlidar/transformed_cloud \
  -r imu:=/utlidar/transformed_raw_imu \
  -p use_sim_time:=false &
CARTO_PID=$!
ALL_PIDS="$ALL_PIDS $CARTO_PID"
sleep 3

echo "  [3/7] Starting Cartographer occupancy grid..."
ros2 run cartographer_ros cartographer_occupancy_grid_node \
  --ros-args \
  -p use_sim_time:=false \
  -p resolution:=0.05 \
  -p publish_period_sec:=1.0 &
GRID_PID=$!
ALL_PIDS="$ALL_PIDS $GRID_PID"
sleep 2

EXTERNAL_MAPPER="false"

case "$MAPPER_TYPE" in
  scan)
    echo "  [4/7] Mapper: simple_scan_mapper_cpp (internal to launch file)"
    EXTERNAL_MAPPER="false"
    ;;

  octomap)
    EXTERNAL_MAPPER="true"
    OCTO_CFG="$REPO_ROOT/src/go2_real_bringup/config/cartographer/octomap_mapping.yaml"
    echo "  [4/7] Starting Octomap server (→ /robot/map)..."
    ros2 run octomap_server octomap_server_node \
      --ros-args \
      --params-file "$OCTO_CFG" \
      -r cloud_in:=/utlidar/transformed_cloud \
      -r projected_map:=/robot/map &
    MAPPER_PID=$!
    ALL_PIDS="$ALL_PIDS $MAPPER_PID"
    ;;

  elevation)
    EXTERNAL_MAPPER="true"
    OCTO_CFG="$REPO_ROOT/src/go2_real_bringup/config/cartographer/octomap_mapping.yaml"
    echo "  [4/7] Starting Octomap + elevation bridge..."
    ros2 run octomap_server octomap_server_node \
      --ros-args \
      --params-file "$OCTO_CFG" \
      -r cloud_in:=/utlidar/transformed_cloud &
    OCTO_ELEV_PID=$!
    ALL_PIDS="$ALL_PIDS $OCTO_ELEV_PID"
    sleep 2

    python3 "$REPO_ROOT/src/go2w_perception/scripts/elevation_to_occupancy.py" \
      --ros-args \
      -p input_topic:=/robot/elevation_map \
      -p output_topic:=/robot/map \
      -p frame_id:=map \
      -p max_step_height:=0.08 \
      -p max_slope_rad:=0.35 &
    ELEV_PID=$!
    ALL_PIDS="$ALL_PIDS $ELEV_PID"
    ;;
esac
sleep 1

OCTO_CFG="$REPO_ROOT/src/go2_real_bringup/config/cartographer/octomap_mapping.yaml"
echo "  [5/7] Starting Octomap 3D visualization..."
ros2 run octomap_server octomap_server_node \
  --ros-args \
  --params-file "$OCTO_CFG" \
  -r cloud_in:=/utlidar/transformed_cloud \
  -r projected_map:=/octomap/projected_map &
OCTO_PID=$!
ALL_PIDS="$ALL_PIDS $OCTO_PID"

echo "  [6/7] Launching navigation stack..."
ros2 launch go2_real_bringup "$NAV_LAUNCH_FILE" \
  robot_namespace:=robot \
  enable_manual_fallback:=true \
  external_mapper:=$EXTERNAL_MAPPER \
  obstacle_avoidance:=$OBSTACLE_AVOIDANCE &
NAV_PID=$!
ALL_PIDS="$ALL_PIDS $NAV_PID"
sleep 2

echo "  [7/7] Starting frontier_3d_markers + RViz..."
ros2 run go2w_perception frontier_3d_markers.py \
  --ros-args \
  -p map_topic:=/robot/map \
  -p marker_topic:=/frontier_cylinders \
  -p frame_id:=map \
  -p free_threshold:=0 \
  -p occ_threshold:=50 \
  -p obstacle_clearance_m:=0.30 \
  -p min_cluster_area_m2:=0.5 \
  -p cylinder_height:=0.8 \
  -p cylinder_radius:=0.12 &
FRONTIER_PID=$!
ALL_PIDS="$ALL_PIDS $FRONTIER_PID"

RVIZ_CFG="$REPO_ROOT/src/go2_real_bringup/config/cartographer/octomap.rviz"
rviz2 -d "$RVIZ_CFG" &
RVIZ_PID=$!
ALL_PIDS="$ALL_PIDS $RVIZ_PID"

echo ""
echo "  ✅ Full autonomy stack running (WiFi)"
echo "     Stop: Ctrl+C or ./go2w_start_autonomy_wifi.sh stop"
echo ""

wait $CARTO_PID
