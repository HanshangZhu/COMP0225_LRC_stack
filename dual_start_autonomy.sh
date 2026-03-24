#!/bin/bash
# dual_start_autonomy.sh
# Real-world dual-robot CFPA2: Go2W over WiFi (robot_a) + Go2 over Ethernet (robot_b).
#
# DDS: each robot's raw /utlidar/* shares the same topic name; we run SLAM in subshells
# with CycloneDDS bound to only that robot's NIC + lo (local discovery). Shared nav +
# coordinator use lo + both NICs so they see namespaced topics and both robots.
#
# Usage:
#   ./dual_start_autonomy.sh                     # scan mapper, OA off on sport bridge
#   ./dual_start_autonomy.sh scan false          # explicit
#   ./dual_start_autonomy.sh octomap true
#   ./dual_start_autonomy.sh stop                # kill stack
#
# Env (variants — auto falls back to defaults below):
#   ROBOT_A_VARIANT / ROBOT_B_VARIANT   go2 | go2w | auto
#   GO2W_WIFI_SSID / GO2W_WIFI_PASSWORD
#   GO2W_WIFI_ROBOT_IP (default 192.168.12.1)
#   GO2_ETH_IP / GO2W_ETH_IP for robot B (default 192.168.123.161)
#   GO2W_ETH_IFACE / GO2W_HOST_IP          Ethernet dongle + host IP for Go2

set -e
REPO_ROOT="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"

NS_A="${DUAL_NS_A:-robot_a}"
NS_B="${DUAL_NS_B:-robot_b}"
ROBOT_A_IP="${GO2W_WIFI_ROBOT_IP:-192.168.12.1}"
ROBOT_B_IP="${GO2_ETH_IP:-${GO2W_ETH_IP:-192.168.123.161}}"
ETH_HOST_IP="${GO2W_HOST_IP:-192.168.123.100}"
ETH_IFACE="${GO2W_ETH_IFACE:-enxc8a36240a4c7}"
SUBNET="24"
WIFI_SSID="${GO2W_WIFI_SSID:-Go2_21585}"
WIFI_PASSWORD="${GO2W_WIFI_PASSWORD:-00000000}"

MAPPER_TYPE="${1:-scan}"
OBSTACLE_AVOIDANCE="${2:-false}"

CARTO_CFG="$REPO_ROOT/src/go2_real_bringup/config/cartographer"

# ── CycloneDDS XML fragments (bash expands iface + peer IPs) ────────
cyclone_lo_wifi_peer() {
  local wifi_iface="$1"
  local peer="$2"
  cat <<XMLEOF
<CycloneDDS><Domain><General><AllowMulticast>false</AllowMulticast><Interfaces>
<NetworkInterface name="lo" priority="default" multicast="true" />
<NetworkInterface name="${wifi_iface}" priority="default" multicast="false" />
</Interfaces></General><Discovery><Peers><Peer address="${peer}"/></Peers>
<ParticipantIndex>auto</ParticipantIndex><MaxAutoParticipantIndex>200</MaxAutoParticipantIndex>
</Discovery></Domain></CycloneDDS>
XMLEOF
}

cyclone_lo_eth_peer() {
  local eth_iface="$1"
  local peer="$2"
  cat <<XMLEOF
<CycloneDDS><Domain><General><AllowMulticast>false</AllowMulticast><Interfaces>
<NetworkInterface name="lo" priority="default" multicast="true" />
<NetworkInterface name="${eth_iface}" priority="default" multicast="true" />
</Interfaces></General><Discovery><Peers><Peer address="${peer}"/></Peers>
<ParticipantIndex>auto</ParticipantIndex><MaxAutoParticipantIndex>200</MaxAutoParticipantIndex>
</Discovery></Domain></CycloneDDS>
XMLEOF
}

cyclone_lo_wifi_eth_two_peers() {
  local wifi_iface="$1"
  local eth_iface="$2"
  local peer_a="$3"
  local peer_b="$4"
  cat <<XMLEOF
<CycloneDDS><Domain><General><AllowMulticast>false</AllowMulticast><Interfaces>
<NetworkInterface name="lo" priority="default" multicast="true" />
<NetworkInterface name="${wifi_iface}" priority="default" multicast="false" />
<NetworkInterface name="${eth_iface}" priority="default" multicast="true" />
</Interfaces></General><Discovery><Peers>
<Peer address="${peer_a}"/><Peer address="${peer_b}"/></Peers>
<ParticipantIndex>auto</ParticipantIndex><MaxAutoParticipantIndex>200</MaxAutoParticipantIndex>
</Discovery></Domain></CycloneDDS>
XMLEOF
}

# ── Optional: probe topics to guess go2 vs go2w (wheel-related names) ─
detect_variant_from_graph() {
  local cyclone_xml="$1"
  local def="$2"
  local topics
  topics="$(
    CYCLONEDDS_URI="$cyclone_xml" RMW_IMPLEMENTATION=rmw_cyclonedds_cpp ROS_DOMAIN_ID=0 bash -c \
      'source /opt/ros/humble/setup.bash 2>/dev/null
       source "'"$REPO_ROOT"'/install/setup.bash" 2>/dev/null
       timeout 6 ros2 topic list 2>/dev/null || true'
  )" || true
  if [[ -z "$topics" ]]; then
    echo "$def"
    return 0
  fi
  if echo "$topics" | grep -qiE 'wheel|go2w|wheeled'; then
    echo "go2w"
  else
    echo "go2"
  fi
}

# ── STOP ───────────────────────────────────────────────────────────
if [[ "$MAPPER_TYPE" == "stop" ]]; then
  echo "Stopping dual autonomy + SLAM subshells..."
  pkill -9 -f cartographer_node 2>/dev/null || true
  pkill -9 -f cartographer_occupancy 2>/dev/null || true
  pkill -9 -f transform_everything 2>/dev/null || true
  pkill -9 -f reactive_nav 2>/dev/null || true
  pkill -9 -f cfpa2 2>/dev/null || true
  pkill -9 -f carto_odom_bridge 2>/dev/null || true
  pkill -9 -f twist_bridge 2>/dev/null || true
  pkill -9 -f cmd_vel_activity_mux 2>/dev/null || true
  pkill -9 -f cmd_vel_to_sport 2>/dev/null || true
  pkill -9 -f octomap_server 2>/dev/null || true
  pkill -9 -f shared_map_fuser 2>/dev/null || true
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
    exit 1
    ;;
esac
case "$OBSTACLE_AVOIDANCE" in
  true|false) ;;
  *)
    echo "ERROR: obstacle_avoidance must be true or false"
    exit 1
    ;;
esac

# ── [1] WiFi: Go2W ──────────────────────────────────────────────────
echo "=== [1/6] WiFi — connect to $WIFI_SSID (robot A / $NS_A) ==="
if ! command -v nmcli &>/dev/null; then
  echo "ERROR: nmcli required for WiFi bring-up." >&2
  exit 1
fi
ros2 daemon stop 2>/dev/null || true
CURRENT=$(nmcli -t -f ACTIVE,SSID dev wifi 2>/dev/null | grep '^yes' | cut -d: -f2 || true)
if [[ "$CURRENT" == "$WIFI_SSID" ]]; then
  echo "  Already on $WIFI_SSID"
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
  echo "ERROR: WiFi interface $WIFI_IFACE not found." >&2
  exit 1
fi

# ── [2] Ethernet: Go2 ───────────────────────────────────────────────
echo ""
echo "=== [2/6] Ethernet — $ETH_IFACE (robot B / $NS_B) ==="
if ! ip link show "$ETH_IFACE" &>/dev/null; then
  echo "ERROR: Ethernet interface $ETH_IFACE not found." >&2
  exit 1
fi
CURRENT_IP=$(ip -4 addr show "$ETH_IFACE" 2>/dev/null | awk '/inet /{print $2}' | cut -d/ -f1)
if [[ -z "$CURRENT_IP" ]]; then
  echo "  Assigning $ETH_HOST_IP/$SUBNET to $ETH_IFACE..."
  sudo ip addr add "$ETH_HOST_IP/$SUBNET" dev "$ETH_IFACE" 2>/dev/null || true
  sudo ip link set "$ETH_IFACE" up
  sleep 1
fi

# ── [3] Ping both robots ───────────────────────────────────────────
echo ""
echo "=== [3/6] Connectivity ==="
if ! ping -c 2 -W 2 "$ROBOT_A_IP" &>/dev/null; then
  echo "ERROR: cannot ping WiFi robot $ROBOT_A_IP" >&2
  exit 1
fi
if ! ping -c 2 -W 2 "$ROBOT_B_IP" &>/dev/null; then
  echo "ERROR: cannot ping Ethernet robot $ROBOT_B_IP" >&2
  exit 1
fi
echo "  A (WiFi)  $ROBOT_A_IP  OK"
echo "  B (Eth)   $ROBOT_B_IP  OK"

CYCLONE_A="$(cyclone_lo_wifi_peer "$WIFI_IFACE" "$ROBOT_A_IP")"
CYCLONE_B="$(cyclone_lo_eth_peer "$ETH_IFACE" "$ROBOT_B_IP")"
CYCLONE_FULL="$(cyclone_lo_wifi_eth_two_peers "$WIFI_IFACE" "$ETH_IFACE" "$ROBOT_A_IP" "$ROBOT_B_IP")"

# ── [4] Resolve variants (auto + env) ───────────────────────────────
echo ""
echo "=== [4/6] Robot variants (reactive_nav profile) ==="
VARG_A="${ROBOT_A_VARIANT:-auto}"
VARG_B="${ROBOT_B_VARIANT:-auto}"
if [[ "$VARG_A" == "auto" ]]; then
  ROBOT_A_VARIANT_RESOLVED="$(detect_variant_from_graph "$CYCLONE_A" "go2w")"
else
  ROBOT_A_VARIANT_RESOLVED="$VARG_A"
fi
if [[ "$VARG_B" == "auto" ]]; then
  ROBOT_B_VARIANT_RESOLVED="$(detect_variant_from_graph "$CYCLONE_B" "go2")"
else
  ROBOT_B_VARIANT_RESOLVED="$VARG_B"
fi
echo "  $NS_A: $ROBOT_A_VARIANT_RESOLVED   $NS_B: $ROBOT_B_VARIANT_RESOLVED"
export ROBOT_A_VARIANT="$ROBOT_A_VARIANT_RESOLVED"
export ROBOT_B_VARIANT="$ROBOT_B_VARIANT_RESOLVED"

# ── Subshell: SLAM side A (WiFi only DDS) ──────────────────────────
launch_slam_side_a() {
  set +e
  export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
  export CYCLONEDDS_URI="$CYCLONE_A"
  export ROS_DOMAIN_ID=0
  source /opt/ros/humble/setup.bash
  # shellcheck disable=SC1090
  source "$REPO_ROOT/install/setup.bash"

  ros2 run transform_sensors transform_everything --ros-args \
    -p output_frame_id:=robot_a_body \
    -r /utlidar/transformed_cloud:=/"$NS_A"/utlidar/transformed_cloud \
    -r /utlidar/transformed_raw_imu:=/"$NS_A"/utlidar/transformed_raw_imu \
    -r /utlidar/transformed_imu:=/"$NS_A"/utlidar/transformed_imu &
  sleep 2

  # Isolate Cartographer internal topics (/submap_list, etc.) per robot.
  ros2 run cartographer_ros cartographer_node \
    -configuration_directory "$CARTO_CFG" \
    -configuration_basename go2w_3d_mapping_robot_a.lua \
    --ros-args \
    -r __ns:=/carto_slam_a \
    -r /carto_slam_a/tf:=/tf \
    -r /carto_slam_a/tf_static:=/tf_static \
    -r points2:=/"$NS_A"/utlidar/transformed_cloud \
    -r imu:=/"$NS_A"/utlidar/transformed_raw_imu \
    -p use_sim_time:=false \
    -r __node:=cartographer_node_"$NS_A" &
  sleep 2

  ros2 run cartographer_ros cartographer_occupancy_grid_node \
    --ros-args \
    -r __ns:=/carto_slam_a \
    -r /carto_slam_a/tf:=/tf \
    -r /carto_slam_a/tf_static:=/tf_static \
    -p use_sim_time:=false \
    -p resolution:=0.05 \
    -p publish_period_sec:=1.0 \
    -r map:=/"$NS_A"/cartographer_map \
    -r __node:=cartographer_occupancy_"$NS_A" &

  OCTO_CFG="$REPO_ROOT/src/go2_real_bringup/config/cartographer/octomap_mapping.yaml"
  ros2 run octomap_server octomap_server_node \
    --ros-args \
    --params-file "$OCTO_CFG" \
    -r cloud_in:=/"$NS_A"/utlidar/transformed_cloud \
    -r projected_map:=/"$NS_A"/octomap/projected_map \
    -r __node:=octomap_vis_"$NS_A" &

  if [[ "$MAPPER_TYPE" == "octomap" ]]; then
    ros2 run octomap_server octomap_server_node \
      --ros-args \
      --params-file "$OCTO_CFG" \
      -r cloud_in:=/"$NS_A"/utlidar/transformed_cloud \
      -r projected_map:=/"$NS_A"/map \
      -r __node:=octomap_mapper_"$NS_A" &
  elif [[ "$MAPPER_TYPE" == "elevation" ]]; then
    ros2 run octomap_server octomap_server_node \
      --ros-args \
      --params-file "$OCTO_CFG" \
      -r cloud_in:=/"$NS_A"/utlidar/transformed_cloud \
      -r __node:=octomap_elev_"$NS_A" &
    sleep 2
    python3 "$REPO_ROOT/src/go2w_perception/scripts/elevation_to_occupancy.py" \
      --ros-args \
      -p input_topic:=/"$NS_A"/elevation_map \
      -p output_topic:=/"$NS_A"/map \
      -p frame_id:=robot_a_map \
      -p max_step_height:=0.08 \
      -p max_slope_rad:=0.35 &
  fi

  wait
}

launch_slam_side_b() {
  set +e
  export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
  export CYCLONEDDS_URI="$CYCLONE_B"
  export ROS_DOMAIN_ID=0
  source /opt/ros/humble/setup.bash
  # shellcheck disable=SC1090
  source "$REPO_ROOT/install/setup.bash"

  ros2 run transform_sensors transform_everything --ros-args \
    -p output_frame_id:=robot_b_body \
    -r /utlidar/transformed_cloud:=/"$NS_B"/utlidar/transformed_cloud \
    -r /utlidar/transformed_raw_imu:=/"$NS_B"/utlidar/transformed_raw_imu \
    -r /utlidar/transformed_imu:=/"$NS_B"/utlidar/transformed_imu &
  sleep 2

  ros2 run cartographer_ros cartographer_node \
    -configuration_directory "$CARTO_CFG" \
    -configuration_basename go2w_3d_mapping_robot_b.lua \
    --ros-args \
    -r __ns:=/carto_slam_b \
    -r /carto_slam_b/tf:=/tf \
    -r /carto_slam_b/tf_static:=/tf_static \
    -r points2:=/"$NS_B"/utlidar/transformed_cloud \
    -r imu:=/"$NS_B"/utlidar/transformed_raw_imu \
    -p use_sim_time:=false \
    -r __node:=cartographer_node_"$NS_B" &
  sleep 2

  ros2 run cartographer_ros cartographer_occupancy_grid_node \
    --ros-args \
    -r __ns:=/carto_slam_b \
    -r /carto_slam_b/tf:=/tf \
    -r /carto_slam_b/tf_static:=/tf_static \
    -p use_sim_time:=false \
    -p resolution:=0.05 \
    -p publish_period_sec:=1.0 \
    -r map:=/"$NS_B"/cartographer_map \
    -r __node:=cartographer_occupancy_"$NS_B" &

  OCTO_CFG="$REPO_ROOT/src/go2_real_bringup/config/cartographer/octomap_mapping.yaml"
  ros2 run octomap_server octomap_server_node \
    --ros-args \
    --params-file "$OCTO_CFG" \
    -r cloud_in:=/"$NS_B"/utlidar/transformed_cloud \
    -r projected_map:=/"$NS_B"/octomap/projected_map \
    -r __node:=octomap_vis_"$NS_B" &

  if [[ "$MAPPER_TYPE" == "octomap" ]]; then
    ros2 run octomap_server octomap_server_node \
      --ros-args \
      --params-file "$OCTO_CFG" \
      -r cloud_in:=/"$NS_B"/utlidar/transformed_cloud \
      -r projected_map:=/"$NS_B"/map \
      -r __node:=octomap_mapper_"$NS_B" &
  elif [[ "$MAPPER_TYPE" == "elevation" ]]; then
    ros2 run octomap_server octomap_server_node \
      --ros-args \
      --params-file "$OCTO_CFG" \
      -r cloud_in:=/"$NS_B"/utlidar/transformed_cloud \
      -r __node:=octomap_elev_"$NS_B" &
    sleep 2
    python3 "$REPO_ROOT/src/go2w_perception/scripts/elevation_to_occupancy.py" \
      --ros-args \
      -p input_topic:=/"$NS_B"/elevation_map \
      -p output_topic:=/"$NS_B"/map \
      -p frame_id:=robot_b_map \
      -p max_step_height:=0.08 \
      -p max_slope_rad:=0.35 &
  fi

  wait
}

echo ""
echo "  Cleaning up stale processes..."
pkill -9 -f cartographer_node 2>/dev/null || true
pkill -9 -f cartographer_occupancy 2>/dev/null || true
pkill -9 -f transform_everything 2>/dev/null || true
pkill -9 -f reactive_nav 2>/dev/null || true
pkill -9 -f cfpa2 2>/dev/null || true
pkill -9 -f carto_odom_bridge 2>/dev/null || true
pkill -9 -f twist_bridge 2>/dev/null || true
pkill -9 -f cmd_vel_activity_mux 2>/dev/null || true
pkill -9 -f cmd_vel_to_sport 2>/dev/null || true
pkill -9 -f octomap_server 2>/dev/null || true
pkill -9 -f shared_map_fuser 2>/dev/null || true
pkill -9 -f elevation_to_occupancy 2>/dev/null || true
pkill -9 -f simple_scan_mapper 2>/dev/null || true
pkill -9 -f frontier_3d_markers 2>/dev/null || true
pkill -9 -f pointcloud_to_laserscan 2>/dev/null || true
killall -9 rviz2 2>/dev/null || true
timeout 5 ros2 daemon stop 2>/dev/null || true
sleep 1

PID_SLAM_A="" PID_SLAM_B="" PID_NAV="" PID_SPORT_A="" PID_SPORT_B=""
PID_FRONTIER="" PID_RVIZ=""
cleanup() {
  echo "Shutting down..."
  kill $PID_RVIZ $PID_FRONTIER $PID_NAV $PID_SPORT_A $PID_SPORT_B 2>/dev/null || true
  kill $PID_SLAM_A $PID_SLAM_B 2>/dev/null || true
  sleep 1
  pkill -9 -f cartographer_node 2>/dev/null || true
  pkill -9 -f cartographer_occupancy 2>/dev/null || true
  pkill -9 -f transform_everything 2>/dev/null || true
  pkill -9 -f octomap_server 2>/dev/null || true
  pkill -9 -f cfpa2 2>/dev/null || true
  pkill -9 -f reactive_nav 2>/dev/null || true
  pkill -9 -f cmd_vel_to_sport 2>/dev/null || true
  pkill -9 -f shared_map_fuser 2>/dev/null || true
  pkill -9 -f carto_odom_bridge 2>/dev/null || true
  pkill -9 -f twist_bridge 2>/dev/null || true
  pkill -9 -f cmd_vel_activity_mux 2>/dev/null || true
  pkill -9 -f elevation_to_occupancy 2>/dev/null || true
  pkill -9 -f simple_scan_mapper 2>/dev/null || true
  pkill -9 -f frontier_3d_markers 2>/dev/null || true
  pkill -9 -f pointcloud_to_laserscan 2>/dev/null || true
  killall -9 rviz2 2>/dev/null || true
  exit 0
}
trap cleanup INT TERM

echo ""
echo "=== [5/6] Starting per-robot SLAM (isolated DDS) ==="
launch_slam_side_a &
PID_SLAM_A=$!
launch_slam_side_b &
PID_SLAM_B=$!
sleep 8

EXTERNAL_MAPPER="false"
if [[ "$MAPPER_TYPE" != "scan" ]]; then
  EXTERNAL_MAPPER="true"
fi

echo ""
echo "=== [6/6] Shared nav + CFPA2 (dual NIC CycloneDDS) ==="
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI="$CYCLONE_FULL"
export ROS_DOMAIN_ID=0
source /opt/ros/humble/setup.bash
# shellcheck disable=SC1090
source "$REPO_ROOT/install/setup.bash"
(ros2 daemon stop &>/dev/null &) ; sleep 1 ; pkill -9 -f _ros2_daemon 2>/dev/null || true

ros2 launch go2_real_bringup dual_real_cfpa2.launch.py \
  robot_a_variant:="$ROBOT_A_VARIANT_RESOLVED" \
  robot_b_variant:="$ROBOT_B_VARIANT_RESOLVED" \
  external_mapper:="$EXTERNAL_MAPPER" \
  obstacle_avoidance:="$OBSTACLE_AVOIDANCE" \
  use_shared_map:=true \
  shared_map_topic:=/disco_slam/global_map \
  robot_a_namespace:="$NS_A" \
  robot_b_namespace:="$NS_B" &
PID_NAV=$!
sleep 4

# Sport API: one bridge per robot, DDS bound to that robot's link (avoid fan-out).
(
  export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
  export CYCLONEDDS_URI="$CYCLONE_A"
  export ROS_DOMAIN_ID=0
  source /opt/ros/humble/setup.bash
  # shellcheck disable=SC1090
  source "$REPO_ROOT/install/setup.bash"
  ros2 run go2w_control cmd_vel_to_sport_bridge.py --ros-args \
    -p cmd_vel_topic:=/"$NS_A"/cmd_vel_to_robot \
    -p sport_topic:=/api/sport/request \
    -p obstacle_avoidance:="$OBSTACLE_AVOIDANCE"
) &
PID_SPORT_A=$!

(
  export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
  export CYCLONEDDS_URI="$CYCLONE_B"
  export ROS_DOMAIN_ID=0
  source /opt/ros/humble/setup.bash
  # shellcheck disable=SC1090
  source "$REPO_ROOT/install/setup.bash"
  ros2 run go2w_control cmd_vel_to_sport_bridge.py --ros-args \
    -p cmd_vel_topic:=/"$NS_B"/cmd_vel_to_robot \
    -p sport_topic:=/api/sport/request \
    -p obstacle_avoidance:="$OBSTACLE_AVOIDANCE"
) &
PID_SPORT_B=$!

ros2 run go2w_perception frontier_3d_markers.py \
  --ros-args \
  -p map_topic:=/disco_slam/global_map \
  -p marker_topic:=/dual_frontier_cylinders \
  -p frame_id:=robot_a_map \
  -p free_threshold:=0 \
  -p occ_threshold:=50 \
  -p obstacle_clearance_m:=0.30 \
  -p min_cluster_area_m2:=0.5 \
  -p cylinder_height:=0.8 \
  -p cylinder_radius:=0.12 &
PID_FRONTIER=$!

RVIZ_CFG="$REPO_ROOT/src/go2_real_bringup/config/cartographer/octomap.rviz"
rviz2 -d "$RVIZ_CFG" &
PID_RVIZ=$!

echo ""
echo "################################################"
echo "  DUAL REAL AUTONOMY running"
echo "  $NS_A (WiFi $ROBOT_A_IP)  variant=$ROBOT_A_VARIANT_RESOLVED"
echo "  $NS_B (Eth  $ROBOT_B_IP)  variant=$ROBOT_B_VARIANT_RESOLVED"
echo "  Shared map: /disco_slam/global_map"
echo "  Stop: Ctrl+C or ./dual_start_autonomy.sh stop"
echo "################################################"

wait "$PID_SLAM_A" || true
wait "$PID_SLAM_B" || true
