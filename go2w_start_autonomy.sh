#!/bin/bash
# go2w_start_autonomy.sh
# Full autonomy pipeline for Go2W: SLAM + frontier exploration + navigation + obstacle avoidance.
#
# Usage: ./go2w_start_autonomy.sh                    # default (scan mapper, OA enabled)
#        ./go2w_start_autonomy.sh scan false         # scan mapper, OA disabled (api_id=1008)
#        ./go2w_start_autonomy.sh octomap true       # octomap mapper, OA enabled
#        ./go2w_start_autonomy.sh elevation false    # elevation mapper, OA disabled
#        ./go2w_start_autonomy.sh stop               # kill all autonomy processes
#
# Pipeline:
#   /utlidar/{cloud,imu}
#     → transform_everything (15.1° pitch + axis flip + IMU LPF)
#     → Cartographer 3D SLAM → TF (map → odom → body)
#     → Octomap 3D (visualization only)
#     → pointcloud_to_laserscan → simple_scan_mapper → /robot/map
#     → CFPA2 frontier exploration → waypoints
#     → reactive_nav (A* + local avoidance) → /cmd_vel
#     → Unitree obstacle avoidance API (api_id=1003)
#     → frontier_3d_markers → /frontier_cylinders (RViz)
#
# Prerequisites:
#   - USB-C ethernet dongle plugged in (ASIX AX88179)
#   - Ethernet cable to Go2W data port
#   - Robot powered on
#   - Workspace built: colcon build (in ~/COMP0225_LRC_stack)

set -e
REPO_ROOT="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"

ROBOT_IP="${GO2W_ETH_IP:-192.168.123.161}"
HOST_IP="${GO2W_HOST_IP:-192.168.123.100}"
ETH_IFACE="${GO2W_ETH_IFACE:-enxc8a36240a4c7}"
SUBNET="24"
MAPPER_TYPE="${1:-scan}"
OBSTACLE_AVOIDANCE="${2:-true}"

# ── DDS config ─────────────────────────────────────────────────────
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
      <MaxAutoParticipantIndex>200</MaxAutoParticipantIndex>
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
  pkill -9 -f reactive_nav 2>/dev/null || true
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

# ── Validate mapper type ──────────────────────────────────────────
case "$MAPPER_TYPE" in
  scan|octomap|elevation) ;;
  *)
    echo "ERROR: Unknown mapper type '$MAPPER_TYPE'"
    echo "Valid options: scan (default), octomap, elevation"
    exit 1
    ;;
esac

# ── Validate obstacle avoidance arg ───────────────────────────────
case "$OBSTACLE_AVOIDANCE" in
  true|false) ;;
  *)
    echo "ERROR: Invalid obstacle avoidance flag '$OBSTACLE_AVOIDANCE'"
    echo "Use: true or false"
    exit 1
    ;;
esac

# ── Ethernet setup ────────────────────────────────────────────────
echo "=== [1/4] Setting up Ethernet ($ETH_IFACE) ==="
if ! ip link show "$ETH_IFACE" &>/dev/null; then
  echo "ERROR: Interface $ETH_IFACE not found." >&2
  echo "  Is the USB-C ethernet dongle plugged in?" >&2
  echo "  Check: ip link show" >&2
  exit 1
fi

# Only assign IP if none exists (don't clobber NetworkManager)
CURRENT_IP=$(ip -4 addr show "$ETH_IFACE" 2>/dev/null | awk '/inet /{print $2}' | cut -d/ -f1)
if [[ -z "$CURRENT_IP" ]]; then
  echo "  Assigning $HOST_IP/$SUBNET to $ETH_IFACE..."
  sudo ip addr add "$HOST_IP/$SUBNET" dev "$ETH_IFACE" 2>/dev/null || true
  sudo ip link set "$ETH_IFACE" up
  sleep 1
  CURRENT_IP="$HOST_IP"
fi
echo "  Interface: $ETH_IFACE   IP: $CURRENT_IP"

# Ping robot
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

# ── Setup ROS/DDS ─────────────────────────────────────────────────
echo ""
echo "=== [3/4] Setting up CycloneDDS ==="
setup_cyclonedds

# ── Verify topics ─────────────────────────────────────────────────
echo ""
echo "=== [4/4] Verifying robot topics ==="
PC_PUB=$(timeout 5 ros2 topic info /utlidar/cloud 2>/dev/null | grep -o 'Publisher count: [0-9]*' || echo "Publisher count: 0")
IMU_PUB=$(timeout 5 ros2 topic info /utlidar/imu 2>/dev/null | grep -o 'Publisher count: [0-9]*' || echo "Publisher count: 0")
echo "  /utlidar/cloud: $PC_PUB"
echo "  /utlidar/imu:   $IMU_PUB"

# ── Print pipeline ────────────────────────────────────────────────
echo ""
echo "################################################"
echo "  FULL AUTONOMY — Go2W"
echo "  Mapper: $MAPPER_TYPE"
echo "  Pipeline:"
echo "    transform_everything → Cartographer 3D → TF"
case "$MAPPER_TYPE" in
  scan)     echo "    → pointcloud_to_laserscan → scan_mapper → /robot/map" ;;
  octomap)  echo "    → octomap (3D→2D projection) → /robot/map" ;;
  elevation) echo "    → elevation → traversability → /robot/map" ;;
esac
echo "    → CFPA2 frontiers → reactive_nav → cmd_vel"
if [[ "$OBSTACLE_AVOIDANCE" == "true" ]]; then
  echo "    → Unitree obstacle avoidance (api_id=1003)"
else
  echo "    → Unitree raw sport move (api_id=1008)"
fi
echo "    → frontier_3d_markers → /frontier_cylinders"
echo "  Ctrl+C to stop"
echo "################################################"
echo ""

# ── Cleanup stale processes ───────────────────────────────────────
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
pkill -9 -f elevation_to_occupancy 2>/dev/null || true
pkill -9 -f simple_scan_mapper 2>/dev/null || true
pkill -9 -f frontier_3d_markers 2>/dev/null || true
pkill -9 -f pointcloud_to_laserscan 2>/dev/null || true
killall -9 rviz2 2>/dev/null || true
timeout 5 ros2 daemon stop 2>/dev/null || true
sleep 1

ALL_PIDS=""

# ── 1) transform_everything ──────────────────────────────────────
echo "  [1/7] Starting transform_everything..."
ros2 run transform_sensors transform_everything &
TE_PID=$!
ALL_PIDS="$TE_PID"
sleep 2

# ── 2) Cartographer 3D SLAM ──────────────────────────────────────
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

# ── 3) Cartographer occupancy grid (reference /map) ──────────────
echo "  [3/7] Starting Cartographer occupancy grid..."
ros2 run cartographer_ros cartographer_occupancy_grid_node \
  --ros-args \
  -p use_sim_time:=false \
  -p resolution:=0.05 \
  -p publish_period_sec:=1.0 &
GRID_PID=$!
ALL_PIDS="$ALL_PIDS $GRID_PID"
sleep 2

# ── 4) Mapper-specific nodes ─────────────────────────────────────
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

# ── 5) Octomap 3D visualization ──────────────────────────────────
OCTO_CFG="$REPO_ROOT/src/go2_real_bringup/config/cartographer/octomap_mapping.yaml"
echo "  [5/7] Starting Octomap 3D visualization..."
ros2 run octomap_server octomap_server_node \
  --ros-args \
  --params-file "$OCTO_CFG" \
  -r cloud_in:=/utlidar/transformed_cloud \
  -r projected_map:=/octomap/projected_map &
OCTO_PID=$!
ALL_PIDS="$ALL_PIDS $OCTO_PID"

# ── 6) Navigation stack (CFPA2 + reactive_nav + obstacle avoidance) ──
echo "  [6/7] Launching navigation stack..."
ros2 launch go2_real_bringup single_go2w_real_cfpa2.launch.py \
  robot_namespace:=robot \
  enable_manual_fallback:=true \
  external_mapper:=$EXTERNAL_MAPPER \
  obstacle_avoidance:=$OBSTACLE_AVOIDANCE &
NAV_PID=$!
ALL_PIDS="$ALL_PIDS $NAV_PID"
sleep 2

# ── 7) Frontier detector + RViz ──────────────────────────────────
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
echo "  ✅ Full autonomy stack running"
echo "     Mapper: $MAPPER_TYPE"
if [[ "$OBSTACLE_AVOIDANCE" == "true" ]]; then
  echo "     Obstacle avoidance: ENABLED (api_id=1003)"
else
  echo "     Obstacle avoidance: DISABLED (using api_id=1008)"
fi
echo "     Frontier cylinders: /frontier_cylinders (red=top utility)"
echo "     Manual override: joystick"
echo "     Stop: Ctrl+C or ./go2w_start_autonomy.sh stop"
echo ""

# ── Trap Ctrl+C → clean shutdown ──────────────────────────────────
cleanup() {
  echo ""
  echo "  Shutting down autonomy stack..."
  kill $ALL_PIDS 2>/dev/null
  sleep 1
  # Force-kill stragglers
  kill -9 $ALL_PIDS 2>/dev/null || true
  echo "  Stopped."
  exit 0
}
trap cleanup INT TERM

# Wait for Cartographer (main process)
wait $CARTO_PID
