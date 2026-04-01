#!/bin/bash
# go2w_ethernet_start.sh
# Connect to Go2W over Ethernet, verify data streams, then live-monitor.
#
# Usage: ./go2w_ethernet_start.sh              # setup + live monitor
#        ./go2w_ethernet_start.sh stop         # stop driver container
#        ./go2w_ethernet_start.sh status       # check topics without monitoring
#        ./go2w_ethernet_start.sh fastlio      # setup + launch Fast-LIO SLAM
#        ./go2w_ethernet_start.sh fastlio_sam  # Fast-LIO + SC-PGO loop closure
#        ./go2w_ethernet_start.sh cartographer # Cartographer 3D SLAM only
#        ./go2w_ethernet_start.sh autonomy     # Cartographer + CFPA2 + default_nav (scan mapper)
#        ./go2w_ethernet_start.sh autonomy octomap    # Same but with Octomap 3D→2D grid
#        ./go2w_ethernet_start.sh autonomy elevation  # Same but with traversability grid
#        ./go2w_ethernet_start.sh mapping      # Carto + Octomap + frontiers, NO controller
#
# Prerequisites:
#   - USB-C ethernet dongle plugged in (ASIX AX88179)
#   - Ethernet cable to Go2W data port
#   - Robot powered on
#
# ─────────────────────────────────────────────────────────────────────
# SENDING COMMANDS (run in a separate terminal after sourcing DDS env)
# ─────────────────────────────────────────────────────────────────────
#
# First, source the DDS environment in your terminal:
#   source /opt/ros/humble/setup.bash
#   source ~/COMP0225_LRC_stack/install/setup.bash
#   export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
#   export CYCLONEDDS_URI="<CycloneDDS><Domain><General><Interfaces>
#     <NetworkInterface name=\"enxc8a36240a4c7\" multicast=\"true\" />
#     </Interfaces></General><Discovery><Peers>
#     <Peer address=\"192.168.123.161\"/></Peers>
#     <ParticipantIndex>auto</ParticipantIndex>
#     </Discovery></Domain></CycloneDDS>"
#   export ROS_DOMAIN_ID=0
#
# ── Velocity (Move) ── api_id=1008
#   # Forward 0.2 m/s for 5 seconds:
#   timeout 5 ros2 topic pub /api/sport/request unitree_api/msg/Request \
#     "{header: {identity: {api_id: 1008}}, parameter: '{\"x\":0.2,\"y\":0.0,\"z\":0.0}'}" --rate 10
#
#   # Rotate in place (0.5 rad/s):
#   timeout 5 ros2 topic pub /api/sport/request unitree_api/msg/Request \
#     "{header: {identity: {api_id: 1008}}, parameter: '{\"x\":0.0,\"y\":0.0,\"z\":0.5}'}" --rate 10
#
#   # Stop (send zero velocity):
#   ros2 topic pub --once /api/sport/request unitree_api/msg/Request \
#     "{header: {identity: {api_id: 1008}}, parameter: '{\"x\":0.0,\"y\":0.0,\"z\":0.0}'}"
#
# ── Mode changes ── (no parameter needed unless noted)
#   # Stand up:
#   ros2 topic pub --once /api/sport/request unitree_api/msg/Request \
#     "{header: {identity: {api_id: 1004}}}"
#
#   # Stand down (lie flat):
#   ros2 topic pub --once /api/sport/request unitree_api/msg/Request \
#     "{header: {identity: {api_id: 1005}}}"
#
#   # Balance stand:
#   ros2 topic pub --once /api/sport/request unitree_api/msg/Request \
#     "{header: {identity: {api_id: 1002}}}"
#
#   # Sit:
#   ros2 topic pub --once /api/sport/request unitree_api/msg/Request \
#     "{header: {identity: {api_id: 1009}}}"
#
# ── Full API ID reference ──
#   1001 = Damp            1002 = BalanceStand     1003 = StopMove
#   1004 = StandUp         1005 = StandDown        1006 = RecoveryStand
#   1007 = Euler           1008 = Move             1009 = Sit
#   1010 = RiseSit         1011 = SwitchGait       1012 = Trigger
#   1013 = BodyHeight      1014 = FootRaiseHeight  1015 = SpeedLevel
#   1016 = Hello           1017 = Stretch          1018 = TrajectoryFollow
#   1019 = ContinuousGait  1020 = Content          1021 = Wallow
#   1022 = Dance1          1023 = Dance2           1027 = SwitchJoystick
#
# ── Body height ── api_id=1013, parameter: {"data": <-0.18 to 0.03>}
#   ros2 topic pub --once /api/sport/request unitree_api/msg/Request \
#     "{header: {identity: {api_id: 1013}}, parameter: '{\"data\":-0.05}'}"
#
# ── Switch gait ── api_id=1011, parameter: {"data": 0=walk, 1=terrain, 2=climb}
#   ros2 topic pub --once /api/sport/request unitree_api/msg/Request \
#     "{header: {identity: {api_id: 1011}}, parameter: '{\"data\":1}'}"
# ─────────────────────────────────────────────────────────────────────
#pkill -9 -f cartographer; pkill -9 -f transform_everything; pkill -9 -f rviz2; sleep 2

set -e
REPO_ROOT="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
COMPOSE="docker compose -f $REPO_ROOT/hardware/docker/go2w_real/docker-compose.yml"

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
      <MaxAutoParticipantIndex>200</MaxAutoParticipantIndex>
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

# Fast-LIO mode — transform_everything + Fast-LIO
if [[ "${1:-}" == "fastlio" || "${1:-}" == "slam" ]]; then
  echo "  Verifying /utlidar/cloud and /utlidar/imu..."
  PC_PUB=$(timeout 5 ros2 topic info /utlidar/cloud 2>/dev/null | grep -o 'Publisher count: [0-9]*' || echo "Publisher count: 0")
  IMU_PUB=$(timeout 5 ros2 topic info /utlidar/imu 2>/dev/null | grep -o 'Publisher count: [0-9]*' || echo "Publisher count: 0")
  echo "  /utlidar/cloud: $PC_PUB"
  echo "  /utlidar/imu:   $IMU_PUB"
  echo ""
  echo "################################################"
  echo "  Launching Fast-LIO (via transform_everything)"
  echo "  Pipeline:"
  echo "    /utlidar/{cloud,imu}"
  echo "      → transform_everything (15.1° pitch + axis flip)"
  echo "      → /utlidar/transformed_{cloud,imu}"
  echo "      → Fast-LIO"
  echo "      → /Odometry, /cloud_registered"
  echo "  Ctrl+C to stop"
  echo "################################################"
  echo ""

  # Load IMU calibration
  CALIB_FILE="$REPO_ROOT/hardware/calibration/imu_calib_data.yaml"
  if [[ -f "$CALIB_FILE" ]]; then
    echo "  ✅ IMU calibration loaded from $CALIB_FILE"
  else
    echo "  ⚠️  No IMU calibration — using zero biases"
  fi

  # 1) Start transform_everything
  echo "  Starting transform_everything..."
  ros2 run transform_sensors transform_everything &
  TE_PID=$!
  sleep 2

  # 2) Launch Fast-LIO
  echo "  Starting Fast-LIO..."
  ros2 launch fast_lio mapping.launch.py \
    config_file:=go2w_utlidar.yaml \
    use_sim_time:=false \
    rviz:=true &
  FLIO_PID=$!

  trap "kill $TE_PID $FLIO_PID 2>/dev/null; echo ''; echo 'Stopped.'; exit 0" INT TERM
  wait $FLIO_PID
  exit 0
fi

# FAST-LIO-SAM mode — transform_everything + Fast-LIO2 + SC-PGO loop closure
if [[ "${1:-}" == "fastlio_sam" ]]; then
  echo "  Verifying /utlidar/cloud and /utlidar/imu..."
  PC_PUB=$(timeout 5 ros2 topic info /utlidar/cloud 2>/dev/null | grep -o 'Publisher count: [0-9]*' || echo "Publisher count: 0")
  IMU_PUB=$(timeout 5 ros2 topic info /utlidar/imu 2>/dev/null | grep -o 'Publisher count: [0-9]*' || echo "Publisher count: 0")
  echo "  /utlidar/cloud: $PC_PUB"
  echo "  /utlidar/imu:   $IMU_PUB"
  echo ""
  echo "################################################"
  echo "  Launching FAST-LIO-SAM (full pipeline)"
  echo "  Pipeline:"
  echo "    /utlidar/{cloud,imu}"
  echo "      → transform_everything (15.1° pitch + axis flip)"
  echo "      → Fast-LIO2 → /cloud_registered + /Odometry"
  echo "      → SC-PGO    → /corrected_odom + /corrected_path"
  echo "                    + /corrected_cloud + /loop_markers"
  echo "  Ctrl+C to stop"
  echo "################################################"
  echo ""

  # Load IMU calibration
  CALIB_FILE="$REPO_ROOT/hardware/calibration/imu_calib_data.yaml"
  if [[ -f "$CALIB_FILE" ]]; then
    echo "  ✅ IMU calibration loaded from $CALIB_FILE"
  else
    echo "  ⚠️  No IMU calibration — using zero biases"
  fi

  # 0) Start transform_everything
  echo "  Starting transform_everything..."
  ros2 run transform_sensors transform_everything &
  TE_PID=$!
  sleep 2

  # 1) Launch Fast-LIO2
  echo "  Starting Fast-LIO2..."
  ros2 launch fast_lio mapping.launch.py \
    config_file:=go2w_utlidar.yaml \
    use_sim_time:=false \
    rviz:=false &
  FLIO_PID=$!
  sleep 3

  # 2) Launch SC-PGO (loop closure)
  echo "  Starting SC-PGO (loop closure)..."
  SC_PGO_CONFIG="$(ros2 pkg prefix sc_pgo)/share/sc_pgo/config/sc_pgo_params.yaml"
  ros2 run sc_pgo sc_pgo_node \
    --ros-args --params-file "$SC_PGO_CONFIG" \
    -r /aft_mapped_to_init:=/Odometry &
  SCPGO_PID=$!
  sleep 2

  # 3) Launch RViz
  echo "  Launching RViz..."
  rviz2 &
  RVIZ_PID=$!
  trap "kill $TE_PID $FLIO_PID $SCPGO_PID $RVIZ_PID 2>/dev/null; echo ''; echo 'Stopped.'; exit 0" INT TERM
  wait $FLIO_PID
  exit 0
fi

# Point-LIO mode — with transform_everything for proper IMU rotation
if [[ "${1:-}" == "pointlio" ]]; then
  echo "  Verifying /utlidar/cloud and /utlidar/imu..."
  PC_PUB=$(timeout 5 ros2 topic info /utlidar/cloud 2>/dev/null | grep -o 'Publisher count: [0-9]*' || echo "Publisher count: 0")
  IMU_PUB=$(timeout 5 ros2 topic info /utlidar/imu 2>/dev/null | grep -o 'Publisher count: [0-9]*' || echo "Publisher count: 0")
  echo "  /utlidar/cloud: $PC_PUB"
  echo "  /utlidar/imu:   $IMU_PUB"
  echo ""
  echo "################################################"
  echo "  Launching Point-LIO (via transform_everything)"
  echo "  Pipeline:"
  echo "    /utlidar/{cloud,imu}"
  echo "      → transform_everything (15.1° pitch + axis flip)"
  echo "      → /utlidar/transformed_{cloud,imu}"
  echo "      → Point-LIO"
  echo "      → /registered_scan, /Odometry"
  echo "  Ctrl+C to stop"
  echo "################################################"
  echo ""

  # 0) Auto-calibrate IMU if no calibration file exists
  CALIB_FILE="$REPO_ROOT/hardware/calibration/imu_calib_data.yaml"
  if [[ ! -f "$CALIB_FILE" ]]; then
    echo "  ⚠️  No IMU calibration found ($CALIB_FILE)"
    echo "  Running auto-calibration — KEEP ROBOT STILL for ~10s..."
    micromamba run -n cmu_env python3 "$REPO_ROOT/hardware/calibration/calibrate_imu.py"
    if [[ -f "$CALIB_FILE" ]]; then
      echo "  ✅ IMU calibration saved"
    else
      echo "  ⚠️  Calibration failed — continuing with default biases"
    fi
    echo ""
  else
    echo "  ✅ IMU calibration loaded from $CALIB_FILE"
  fi

  # 1) Launch transform_everything (QoS-fixed, BEST_EFFORT subscribers)
  echo "  Starting transform_everything..."
  ros2 run transform_sensors transform_everything &
  TE_PID=$!
  sleep 2

  # Verify transformed topics appear
  echo "  Waiting for transformed topics..."
  for i in $(seq 1 10); do
    TC_PUB=$(timeout 3 ros2 topic info /utlidar/transformed_cloud 2>/dev/null | grep -o 'Publisher count: [0-9]*' || echo "Publisher count: 0")
    if echo "$TC_PUB" | grep -q "[1-9]"; then
      echo "  /utlidar/transformed_cloud: $TC_PUB ✅"
      break
    fi
    if [[ $i -eq 10 ]]; then
      echo "  ⚠️  transform_everything not publishing yet — launching Point-LIO anyway"
    fi
    sleep 1
  done

  # 2) Launch Point-LIO (reads from transformed topics via config)
  RVIZ_CFG="$(ros2 pkg prefix point_lio_unilidar)/share/point_lio_unilidar/rviz_cfg/pointlio_go2w.rviz"
  POINTLIO_CFG="$(ros2 pkg prefix point_lio_unilidar)/share/point_lio_unilidar/config/utlidar.yaml"
  echo "  Starting Point-LIO..."
  ros2 run point_lio_unilidar pointlio_mapping \
    --ros-args --params-file "$POINTLIO_CFG" \
    -p use_imu_as_input:=false \
    -p prop_at_freq_of_imu:=true \
    -p check_satu:=true \
    -p init_map_size:=10 \
    -p point_filter_num:=1 \
    -p space_down_sample:=true \
    -p filter_size_surf:=0.1 \
    -p filter_size_map:=0.1 \
    -r /cloud_registered:=/registered_scan \
    -r /aft_mapped_to_init:=/state_estimation &
  PLIO_PID=$!

  # Wait for IMU init to complete (polls for /state_estimation — only publishes after 100%)
  echo "  Waiting for Point-LIO IMU initialization (100%)..."
  for i in $(seq 1 60); do
    OD_PUB=$(timeout 3 ros2 topic info /state_estimation 2>/dev/null | grep -o 'Publisher count: [0-9]*' || echo "Publisher count: 0")
    if echo "$OD_PUB" | grep -q "[1-9]"; then
      echo "  ✅ Point-LIO IMU init complete — /state_estimation publishing"
      break
    fi
    if [[ $i -eq 60 ]]; then
      echo "  ⚠️  Timeout waiting for init — launching RViz anyway"
    fi
    sleep 1
  done

  echo "  Launching RViz..."
  rviz2 -d "$RVIZ_CFG" &
  RVIZ_PID=$!
  trap "kill $TE_PID $PLIO_PID $RVIZ_PID 2>/dev/null; echo ''; echo 'Stopped.'; exit 0" INT TERM
  wait $PLIO_PID
  exit 0
fi

# Point-LIO-SAM mode — transform_everything + Point-LIO + SC-PGO loop closure
if [[ "${1:-}" == "pointlio_sam" ]]; then
  echo "  Verifying /utlidar/cloud and /utlidar/imu..."
  PC_PUB=$(timeout 5 ros2 topic info /utlidar/cloud 2>/dev/null | grep -o 'Publisher count: [0-9]*' || echo "Publisher count: 0")
  IMU_PUB=$(timeout 5 ros2 topic info /utlidar/imu 2>/dev/null | grep -o 'Publisher count: [0-9]*' || echo "Publisher count: 0")
  echo "  /utlidar/cloud: $PC_PUB"
  echo "  /utlidar/imu:   $IMU_PUB"
  echo ""
  echo "################################################"
  echo "  Launching Point-LIO-SAM (full pipeline)"
  echo "  Pipeline:"
  echo "    /utlidar/{cloud,imu}"
  echo "      → transform_everything (15.1° pitch + axis flip)"
  echo "      → Point-LIO → /state_estimation + /registered_scan"
  echo "      → SC-PGO    → /corrected_odom + /corrected_path"
  echo "                    + /corrected_cloud + /loop_markers"
  echo "  Ctrl+C to stop"
  echo "################################################"
  echo ""

  CALIB_FILE="$REPO_ROOT/hardware/calibration/imu_calib_data.yaml"
  if [[ -f "$CALIB_FILE" ]]; then
    echo "  ✅ IMU calibration loaded from $CALIB_FILE"
  else
    echo "  ⚠️  No IMU calibration — using zero biases"
  fi

  # 0) Start transform_everything
  echo "  Starting transform_everything..."
  ros2 run transform_sensors transform_everything &
  TE_PID=$!
  sleep 2

  # 1) Launch Point-LIO
  POINTLIO_CFG="$(ros2 pkg prefix point_lio_unilidar)/share/point_lio_unilidar/config/utlidar.yaml"
  echo "  Starting Point-LIO..."
  ros2 run point_lio_unilidar pointlio_mapping \
    --ros-args --params-file "$POINTLIO_CFG" \
    -p use_imu_as_input:=false \
    -p prop_at_freq_of_imu:=true \
    -p check_satu:=true \
    -p init_map_size:=10 \
    -p point_filter_num:=1 \
    -p space_down_sample:=true \
    -p filter_size_surf:=0.1 \
    -p filter_size_map:=0.1 \
    -r /cloud_registered:=/registered_scan \
    -r /aft_mapped_to_init:=/state_estimation &
  PLIO_PID=$!

  # Wait for IMU init — must see actual odom data, not just publisher existing
  echo "  Waiting for Point-LIO IMU initialization..."
  echo "  (Keep robot STILL until you see '100.0%' in the output above)"
  for i in $(seq 1 90); do
    # Try to get one actual odom message — only succeeds after init complete
    GOT_MSG=$(timeout 2 ros2 topic echo --once /state_estimation nav_msgs/msg/Odometry 2>/dev/null | head -1 || true)
    if [[ -n "$GOT_MSG" ]]; then
      echo "  ✅ Point-LIO publishing odom — waiting 8s for scan matching to stabilize..."
      sleep 8
      echo "  ✅ Init complete — safe to walk now"
      break
    fi
    if [[ $i -eq 90 ]]; then
      echo "  ⚠️  Timeout (90s) waiting for init — launching SC-PGO anyway"
    fi
    sleep 1
  done

  # 2) Launch SC-PGO (loop closure)
  # SC-PGO expects /aft_mapped_to_init (odom) and /cloud_registered (scans)
  # Point-LIO remaps these to /state_estimation and /registered_scan
  # So remap SC-PGO's inputs back
  echo "  Starting SC-PGO (loop closure)..."
  SC_PGO_CONFIG="$(ros2 pkg prefix sc_pgo)/share/sc_pgo/config/sc_pgo_params.yaml"
  ros2 run sc_pgo sc_pgo_node \
    --ros-args --params-file "$SC_PGO_CONFIG" \
    -r /aft_mapped_to_init:=/state_estimation \
    -r /cloud_registered:=/registered_scan &
  SCPGO_PID=$!
  sleep 2

  # 3) Launch RViz — two windows: raw (left) and corrected (right)
  RVIZ_CFG="$(ros2 pkg prefix point_lio_unilidar)/share/point_lio_unilidar/rviz_cfg/pointlio_go2w.rviz"
  RVIZ_CFG2="$(ros2 pkg prefix point_lio_unilidar)/share/point_lio_unilidar/rviz_cfg/pointlio_corrected.rviz"
  echo "  Launching RViz (raw + corrected)..."
  rviz2 -d "$RVIZ_CFG" &
  RVIZ_PID=$!
  rviz2 -d "$RVIZ_CFG2" &
  RVIZ2_PID=$!
  trap "kill $TE_PID $PLIO_PID $SCPGO_PID $RVIZ_PID $RVIZ2_PID 2>/dev/null; echo ''; echo 'Stopped.'; exit 0" INT TERM
  wait $PLIO_PID
  exit 0
fi

# Cartographer 3D mode — transform_everything + Cartographer SLAM (built-in loop closure + occupancy grid)
if [[ "${1:-}" == "cartographer" ]]; then
  echo "  Verifying /utlidar/cloud and /utlidar/imu..."
  PC_PUB=$(timeout 5 ros2 topic info /utlidar/cloud 2>/dev/null | grep -o 'Publisher count: [0-9]*' || echo "Publisher count: 0")
  IMU_PUB=$(timeout 5 ros2 topic info /utlidar/imu 2>/dev/null | grep -o 'Publisher count: [0-9]*' || echo "Publisher count: 0")
  echo "  /utlidar/cloud: $PC_PUB"
  echo "  /utlidar/imu:   $IMU_PUB"
  echo ""
  echo "################################################"
  echo "  Launching Cartographer 3D SLAM"
  echo "  Pipeline:"
  echo "    /utlidar/{cloud,imu}"
  echo "      → transform_everything (15.1° pitch + axis flip)"
  echo "      → Cartographer 3D (local + global SLAM)"
  echo "      → /map (OccupancyGrid) + TF + submap list"
  echo "  Ctrl+C to stop"
  echo "################################################"
  echo ""

  # 0) Use the proven transform_everything (same as Point-LIO mode)
  echo "  Starting transform_everything..."
  ros2 run transform_sensors transform_everything &
  TE_PID=$!
  sleep 2

  # 1) Launch Cartographer
  CARTO_CFG="$REPO_ROOT/src/go2_real_bringup/config/cartographer"
  echo "  Starting Cartographer 3D..."
  ros2 run cartographer_ros cartographer_node \
    -configuration_directory "$CARTO_CFG" \
    -configuration_basename go2w_3d_mapping.lua \
    --ros-args \
    -r points2:=/utlidar/transformed_cloud \
    -r imu:=/utlidar/transformed_raw_imu \
    -p use_sim_time:=false &
  CARTO_PID=$!
  sleep 3

  # 2) Launch occupancy grid node
  echo "  Starting occupancy grid publisher..."
  ros2 run cartographer_ros cartographer_occupancy_grid_node \
    --ros-args \
    -p use_sim_time:=false \
    -p resolution:=0.05 \
    -p publish_period_sec:=1.0 &
  GRID_PID=$!

  # 3) Launch RViz — 3D cloud view (left) + occupancy grid view (right)
  RVIZ_CFG="$REPO_ROOT/src/go2_real_bringup/config/cartographer/cartographer.rviz"
  RVIZ_GRID_CFG="$REPO_ROOT/src/go2_real_bringup/config/cartographer/cartographer_grid.rviz"
  echo "  Launching RViz (3D cloud + occupancy grid)..."
  rviz2 -d "$RVIZ_CFG" &
  RVIZ_PID=$!
  rviz2 -d "$RVIZ_GRID_CFG" &
  RVIZ_GRID_PID=$!
  trap "kill $TE_PID $CARTO_PID $GRID_PID $RVIZ_PID $RVIZ_GRID_PID 2>/dev/null; echo ''; echo 'Stopped.'; exit 0" INT TERM
  wait $CARTO_PID
  exit 0
fi

# Half-auto mapping mode — Carto SLAM + Octomap + frontier detection, NO controller
# Manually drive the robot while it builds the map and visualizes frontiers
# Usage: ./go2w_ethernet_start.sh mapping
if [[ "${1:-}" == "mapping" ]]; then
  echo "  Verifying /utlidar/cloud and /utlidar/imu..."
  PC_PUB=$(timeout 5 ros2 topic info /utlidar/cloud 2>/dev/null | grep -o 'Publisher count: [0-9]*' || echo "Publisher count: 0")
  IMU_PUB=$(timeout 5 ros2 topic info /utlidar/imu 2>/dev/null | grep -o 'Publisher count: [0-9]*' || echo "Publisher count: 0")
  echo "  /utlidar/cloud: $PC_PUB"
  echo "  /utlidar/imu:   $IMU_PUB"
  echo ""
  echo "################################################"
  echo "  Launching MAPPING (half-auto — no controller)"
  echo "  Pipeline:"
  echo "    /utlidar/{cloud,imu}"
  echo "      → transform_everything"
  echo "      → Cartographer 3D SLAM → TF"
  echo "      → Octomap 3D (visualization)"
  echo "      → pointcloud_to_laserscan → scan_mapper → /robot/map"
  echo "      → frontier_3d_markers → /frontier_cylinders"
  echo "  Drive the robot manually with the controller."
  echo "  Ctrl+C to stop"
  echo "################################################"
  echo ""

  # Kill stale processes
  echo "  Cleaning up stale processes..."
  pkill -9 -f cartographer_node 2>/dev/null || true
  pkill -9 -f cartographer_occupancy 2>/dev/null || true
  pkill -9 -f transform_everything 2>/dev/null || true
  pkill -9 -f octomap_server 2>/dev/null || true
  pkill -9 -f cfpa2 2>/dev/null || true
  pkill -9 -f carto_odom_bridge 2>/dev/null || true
  pkill -9 -f pointcloud_to_laserscan 2>/dev/null || true
  pkill -9 -f simple_scan_mapper 2>/dev/null || true
  pkill -9 -f frontier_3d_markers 2>/dev/null || true
  killall -9 rviz2 2>/dev/null || true
  ros2 daemon stop 2>/dev/null || true
  sleep 1

  # 0) transform_everything
  echo "  Starting transform_everything..."
  ros2 run transform_sensors transform_everything &
  TE_PID=$!
  sleep 2

  # 1) Cartographer 3D SLAM
  CARTO_CFG="$REPO_ROOT/src/go2_real_bringup/config/cartographer"
  echo "  Starting Cartographer 3D..."
  ros2 run cartographer_ros cartographer_node \
    -configuration_directory "$CARTO_CFG" \
    -configuration_basename go2w_3d_mapping.lua \
    --ros-args \
    -r points2:=/utlidar/transformed_cloud \
    -r imu:=/utlidar/transformed_raw_imu \
    -p use_sim_time:=false &
  CARTO_PID=$!
  sleep 3

  # 2) Cartographer occupancy grid (native /map — keeps running as reference)
  echo "  Starting Cartographer occupancy grid..."
  ros2 run cartographer_ros cartographer_occupancy_grid_node \
    --ros-args \
    -p use_sim_time:=false \
    -p resolution:=0.05 \
    -p publish_period_sec:=1.0 &
  CGRID_PID=$!

  # 3) Static TF: body → base_link (identity)
  ros2 run tf2_ros static_transform_publisher \
    --frame-id body --child-frame-id base_link \
    --x 0 --y 0 --z 0 --qx 0 --qy 0 --qz 0 --qw 1 \
    --ros-args -p use_sim_time:=false &
  STATIC_TF_PID=$!

  # 4) Carto odom bridge (TF map→body → /robot/odom/nav)
  echo "  Starting carto_odom_bridge..."
  ros2 run go2w_perception carto_odom_bridge.py \
    --ros-args \
    -r __ns:=/robot \
    -p parent_frame:=map \
    -p child_frame:=body \
    -p output_topic:=/robot/odom/nav \
    -p output_frame_id:=map \
    -p output_child_frame_id:=base_link \
    -p rate:=50.0 &
  ODOM_PID=$!
  sleep 1

  # 5) Octomap server (3D voxels — for visualization only, /robot/map is broken)
  OCTO_CFG="$REPO_ROOT/src/go2_real_bringup/config/cartographer/octomap_mapping.yaml"
  echo "  Starting Octomap server..."
  ros2 run octomap_server octomap_server_node \
    --ros-args \
    --params-file "$OCTO_CFG" \
    -r cloud_in:=/utlidar/transformed_cloud \
    -r projected_map:=/octomap/projected_map &
  OCTO_PID=$!

  # 6) PointCloud → LaserScan (feeds scan mapper for proper 2D raycasting)
  echo "  Starting pointcloud_to_laserscan..."
  ros2 run pointcloud_to_laserscan pointcloud_to_laserscan_node \
    --ros-args \
    -r __ns:=/robot \
    -r cloud_in:=/utlidar/transformed_cloud \
    -r scan:=/robot/scan_3d \
    -p use_sim_time:=false \
    -p target_frame:=base_link \
    -p transform_tolerance:=0.3 \
    -p min_height:=-0.25 \
    -p max_height:=0.60 \
    -p angle_min:=-3.14159 \
    -p angle_max:=3.14159 \
    -p angle_increment:=0.006135923151543 \
    -p range_min:=0.10 \
    -p range_max:=8.0 \
    -p use_inf:=true &
  SCAN_PID=$!
  sleep 1

  # 7) Scan-based 2D mapper (proper raycasting → clear free/occupied/unknown)
  #    Produces /robot/map with value 0=free, 100=occupied, -1=unknown
  SCANMAP_CFG="$REPO_ROOT/src/go2_gazebo_sim/config/nav/simple_scan_mapper_single_go2w.yaml"
  echo "  Starting simple_scan_mapper_cpp..."
  ros2 run go2_nav_algorithms simple_scan_mapper_cpp \
    --ros-args \
    -r __ns:=/robot \
    --params-file "$SCANMAP_CFG" \
    -p scan_topic:=/robot/scan_3d \
    -p odom_topic:=/robot/odom/nav \
    -p map_topic:=/robot/map \
    -p map_frame:=map \
    -p broadcast_tf:=false &
  SCANMAP_PID=$!
  sleep 1

  # 8) Frontier detector — subscribes to /robot/map (scan mapper with proper
  #    free-space raycasting), publishes vertical cylinder markers.
  echo "  Starting frontier_3d_markers (on scan mapper /robot/map)..."
  ros2 run go2w_perception frontier_3d_markers.py \
    --ros-args \
    -p map_topic:=/robot/map \
    -p marker_topic:=/frontier_cylinders \
    -p frame_id:=map \
    -p free_threshold:=0 \
    -p occ_threshold:=50 \
    -p frontier_stride:=2 \
    -p min_cluster_area_m2:=0.5 \
    -p obstacle_clearance_m:=0.30 \
    -p max_frontiers:=180 \
    -p cylinder_height:=0.8 \
    -p cylinder_radius:=0.12 \
    -p cylinder_z_base:=0.0 \
    -p color_r:=0.0 \
    -p color_g:=1.0 \
    -p color_b:=0.3 \
    -p color_a:=0.75 &
  FRONTIER_PID=$!
  sleep 1

  # 9) RViz — octomap view (3D voxels + 2D grid + frontier cylinders)
  RVIZ_CFG="$REPO_ROOT/src/go2_real_bringup/config/cartographer/octomap.rviz"
  echo "  Launching RViz..."
  rviz2 -d "$RVIZ_CFG" &
  RVIZ_PID=$!

  echo ""
  echo "  ✅ Mapping mode running (no controller)"
  echo "     Drive the robot with the physical controller"
  echo "     Frontiers shown as green cylinders in RViz (/frontier_cylinders)"
  echo "     Ctrl+C to stop everything"
  echo ""

  trap "kill $TE_PID $CARTO_PID $CGRID_PID $STATIC_TF_PID $ODOM_PID $OCTO_PID $SCAN_PID $SCANMAP_PID $FRONTIER_PID $RVIZ_PID 2>/dev/null; echo ''; echo 'Stopped.'; exit 0" INT TERM
  wait $CARTO_PID
  exit 0
fi

# Full autonomy mode — Cartographer SLAM + CFPA2 exploration + reactive navigation
# Usage: ./go2w_ethernet_start.sh autonomy [scan|octomap|elevation]
if [[ "${1:-}" == "autonomy" ]]; then
  MAPPER_TYPE="${2:-scan}"  # default: scan (simple_scan_mapper_cpp)
  echo "  Verifying /utlidar/cloud and /utlidar/imu..."
  PC_PUB=$(timeout 5 ros2 topic info /utlidar/cloud 2>/dev/null | grep -o 'Publisher count: [0-9]*' || echo "Publisher count: 0")
  IMU_PUB=$(timeout 5 ros2 topic info /utlidar/imu 2>/dev/null | grep -o 'Publisher count: [0-9]*' || echo "Publisher count: 0")
  echo "  /utlidar/cloud: $PC_PUB"
  echo "  /utlidar/imu:   $IMU_PUB"
  echo ""
  echo "################################################"
  echo "  Launching FULL AUTONOMY"
  echo "  Mapper: $MAPPER_TYPE"
  echo "  Pipeline:"
  echo "    /utlidar/{cloud,imu}"
  echo "      → transform_everything (15.1° pitch + axis flip)"
  echo "      → Cartographer 3D SLAM → /map + TF"
  case "$MAPPER_TYPE" in
    scan)
      echo "      → pointcloud_to_laserscan → simple_scan_mapper → /robot/map"
      ;;
    octomap)
      echo "      → octomap_server (3D voxels → 2D projection) → /robot/map"
      ;;
    elevation)
      echo "      → grid_map elevation → traversability → /robot/map"
      ;;
    *)
      echo "  ERROR: Unknown mapper type '$MAPPER_TYPE'"
      echo "  Valid options: scan, octomap, elevation"
      exit 1
      ;;
  esac
  echo "      → CFPA2 frontier exploration → waypoints"
  echo "      → default_nav (A* + local avoid) → /cmd_vel"
  echo "      → Unitree obstacle avoidance API (api_id=1003)"
  echo "      → frontier_3d_markers → /frontier_cylinders (RViz)"
  echo "  Ctrl+C to stop"
  echo "################################################"
  echo ""

  # Kill stale ROS/DDS processes from previous runs to avoid participant exhaustion
  echo "  Cleaning up stale processes..."
  pkill -9 -f cartographer_node 2>/dev/null || true
  pkill -9 -f cartographer_occupancy 2>/dev/null || true
  pkill -9 -f transform_everything 2>/dev/null || true
  pkill -9 -f default_nav 2>/dev/null || true
  pkill -9 -f cfpa2 2>/dev/null || true
  pkill -9 -f carto_odom_bridge 2>/dev/null || true
  pkill -9 -f twist_bridge 2>/dev/null || true
  pkill -9 -f cmd_vel_activity_mux 2>/dev/null || true
  pkill -9 -f octomap_server 2>/dev/null || true
  pkill -9 -f elevation_to_occupancy 2>/dev/null || true
  pkill -9 -f simple_scan_mapper 2>/dev/null || true
  pkill -9 -f frontier_3d_markers 2>/dev/null || true
  pkill -9 -f cmd_vel_to_sport 2>/dev/null || true
  killall -9 rviz2 2>/dev/null || true
  ros2 daemon stop 2>/dev/null || true
  sleep 1

  # 0) transform_everything
  echo "  Starting transform_everything..."
  ros2 run transform_sensors transform_everything &
  TE_PID=$!
  sleep 2

  # 1) Cartographer
  CARTO_CFG="$REPO_ROOT/src/go2_real_bringup/config/cartographer"
  echo "  Starting Cartographer 3D..."
  ros2 run cartographer_ros cartographer_node \
    -configuration_directory "$CARTO_CFG" \
    -configuration_basename go2w_3d_mapping.lua \
    --ros-args \
    -r points2:=/utlidar/transformed_cloud \
    -r imu:=/utlidar/transformed_raw_imu \
    -p use_sim_time:=false &
  CARTO_PID=$!
  sleep 3

  # 2) Occupancy grid (Cartographer's native /map — used as reference/fallback)
  echo "  Starting Cartographer occupancy grid publisher..."
  ros2 run cartographer_ros cartographer_occupancy_grid_node \
    --ros-args \
    -p use_sim_time:=false \
    -p resolution:=0.05 \
    -p publish_period_sec:=1.0 &
  GRID_PID=$!
  sleep 2

  # 3) Launch mapper-specific nodes
  MAPPER_PIDS=""
  EXTERNAL_MAPPER="false"

  case "$MAPPER_TYPE" in
    scan)
      # Default: simple_scan_mapper launched inside the nav launch file
      echo "  Mapper: simple_scan_mapper_cpp (internal to launch file)"
      EXTERNAL_MAPPER="false"
      ;;

    octomap)
      # Octomap: 3D voxel map → 2D projected occupancy grid
      EXTERNAL_MAPPER="true"
      OCTO_CFG="$REPO_ROOT/src/go2_real_bringup/config/cartographer/octomap_mapping.yaml"
      echo "  Starting Octomap server..."
      ros2 run octomap_server octomap_server_node \
        --ros-args \
        --params-file "$OCTO_CFG" \
        -r cloud_in:=/utlidar/transformed_cloud \
        -r projected_map:=/robot/map &
      MAPPER_PIDS="$!"
      echo "  Octomap server PID: $MAPPER_PIDS"
      ;;

    elevation)
      # Grid map elevation → traversability → OccupancyGrid
      EXTERNAL_MAPPER="true"
      # For now, use octomap for the 3D representation and elevation bridge for traversability
      OCTO_CFG="$REPO_ROOT/src/go2_real_bringup/config/cartographer/octomap_mapping.yaml"
      echo "  Starting Octomap server (for 3D representation)..."
      ros2 run octomap_server octomap_server_node \
        --ros-args \
        --params-file "$OCTO_CFG" \
        -r cloud_in:=/utlidar/transformed_cloud &
      OCTO_PID=$!
      sleep 2

      echo "  Starting elevation→occupancy bridge..."
      python3 "$REPO_ROOT/src/go2w_perception/scripts/elevation_to_occupancy.py" \
        --ros-args \
        -p input_topic:=/robot/elevation_map \
        -p output_topic:=/robot/map \
        -p frame_id:=map \
        -p max_step_height:=0.08 \
        -p max_slope_rad:=0.35 &
      ELEV_PID=$!
      MAPPER_PIDS="$OCTO_PID $ELEV_PID"
      echo "  Elevation mapper PIDs: $MAPPER_PIDS"
      ;;
  esac
  sleep 1

  # 4) Octomap server (3D visualization only)
  OCTO_CFG="$REPO_ROOT/src/go2_real_bringup/config/cartographer/octomap_mapping.yaml"
  echo "  Starting Octomap server (3D visualization)..."
  ros2 run octomap_server octomap_server_node \
    --ros-args \
    --params-file "$OCTO_CFG" \
    -r cloud_in:=/utlidar/transformed_cloud \
    -r projected_map:=/octomap/projected_map &
  OCTO_PID=$!

  # 5) Full navigation stack (CFPA2 + default_nav + Unitree obstacle avoidance)
  echo "  Launching navigation stack (CFPA2 + default_nav + obstacle_avoidance)..."
  ros2 launch go2_real_bringup single_go2w_real_cfpa2.launch.py \
    robot_namespace:=robot \
    enable_manual_fallback:=true \
    external_mapper:=$EXTERNAL_MAPPER \
    obstacle_avoidance:=true &
  NAV_PID=$!
  sleep 2

  # 6) Frontier detector (subscribes to /robot/map from scan mapper inside launch)
  echo "  Starting frontier_3d_markers..."
  ros2 run go2w_perception frontier_3d_markers.py \
    --ros-args \
    -p map_topic:=/robot/map \
    -p marker_topic:=/frontier_cylinders \
    -p frame_id:=map \
    -p free_threshold:=0 \
    -p occ_threshold:=50 \
    -p obstacle_clearance_m:=0.30 \
    -p cylinder_height:=0.8 \
    -p cylinder_radius:=0.12 &
  FRONTIER_PID=$!

  # 7) RViz — octomap 3D view + scan mapper 2D grid + frontier cylinders
  RVIZ_CFG="$REPO_ROOT/src/go2_real_bringup/config/cartographer/octomap.rviz"
  echo "  Launching RViz..."
  rviz2 -d "$RVIZ_CFG" &
  RVIZ_PID=$!

  echo ""
  echo "  ✅ Full autonomy stack running (mapper=$MAPPER_TYPE)"
  echo "     Unitree obstacle avoidance: ENABLED (api_id=1003)"
  echo "     Frontiers shown as green cylinders in RViz"
  echo "     Manual override: use joystick"
  echo "     Ctrl+C to stop everything"
  echo ""

  trap "kill $TE_PID $CARTO_PID $GRID_PID $OCTO_PID $NAV_PID $FRONTIER_PID $RVIZ_PID $MAPPER_PIDS 2>/dev/null; echo ''; echo 'Stopped.'; exit 0" INT TERM
  wait $CARTO_PID
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
echo "  Ctrl+C to stop live monitor"
echo "################################################"
echo ""

# ── LIVE MONITOR ─────────────────────────────────────────────────────
trap 'echo ""; echo "Monitor stopped."; kill $(jobs -p) 2>/dev/null; exit 0' INT TERM

echo "=== LIVE MONITOR — Ctrl+C to stop ==="
echo ""

# Background: point cloud rate (continuously printed)
(
  while true; do
    timeout 4 ros2 topic hz /utlidar/cloud 2>/dev/null | tail -1 | \
      sed 's/^/  [PC] /'
    sleep 1
  done
) &

# Background: joint state — prefer /joint_states (go2w_driver), fallback /lowstate
(
  # Detect which topic is available
  JS_TOPIC=""
  if timeout 3 ros2 topic info /joint_states 2>/dev/null | grep -q "Publisher count: [1-9]"; then
    JS_TOPIC="/joint_states"
  else
    JS_TOPIC="/lowstate"
  fi
  echo "  [JOINTS] using $JS_TOPIC"

  while true; do
    if [[ "$JS_TOPIC" == "/joint_states" ]]; then
      JOINTS=$(timeout 3 ros2 topic echo /joint_states --once 2>/dev/null | \
        grep -A1 "position:" | tail -1 | tr -d '[]' | \
        awk -F, '{for(i=1;i<=NF;i++) printf "%.3f ", $i}')
    else
      JOINTS=$(timeout 3 ros2 topic echo /lowstate --once 2>/dev/null | \
        grep "q:" | head -16 | awk '{printf "%.3f ", $2}')
    fi
    if [[ -n "$JOINTS" ]]; then
      echo "  [JOINTS] $JOINTS"
    fi
    sleep 3
  done
) &

# Background: robot pose (throttled)
(
  while true; do
    POSE=$(timeout 3 ros2 topic echo /utlidar/robot_pose --once 2>/dev/null | \
      awk '/position:/{getline; x=$2; getline; y=$2; getline; z=$2; printf "x=%.3f y=%.3f z=%.3f", x, y, z}')
    if [[ -n "$POSE" ]]; then
      echo "  [POSE]   $POSE"
    fi
    sleep 2
  done
) &

wait
