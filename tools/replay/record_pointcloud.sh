#!/bin/bash
# record_pointcloud.sh — Record point cloud, IMU, and odometry data for offline SLAM
#
# Usage: ./record_pointcloud.sh              # record with timestamp dirname
#        ./record_pointcloud.sh my_run       # record to data/my_run/
#        ./record_pointcloud.sh --topics     # show what will be recorded
#
# Output: data/<name>/  (rosbag2 format, playable with ros2 bag play)

set -e
REPO_ROOT="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"

# DDS setup (same as go2w_ethernet_start.sh)
source /opt/ros/humble/setup.bash
[[ -f "$REPO_ROOT/install/setup.bash" ]] && source "$REPO_ROOT/install/setup.bash"
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ETH_IFACE="${GO2W_ETH_IFACE:-enxc8a36240a4c7}"
ROBOT_IP="${GO2W_ETH_IP:-192.168.123.161}"
export CYCLONEDDS_URI="<CycloneDDS><Domain>
  <General><Interfaces>
    <NetworkInterface name=\"${ETH_IFACE}\" priority=\"default\" multicast=\"true\" />
  </Interfaces></General>
  <Discovery>
    <Peers><Peer address=\"${ROBOT_IP}\"/></Peers>
    <ParticipantIndex>auto</ParticipantIndex>
    <MaxAutoParticipantIndex>120</MaxAutoParticipantIndex>
  </Discovery>
</Domain></CycloneDDS>"
export ROS_DOMAIN_ID=0

# Topics to record
TOPICS=(
  /utlidar/cloud                    # raw 3D point cloud (~15 Hz)
  /utlidar/imu                      # raw IMU (high rate)
  /utlidar/transformed_cloud        # pitch-corrected cloud (if transform_everything running)
  /utlidar/transformed_imu          # axis-flipped IMU
  /registered_scan                  # Point-LIO registered scan (if running)
  /state_estimation                 # Point-LIO odometry (if running)
  /tf                               # all transforms
  /tf_static                        # static transforms
)

if [[ "${1:-}" == "--topics" ]]; then
  echo "Topics to record:"
  for t in "${TOPICS[@]}"; do
    echo "  $t"
  done
  exit 0
fi

# Output directory
RUN_NAME="${1:-$(date +%Y%m%d_%H%M%S)}"
OUT_DIR="$REPO_ROOT/data/$RUN_NAME"
mkdir -p "$REPO_ROOT/data"

echo "################################################"
echo "  Point Cloud Data Recorder"
echo ""
echo "  Output: $OUT_DIR/"
echo "  Format: rosbag2 (sqlite3)"
echo ""
echo "  Topics:"
for t in "${TOPICS[@]}"; do
  echo "    $t"
done
echo ""
echo "  Ctrl+C to stop recording"
echo "################################################"
echo ""

# Write QoS overrides to temp file
QOS_FILE=$(mktemp /tmp/qos_overrides_XXXX.yaml)
trap "rm -f $QOS_FILE" EXIT
cat > "$QOS_FILE" <<EOF
/utlidar/cloud:
  reliability: best_effort
  durability: volatile
  history: keep_last
  depth: 10
/utlidar/imu:
  reliability: best_effort
  durability: volatile
  history: keep_last
  depth: 10
/utlidar/transformed_cloud:
  reliability: best_effort
  durability: volatile
  history: keep_last
  depth: 10
/utlidar/transformed_imu:
  reliability: best_effort
  durability: volatile
  history: keep_last
  depth: 10
EOF

exec ros2 bag record \
  -o "$OUT_DIR" \
  -s sqlite3 \
  --qos-profile-overrides-path "$QOS_FILE" \
  "${TOPICS[@]}"
