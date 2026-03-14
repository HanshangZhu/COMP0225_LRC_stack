#!/bin/bash
# replay_fastlio_sam.sh — Replay bag through FAST-LIO2 + SC-PGO (loop closure) + mapper
#
# Usage: ./replay_fastlio_sam.sh data/hallway_run/
#        ./replay_fastlio_sam.sh data/hallway_run/ --rate 0.5
#        ./replay_fastlio_sam.sh data/hallway_run/ --no-rviz
#
# Pipeline:
#   rosbag2 → /utlidar/{cloud,imu}
#     → Fast-LIO2 → /Odometry + /cloud_registered
#     → SC-PGO   → /corrected_odom + /corrected_cloud + /loop_markers
#     → RViz

set -e
REPO_ROOT="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"

source /opt/ros/humble/setup.bash
[[ -f "$REPO_ROOT/install/setup.bash" ]] && source "$REPO_ROOT/install/setup.bash"

# ── Parse args ───────────────────────────────────────────────────
BAG_PATH=""
RATE="1.0"
NO_RVIZ=false

for arg in "$@"; do
  case "$arg" in
    --no-rviz) NO_RVIZ=true ;;
    --rate)    shift_next=rate ;;
    *)
      if [[ "${shift_next:-}" == "rate" ]]; then
        RATE="$arg"
        shift_next=""
      elif [[ -z "$BAG_PATH" ]]; then
        BAG_PATH="$arg"
      fi
      ;;
  esac
done

if [[ -z "$BAG_PATH" ]]; then
  echo "Usage: $0 <bag_path> [--rate N] [--no-rviz]"
  echo ""
  echo "Available bags:"
  ls -1d "$REPO_ROOT"/data/*/ 2>/dev/null | while read d; do
    info=$(ros2 bag info "$d" 2>/dev/null | grep -E "Duration|Messages" | head -2 | tr '\n' ' ')
    echo "  $(basename "$d")  $info"
  done
  exit 1
fi

if [[ ! -d "$BAG_PATH" ]] && [[ -d "$REPO_ROOT/data/$BAG_PATH" ]]; then
  BAG_PATH="$REPO_ROOT/data/$BAG_PATH"
fi

if [[ ! -d "$BAG_PATH" ]]; then
  echo "ERROR: Bag not found: $BAG_PATH" >&2
  exit 1
fi

echo "################################################"
echo "  FAST-LIO-SAM Replay (Loop Closure)"
echo ""
echo "  Bag:  $BAG_PATH"
echo "  Rate: ${RATE}x"
echo ""
echo "  Pipeline:"
echo "    rosbag → Fast-LIO2 → SC-PGO (loop closure) → RViz"
echo ""
echo "  Topics published by SC-PGO:"
echo "    /corrected_odom    (nav_msgs/Odometry)"
echo "    /corrected_path    (nav_msgs/Path)"
echo "    /corrected_cloud   (PointCloud2)"
echo "    /loop_markers      (MarkerArray)"
echo ""
echo "  Ctrl+C to stop"
echo "################################################"
echo ""

PIDS=()
cleanup() {
  echo ""
  echo "Stopping all processes..."
  for pid in "${PIDS[@]}"; do
    kill "$pid" 2>/dev/null || true
  done
  wait 2>/dev/null
  echo "Done."
}
trap cleanup EXIT INT TERM

# 1. Start Fast-LIO2
echo "  Starting Fast-LIO2..."
ros2 launch fast_lio mapping.launch.py \
  config_file:=go2w_utlidar.yaml \
  use_sim_time:=false \
  rviz:=false &
PIDS+=($!)
sleep 3

# 2. Start SC-PGO (loop closure)
echo "  Starting SC-PGO (loop closure)..."
SC_PGO_CONFIG="$REPO_ROOT/install/sc_pgo/share/sc_pgo/config/sc_pgo_params.yaml"
ros2 run sc_pgo sc_pgo_node \
  --ros-args --params-file "$SC_PGO_CONFIG" \
  -r /aft_mapped_to_init:=/Odometry \
  -r /cloud_registered:=/cloud_registered &
PIDS+=($!)
sleep 2

# 3. Start RViz
if [[ "$NO_RVIZ" == "false" ]]; then
  echo "  Starting RViz..."
  rviz2 &
  PIDS+=($!)
  sleep 1
fi

# 4. Play bag
echo "  Playing bag at ${RATE}x..."
echo ""
ros2 bag play "$BAG_PATH" \
  --rate "$RATE" \
  --topics /utlidar/cloud /utlidar/imu /tf /tf_static

echo ""
echo "Playback complete."
# Keep alive so SC-PGO finishes processing
echo "  Waiting for SC-PGO to finish optimization..."
sleep 10
