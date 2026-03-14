#!/bin/bash
# replay_pointcloud.sh — Replay recorded bag through Point-LIO SLAM + RViz
#
# Usage: ./replay_pointcloud.sh data/hallway_run/
#        ./replay_pointcloud.sh data/hallway_run/ --rate 0.5
#        ./replay_pointcloud.sh data/hallway_run/ --no-rviz
#
# Pipeline:
#   rosbag2 → /utlidar/{cloud,imu}
#     → transform_everything → /utlidar/transformed_{cloud,imu}
#     → Point-LIO → /registered_scan + /state_estimation
#     → RViz (visualize)

set -e
REPO_ROOT="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"

source /opt/ros/humble/setup.bash
[[ -f "$REPO_ROOT/install/setup.bash" ]] && source "$REPO_ROOT/install/setup.bash"

# ── Parse args ───────────────────────────────────────────────────────
BAG_PATH=""
RATE="1.0"
NO_RVIZ=false
EXTRA_ARGS=()

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
      else
        EXTRA_ARGS+=("$arg")
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
echo "  Point Cloud Replay through Point-LIO"
echo ""
echo "  Bag:  $BAG_PATH"
echo "  Rate: ${RATE}x"
echo ""
echo "  Pipeline:"
echo "    rosbag → transform_everything → Point-LIO → RViz"
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

# 1. Start transform_everything
echo "  Starting transform_everything..."
ros2 run transform_sensors transform_everything &
PIDS+=($!)
sleep 2

# 2. Start Point-LIO
echo "  Starting Point-LIO..."
ros2 launch point_lio_unilidar mapping_utlidar.launch rviz:=false &
PIDS+=($!)
sleep 2

# 3. Start RViz
if [[ "$NO_RVIZ" == "false" ]]; then
  echo "  Starting RViz..."
  RVIZ_CONFIG="$REPO_ROOT/install/point_lio_unilidar/share/point_lio_unilidar/rviz_cfg/loam_livox2.rviz"
  if [[ -f "$RVIZ_CONFIG" ]]; then
    rviz2 -d "$RVIZ_CONFIG" &
  else
    rviz2 &
  fi
  PIDS+=($!)
  sleep 1
fi

# 4. Play bag (only raw topics — let transform_everything process them)
echo "  Playing bag at ${RATE}x..."
echo ""
ros2 bag play "$BAG_PATH" \
  --rate "$RATE" \
  --clock \
  --topics /utlidar/cloud /utlidar/imu /tf /tf_static \
  "${EXTRA_ARGS[@]}"

echo ""
echo "Playback complete."
# Keep alive briefly so Point-LIO finishes processing
sleep 3
