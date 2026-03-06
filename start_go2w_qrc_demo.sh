#!/usr/bin/env bash
set -euo pipefail

WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
QRC_WS_DIR="${WS_DIR}/COMP0225_QRC_DEMO"
ROS2_SETUP_BASH="${ROS2_SETUP_BASH:-/opt/ros/humble/setup.bash}"

safe_source() {
  set +u
  # shellcheck disable=SC1090
  source "$1"
  set -u
}

prepend_path() {
  local var_name="$1"
  local path_value="$2"
  local current_value="${!var_name:-}"
  if [[ -z "${current_value}" ]]; then
    export "${var_name}=${path_value}"
    return
  fi
  case ":${current_value}:" in
    *":${path_value}:"*) ;;
    *) export "${var_name}=${path_value}:${current_value}" ;;
  esac
}

if [[ -f "${HOME}/miniforge3/etc/profile.d/conda.sh" ]]; then
  safe_source "${HOME}/miniforge3/etc/profile.d/conda.sh"
  conda activate cmu_env
elif command -v micromamba >/dev/null 2>&1; then
  eval "$(micromamba shell hook -s bash)"
  micromamba activate cmu_env
else
  echo "Could not find conda/micromamba activation script for cmu_env." >&2
  exit 1
fi

if [[ ! -f "${ROS2_SETUP_BASH}" ]]; then
  echo "Missing ROS2 setup script: ${ROS2_SETUP_BASH}" >&2
  exit 1
fi
safe_source "${ROS2_SETUP_BASH}"

if [[ ! -f "${WS_DIR}/install/setup.bash" ]]; then
  echo "Missing ${WS_DIR}/install/setup.bash. Build the root workspace first." >&2
  exit 1
fi
safe_source "${WS_DIR}/install/setup.bash"

if [[ ! -f "${QRC_WS_DIR}/install/setup.bash" ]]; then
  echo "Missing ${QRC_WS_DIR}/install/setup.bash. Build COMP0225_QRC_DEMO first." >&2
  exit 1
fi
safe_source "${QRC_WS_DIR}/install/setup.bash"

prepend_path GAZEBO_RESOURCE_PATH "${QRC_WS_DIR}/install/comp0225_qrc_demo_bringup/share/comp0225_qrc_demo_bringup"
prepend_path GAZEBO_MODEL_PATH "${QRC_WS_DIR}/install/comp0225_qrc_demo_bringup/share/comp0225_qrc_demo_bringup"
prepend_path GAZEBO_RESOURCE_PATH "${WS_DIR}/install"
prepend_path GAZEBO_MODEL_PATH "${WS_DIR}/install"

export ROS_LOG_DIR="${ROS_LOG_DIR:-/tmp/ros_logs}"
mkdir -p "${ROS_LOG_DIR}"

kill_pattern() {
  local pattern="$1"
  for pid in $(pgrep -f "$pattern" 2>/dev/null || true); do
    [[ "$pid" == "$$" ]] && continue
    [[ "$pid" == "$PPID" ]] && continue
    kill "$pid" 2>/dev/null || true
  done
}

kill_pattern '[g]zserver'
kill_pattern '(^|/)gzclient( |$)'
kill_pattern '/go2_nav_algorithms/lib/go2_nav_algorithms/simple_scan_mapper_cpp'
kill_pattern '/go2_nav_algorithms/lib/go2_nav_algorithms/simple_frontier_explorer.py'
kill_pattern '/go2w_control/lib/go2w_control/reactive_nav.py'
kill_pattern '/go2w_control/lib/go2w_control/wall_collision_checker.py'
kill_pattern '/go2w_spawn/lib/go2w_spawn/initial_pose_guard.py'
kill_pattern '/go2w_spawn/lib/go2w_spawn/spawn_entity_direct.py'
kill_pattern '/go2w_perception/lib/go2w_perception/pointcloud_adapter.py'
kill_pattern '/go2w_spawn/lib/go2w_spawn/stand_up_slowly.py'
kill_pattern '/comp0225_qrc_demo_bringup/lib/comp0225_qrc_demo_bringup/qos_bridge.py'
kill_pattern '/comp0225_qrc_demo_bringup/lib/comp0225_qrc_demo_bringup/pointcloud_adapter.py'
kill_pattern '/comp0225_qrc_demo_bringup/lib/comp0225_qrc_demo_bringup/slam_odom_relay.py'
kill_pattern '/comp0225_qrc_demo_bringup/lib/comp0225_qrc_demo_bringup/autonomy_enabler.py'
kill_pattern '/comp0225_qrc_demo_bringup/lib/comp0225_qrc_demo_bringup/twist_bridge.py'
kill_pattern '/pointcloud_to_laserscan_node'
sleep 1

DEFAULT_LAUNCH_ARGS=(
  "robot_variant:=go2w"
  "robot_name:=go2w"
  "use_sim_time:=true"
  "gui:=true"
  "rviz:=false"
  "cleanup_stale:=false"
  "use_slam:=false"
  "spawn_x:=2.5"
  "spawn_y:=0.0"
  "spawn_z:=0.45"
  "spawn_heading:=0.0"
)

exec ros2 launch comp0225_qrc_demo_bringup gazebo_demo.launch.py \
  "${DEFAULT_LAUNCH_ARGS[@]}" \
  "$@"
