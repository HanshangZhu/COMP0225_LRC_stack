#!/usr/bin/env bash
set -eo pipefail

WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROS2_SETUP_BASH="${ROS2_SETUP_BASH:-/opt/ros/humble/setup.bash}"
PROFILE="${1:-autonomy_baseline}"

safe_source() {
  local had_u=0
  if [[ $- == *u* ]]; then
    had_u=1
    set +u
  fi
  # shellcheck disable=SC1090
  source "$1"
  local rc=$?
  if [[ ${had_u} -eq 1 ]]; then
    set -u
  fi
  return ${rc}
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

safe_source "${ROS2_SETUP_BASH}"

if [[ ! -f "${WS_DIR}/install/setup.bash" ]]; then
  echo "Missing ${WS_DIR}/install/setup.bash. Build first: colcon build --symlink-install" >&2
  exit 1
fi
safe_source "${WS_DIR}/install/setup.bash"

USD_PATH="${WS_DIR}/src/go2_issac_stack/assets/unitree_model/Go2/usd/go2.usd"

case "$PROFILE" in
  sensor_realism|perf)
    # Sensor realism branch: native RTX lidar stream.
    ros2 launch go2_issac_stack two_go2_isaac_coordinated_autonomy.launch.py \
      isaac_headless:=head \
      rviz:=true \
      isaac_lidar_mode:=rtx \
      isaac_renderer:=RayTracedLighting \
      isaac_rtx_lidar_config:=NVIDIA/Debug_Rotary \
      isaac_lidar_rays:=360 \
      isaac_pointcloud_hz:=5.0 \
      isaac_rtx_full_scan:=true \
      use_cpp_mapper:=true \
      use_fast_lio:=false \
      enable_dynamic_physics:=false \
      enable_champ_stack:=false \
      enable_goal_assigner:=false \
      enable_status_monitor:=false \
      enable_coverage_visualizer:=true \
      frontier_prefer_costmap:=true \
      mapper_update_rate:=2.0 \
      frontier_update_rate:=1.5 \
      planning_scan_min_height:=-1.0 \
      planning_scan_max_height:=2.0 \
      planning_scan_range_max:=12.0 \
      isaac_robot_usd:="$USD_PATH"
    ;;

  autonomy_baseline|balanced)
    # Autonomy baseline branch: synthetic CPU raycast pointcloud (GT geometry).
    ros2 launch go2_issac_stack two_go2_isaac_coordinated_autonomy.launch.py \
      isaac_headless:=true \
      rviz:=true \
      isaac_lidar_mode:=cpu \
      isaac_lidar_rays:=720 \
      isaac_pointcloud_hz:=5.0 \
      use_cpp_mapper:=true \
      use_fast_lio:=false \
      enable_dynamic_physics:=false \
      enable_champ_stack:=false \
      enable_goal_assigner:=true \
      enable_status_monitor:=true \
      enable_coverage_visualizer:=true \
      frontier_prefer_costmap:=true \
      mapper_update_rate:=2.0 \
      frontier_update_rate:=2.0 \
      planning_scan_min_height:=0.10 \
      planning_scan_max_height:=0.80 \
      planning_scan_range_min:=0.25 \
      planning_scan_range_max:=12.0 \
      isaac_robot_usd:="$USD_PATH"
    ;;

  debug)
    # Fidelity/debug profile (matches prior behavior closely).
    ros2 launch go2_issac_stack two_go2_isaac_coordinated_autonomy.launch.py \
      isaac_headless:=false \
      rviz:=true \
      isaac_lidar_mode:=cpu \
      isaac_lidar_rays:=3600 \
      frontier_prefer_costmap:=false \
      use_cpp_mapper:=true \
      enable_dynamic_physics:=true \
      enable_champ_stack:=true \
      enable_goal_assigner:=true \
      enable_status_monitor:=true \
      enable_coverage_visualizer:=true \
      planning_scan_min_height:=-1.0 \
      planning_scan_max_height:=2.0 \
      planning_scan_range_max:=12.0 \
      isaac_robot_usd:="$USD_PATH"
    ;;

  *)
    echo "Unknown profile '$PROFILE'. Use one of: autonomy_baseline | sensor_realism | debug" >&2
    echo "Aliases: balanced -> autonomy_baseline, perf -> sensor_realism" >&2
    exit 2
    ;;
esac
