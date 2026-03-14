#!/usr/bin/env bash
set -euo pipefail

WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ROS2_SETUP_BASH="${ROS2_SETUP_BASH:-/opt/ros/humble/setup.bash}"

safe_source() {
  set +u
  # shellcheck disable=SC1090
  source "$1"
  set -u
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
  echo "Missing ${WS_DIR}/install/setup.bash. Build first: colcon build --symlink-install" >&2
  exit 1
fi
safe_source "${WS_DIR}/install/setup.bash"

export ROS_LOG_DIR="${ROS_LOG_DIR:-/tmp/ros_logs}"
mkdir -p "${ROS_LOG_DIR}"

terminate_matching_processes() {
  local pattern="$1"
  local signal="$2"
  local pid
  for pid in $(pgrep -f "$pattern" 2>/dev/null || true); do
    [[ "$pid" == "$$" ]] && continue
    [[ "$pid" == "$PPID" ]] && continue
    kill "-${signal}" "$pid" 2>/dev/null || true
  done
}

if [[ "${CLEANUP_STALE_GAZEBO:-1}" == "1" ]]; then
  CLEANUP_PATTERNS=(
    'ros2 launch go2_gazebo_sim single_go2w_gazebo_cfpa2.launch.py'
    'ros2 launch go2_gazebo_sim dual_go2_modular.launch.py'
    'ros2 launch go2_gazebo_sim dual_go2w_modular.launch.py'
    '[g]zserver'
    '(^|/)gzclient( |$)'
    '(^|/)gazebo( |$)'
    '/go2_nav_algorithms/lib/go2_nav_algorithms/simple_scan_mapper_cpp'
    '/go2_nav_algorithms/lib/go2_nav_algorithms/simple_frontier_explorer.py'
    '/go2w_observability/lib/go2w_observability/dual_map_coverage_visualizer.py'
    '/go2_gazebo_sim/lib/go2_gazebo_sim/shared_map_fuser.py'
    '/go2w_control/lib/go2w_control/reactive_nav.py'
    '/go2w_control/lib/go2w_control/autonomy_enabler.py'
    '/go2w_perception/lib/go2w_perception/twist_bridge.py'
    '/go2w_control/lib/go2w_control/go2w_hybrid_cmd_router.py'
    '/go2w_perception/lib/go2w_perception/qos_bridge.py'
    '/go2w_observability/lib/go2w_observability/robot_status_monitor.py'
    '/go2w_spawn/lib/go2w_spawn/initial_pose_guard.py'
    '/go2w_spawn/lib/go2w_spawn/spawn_entity_direct.py'
    '/go2w_perception/lib/go2w_perception/pointcloud_adapter.py'
    '/go2w_perception/lib/go2w_perception/slam_odom_relay.py'
    '/fast_lio/lib/fast_lio/fastlio_mapping'
    '/cfpa2_collaborative_autonomy/lib/cfpa2_collaborative_autonomy/cfpa2_coordinator_node'
    '/champ_base/lib/champ_base/quadruped_controller_node'
    '/champ_base/lib/champ_base/state_estimation_node'
    '/robot_localization/ekf_node'
    '/robot_state_publisher'
    '/champ_gazebo/lib/champ_gazebo/contact_sensor'
    '/opt/ros/.*/lib/controller_manager/spawner'
    '/pointcloud_to_laserscan_node'
  )

  for pattern in "${CLEANUP_PATTERNS[@]}"; do
    terminate_matching_processes "$pattern" TERM
  done
  sleep 1
  for pattern in "${CLEANUP_PATTERNS[@]}"; do
    terminate_matching_processes "$pattern" KILL
  done
  sleep 1
fi

if [[ -z "${GAZEBO_MASTER_URI:-}" ]]; then
  pick_port() {
    local port
    for port in "$@"; do
      if ! ss -H -ltn "sport = :${port}" 2>/dev/null | grep -q .; then
        echo "${port}"
        return 0
      fi
    done
    return 1
  }

  GAZEBO_PORT="$(pick_port 11345 11346 11347 11348 11349 11350)" || {
    echo "ERROR: no free Gazebo master port found in 11345-11350" >&2
    exit 1
  }
  export GAZEBO_MASTER_URI="http://127.0.0.1:${GAZEBO_PORT}"
fi

if [[ -z "${FASTRTPS_DEFAULT_PROFILES_FILE:-}" && -f "${WS_DIR}/config/fastdds_no_shm.xml" ]]; then
  export FASTRTPS_DEFAULT_PROFILES_FILE="${WS_DIR}/config/fastdds_no_shm.xml"
fi

echo "Using GAZEBO_MASTER_URI=${GAZEBO_MASTER_URI}"
if [[ -n "${FASTRTPS_DEFAULT_PROFILES_FILE:-}" ]]; then
  echo "Using FASTRTPS_DEFAULT_PROFILES_FILE=${FASTRTPS_DEFAULT_PROFILES_FILE}"
fi

# --------------------------------------------------------------------------- #
# CPU budget note (dual Go2W = ~50 processes):
#   gzserver alone uses 1+ cores.  gzclient + rviz add 1-2 more.
#   Two full CHAMP stacks (controller_manager @250Hz, RSP @200Hz,
#   reactive_nav @20Hz, EKFs, lidar, mapper, frontier, hybrid router)
#   easily saturate a 4-core machine.
#
#   Defaults below disable the GUI/RViz to keep headless runs stable.
#   Override from the command line:  ./run_cfpa2_go2w_gazebo.sh gui:=true rviz:=true
# --------------------------------------------------------------------------- #

DEFAULT_LAUNCH_ARGS=(
  # Core sim runtime toggles.
  "use_sim_time:=true"
  "gui:=${GO2W_GUI:-true}"
  "rviz:=${GO2W_RVIZ:-true}"
  "cleanup_stale:=false"
  "use_fast_lio:=${GO2W_USE_FAST_LIO:-false}"
  "pointcloud_noise_enabled:=${GO2W_POINTCLOUD_NOISE:-false}"
  "pointcloud_noise_mean:=${GO2W_POINTCLOUD_NOISE_MEAN:-0.0}"
  "pointcloud_noise_stddev:=${GO2W_POINTCLOUD_NOISE_STDDEV:-0.015}"
  "enable_frontier_aux:=false"

  # Shared map ingestion for coordinator (optional global map topic).
  "use_shared_map:=true"
  "shared_map_topic:=/disco_slam/global_map"
  "shared_map_wait_sec:=8.0"

  # Coordinator behavior (generic M-TARE knobs used by CFPA2 wrapper too).
  "mtare_algorithm_mode:=cfpa2"
  "mtare_goal_publish_rate:=2.0"
  "mtare_overlap_weight:=1.0"
  "mtare_communication_timeout_sec:=6.0"
  "mtare_prediction_horizon_sec:=4.0"
  "mtare_pursuit_weight:=2.0"
  "mtare_pursuit_switch_margin:=0.10"
  "switch_hysteresis:=0.08"
  "goal_lock_sec:=5.0"
  "mtare_exploration_gain_radius_cells:=4"
  "mtare_meeting_min_distance:=1.5"
  "mtare_teammate_stale_ttl_sec:=120.0"

  # CFPA2 frontier utility weights.
  "cfpa2_w_ig:=1.2"
  "cfpa2_w_c:=0.6"
  "cfpa2_w_sw:=0.2"
  "cfpa2_lambda_overlap:=6.0"
  "cfpa2_sigma_overlap_m:=1.5"

  # CFPA2 stuck recovery + close-range arbitration.
  "cfpa2_stuck_lock_sec:=8.0"
  "cfpa2_stuck_min_motion_m:=0.20"
  "cfpa2_stuck_blacklist_sec:=15.0"
  "cfpa2_close_stop_radius_m:=0.35"
  "cfpa2_close_stop_speed_epsilon:=0.02"

  # CFPA2 space-time A* handoff waypointing.
  "cfpa2_space_time_enabled:=true"
  "cfpa2_space_time_horizon_sec:=5.0"
  "cfpa2_space_time_dt_sec:=0.40"
  "cfpa2_space_time_safety_radius_m:=0.55"
  "cfpa2_space_time_waypoint_lookahead_m:=0.90"
  "cfpa2_space_time_window_margin_m:=3.0"
  "cfpa2_space_time_max_expansions:=12000"
  "cfpa2_space_time_assumed_speed_mps:=0.25"
  "cfpa2_space_time_max_speed_mps:=0.60"

  # CFPA2 frontier cluster gate (connected frontier area in m^2).
  "cfpa2_frontier_min_cluster_area_m2:=0.1"

  # Minimum clearance from occupied cells for a frontier cell to be a valid goal.
  # Must be >= planner_inflation_radius to ensure goals are always reachable.
  "cfpa2_frontier_obstacle_clearance_m:=0.40"

  # Spawn poses.
  "robot_a_spawn_x:=1.0"
  "robot_a_spawn_y:=0.0"
  "robot_a_spawn_yaw:=0"
  "robot_b_spawn_x:=2.0"
  "robot_b_spawn_y:=0.0"
  "robot_b_spawn_yaw:=3.14159"

  # Planner/backend selection.
  "planner_backend:=cfpa2"
  "require_shared_graph:=true"
  "exact_far_world_frame:=world"

  # Modular stack feature gates.
  "enable_assets:=true"
  "enable_perception:=true"
  "enable_slam:=true"
  "enable_control:=true"
  "enable_navigation:=true"

  # Point-LIO debug profile passthrough (kept for compatibility).
  "pointlio_autonomous:=false"
  "pointlio_spawn_x:=2.5"
  "pointlio_spawn_y:=0.0"
  "pointlio_spawn_z:=0.32"
  "pointlio_spawn_heading:=0.0"
)

exec ros2 launch go2_gazebo_sim dual_go2w_modular.launch.py \
  "profile:=mtare_ros2" \
  "${DEFAULT_LAUNCH_ARGS[@]}" \
  "$@"
