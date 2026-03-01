#!/usr/bin/env bash
set -euo pipefail

WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
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

DEFAULT_LAUNCH_ARGS=(
  # Core sim runtime toggles.
  "use_sim_time:=true"
  "gui:=true"
  "rviz:=true"
  "cleanup_stale:=true"
  "use_fast_lio:=false"
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
  "cfpa2_w_ig:=1.1" # information gain()
  "cfpa2_w_c:=0.6"
  "cfpa2_w_sw:=0.2" 
  "cfpa2_lambda_overlap:=4.0"
  "cfpa2_sigma_overlap_m:=1.0"

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

  # Spawn poses.
  "robot_a_spawn_x:=1.0"
  "robot_a_spawn_y:=0.0"
  "robot_a_spawn_yaw:=0"
  "robot_b_spawn_x:=2.0" #18, alternatively
  "robot_b_spawn_y:=0.0" #0
  "robot_b_spawn_yaw:=3.14159" #3.14159

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

exec ros2 launch go2_gazebo_sim dual_go2_modular.launch.py \
  "profile:=mtare_ros2" \
  "${DEFAULT_LAUNCH_ARGS[@]}" \
  "$@"
