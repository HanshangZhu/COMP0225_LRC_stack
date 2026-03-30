#!/usr/bin/env bash
# A/B: FAR VLM launch with Cartographer vs Fast-LIO only (same world, same nav).
# Measures realtime factor (sim_s / wall_s) via tools/measure_sim_speed.py.
#
# Usage:
#   ./scripts/benchmark_slam_overhead.sh
#   GUI=false RVIZ=false WARMUP_SEC=90 SAMPLE_SEC=45 ./scripts/benchmark_slam_overhead.sh
#
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
  conda activate cmu_env 2>/dev/null || true
elif command -v micromamba >/dev/null 2>&1; then
  eval "$(micromamba shell hook -s bash)"
  micromamba activate cmu_env 2>/dev/null || true
fi

safe_source "${ROS2_SETUP_BASH}"
safe_source "${WS_DIR}/install/setup.bash"

GUI="${GUI:-true}"
RVIZ="${RVIZ:-false}"
WARMUP_SEC="${WARMUP_SEC:-90}"
SAMPLE_SEC="${SAMPLE_SEC:-45}"
# Disable VLM API work; delay VLM nodes effectively forever
VLM_DELAY="${VLM_DELAY:-9999}"

if [[ "${GUI}" == "true" && -z "${DISPLAY:-}" ]]; then
  echo "WARN: DISPLAY unset; forcing GUI=false" >&2
  GUI=false
fi

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

cleanup_stack() {
  local signal="${1:-TERM}"
  local -a patterns=(
    'ros2 launch vlm_explorer single_vlm_gazebo_far.launch.py'
    'ros2 launch go2_gazebo_sim single_go2w_gazebo_cfpa2.launch.py'
    '/go2_gazebo_sim/lib/go2_gazebo_sim/waypoint_mux.py'
    '/vlm_explorer/lib/vlm_explorer/'
    'carto_odom_bridge.py'
    'cartographer_node'
    'fastlio_mapping'
    'fast_lio/'
    '[g]zserver'
    '(^|/)gzclient( |$)'
  )
  local pattern
  for pattern in "${patterns[@]}"; do
    terminate_matching_processes "$pattern" "$signal"
  done
}

sample_cpu_hot() {
  # One-line snapshot: gz + cartographer + fastlio-related
  ps -eo pid,pcpu,comm,args --sort=-pcpu 2>/dev/null \
    | head -n 15 || true
}

run_trial() {
  local slam_source="$1"
  echo ""
  echo "========== TRIAL: slam_source=${slam_source} =========="
  cleanup_stack TERM
  sleep 1
  cleanup_stack KILL
  sleep 1

  ros2 launch vlm_explorer single_vlm_gazebo_far.launch.py \
    slam_source:="${slam_source}" \
    gui:=${GUI} \
    rviz:=${RVIZ} \
    vlm_enabled:=false \
    vlm_delay:=${VLM_DELAY} \
    use_sim_time:=true \
    >"${WS_DIR}/log/benchmark_${slam_source}_launch.log" 2>&1 &
  local launch_pid=$!

  echo "Launch PID=${launch_pid}; warming up ${WARMUP_SEC}s..."
  sleep "${WARMUP_SEC}"

  echo "--- top CPU (snapshot) ---"
  sample_cpu_hot

  echo "--- /clock sample (${SAMPLE_SEC}s wall) ---"
  python3 "${WS_DIR}/tools/measure_sim_speed.py" --duration "${SAMPLE_SEC}" \
    || true

  echo "Stopping stack..."
  kill "${launch_pid}" 2>/dev/null || true
  cleanup_stack TERM
  sleep 2
  cleanup_stack KILL
  sleep 1
}

mkdir -p "${WS_DIR}/log"

echo "benchmark_slam_overhead: GUI=${GUI} RVIZ=${RVIZ} WARMUP_SEC=${WARMUP_SEC} SAMPLE_SEC=${SAMPLE_SEC}"
echo "Higher realtime_factor => sim advances faster vs wall (less overhead)."

run_trial cartographer
run_trial fast_lio

echo ""
echo "Done. Full launch logs: ${WS_DIR}/log/benchmark_*_launch.log"
