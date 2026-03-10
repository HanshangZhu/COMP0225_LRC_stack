#!/usr/bin/env bash
set -euo pipefail

WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROS2_SETUP_BASH="${ROS2_SETUP_BASH:-/opt/ros/humble/setup.bash}"

# --------------------------------------------------------------------------- #
# Environment activation
# --------------------------------------------------------------------------- #
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

# --------------------------------------------------------------------------- #
# Kill stale processes from previous runs
# --------------------------------------------------------------------------- #
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

CLEANUP_PATTERNS=(
  'ros2 launch go2_gazebo_sim single_go2w_gazebo_cfpa2.launch.py'
  '[g]zserver'
  '(^|/)gzclient( |$)'
  '(^|/)gazebo( |$)'
)
for pattern in "${CLEANUP_PATTERNS[@]}"; do
  terminate_matching_processes "$pattern" TERM
done
sleep 1
for pattern in "${CLEANUP_PATTERNS[@]}"; do
  terminate_matching_processes "$pattern" KILL
done
sleep 0.5

# --------------------------------------------------------------------------- #
# Pick a free Gazebo master port
# --------------------------------------------------------------------------- #
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

if [[ -z "${FASTRTPS_DEFAULT_PROFILES_FILE:-}" && -f "${WS_DIR}/fastdds_no_shm.xml" ]]; then
  export FASTRTPS_DEFAULT_PROFILES_FILE="${WS_DIR}/fastdds_no_shm.xml"
fi

echo "Using GAZEBO_MASTER_URI=${GAZEBO_MASTER_URI}"
if [[ -n "${FASTRTPS_DEFAULT_PROFILES_FILE:-}" ]]; then
  echo "Using FASTRTPS_DEFAULT_PROFILES_FILE=${FASTRTPS_DEFAULT_PROFILES_FILE}"
fi

# --------------------------------------------------------------------------- #
# Launch single Go2W Gazebo with CFPA2
# --------------------------------------------------------------------------- #
# CFPA2 utility weights — tune here to control frontier selection:
#   cfpa2_w_ig       : info-gain weight   (lower → less pull from far "big" frontiers)
#   cfpa2_w_c        : distance-cost weight (higher → strongly prefer nearby frontiers)
#   cfpa2_w_momentum : heading momentum    (higher → avoid turning back)
#   cfpa2_min_utility: stop threshold      (best utility below this → robot holds position)
exec ros2 launch go2_gazebo_sim single_go2w_gazebo_cfpa2.launch.py \
  gui:=true \
  rviz:=true \
  cleanup_stale:=false \
  cfpa2_w_ig:=0.5 \
  cfpa2_w_c:=0.8 \
  cfpa2_w_momentum:=2.5 \
  cfpa2_min_utility:=-1.0 \
  "$@"
