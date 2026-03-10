#!/usr/bin/env bash
set -eo pipefail

WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ROS2_SETUP_BASH="${ROS2_SETUP_BASH:-/opt/ros/humble/setup.bash}"

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
  conda activate "${CONDA_ENV:-cmu_env}"
elif command -v micromamba >/dev/null 2>&1; then
  eval "$(micromamba shell hook -s bash)"
  micromamba activate "${CONDA_ENV:-cmu_env}"
else
  echo "Could not find conda/micromamba activation script for ${CONDA_ENV:-cmu_env}." >&2
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

if [[ "${CLEANUP_STALE_GAZEBO:-1}" == "1" ]]; then
  pkill -f '[g]zserver' 2>/dev/null || true
  pkill -f '(^|/)gzclient( |$)' 2>/dev/null || true
  pkill -f '(^|/)gazebo( |$)' 2>/dev/null || true
  sleep 1
fi

if [[ -z "${GAZEBO_MASTER_URI:-}" ]]; then
  pick_port() {
    local p
    for p in "$@"; do
      if ! ss -H -ltn "sport = :${p}" 2>/dev/null | grep -q .; then
        echo "${p}"
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

echo "=== M-TARE Baseline ==="
echo "Using GAZEBO_MASTER_URI=${GAZEBO_MASTER_URI}"
if [[ -n "${FASTRTPS_DEFAULT_PROFILES_FILE:-}" ]]; then
  echo "Using FASTRTPS_DEFAULT_PROFILES_FILE=${FASTRTPS_DEFAULT_PROFILES_FILE}"
fi

exec ros2 launch go2_gazebo_sim dual_go2_modular.launch.py \
  profile:=mtare_ros2 \
  planner_backend:=mtare_ros2 \
  mtare_algorithm_mode:=mtare \
  cleanup_stale:=false \
  gui:=false \
  rviz:=true \
  use_shared_map:=true \
  shared_map_topic:=/disco_slam/global_map \
  shared_map_wait_sec:=20.0 \
  "$@"
