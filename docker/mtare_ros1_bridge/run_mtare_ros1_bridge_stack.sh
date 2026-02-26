#!/usr/bin/env bash
set -euo pipefail

ROS1_SETUP="${ROS1_SETUP:-/opt/ros/noetic/setup.bash}"
ROS2_SETUP="${ROS2_SETUP:-/opt/ros/foxy/setup.bash}"
MTARE_ROS1_WS_PATH="${MTARE_ROS1_WS_PATH:-/mtare_ros1_ws}"
MTARE_SCENARIO="${MTARE_SCENARIO:-indoor_go2_bridge}"
MTARE_AUTO_BUILD_ROS1_WS="${MTARE_AUTO_BUILD_ROS1_WS:-1}"
MTARE_RESTART_DELAY_SEC="${MTARE_RESTART_DELAY_SEC:-8}"

if [[ ! -f "${ROS1_SETUP}" ]]; then
  echo "ROS1 setup file not found: ${ROS1_SETUP}" >&2
  exit 1
fi
if [[ ! -f "${ROS2_SETUP}" ]]; then
  echo "ROS2 setup file not found: ${ROS2_SETUP}" >&2
  exit 1
fi
if [[ ! -d "${MTARE_ROS1_WS_PATH}" ]]; then
  echo "M-TARE workspace not found: ${MTARE_ROS1_WS_PATH}" >&2
  exit 1
fi

cleanup() {
  local pids=()
  [[ -n "${BRIDGE_PID:-}" ]] && pids+=("${BRIDGE_PID}")
  [[ -n "${MTARE_SUPERVISOR_PID:-}" ]] && pids+=("${MTARE_SUPERVISOR_PID}")
  [[ -n "${ROSCORE_PID:-}" ]] && pids+=("${ROSCORE_PID}")
  if [[ ${#pids[@]} -gt 0 ]]; then
    kill "${pids[@]}" >/dev/null 2>&1 || true
    wait "${pids[@]}" >/dev/null 2>&1 || true
  fi
}
trap cleanup EXIT INT TERM

start_mtare_supervisor() {
  (
    set +e
    local launch_pid=""

    cleanup_child() {
      if [[ -n "${launch_pid}" ]]; then
        kill "${launch_pid}" >/dev/null 2>&1 || true
        wait "${launch_pid}" >/dev/null 2>&1 || true
      fi
      exit 0
    }
    trap cleanup_child INT TERM

    while true; do
      echo "Starting ROS1 M-TARE launch (scenario=${MTARE_SCENARIO})..."
      roslaunch --wait tare_planner explore_dual_go2_bridge.launch scenario:="${MTARE_SCENARIO}" robot_num:=2 rviz:=false &
      launch_pid=$!
      wait "${launch_pid}"
      exit_code=$?
      launch_pid=""
      echo "ROS1 M-TARE launch exited (code=${exit_code}); restarting in ${MTARE_RESTART_DELAY_SEC}s..." >&2
      sleep "${MTARE_RESTART_DELAY_SEC}"
    done
  ) &
  MTARE_SUPERVISOR_PID=$!
}

set +u
# shellcheck disable=SC1090
source "${ROS1_SETUP}"
set -u

if [[ ! -f "${MTARE_ROS1_WS_PATH}/devel/setup.bash" ]]; then
  if [[ "${MTARE_AUTO_BUILD_ROS1_WS}" == "1" ]]; then
    echo "ROS1 workspace is not built; running catkin_make in ${MTARE_ROS1_WS_PATH}"
    (
      cd "${MTARE_ROS1_WS_PATH}"
      catkin_make
    )
  fi
fi

if [[ ! -f "${MTARE_ROS1_WS_PATH}/devel/setup.bash" ]]; then
  echo "Missing ${MTARE_ROS1_WS_PATH}/devel/setup.bash after build attempt." >&2
  exit 1
fi

echo "Starting roscore..."
roscore &
ROSCORE_PID=$!

for _ in $(seq 1 40); do
  if rosparam list >/dev/null 2>&1; then
    break
  fi
  sleep 0.5
done
if ! rosparam list >/dev/null 2>&1; then
  echo "roscore did not become ready in time." >&2
  exit 1
fi

set +u
# shellcheck disable=SC1090
source "${ROS1_SETUP}"
# shellcheck disable=SC1090
source "${MTARE_ROS1_WS_PATH}/devel/setup.bash"
set -u

start_mtare_supervisor

set +u
# shellcheck disable=SC1090
source "${ROS1_SETUP}"
# shellcheck disable=SC1090
source "${ROS2_SETUP}"
# shellcheck disable=SC1090
source "${MTARE_ROS1_WS_PATH}/devel/setup.bash"
set -u

echo "Starting ros1_bridge dynamic_bridge..."
ros2 run ros1_bridge dynamic_bridge --bridge-all-1to2-topics --bridge-all-2to1-topics &
BRIDGE_PID=$!

wait -n "${ROSCORE_PID}" "${MTARE_SUPERVISOR_PID}" "${BRIDGE_PID}"
exit_code=$?
echo "One of ROS1 runtime processes exited (code=${exit_code})." >&2
exit "${exit_code}"
