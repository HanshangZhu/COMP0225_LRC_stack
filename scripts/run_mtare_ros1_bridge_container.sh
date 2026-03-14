#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"

COMMAND="start"
IMAGE="cmu-mtare-ros1-bridge:foxy-noetic"
NAME="mtare_ros1_bridge"
BUILD_POLICY="auto"
NO_CACHE="0"
BRIDGE_REV_LABEL_KEY="cmu.mtare.bridge_rev"
MTARE_SCENARIO="indoor_go2_bridge"
MTARE_ROS1_WS_PATH="${WORKSPACE_DIR}/src/mtare_ros1_ws"
FASTDDS_TRANSPORT="udp"
ROS_DOMAIN_ID_VALUE="${ROS_DOMAIN_ID:-0}"

print_usage() {
  cat <<'EOF'
Usage:
  ./run_mtare_ros1_bridge_container.sh [start|stop|logs] [options]

Commands:
  start   Build (if needed) and start ROS1+M-TARE+ros1_bridge container. (default)
  stop    Stop/remove the container.
  logs    Follow container logs.

Options:
  --workspace PATH
  --image IMAGE
  --name NAME
  --build-policy {auto|always|never}
  --no-cache
  --scenario NAME
  --mtare-ros1-ws-path PATH
  --fastdds-transport {udp|auto}
  --ros-domain-id ID
  -h, --help
EOF
}

to_lower() {
  echo "$1" | tr '[:upper:]' '[:lower:]'
}

normalize_build_policy() {
  local value
  value="$(to_lower "$1")"
  case "${value}" in
    auto|always|never) echo "${value}" ;;
    *)
      echo "Invalid build policy: $1 (use auto|always|never)" >&2
      exit 2
      ;;
  esac
}

normalize_fastdds_transport() {
  local value
  value="$(to_lower "$1")"
  case "${value}" in
    udp|auto) echo "${value}" ;;
    *)
      echo "Invalid fastdds transport: $1 (use udp|auto)" >&2
      exit 2
      ;;
  esac
}

compute_bridge_rev() {
  local dockerfile_path="${WORKSPACE_DIR}/hardware/docker/mtare_ros1_bridge/Dockerfile"
  local stack_script_path="${WORKSPACE_DIR}/hardware/docker/mtare_ros1_bridge/run_mtare_ros1_bridge_stack.sh"
  sha256sum "${dockerfile_path}" "${stack_script_path}" | sha256sum | awk '{print $1}'
}

if [[ $# -gt 0 ]]; then
  case "$1" in
    start|stop|logs)
      COMMAND="$1"
      shift
      ;;
  esac
fi

while [[ $# -gt 0 ]]; do
  case "$1" in
    --workspace)
      WORKSPACE_DIR="$2"
      shift 2
      ;;
    --image)
      IMAGE="$2"
      shift 2
      ;;
    --name)
      NAME="$2"
      shift 2
      ;;
    --build-policy)
      BUILD_POLICY="$(normalize_build_policy "$2")"
      shift 2
      ;;
    --no-cache)
      NO_CACHE="1"
      shift
      ;;
    --scenario)
      MTARE_SCENARIO="$2"
      shift 2
      ;;
    --mtare-ros1-ws-path)
      MTARE_ROS1_WS_PATH="$2"
      shift 2
      ;;
    --fastdds-transport)
      FASTDDS_TRANSPORT="$(normalize_fastdds_transport "$2")"
      shift 2
      ;;
    --ros-domain-id)
      ROS_DOMAIN_ID_VALUE="$2"
      shift 2
      ;;
    -h|--help)
      print_usage
      exit 0
      ;;
    *)
      echo "Unknown argument: $1" >&2
      print_usage
      exit 2
      ;;
  esac
done

if ! command -v docker >/dev/null 2>&1; then
  echo "docker is required but was not found on PATH." >&2
  exit 1
fi

if [[ "${COMMAND}" == "stop" ]]; then
  docker rm -f "${NAME}" >/dev/null 2>&1 || true
  echo "Stopped container '${NAME}' (if it was running)."
  exit 0
fi

if [[ "${COMMAND}" == "logs" ]]; then
  exec docker logs -f "${NAME}"
fi

if [[ ! -d "${MTARE_ROS1_WS_PATH}" ]]; then
  echo "M-TARE ROS1 workspace not found: ${MTARE_ROS1_WS_PATH}" >&2
  exit 1
fi

if [[ ! -f "${WORKSPACE_DIR}/hardware/docker/mtare_ros1_bridge/Dockerfile" ]]; then
  echo "Dockerfile not found at ${WORKSPACE_DIR}/hardware/docker/mtare_ros1_bridge/Dockerfile" >&2
  exit 1
fi
if [[ ! -f "${WORKSPACE_DIR}/hardware/docker/mtare_ros1_bridge/run_mtare_ros1_bridge_stack.sh" ]]; then
  echo "Container stack script not found at ${WORKSPACE_DIR}/hardware/docker/mtare_ros1_bridge/run_mtare_ros1_bridge_stack.sh" >&2
  exit 1
fi

LOCAL_BRIDGE_REV="$(compute_bridge_rev)"

need_build="0"
if [[ "${BUILD_POLICY}" == "always" ]]; then
  need_build="1"
elif [[ "${BUILD_POLICY}" == "auto" ]]; then
  if ! docker image inspect "${IMAGE}" >/dev/null 2>&1; then
    need_build="1"
  else
    image_bridge_rev="$(docker image inspect -f "{{ index .Config.Labels \"${BRIDGE_REV_LABEL_KEY}\" }}" "${IMAGE}" 2>/dev/null || true)"
    if [[ "${image_bridge_rev}" != "${LOCAL_BRIDGE_REV}" ]]; then
      need_build="1"
      echo "Image '${IMAGE}' is stale for current ROS1 bridge scripts; rebuilding."
    fi
  fi
elif [[ "${BUILD_POLICY}" == "never" ]]; then
  if ! docker image inspect "${IMAGE}" >/dev/null 2>&1; then
    echo "Container image '${IMAGE}' not found and --build-policy=never." >&2
    exit 1
  fi
fi

if [[ "${need_build}" == "1" ]]; then
  DOCKER_BUILD_ARGS=(
    build
    -t "${IMAGE}"
    --label "${BRIDGE_REV_LABEL_KEY}=${LOCAL_BRIDGE_REV}"
    -f "${WORKSPACE_DIR}/hardware/docker/mtare_ros1_bridge/Dockerfile"
  )
  if [[ "${NO_CACHE}" == "1" ]]; then
    DOCKER_BUILD_ARGS+=(--no-cache)
  fi
  DOCKER_BUILD_ARGS+=("${WORKSPACE_DIR}")
  echo "Building ROS1 bridge image '${IMAGE}'..."
  docker "${DOCKER_BUILD_ARGS[@]}"
fi

docker rm -f "${NAME}" >/dev/null 2>&1 || true

DOCKER_RUN_ARGS=(
  run
  --detach
  --network host
  --name "${NAME}"
  -e "MTARE_SCENARIO=${MTARE_SCENARIO}"
  -e "MTARE_ROS1_WS_PATH=/mtare_ros1_ws"
  -e "ROS_DOMAIN_ID=${ROS_DOMAIN_ID_VALUE}"
  -e "RMW_IMPLEMENTATION=rmw_fastrtps_cpp"
  -v "${MTARE_ROS1_WS_PATH}:/mtare_ros1_ws"
)

if [[ "${FASTDDS_TRANSPORT}" == "udp" ]]; then
  DOCKER_RUN_ARGS+=(-e "FASTDDS_BUILTIN_TRANSPORTS=UDPv4")
fi

DOCKER_RUN_ARGS+=("${IMAGE}")

container_id="$(docker "${DOCKER_RUN_ARGS[@]}")"
sleep 2
status="$(docker inspect -f '{{.State.Status}}' "${NAME}" 2>/dev/null || true)"
if [[ "${status}" != "running" ]]; then
  echo "ROS1 bridge container '${NAME}' failed to stay running (status='${status}')." >&2
  echo "---- container logs ----" >&2
  docker logs "${NAME}" >&2 || true
  echo "---- end logs ----" >&2
  exit 1
fi

echo "Started ROS1 bridge container '${NAME}' (${container_id})."
echo "Use './run_mtare_ros1_bridge_container.sh logs --name ${NAME}' to inspect logs."
