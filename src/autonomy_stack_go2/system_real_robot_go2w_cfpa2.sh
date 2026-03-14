#!/bin/bash

set -e

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
REPO_ROOT="$( cd "$SCRIPT_DIR/../.." &> /dev/null && pwd )"
SETUP_FILE="$REPO_ROOT/install/setup.bash"

if [[ ! -f "$SETUP_FILE" ]]; then
  echo "Workspace setup file not found: $SETUP_FILE" >&2
  echo "Build the workspace first, then rerun this script." >&2
  exit 1
fi

cd "$REPO_ROOT"
source "$SETUP_FILE"

# --- CycloneDDS over WiFi ---
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
WIFI_IFACE="${WIFI_IFACE:-$(ip -br link show type wifi | awk 'NR==1{print $1}')}"
if [[ -z "$WIFI_IFACE" ]]; then
  echo "WARNING: No WiFi interface found. DDS may use wrong NIC." >&2
else
  echo "CycloneDDS bound to WiFi interface: $WIFI_IFACE"
  export CYCLONEDDS_URI="<CycloneDDS><Domain><General><Interfaces>
    <NetworkInterface name=\"${WIFI_IFACE}\" priority=\"default\" multicast=\"default\" />
  </Interfaces></General></Domain></CycloneDDS>"
fi

exec ros2 launch go2_real_bringup single_go2w_real_cfpa2.launch.py "$@"
