#!/bin/bash
# Wrapper to launch the real-robot autonomy stack with TARE waypoint wiring.
#
# Usage:
#   ./go2w_start_tare.sh
#   ./go2w_start_tare.sh scan false
#   ./go2w_start_tare.sh octomap true
#
# Args are forwarded to go2w_start_autonomy.sh:
#   $1 mapper type: scan|octomap|elevation
#   $2 obstacle avoidance: true|false

set -e
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"

export GO2W_NAV_LAUNCH_FILE="single_go2w_real_tare.launch.py"
exec "$SCRIPT_DIR/go2w_start_autonomy.sh" "$@"
