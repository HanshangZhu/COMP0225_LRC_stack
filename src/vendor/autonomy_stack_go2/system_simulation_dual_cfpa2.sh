#!/bin/bash

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"

cd "$SCRIPT_DIR"
source ./install/setup.bash
ros2 launch vehicle_simulator two_go2_t_world_cfpa2.launch.py "$@"
