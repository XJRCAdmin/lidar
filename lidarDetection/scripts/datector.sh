#!/bin/bash

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
WS_DIR="$( dirname "$SCRIPT_DIR" )"

source "$WS_DIR/install/setup.bash"

ros2 launch lidar_detection go2.launch.py