rm -rf ../devel
rm -rf ../build

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
WS_DIR="$( dirname "$SCRIPT_DIR" )"

"$WS_DIR/src/livox_ros_driver2/build.sh" ROS2
