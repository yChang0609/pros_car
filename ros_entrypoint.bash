#!/bin/bash
set -e
# setup ros2 environment
source /.bashrc
source "$ROS2_WS/install/setup.bash"
exec "$@"