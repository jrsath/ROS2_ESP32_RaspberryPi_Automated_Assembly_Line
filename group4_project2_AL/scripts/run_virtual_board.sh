#!/usr/bin/env bash
# Keep the micro-ROS RMW confined to this client process.
set -e
source /opt/ros/humble/setup.bash
source "$1/install/local_setup.bash"
export RMW_IMPLEMENTATION=rmw_microxrcedds
exec "$1/install/project2_virtual_board/lib/project2_virtual_board/virtual_board"
