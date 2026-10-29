#!/usr/bin/env bash
set -e
source /opt/ros/humble/setup.bash
source "$1/install/local_setup.bash"
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
exec "$1/install/micro_ros_agent/lib/micro_ros_agent/micro_ros_agent" udp4 --port "${PROJECT2_AGENT_PORT:-8889}" -v 3
