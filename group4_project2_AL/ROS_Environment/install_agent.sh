#!/usr/bin/env bash
# Install only the agent here; leave the lab's arm-driver workspace alone.
set -eo pipefail
source /opt/ros/humble/setup.bash
sudo apt-get update
sudo apt-get install -y git python3-colcon-common-extensions python3-rosdep python3-yaml
workspace="${PROJECT2_AGENT_WORKSPACE:-$HOME/ROS2/group4_microros_ws}"
mkdir -p "$workspace/src"
if [[ ! -d "$workspace/src/micro_ros_setup/.git" ]]; then
    git clone --branch humble https://github.com/micro-ROS/micro_ros_setup.git "$workspace/src/micro_ros_setup"
fi
cd "$workspace"
[[ -f /etc/ros/rosdep/sources.list.d/20-default.list ]] || sudo rosdep init
rosdep update
rosdep install --from-paths src --ignore-src -r -y --rosdistro humble
colcon build --packages-select micro_ros_setup
source install/local_setup.bash
if [[ ! -d src/micro_ros_agent ]]; then
    ros2 run micro_ros_setup create_agent_ws.sh
fi
ros2 run micro_ros_setup build_agent.sh
echo 'Agent built. Next: bash run_agent.sh usb (from the ROS_Environment folder).'
