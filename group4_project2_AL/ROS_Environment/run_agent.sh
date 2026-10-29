#!/usr/bin/env bash
# The agent belongs on the computer connected to the ESP32, not both computers.
set -eo pipefail
folder="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
identity="${PROJECT2_ID_FILE:-$folder/../ID.yaml}"
[[ -f "$identity" ]] || identity="$folder/ID.yaml"
[[ -f "$identity" ]] || { echo 'Copy ID.example.yaml to ID.yaml in this folder and set the hardware domain.' >&2; exit 1; }
transport="${1:-usb}"
[[ $transport == usb || $transport == wifi ]] || { echo 'Usage: bash run_agent.sh usb|wifi'; exit 2; }
source /opt/ros/humble/setup.bash
export ROS_LOCALHOST_ONLY=0 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID="$(python3 -c 'import sys,yaml; print(int(yaml.safe_load(open(sys.argv[1]))["ros_domain_id"]))' "$identity")"
agent_ws="${PROJECT2_AGENT_WORKSPACE:-$HOME/ROS2/group4_microros_ws}"
agent="$agent_ws/install/micro_ros_agent/lib/micro_ros_agent/micro_ros_agent"
if [[ ! -x "$agent" ]]; then
    agent_ws="$HOME/ROS2/group4_project2_ws/.project2/agent_ws"
    agent="$agent_ws/install/micro_ros_agent/lib/micro_ros_agent/micro_ros_agent"
fi
[[ -x "$agent" ]] || { echo 'micro-ROS agent not found. Run bash install_agent.sh in ROS_Environment first.' >&2; exit 1; }
source "$agent_ws/install/local_setup.bash"
exec python3 "$folder/start_agent.py" --identity "$identity" --transport "$transport" --agent "$agent"
