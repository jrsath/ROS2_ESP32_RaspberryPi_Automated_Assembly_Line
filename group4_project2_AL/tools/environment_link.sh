#!/usr/bin/env bash
# Use the lab/laptop hardware domain without inheriting the simulator's overlay.
set -eo pipefail
project_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
if [[ ${1:-} != --clean ]]; then
    exec bash "$project_dir/tools/clean_env.sh" bash "$0" --clean "$@"
fi
shift
source /opt/ros/humble/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp ROS_LOCALHOST_ONLY=0
exec python3 -u "$project_dir/scripts/environment_link.py" --identity "$project_dir/ID.yaml"
