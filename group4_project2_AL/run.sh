#!/usr/bin/env bash
# Start with only ROS Humble and this build, even from an old ROS terminal.
set -e -o pipefail
PROJECT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
if [[ ${1:-} != --clean ]]; then
    exec bash "$PROJECT_DIR/tools/clean_env.sh" bash --noprofile --norc "$0" --clean "$@"
fi
shift
WORKSPACE_DIR="$(cd "$PROJECT_DIR/../.." && pwd)"
if [[ -f "$PROJECT_DIR/.project2_workspace" ]]; then
    WORKSPACE_DIR="$(cat "$PROJECT_DIR/.project2_workspace")"
fi
[[ -f "$WORKSPACE_DIR/install/local_setup.bash" ]] || { echo 'Run bash install_build.sh first.' >&2; exit 1; }
source /opt/ros/humble/setup.bash
source "$WORKSPACE_DIR/install/local_setup.bash"
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp ROS_LOCALHOST_ONLY=0
export ROS2CLI_NO_DAEMON=1
if [[ -f "$WORKSPACE_DIR/.project2-managed" ]]; then
    export PROJECT2_AGENT_WORKSPACE="$WORKSPACE_DIR/.project2/agent_ws"
    export PROJECT2_HOST_WORKSPACE="$WORKSPACE_DIR/.project2/host_ws"
fi
MODE="${1:-sim}"
REMOTE_AGENT=false
for option in "$@"; do
    [[ $option == agent:=false ]] && REMOTE_AGENT=true
done
CONFIG_MODE=sim
if [[ $MODE == hardware || $MODE == camera || $MODE == doctor || ( $MODE == shell && ${2:-} == hardware ) ]]; then CONFIG_MODE=hardware; fi
export PROJECT2_PROJECT_DIR="$PROJECT_DIR"
export PROJECT2_ID_FILE="$PROJECT_DIR/ID.yaml"
export ROS_DOMAIN_ID="$(python3 - "$PROJECT_DIR/ID.yaml" "$CONFIG_MODE" <<'PY'
import sys,yaml
settings=yaml.safe_load(open(sys.argv[1]))
if sys.argv[2]=='sim' and int(settings.get('simulation_ros_domain_id',42)) == int(settings['ros_domain_id']):
    raise SystemExit('Use different ros_domain_id and simulation_ros_domain_id in ID.yaml so simulated commands cannot reach the rig.')
print(int(settings['ros_domain_id'] if sys.argv[2]=='hardware' else settings.get('simulation_ros_domain_id',42)))
PY
)"
export IGN_PARTITION="project2_$(printf '%s' "$WORKSPACE_DIR" | sha256sum | cut -c1-12)_$ROS_DOMAIN_ID"
export GZ_PARTITION="$IGN_PARTITION"
cd "$PROJECT_DIR"
runner=()
if [[ $MODE == sim || $MODE == qa || $MODE == manual || $MODE == hardware ]]; then
    python3 "$PROJECT_DIR/tools/check_install.py" --workspace "$WORKSPACE_DIR"
    port="$(python3 - "$PROJECT_DIR/ID.yaml" "$MODE" <<'PY'
import sys, yaml
settings = yaml.safe_load(open(sys.argv[1]))
key = 'agent_port' if sys.argv[2] == 'hardware' else 'simulation_agent_port'
print(settings['network'][key] if sys.argv[2] != 'hardware' or settings['transport'] == 'wifi' else 0)
PY
)"
    [[ $MODE == hardware && $REMOTE_AGENT == true ]] && port=0
    runner=(python3 "$PROJECT_DIR/tools/session.py" --port "$port" --)
fi
case "$MODE" in
    sim) exec "${runner[@]}" ros2 launch group4_project2_al production.launch.py toys:="${2:-0}" "${@:3}" ;;
    qa) exec "${runner[@]}" ros2 launch group4_project2_al production.launch.py toys:=6 qa_trial:=true cad_camera:=true qa_seed:="${2:-4}" ;;
    manual) exec "${runner[@]}" ros2 launch group4_project2_al gazebo.launch.py control:=manual master_slave:=false assembly_scene:=true ;;
    hardware) if [[ $REMOTE_AGENT == false ]]; then python3 tools/check_connection.py; fi; exec "${runner[@]}" ros2 launch group4_project2_al hardware.launch.py "${@:2}" ;;
    doctor) exec python3 tools/check_connection.py ;;
    camera) exec ros2 run group4_project2_al camera_inspection.py ;;
    test) python3 -m pytest test/ -q; exec python3 tools/check_geometry.py --section "${2:-all}" ;;
    shell) echo "Clean ROS shell: $WORKSPACE_DIR | domain $ROS_DOMAIN_ID"; exec bash --noprofile --norc ;;
    *) echo 'Usage: bash run.sh sim [1-5] | qa [seed] | manual | hardware | camera | doctor | test | shell [hardware]'; exit 2 ;;
esac
