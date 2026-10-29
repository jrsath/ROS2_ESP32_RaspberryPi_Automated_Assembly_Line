#!/usr/bin/env bash
# Install dependencies and build in a separate workspace without removing old workspaces.
set -e -o pipefail
PROJECT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
if [[ ${1:-} != --clean ]]; then
    exec bash "$PROJECT_DIR/tools/clean_env.sh" bash --noprofile --norc "$0" --clean "$@"
fi
shift
WORKSPACE="$HOME/ROS2/group4_project2_ws"
SYSTEM=1
EXPLICIT_WORKSPACE=0
while (( $# )); do
    case "$1" in
        --workspace) WORKSPACE="$2"; EXPLICIT_WORKSPACE=1; shift 2 ;;
        --skip-system) SYSTEM=0; shift ;;
        --help) echo 'Usage: bash install_build.sh [--workspace /absolute/path] [--skip-system]'; exit 0 ;;
        *) echo "Unknown option: $1" >&2; exit 2 ;;
    esac
done
source /etc/os-release
[[ $ID == ubuntu && $VERSION_ID == 22.04 ]] || { echo 'Use Ubuntu 22.04 (native or WSL2).' >&2; exit 1; }
[[ $WORKSPACE == /* ]] || { echo 'Workspace must be an absolute path.' >&2; exit 1; }
WORKSPACE="$(realpath -m "$WORKSPACE")"
RIG_DIR="$PROJECT_DIR/vendor/group4_assembly_line"
[[ -f "$RIG_DIR/package.xml" ]] || { echo 'Missing bundled robot package under vendor/group4_assembly_line.' >&2; exit 1; }
# A different extracted copy gets its own build instead of reusing stale source links.
if (( ! EXPLICIT_WORKSPACE )); then
    if [[ -f "$PROJECT_DIR/.project2_workspace" ]]; then
        recorded="$(cat "$PROJECT_DIR/.project2_workspace")"
        if [[ -f "$recorded/.project2-managed" && $(readlink -f "$recorded/src/group4_project2_AL") == "$PROJECT_DIR" ]]; then
            WORKSPACE="$recorded"
        fi
    fi
    if [[ -d "$WORKSPACE" && -n $(ls -A "$WORKSPACE") && $(readlink -f "$WORKSPACE/src/group4_project2_AL") != "$PROJECT_DIR" ]]; then
        suffix="$(printf '%s' "$PROJECT_DIR" | sha256sum | cut -c1-10)"
        WORKSPACE="$HOME/ROS2/group4_project2_ws_$suffix"
    fi
fi
# Refuse to build into an unrelated existing directory; nothing is deleted or renamed.
if [[ -d "$WORKSPACE" && ! -f "$WORKSPACE/.project2-managed" && -n $(ls -A "$WORKSPACE") ]]; then
    echo "Existing workspace left untouched: $WORKSPACE. Choose another --workspace path." >&2
    exit 1
fi
mkdir -p "$WORKSPACE/src" "$WORKSPACE/.project2"
printf '%s\n' 'Group 4 Project 2 isolated build' > "$WORKSPACE/.project2-managed"
for name in group4_project2_AL group4_assembly_line; do
    source_dir="$PROJECT_DIR"; [[ $name == group4_assembly_line ]] && source_dir="$RIG_DIR"
    target="$WORKSPACE/src/$name"
    if [[ -e $target || -L $target ]]; then
        [[ $(readlink -f "$target") == $(readlink -f "$source_dir") ]] || { echo "Conflicting path left untouched: $target" >&2; exit 1; }
    else
        ln -s "$source_dir" "$target"
    fi
done
exec > >(tee -a "$WORKSPACE/.project2/install_build.log") 2>&1
if (( SYSTEM )); then
    sudo -v
    sudo apt-get update
    sudo apt-get --no-remove install -y locales software-properties-common curl ca-certificates git build-essential cmake python3
    sudo locale-gen en_US.UTF-8
    sudo add-apt-repository universe -y
    sudo apt-get --no-remove install -y systemd udev
    if [[ ! -f /opt/ros/humble/setup.bash ]]; then
        # The ROS repository package manages its signing key and apt source.
        version=$(curl -fsSL --retry 3 https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | python3 -c 'import json,sys; print(json.load(sys.stdin)["tag_name"])')
        curl -fL --retry 3 -o "$WORKSPACE/.project2/ros2-apt-source.deb" "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${version}/ros2-apt-source_${version}.jammy_all.deb"
        sudo dpkg -i "$WORKSPACE/.project2/ros2-apt-source.deb"
        sudo apt-get update
    fi
    sudo apt-get --no-remove install -y ros-humble-desktop ros-dev-tools python3-colcon-common-extensions python3-rosdep python3-vcstool python3-pip python3-yaml python3-scipy python3-opencv python3-pyqt5 python3-pytest ros-humble-ros-gz ros-humble-gz-ros2-control ros-humble-ros2-controllers ros-humble-interactive-markers ros-humble-cv-bridge ros-humble-ament-cmake-pytest
fi
[[ -f /opt/ros/humble/setup.bash ]] || { echo 'ROS Humble is missing; rerun without --skip-system.' >&2; exit 1; }
python3 "$PROJECT_DIR/tools/check_install.py" --repair
source /opt/ros/humble/setup.bash
export PROJECT2_WORKSPACE="$WORKSPACE" PROJECT2_RUNTIME_ROOT="$WORKSPACE/.project2"
# Keep downloads, the agent and the desktop micro-ROS client private to this build.
AGENT="$PROJECT2_RUNTIME_ROOT/agent_ws"
mkdir -p "$AGENT/src"
if [[ ! -d "$AGENT/src/micro_ros_setup" ]]; then
    git clone --depth 1 -b humble https://github.com/micro-ROS/micro_ros_setup.git "$AGENT/src/micro_ros_setup"
fi
if (( SYSTEM )); then
    if [[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then sudo rosdep init; fi
    rosdep update
    rosdep install --from-paths "$PROJECT_DIR" "$RIG_DIR" "$AGENT/src" --ignore-src -y
fi
(
    cd "$AGENT"
    colcon build --symlink-install --packages-select micro_ros_setup --parallel-workers 2
    source install/local_setup.bash
    if [[ ! -d src/uros/micro-ROS-Agent ]]; then
        ros2 run micro_ros_setup create_agent_ws.sh
    fi
    ros2 run micro_ros_setup build_agent.sh --parallel-workers 2
)
python3 "$PROJECT_DIR/tools/setup_simulation.py"
python3 "$PROJECT_DIR/tools/apply_id.py"
(
    source "$WORKSPACE/install/local_setup.bash"
    python3 "$PROJECT_DIR/tools/check_install.py" --workspace "$WORKSPACE"
)
# Record this build only after every build step succeeds. run.sh reads this file.
printf '%s\n' "$WORKSPACE" > "$PROJECT_DIR/.project2_workspace"
echo "Ready. Old workspaces and shell startup files were not changed."
echo "Run: bash $PROJECT_DIR/run.sh sim 1"
