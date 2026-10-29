#!/usr/bin/env python3
# Build a local controller fix and a separate desktop micro-ROS client workspace.
import json
import os
from pathlib import Path
import shlex
import shutil
import subprocess

ROOT = Path(__file__).resolve().parents[1]
# A fresh Git checkout starts with the shared settings template.
if not (ROOT / "ID.yaml").exists():
    shutil.copy2(ROOT / "ID.example.yaml", ROOT / "ID.yaml")
WS = Path(os.environ.get("PROJECT2_WORKSPACE", ROOT.parents[1]))
RUNTIME = Path(os.environ.get("PROJECT2_RUNTIME_ROOT", Path.home() / "ROS2"))
HOST = RUNTIME / (
    "host_ws" if "PROJECT2_RUNTIME_ROOT" in os.environ else "project2_microros_host_ws"
)


def run(command, cwd=None):
    subprocess.run(command, cwd=cwd, check=True)


def ros_build(arguments, cwd, overlay=False):
    script = "source /opt/ros/humble/setup.bash\n"
    if overlay:
        script += "source install/local_setup.bash\n"
    script += shlex.join(["colcon", "build", "--base-paths", "src", *arguments])
    run(["bash", "--noprofile", "--norc", "-c", script], cwd)


def clone(url, branch, destination):
    if not destination.exists():
        destination.parent.mkdir(parents=True, exist_ok=True)
        run(["git", "clone", "--depth", "1", "--branch", branch, url, str(destination)])


controller = RUNTIME / "project2_vendor/ros2_controllers"
clone("https://github.com/ros-controls/ros2_controllers.git", "2.54.0", controller)
source = controller / "joint_trajectory_controller/src/joint_trajectory_controller.cpp"
old = "static std::vector<double> output(dof_, 0.0);"
new = "// Gazebo updates different controller instances concurrently.\n    std::vector<double> output(dof_, 0.0);"
text = source.read_text()
if old in text:
    source.write_text(text.replace(old, new))
elif new not in text:
    raise RuntimeError(
        "Unexpected controller source; inspect before applying the patch"
    )
link = WS / "src/joint_trajectory_controller"
if not link.exists():
    link.symlink_to(
        controller / "joint_trajectory_controller", target_is_directory=True
    )
elif link.resolve() != (controller / "joint_trajectory_controller").resolve():
    raise RuntimeError("An existing controller package occupies " + str(link))
ros_build(
    [
        "--packages-select",
        "joint_trajectory_controller",
        "--allow-overriding",
        "joint_trajectory_controller",
        "--symlink-install",
        "--parallel-workers",
        "2",
        "--cmake-args",
        "-DBUILD_TESTING=OFF",
    ],
    WS,
)

repos = {
    "eProsima/Micro-CDR": ("eProsima/Micro-CDR", "ros2"),
    "eProsima/Micro-XRCE-DDS-Client": ("eProsima/Micro-XRCE-DDS-Client", "ros2"),
    "uros/rclc": ("ros2/rclc", "humble"),
    "uros/rmw_microxrcedds": ("micro-ROS/rmw-microxrcedds", "humble"),
    "uros/micro_ros_msgs": ("micro-ROS/micro_ros_msgs", "humble"),
    "uros/rosidl_typesupport_microxrcedds": (
        "micro-ROS/rosidl_typesupport_microxrcedds",
        "humble",
    ),
    "ros2/common_interfaces": ("ros2/common_interfaces", "humble"),
    "ros2/rcl_interfaces": ("ros2/rcl_interfaces", "humble"),
}
for name, (repo, branch) in repos.items():
    clone("https://github.com/" + repo + ".git", branch, HOST / "src" / name)
meta = {
    "names": {
        "microcdr": {"cmake-args": ["-DBUILD_SHARED_LIBS=ON"]},
        "microxrcedds_client": {
            "cmake-args": ["-DBUILD_SHARED_LIBS=ON", "-DUCLIENT_PROFILE_MULTITHREAD=ON"]
        },
        "rmw_microxrcedds": {
            "cmake-args": [
                "-DRMW_UXRCE_TRANSPORT=udp",
                "-DRMW_UXRCE_MAX_PUBLISHERS=15",
                "-DRMW_UXRCE_MAX_SUBSCRIPTIONS=15",
                "-DRMW_UXRCE_STREAM_HISTORY=32",
                "-DRMW_UXRCE_MAX_HISTORY=10",
            ]
        },
    }
}
(HOST / "src/colcon.meta").write_text(json.dumps(meta, indent=2) + "\n")
# rclc test-only messages are unnecessary when BUILD_TESTING is off.
common = [
    "--packages-ignore",
    "test_msgs",
    "--parallel-workers",
    "2",
    "--cmake-args",
    "-DBUILD_TESTING=OFF",
    "-DBUILD_SHARED_LIBS=ON",
]
ros_build(
    [
        "--packages-up-to",
        "rosidl_typesupport_microxrcedds_c",
        "rosidl_typesupport_microxrcedds_cpp",
        *common,
    ],
    HOST,
)
# Build the full client dependency graph, including messages used by rclc.
# Type support must already exist when CMake generates the message libraries.
ros_build(
    [
        "--packages-up-to",
        "rclc",
        "rmw_microxrcedds",
        "std_msgs",
        "micro_ros_msgs",
        "--allow-overriding",
        "builtin_interfaces",
        "std_msgs",
        "action_msgs",
        "test_msgs",
        *common,
    ],
    HOST,
    True,
)
app = HOST / "src/project2_virtual_board"
app.mkdir(exist_ok=True)
for filename in ("CMakeLists.txt", "package.xml", "main.c"):
    shutil.copy2(ROOT / "microros_host" / filename, app / filename)
ros_build(["--packages-select", "project2_virtual_board", *common], HOST, True)
ros_build(
    [
        "--packages-select",
        "group4_assembly_line",
        "group4_project2_al",
        "--symlink-install",
    ],
    WS,
    True,
)
print(f"Simulation and micro-ROS host built in {WS}.")
