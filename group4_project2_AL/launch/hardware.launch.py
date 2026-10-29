# Physical I/O and read-only arm display. The university driver owns the motors.
from pathlib import Path
import yaml
import os
import xacro
from launch import LaunchDescription
from launch.actions import ExecuteProcess, OpaqueFunction, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from group4_project2_al.connection import read_identity, agent_args
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def start(context):
    share = Path(get_package_share_directory("group4_project2_al"))
    rig = Path(get_package_share_directory("group4_assembly_line"))
    data = read_identity(share)
    nodes = [
        Node(package="group4_project2_al", executable="io_monitor.py", output="screen")
    ]
    if LaunchConfiguration("agent").perform(context).lower() == "true":
        # Run the agent only on the computer physically connected to the ESP32.
        agent = (
            Path(
                os.environ.get(
                    "PROJECT2_AGENT_WORKSPACE",
                    str(Path.home() / "ROS2/group4_microros_ws"),
                )
            )
            / "install/micro_ros_agent/lib/micro_ros_agent/micro_ros_agent"
        )
        if not agent.is_file():
            raise RuntimeError("Install the micro-ROS agent first; see README.md")
        import shlex

        command = (
            "source /opt/ros/humble/setup.bash\nsource "
            + shlex.quote(str(agent.parents[3] / "local_setup.bash"))
            + "\nexec "
            + shlex.join([str(agent), *agent_args(data)])
        )
        nodes.insert(0, ExecuteProcess(cmd=["bash", "-c", command], output="screen"))
    if LaunchConfiguration("io_only").perform(context).lower() == "true":
        return nodes
    nodes.extend(
        [
            Node(
                package="group4_project2_al",
                executable="hardware_monitor.py",
                output="screen",
            ),
            Node(
                package="group4_project2_al",
                executable="camera_inspection.py",
                output="screen",
            ),
            Node(
                package="group4_assembly_line",
                executable="rig_markers.py",
                parameters=[{"show_reference_boxes": False}],
            ),
        ]
    )
    layout = yaml.safe_load((rig / "config/rig_layout.yaml").read_text())
    for arm, base in layout["robots"].items():
        document = xacro.process_file(
            str(rig / "urdf/so101.urdf.xacro"),
            mappings={
                "robot_namespace": "/" + arm,
                "mesh_root": str(rig / "models/project2_rig/meshes"),
            },
        )
        nodes.append(
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                namespace=arm,
                parameters=[
                    {"robot_description": document.toxml(), "frame_prefix": arm + "/"}
                ],
                remappings=[("joint_states", "joint_state_broadcaster/joint_states")],
            )
        )
        nodes.append(
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name=arm + "_world",
                arguments=[
                    *map(str, base["xyz"]),
                    *map(str, [base["rpy"][2], base["rpy"][1], base["rpy"][0]]),
                    "world",
                    arm + "/world",
                ],
            )
        )
    nodes.append(
        Node(
            package="rviz2",
            executable="rviz2",
            arguments=["-d", str(share / "config/gazebo.rviz")],
        )
    )
    return nodes


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("agent", default_value="true"),
            DeclareLaunchArgument("io_only", default_value="false"),
            OpaqueFunction(function=start),
        ]
    )
