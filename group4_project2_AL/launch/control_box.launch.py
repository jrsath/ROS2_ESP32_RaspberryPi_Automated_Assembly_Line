# Real micro-ROS transport, with a desktop board and virtual operator switches.
from pathlib import Path
import yaml
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_prefix, get_package_share_directory


def start_board(context):
    share = Path(get_package_share_directory("group4_project2_al"))
    data = yaml.safe_load((share / "ID.yaml").read_text())["network"]
    environment = {
        "PROJECT2_AGENT_IP": str(data["simulation_agent_ip"]),
        "PROJECT2_AGENT_PORT": str(data["simulation_agent_port"]),
    }
    host = Path(LaunchConfiguration("host_workspace").perform(context))
    if not (
        host / "install/project2_virtual_board/lib/project2_virtual_board/virtual_board"
    ).is_file():
        raise RuntimeError("Build the micro-ROS host first: see README.md")
    # Use the existing agent workspace without adding the host RMW to Gazebo.
    agent = Path(LaunchConfiguration("agent_workspace").perform(context))
    binary = agent / "install/micro_ros_agent/lib/micro_ros_agent/micro_ros_agent"
    if not binary.is_file():
        raise RuntimeError("micro-ROS agent not found in " + str(agent))
    wrapper = (
        Path(get_package_prefix("group4_project2_al"))
        / "lib/group4_project2_al/run_virtual_board.sh"
    )
    return [
        ExecuteProcess(
            cmd=["bash", str(wrapper.with_name("run_sim_agent.sh")), str(agent)],
            output="screen",
            additional_env=environment,
        ),
        ExecuteProcess(
            cmd=["bash", str(wrapper), str(host)],
            output="screen",
            respawn=True,
            respawn_delay=2.0,
            additional_env=environment,
        ),
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("panel", default_value="true"),
            DeclareLaunchArgument("cad_camera", default_value="false"),
            DeclareLaunchArgument(
                "host_workspace",
                default_value=os.environ.get(
                    "PROJECT2_HOST_WORKSPACE",
                    str(Path.home() / "ROS2/project2_microros_host_ws"),
                ),
            ),
            DeclareLaunchArgument(
                "agent_workspace",
                default_value=os.environ.get(
                    "PROJECT2_AGENT_WORKSPACE",
                    str(Path.home() / "ROS2/group4_microros_ws"),
                ),
            ),
            OpaqueFunction(function=start_board),
            Node(
                package="group4_project2_al",
                executable="control_panel.py",
                parameters=[
                    {
                        "cad_camera": ParameterValue(
                            LaunchConfiguration("cad_camera"), value_type=bool
                        )
                    }
                ],
                output="screen",
                condition=IfCondition(LaunchConfiguration("panel")),
                additional_env={"QT_QPA_PLATFORM": "xcb"},
            ),
        ]
    )
