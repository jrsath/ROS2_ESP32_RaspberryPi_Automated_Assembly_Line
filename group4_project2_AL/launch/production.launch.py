# Full assembly, conveyor, camera and packaging run using the supplied rig.
from pathlib import Path
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    share = Path(get_package_share_directory("group4_project2_al"))
    defaults = {
        "toys": "0",
        "gui": "true",
        "rviz": "false",
        "micro_ros": "true",
        "qa_trial": "false",
        "cad_camera": "true",
        "qa_seed": "4",
        "panel": "true",
        "autostart": "false",
        "camera_window": "true",
        "production_config": str(
            Path(os.environ.get("PROJECT2_PROJECT_DIR", share))
            / "config/production.yaml"
        ),
    }
    arguments = {name: LaunchConfiguration(name) for name in defaults}
    arguments.update(
        production="true",
        assembly_scene="true",
        contact_assembly="false",
        control="external",
        master_slave="false",
        mode="full",
    )
    return LaunchDescription(
        [
            *[
                DeclareLaunchArgument(name, default_value=value)
                for name, value in defaults.items()
            ],
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(share / "launch/gazebo.launch.py")),
                launch_arguments=arguments.items(),
            ),
        ]
    )
