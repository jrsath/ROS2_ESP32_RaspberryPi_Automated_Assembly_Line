# Launch only software simulation; all topics are under /project2/sim.
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("mode", default_value="full"),
            DeclareLaunchArgument("cycles", default_value="5"),
            DeclareLaunchArgument("step_seconds", default_value="0.5"),
            DeclareLaunchArgument("autostart", default_value="false"),
            DeclareLaunchArgument("wait_motion", default_value="false"),
            DeclareLaunchArgument("wait_scene", default_value="false"),
            DeclareLaunchArgument("motion_timeout", default_value="60.0"),
            DeclareLaunchArgument("require_io", default_value="false"),
            DeclareLaunchArgument("manual_mode", default_value="false"),
            Node(
                package="group4_project2_al",
                executable="cell_sim.py",
                output="screen",
                parameters=[
                    {
                        "mode": LaunchConfiguration("mode"),
                        "require_io": ParameterValue(
                            LaunchConfiguration("require_io"), value_type=bool
                        ),
                        "manual_mode": ParameterValue(
                            LaunchConfiguration("manual_mode"), value_type=bool
                        ),
                        "wait_motion": ParameterValue(
                            LaunchConfiguration("wait_motion"), value_type=bool
                        ),
                        "wait_scene": ParameterValue(
                            LaunchConfiguration("wait_scene"), value_type=bool
                        ),
                        "motion_timeout": ParameterValue(
                            LaunchConfiguration("motion_timeout"), value_type=float
                        ),
                        "cycles": ParameterValue(
                            LaunchConfiguration("cycles"), value_type=int
                        ),
                        "step_seconds": ParameterValue(
                            LaunchConfiguration("step_seconds"), value_type=float
                        ),
                        "autostart": ParameterValue(
                            LaunchConfiguration("autostart"), value_type=bool
                        ),
                    }
                ],
            ),
        ]
    )
