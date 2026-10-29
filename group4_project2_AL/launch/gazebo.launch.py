# The supplied CAD scene and its real Gazebo ros2_control controllers.
from pathlib import Path
from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory, get_package_prefix


def generate_launch_description():
    share = Path(get_package_share_directory("group4_project2_al"))
    rig = Path(get_package_share_directory("group4_assembly_line"))
    process = IfCondition(
        PythonExpression(["'", LaunchConfiguration("control"), "' == 'process'"])
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument("table_height", default_value="1.0"),
            DeclareLaunchArgument("rviz", default_value="true"),
            DeclareLaunchArgument("gui", default_value="true"),
            DeclareLaunchArgument("software_rendering", default_value="true"),
            DeclareLaunchArgument(
                "control",
                default_value="manual",
                choices=["manual", "process", "external"],
            ),
            DeclareLaunchArgument("master_slave", default_value="true"),
            DeclareLaunchArgument("mode", default_value="full"),
            DeclareLaunchArgument("cycles", default_value="5"),
            DeclareLaunchArgument("autostart", default_value="false"),
            DeclareLaunchArgument("micro_ros", default_value="true"),
            DeclareLaunchArgument("panel", default_value="true"),
            DeclareLaunchArgument(
                "waypoints", default_value=str(share / "config/task_waypoints.yaml")
            ),
            DeclareLaunchArgument("assembly_scene", default_value="false"),
            DeclareLaunchArgument("contact_assembly", default_value="false"),
            DeclareLaunchArgument("toys", default_value="1"),
            DeclareLaunchArgument("qa_trial", default_value="false"),
            DeclareLaunchArgument("cad_camera", default_value="false"),
            DeclareLaunchArgument("qa_seed", default_value="4"),
            DeclareLaunchArgument("production", default_value="false"),
            DeclareLaunchArgument("camera_window", default_value="true"),
            DeclareLaunchArgument(
                "production_config", default_value=str(share / "config/production.yaml")
            ),
            SetEnvironmentVariable("QT_QPA_PLATFORM", "xcb"),
            SetEnvironmentVariable(
                "LIBGL_ALWAYS_SOFTWARE",
                "1",
                condition=IfCondition(LaunchConfiguration("software_rendering")),
            ),
            AppendEnvironmentVariable(
                "IGN_GAZEBO_RESOURCE_PATH", str(share / "models")
            ),
            AppendEnvironmentVariable("GZ_SIM_RESOURCE_PATH", str(share / "models")),
            AppendEnvironmentVariable(
                "IGN_GAZEBO_SYSTEM_PLUGIN_PATH",
                str(Path(get_package_prefix("group4_project2_al")) / "lib"),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(rig / "launch/simulation.launch.py")),
                launch_arguments={
                    "gui": LaunchConfiguration("gui"),
                    "world_file": str(share / "worlds/conveyor.sdf"),
                    "kinematic_grasp": LaunchConfiguration("production"),
                    "reference_boxes": "false",
                    "relocate_controls": "true",
                    "table_height": LaunchConfiguration("table_height"),
                }.items(),
            ),
            # Display the three supplied STL parts in the general process scene.
            TimerAction(
                period=14.0,
                actions=[
                    Node(
                        package="ros_gz_sim",
                        executable="create",
                        name="spawn_product_parts",
                        output="screen",
                        arguments=[
                            "-world",
                            "empty",
                            "-file",
                            str(share / "models/product/model.sdf"),
                            "-name",
                            "product_parts",
                        ],
                        condition=UnlessCondition(
                            LaunchConfiguration("assembly_scene")
                        ),
                    )
                ],
            ),
            Node(
                package="group4_assembly_line",
                executable="rig_markers.py",
                parameters=[
                    {
                        "use_sim_time": True,
                        "show_reference_boxes": False,
                        "relocate_controls": True,
                        "table_height": ParameterValue(
                            LaunchConfiguration("table_height"), value_type=float
                        ),
                        "product_mesh_dir": PythonExpression(
                            [
                                "'' if '",
                                LaunchConfiguration("assembly_scene"),
                                "' == 'true' else '",
                                str(share / "models/product"),
                                "'",
                            ]
                        ),
                    }
                ],
                output="log",
            ),
            Node(
                package="ros_gz_bridge",
                executable="parameter_bridge",
                output="screen",
                arguments=[
                    "/project2/sim/proximity_scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan",
                    "/world/empty/create@ros_gz_interfaces/srv/SpawnEntity",
                    "/world/empty/set_pose@ros_gz_interfaces/srv/SetEntityPose",
                    "/world/empty/pose/info@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V",
                    "/project2/grasp/request@std_msgs/msg/String@gz.msgs.StringMsg",
                    "/project2/grasp/reply@std_msgs/msg/String@gz.msgs.StringMsg",
                ],
                remappings=[("/world/empty/pose/info", "/project2/sim/entity_poses")],
                condition=IfCondition(LaunchConfiguration("assembly_scene")),
            ),
            Node(
                package="group4_project2_al",
                executable="scene_markers.py",
                output="log",
                condition=IfCondition(LaunchConfiguration("production")),
            ),
            Node(
                package="group4_project2_al",
                executable="assembly_scene.py",
                output="screen",
                parameters=[
                    {
                        "production": ParameterValue(LaunchConfiguration("production"), value_type=bool),
                        "table_height": ParameterValue(
                            LaunchConfiguration("table_height"), value_type=float
                        ),
                        "toys": ParameterValue(
                            LaunchConfiguration("toys"), value_type=int
                        ),
                        "qa_trial": ParameterValue(
                            LaunchConfiguration("qa_trial"), value_type=bool
                        ),
                        "contact_assembly": ParameterValue(
                            PythonExpression(
                                [
                                    "'",
                                    LaunchConfiguration("contact_assembly"),
                                    "' == 'true' or '",
                                    LaunchConfiguration("production"),
                                    "' == 'true'",
                                ]
                            ),
                            value_type=bool,
                        ),
                        "layout_file": str(share / "config/assembly_run.yaml"),
                    }
                ],
                condition=IfCondition(LaunchConfiguration("assembly_scene")),
            ),
            Node(
                package="group4_project2_al",
                executable="conveyor_sensor.py",
                output="screen",
                parameters=[
                    {
                        "table_height": ParameterValue(
                            LaunchConfiguration("table_height"), value_type=float
                        ),
                        "config": LaunchConfiguration("production_config"),
                    }
                ],
                condition=IfCondition(LaunchConfiguration("production")),
            ),
            Node(
                package="group4_project2_al",
                executable="production_run.py",
                output="screen",
                parameters=[
                    {
                        "table_height": ParameterValue(
                            LaunchConfiguration("table_height"), value_type=float
                        ),
                        "toys": ParameterValue(
                            LaunchConfiguration("toys"), value_type=int
                        ),
                        "qa_trial": ParameterValue(
                            LaunchConfiguration("qa_trial"), value_type=bool
                        ),
                        "qa_seed": ParameterValue(
                            LaunchConfiguration("qa_seed"), value_type=int
                        ),
                        "require_io": ParameterValue(
                            LaunchConfiguration("micro_ros"), value_type=bool
                        ),
                        "autostart": ParameterValue(
                            LaunchConfiguration("autostart"), value_type=bool
                        ),
                        "config": LaunchConfiguration("production_config"),
                    }
                ],
                condition=IfCondition(LaunchConfiguration("production")),
            ),
            Node(
                package="group4_project2_al",
                executable="camera_inspection.py",
                output="screen",
                parameters=[
                    {
                        "cad_camera": ParameterValue(
                            LaunchConfiguration("cad_camera"), value_type=bool
                        ),
                        "show_window": ParameterValue(
                            LaunchConfiguration("camera_window"), value_type=bool
                        ),
                        "config": LaunchConfiguration("production_config"),
                    }
                ],
                condition=IfCondition(LaunchConfiguration("production")),
            ),
            Node(
                package="group4_project2_al",
                executable="arm_sim.py",
                output="screen",
                parameters=[
                    {
                        "backend": "gazebo",
                        "table_height": ParameterValue(
                            LaunchConfiguration("table_height"), value_type=float
                        ),
                        "control": LaunchConfiguration("control"),
                        "waypoints_file": LaunchConfiguration("waypoints"),
                        "require_io": ParameterValue(
                            LaunchConfiguration("micro_ros"), value_type=bool
                        ),
                        "master_slave": ParameterValue(
                            LaunchConfiguration("master_slave"), value_type=bool
                        ),
                    }
                ],
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    str(share / "launch/simulation.launch.py")
                ),
                condition=UnlessCondition(LaunchConfiguration("production")),
                launch_arguments={
                    "mode": LaunchConfiguration("mode"),
                    "cycles": LaunchConfiguration("cycles"),
                    "manual_mode": PythonExpression(
                        ["'", LaunchConfiguration("control"), "' == 'manual'"]
                    ),
                    "require_io": LaunchConfiguration("micro_ros"),
                    "autostart": LaunchConfiguration("autostart"),
                    "wait_motion": "true",
                    "wait_scene": LaunchConfiguration("assembly_scene"),
                    "motion_timeout": PythonExpression(
                        [
                            "180.0 if '",
                            LaunchConfiguration("contact_assembly"),
                            "' == 'true' else 60.0",
                        ]
                    ),
                }.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    str(share / "launch/control_box.launch.py")
                ),
                condition=IfCondition(LaunchConfiguration("micro_ros")),
                launch_arguments={"panel": LaunchConfiguration("panel"), "cad_camera": LaunchConfiguration("cad_camera")}.items(),
            ),
            Node(
                package="group4_project2_al",
                executable="sim_power.py",
                output="screen",
                condition=IfCondition(LaunchConfiguration("micro_ros")),
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                arguments=["-d", str(share / "config/gazebo.rviz")],
                parameters=[{"use_sim_time": True}],
                output="log",
                condition=IfCondition(LaunchConfiguration("rviz")),
            ),
        ]
    )
