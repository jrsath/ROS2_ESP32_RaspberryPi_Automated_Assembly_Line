import os
import xml.etree.ElementTree as ET
from pathlib import Path

import yaml
import xacro
from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
    LogInfo,
    TimerAction,
    DeclareLaunchArgument,
    OpaqueFunction,
)
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import (
    IncludeLaunchDescription,
    RegisterEventHandler,
    EmitEvent,
    ExecuteProcess,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch_ros.actions import Node


ROBOTS = ("assembly_left", "assembly_right", "inspection", "packaging")


def make_scene(context):
    share = Path(get_package_share_directory("group4_assembly_line"))
    models = share / "models"
    height = float(LaunchConfiguration("table_height").perform(context))
    relocate = LaunchConfiguration("relocate_controls").perform(context) == "true"
    rig_document = ET.parse(models / "project2_rig/model.sdf")
    if relocate:
        cad = rig_document.getroot().find(".//link")
        for visual in list(cad.findall("visual")):
            if visual.get("name", "").startswith(("estop_w_buttons_", "estop_mount_")):
                cad.remove(visual)
    rig_sdf = ET.tostring(rig_document.getroot(), encoding="unicode")
    reference_boxes = LaunchConfiguration("reference_boxes").perform(context) == "true"
    with open(share / "config/rig_layout.yaml", encoding="utf-8") as f:
        layout = yaml.safe_load(f)

    # Separate foreground processes let ros2 launch stop both WSLg and physics.
    gazebo = ExecuteProcess(
        cmd=[
            "ign",
            "gazebo",
            "-s",
            "-r",
            LaunchConfiguration("world_file").perform(context),
        ],
        output="screen",
        sigterm_timeout="5",
        sigkill_timeout="3",
    )
    gui = ExecuteProcess(
        cmd=["ign", "gazebo", "-g"],
        output="screen",
        condition=IfCondition(LaunchConfiguration("gui")),
        sigterm_timeout="5",
        sigkill_timeout="3",
    )
    clock_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        output="screen",
        arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
    )

    actions = [
        DeclareLaunchArgument("gui", default_value="true"),
        DeclareLaunchArgument("dynamic_boxes", default_value="false"),
        AppendEnvironmentVariable("IGN_GAZEBO_RESOURCE_PATH", str(models)),
        AppendEnvironmentVariable("GZ_SIM_RESOURCE_PATH", str(models)),
        AppendEnvironmentVariable(
            "IGN_GAZEBO_SYSTEM_PLUGIN_PATH",
            str(Path(get_package_prefix("gz_ros2_control")) / "lib"),
        ),
        gazebo,
        gui,
        clock_bridge,
        LogInfo(
            msg="Project 2 simulation: starting Gazebo and loading supplied CAD rig..."
        ),
    ]

    actions.append(
        TimerAction(
            period=1.0,
            actions=[
                LogInfo(msg="Project 2 simulation: spawning Conveyor Robot Rig CAD..."),
                Node(
                    package="ros_gz_sim",
                    executable="create",
                    name="spawn_project2_rig",
                    output="screen",
                    arguments=[
                        "-world",
                        "empty",
                        "-string",
                        rig_sdf,
                        "-name",
                        "project2_rig",
                        "-z",
                        str(height),
                    ],
                ),
            ],
        )
    )

    box_actions = []
    for box in layout["boxes"] if reference_boxes else []:
        xyz = box["xyz"]
        rpy = box["rpy"]
        box_actions.append(
            Node(
                package="ros_gz_sim",
                executable="create",
                name=f"spawn_{box['name']}",
                output="screen",
                arguments=[
                    "-world",
                    "empty",
                    "-file",
                    PythonExpression(
                        [
                            "'",
                            str(models),
                            "/box_lower' + ('' if '",
                            LaunchConfiguration("dynamic_boxes"),
                            "' == 'true' else '_static') + '/model.sdf'",
                        ]
                    ),
                    "-name",
                    box["name"],
                    "-x",
                    str(xyz[0]),
                    "-y",
                    str(xyz[1]),
                    "-z",
                    str(xyz[2] + height),
                    "-R",
                    str(rpy[0]),
                    "-P",
                    str(rpy[1]),
                    "-Y",
                    str(rpy[2]),
                ],
            )
        )
    # Queue box requests so model creation does not flood Gazebo at startup.
    actions.append(TimerAction(period=1.8, actions=box_actions[:1]))
    for previous, following in zip(box_actions, box_actions[1:]):
        actions.append(
            RegisterEventHandler(
                OnProcessExit(target_action=previous, on_exit=[following])
            )
        )

    xacro_path = share / "urdf/so101.urdf.xacro"
    controllers = share / "config/so101_controllers.yaml"
    robot_spawns = []
    robot_nodes = []
    controller_spawns = [
        LogInfo(
            msg="Project 2 simulation: waiting for robot controller managers, then activating controllers..."
        )
    ]
    tf_nodes = []

    for robot in ROBOTS:
        document = xacro.process_file(
            str(xacro_path),
            mappings={
                "robot_namespace": f"/{robot}",
                "prefix": f"{robot}/",
                "controllers_file": str(controllers),
                "mesh_root": str(models / "project2_rig/meshes"),
            },
        )
        description = document.toxml()
        rsp = Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            namespace=robot,
            name="robot_state_publisher",
            output="screen",
            parameters=[
                {
                    "robot_description": description,
                    "use_sim_time": True,
                    "frame_prefix": f"{robot}/",
                }
            ],
            remappings=[("joint_states", "joint_state_broadcaster/joint_states")],
        )
        p = layout["robots"][robot]
        xyz, rpy = p["xyz"], p["rpy"]
        spawn = Node(
            package="ros_gz_sim",
            executable="create",
            name=f"spawn_{robot}",
            output="screen",
            arguments=[
                "-world",
                "empty",
                "-topic",
                f"/{robot}/robot_description",
                "-name",
                robot,
                "-x",
                str(xyz[0]),
                "-y",
                str(xyz[1]),
                "-z",
                str(xyz[2] + height),
                "-R",
                str(rpy[0]),
                "-P",
                str(rpy[1]),
                "-Y",
                str(rpy[2]),
            ],
        )
        spawner = Node(
            package="group4_assembly_line",
            executable="start_controllers.py",
            name=f"{robot}_controller_spawner",
            output="screen",
            arguments=["--controller-manager", f"/{robot}/controller_manager"],
        )
        tf = Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name=f"{robot}_world_pose",
            output="log",
            arguments=[
                "--x",
                str(xyz[0]),
                "--y",
                str(xyz[1]),
                "--z",
                str(xyz[2] + height),
                "--roll",
                str(rpy[0]),
                "--pitch",
                str(rpy[1]),
                "--yaw",
                str(rpy[2]),
                "--frame-id",
                "world",
                "--child-frame-id",
                f"{robot}/world",
            ],
        )
        robot_nodes.append(rsp)
        robot_spawns.append(spawn)
        controller_spawns.append(spawner)
        tf_nodes.append(tf)

    actions.append(TimerAction(period=2.5, actions=robot_nodes + tf_nodes))

    # Spawn robots after boxes, one at a time, to avoid concurrent plugin setup.
    def begin_robots(event, context):
        if event.returncode != 0:
            return [EmitEvent(event=Shutdown(reason="Packaging box creation failed"))]
        return [
            LogInfo(
                msg="Project 2 simulation: spawning four articulated SO101 arms..."
            ),
            TimerAction(period=1.0, actions=robot_spawns[:1]),
        ]

    if box_actions:
        actions.append(
            RegisterEventHandler(
                OnProcessExit(target_action=box_actions[-1], on_exit=begin_robots)
            )
        )
    else:
        actions.append(TimerAction(period=3.5, actions=robot_spawns[:1]))
    for previous, following in zip(robot_spawns, robot_spawns[1:]):

        def next_robot(event, context, node=following):
            if event.returncode != 0:
                return [EmitEvent(event=Shutdown(reason="Robot creation failed"))]
            return [TimerAction(period=3.0, actions=[node])]

        actions.append(
            RegisterEventHandler(
                OnProcessExit(target_action=previous, on_exit=next_robot)
            )
        )

    def begin_controllers(event, context):
        if event.returncode != 0:
            return [EmitEvent(event=Shutdown(reason="Robot creation failed"))]
        return [TimerAction(period=5.0, actions=controller_spawns[:2])]

    actions.append(
        RegisterEventHandler(
            OnProcessExit(target_action=robot_spawns[-1], on_exit=begin_controllers)
        )
    )
    for previous, following in zip(controller_spawns[1:], controller_spawns[2:]):

        def next_controller(event, context, node=following):
            if event.returncode != 0:
                return [EmitEvent(event=Shutdown(reason="Controller startup failed"))]
            return [node]

        actions.append(
            RegisterEventHandler(
                OnProcessExit(target_action=previous, on_exit=next_controller)
            )
        )

    if relocate:
        # Rotate the operator controls toward the person at the front of the table.
        box_sdf = '<sdf version="1.9"><model name="operator_control_box"><static>true</static><link name="case"><visual name="controls"><geometry><mesh><uri>model://project2_rig/meshes/EStop_w_buttons.stl</uri></mesh></geometry><material><ambient>0.3 0.3 0.31 1</ambient></material></visual><collision name="housing"><pose>0 0 0.025 0 0 0</pose><geometry><box><size>0.17 0.071 0.05</size></box></geometry></collision></link></model></sdf>'
        actions.append(
            TimerAction(
                period=1.6,
                actions=[
                    Node(
                        package="ros_gz_sim",
                        executable="create",
                        arguments=[
                            "-world",
                            "empty",
                            "-string",
                            box_sdf,
                            "-name",
                            "operator_control_box",
                            "-x",
                            "0",
                            "-y",
                            "-0.15",
                            "-z",
                            str(height - 0.07),
                            "-R",
                            "1.570796327",
                        ],
                        output="log",
                    )
                ],
            )
        )
    if height > 0:
        # Table top ends at the rig mounting plane; legs extend to the floor.
        pieces = [((0, 0.28, height - 0.025), (1.8, 0.85, 0.05))]
        pieces += [
            ((x, y, (height - 0.05) / 2), (0.055, 0.055, height - 0.05))
            for x in (-0.82, 0.82)
            for y in (-0.08, 0.64)
        ]
        links = ""
        for i, (xyz, size) in enumerate(pieces):
            pose = " ".join(map(str, xyz)) + " 0 0 0"
            dims = " ".join(map(str, size))
            geometry = "<geometry><box><size>" + dims + "</size></box></geometry>"
            links += f'<link name="piece{i}"><pose>{pose}</pose><collision name="solid">{geometry}</collision><visual name="shape">{geometry}<material><ambient>0.32 0.34 0.36 1</ambient><diffuse>0.32 0.34 0.36 1</diffuse></material></visual></link>'
        sdf = (
            '<sdf version="1.9"><model name="work_table"><static>true</static>'
            + links
            + "</model></sdf>"
        )
        actions.append(
            TimerAction(
                period=1.3,
                actions=[
                    Node(
                        package="ros_gz_sim",
                        executable="create",
                        arguments=[
                            "-world",
                            "empty",
                            "-string",
                            sdf,
                            "-name",
                            "work_table",
                        ],
                        output="log",
                    )
                ],
            )
        )

    # The calling package supplies RViz, markers and the process coordinator.
    return actions


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("world_file", default_value="empty.sdf"),
            DeclareLaunchArgument("relocate_controls", default_value="false"),
            DeclareLaunchArgument("table_height", default_value="0.0"),
            DeclareLaunchArgument("reference_boxes", default_value="true"),
            DeclareLaunchArgument("kinematic_grasp", default_value="false"),
            OpaqueFunction(function=make_scene),
        ]
    )
