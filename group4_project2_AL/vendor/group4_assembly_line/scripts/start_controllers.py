#!/usr/bin/env python3
# Load each controller once and wait for slow Gazebo service replies.
import argparse
import rclpy
from rclpy.utilities import remove_ros_args
from controller_manager.controller_manager_services import (
    list_controllers,
    load_controller,
    configure_controller,
    switch_controllers,
)


def start(node, manager):
    def states():
        reply = list_controllers(
            node, manager, service_timeout=180.0, call_timeout=90.0
        )
        return {item.name: item.state for item in reply.controller}

    for name in ("joint_state_broadcaster", "arm_controller", "gripper_controller"):
        current = states().get(name)
        if current is None:
            # Humble's stock spawner uses a 10s load timeout even with its longer option.
            reply = load_controller(
                node, manager, name, service_timeout=180.0, call_timeout=90.0
            )
            current = states().get(name)
            if not reply.ok and current is None:
                raise RuntimeError(f"Could not load {name}")
        if current == "unconfigured":
            reply = configure_controller(
                node, manager, name, service_timeout=180.0, call_timeout=90.0
            )
            if not reply.ok and states().get(name) != "inactive":
                raise RuntimeError(f"Could not configure {name}")
        if states().get(name) != "active":
            reply = switch_controllers(
                node, manager, [], [name], True, True, 60.0, call_timeout=90.0
            )
            if not reply.ok or states().get(name) != "active":
                raise RuntimeError(f"Could not activate {name}")
        node.get_logger().info(f"{name} active")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--controller-manager", required=True)
    args = parser.parse_args(remove_ros_args()[1:])
    rclpy.init()
    node = rclpy.create_node("start_controllers")
    try:
        start(node, args.controller_manager)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
