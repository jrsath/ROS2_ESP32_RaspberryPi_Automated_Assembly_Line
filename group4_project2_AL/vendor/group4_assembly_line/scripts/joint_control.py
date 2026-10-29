#!/usr/bin/env python3
import argparse
import math
import threading

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from group4_assembly_line.arm import ALL_JOINTS, SO101Arm, load_hardware_config


def load_poses():
    path = (
        get_package_share_directory("group4_assembly_line") + "/config/robot_poses.yaml"
    )
    with open(path, encoding="utf-8") as config_file:
        return yaml.safe_load(config_file)["poses"]


def main():
    parser = argparse.ArgumentParser(
        description="Move one SO101 in Gazebo or on the real ROS interface."
    )
    parser.add_argument("--backend", choices=["sim", "real"], default="sim")
    parser.add_argument(
        "--robot",
        choices=["assembly_left", "assembly_right", "inspection", "packaging"],
        default="assembly_left",
    )
    parser.add_argument("--pose", help="Named pose from config/robot_poses.yaml")
    parser.add_argument(
        "--radians",
        type=float,
        nargs=6,
        metavar=("PAN", "LIFT", "ELBOW", "WRIST", "ROLL", "GRIPPER"),
    )
    parser.add_argument(
        "--degrees",
        type=float,
        nargs=6,
        metavar=("PAN", "LIFT", "ELBOW", "WRIST", "ROLL", "GRIPPER"),
    )
    parser.add_argument("--xyz", type=float, nargs=3, metavar=("X", "Y", "Z"))
    parser.add_argument("--gripper", type=float, help="Gripper position in radians")
    parser.add_argument("--duration", type=float, default=2.0)
    parser.add_argument(
        "--wait-timeout",
        type=float,
        default=120.0,
        help="Seconds to wait for the robot controllers and joint feedback",
    )
    parser.add_argument(
        "--sequence", action="store_true", help="Run the neutral/demo motion sequence"
    )
    parser.add_argument(
        "--read", action="store_true", help="Print the current joint positions"
    )
    parser.add_argument(
        "--jog",
        nargs=2,
        metavar=("JOINT", "DELTA"),
        help="Move one joint by a small amount in radians",
    )
    args, ros_args = parser.parse_known_args()

    selected = sum(
        value is not None
        for value in (
            args.pose,
            args.radians,
            args.degrees,
            args.xyz,
            args.gripper,
            args.jog,
        )
    )
    selected += int(args.sequence) + int(args.read)
    if selected == 0:
        parser.error(
            "choose --pose, --radians, --degrees, --xyz, --gripper, --sequence, --read or --jog"
        )
    if selected > 1:
        parser.error("choose only one target type at a time")

    rclpy.init(args=ros_args)
    node = Node("so101_joint_control")
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    try:
        arm = SO101Arm(node, args.robot, args.backend, load_hardware_config())
        node.get_logger().info(f"{args.robot}: waiting for robot command interface...")
        if not arm.wait_ready(timeout=max(1.0, args.wait_timeout)):
            raise RuntimeError(f"{args.robot}: robot command interface is not ready")
        poses = load_poses()

        if args.read:
            values = arm.positions()
            for name, value in zip(ALL_JOINTS, values):
                print(f"{name}: {value:.6f} rad ({math.degrees(value):.2f} deg)")
        elif args.jog:
            joint, delta_text = args.jog
            if joint not in ALL_JOINTS:
                raise KeyError(f"Unknown joint {joint}. Choices: {ALL_JOINTS}")
            delta = float(delta_text)
            values = arm.positions()
            index = ALL_JOINTS.index(joint)
            values[index] += delta
            if joint == "gripper":
                if not arm.set_gripper(values[5], duration=args.duration):
                    raise RuntimeError(f"{args.robot}: gripper jog failed")
            else:
                if not arm.move_joints(values[:5], duration=args.duration):
                    raise RuntimeError(f"{args.robot}: joint jog failed")
        elif args.sequence:
            for name in ("neutral", "demo_left", "demo_right", "neutral"):
                node.get_logger().info(f"{args.robot}: {name}")
                if not arm.move_pose(poses[name], duration=args.duration):
                    raise RuntimeError(f"{args.robot}: motion failed at {name}")
        elif args.pose:
            if args.pose not in poses:
                raise KeyError(f"Unknown pose {args.pose}. Choices: {sorted(poses)}")
            if not arm.move_pose(poses[args.pose], duration=args.duration):
                raise RuntimeError(f"{args.robot}: motion failed")
        elif args.radians:
            if not arm.move_pose(args.radians, duration=args.duration):
                raise RuntimeError(f"{args.robot}: motion failed")
        elif args.degrees:
            values = [math.radians(value) for value in args.degrees]
            if not arm.move_pose(values, duration=args.duration):
                raise RuntimeError(f"{args.robot}: motion failed")
        elif args.xyz:
            if not arm.move_xyz(*args.xyz, duration=args.duration):
                raise RuntimeError(f"{args.robot}: IK motion failed")
        elif args.gripper is not None:
            if not arm.set_gripper(args.gripper, duration=min(args.duration, 1.0)):
                raise RuntimeError(f"{args.robot}: gripper motion failed")
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()
        spin_thread.join(timeout=1.0)


if __name__ == "__main__":
    main()
