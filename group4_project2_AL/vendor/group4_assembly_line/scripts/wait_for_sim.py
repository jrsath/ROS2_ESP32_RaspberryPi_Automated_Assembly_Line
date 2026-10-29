#!/usr/bin/env python3
# Wait for arm and gripper action servers and six-joint feedback on every arm.
# A zero timeout waits until slow Gazebo controllers finish starting.

import time

import rclpy
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState

ROBOTS = ("assembly_left", "assembly_right", "inspection", "packaging")
JOINTS = {
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_roll",
    "gripper",
}


class SimulationReadiness(Node):
    def __init__(self):
        super().__init__("project2_simulation_readiness")
        self.timeout_s = float(self.declare_parameter("timeout_s", 0.0).value)
        self.started = time.monotonic()
        self.last_report = 0.0
        self.feedback = {name: False for name in ROBOTS}
        self.arm_actions = {}
        self.gripper_actions = {}
        self.subscriptions = []

        for robot in ROBOTS:
            self.arm_actions[robot] = ActionClient(
                self,
                FollowJointTrajectory,
                f"/{robot}/arm_controller/follow_joint_trajectory",
            )
            self.gripper_actions[robot] = ActionClient(
                self,
                FollowJointTrajectory,
                f"/{robot}/gripper_controller/follow_joint_trajectory",
            )
            self.subscriptions.append(
                self.create_subscription(
                    JointState,
                    f"/{robot}/joint_state_broadcaster/joint_states",
                    lambda msg, name=robot: self._joint_state(name, msg),
                    20,
                )
            )

        self.get_logger().info("Waiting for complete Project 2 Gazebo simulation...")

    def _joint_state(self, robot, msg):
        if msg.name:
            self.feedback[robot] = JOINTS.issubset(set(msg.name))
        else:
            self.feedback[robot] = len(msg.position) >= len(JOINTS)

    def robot_status(self, robot):
        arm = self.arm_actions[robot].server_is_ready()
        gripper = self.gripper_actions[robot].server_is_ready()
        feedback = self.feedback[robot]
        return arm, gripper, feedback

    def ready(self):
        return all(all(self.robot_status(robot)) for robot in ROBOTS)

    def report(self):
        now = time.monotonic()
        if now - self.last_report < 2.0:
            return
        self.last_report = now
        for robot in ROBOTS:
            arm, gripper, feedback = self.robot_status(robot)
            self.get_logger().info(
                f'{robot}: arm_action={"ready" if arm else "waiting"}, '
                f'gripper_action={"ready" if gripper else "waiting"}, '
                f'joint_feedback={"ready" if feedback else "waiting"}'
            )

    def timed_out(self):
        return (
            self.timeout_s > 0.0 and (time.monotonic() - self.started) >= self.timeout_s
        )


def main(args=None):
    rclpy.init(args=args)
    node = SimulationReadiness()
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            if node.ready():
                node.get_logger().info(
                    "Simulation READY: all four SO101 controllers and joint feedback are available."
                )
                return
            node.report()
            if node.timed_out():
                raise RuntimeError(
                    "Timed out waiting for the complete Gazebo simulation"
                )
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
