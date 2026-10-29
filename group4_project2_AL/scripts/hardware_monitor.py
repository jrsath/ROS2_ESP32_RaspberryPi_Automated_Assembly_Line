#!/usr/bin/env python3
# Read lab JointState feedback for RViz. This node never publishes motor commands.
from pathlib import Path
from group4_project2_al.connection import read_identity
import yaml
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from ament_index_python.packages import get_package_share_directory
from group4_project2_al.arm_model import JOINTS


class Monitor(Node):
    def __init__(self):
        super().__init__("project2_hardware_monitor")
        identity = read_identity(get_package_share_directory("group4_project2_al"))
        self.publishers_by_arm = {}
        for arm, config in identity["hardware"]["arms"].items():
            topic = config["state_topic"]
            if not topic:
                self.get_logger().warning(
                    f"{arm}: set the lab JointState topic in ID.yaml"
                )
                continue
            destination = f"/{arm}/joint_state_broadcaster/joint_states"
            if topic == destination:
                continue  # The lab already publishes the topic RViz uses.
            self.publishers_by_arm[arm] = self.create_publisher(
                JointState, f"/{arm}/joint_state_broadcaster/joint_states", 10
            )
            self.create_subscription(
                JointState,
                topic,
                lambda msg, name=arm: self.receive(name, msg),
                qos_profile_sensor_data,
            )

    def receive(self, arm, message):
        if not set(JOINTS).issubset(message.name):
            self.get_logger().warning(
                f"{arm}: lab joint names differ; map names/units before displaying"
            )
            return
        self.publishers_by_arm[arm].publish(message)


def main():
    rclpy.init()
    node = Monitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
