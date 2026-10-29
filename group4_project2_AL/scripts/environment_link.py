#!/usr/bin/env python3
# Observe the physical ROS domain in a separate process from the simulation.
import argparse
import json
import os
import time
from pathlib import Path

import yaml
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_msgs.msg import UInt64MultiArray


class EnvironmentLink(Node):
    def __init__(self, identity):
        super().__init__("project2_panel_environment_link")
        self.io = None
        self.io_time = 0.0
        self.arms = {}
        self.configured = []
        self.create_subscription(
            UInt64MultiArray,
            "/project2/io/state",
            self.receive_io,
            qos_profile_sensor_data,
        )
        for arm, settings in identity.get("hardware", {}).get("arms", {}).items():
            topic = settings.get("state_topic", "")
            if topic:
                self.configured.append(arm)
                self.create_subscription(
                    JointState,
                    topic,
                    lambda msg, name=arm: self.receive_arm(name, msg),
                    qos_profile_sensor_data,
                )
        self.create_timer(0.25, self.report)

    def receive_io(self, message):
        if len(message.data) == 14:
            self.io = list(message.data)
            self.io_time = time.monotonic()

    def receive_arm(self, arm, message):
        # Report the original names and values; never guess a joint-name mapping.
        self.arms[arm] = (time.monotonic(), list(message.name), list(message.position))

    def report(self):
        now = time.monotonic()
        arms = {
            arm: {"names": names, "positions": positions}
            for arm, (seen, names, positions) in self.arms.items()
            if now - seen < 1.0
        }
        print(
            json.dumps(
                {
                    "environment_status": True,
                    "io": self.io if now - self.io_time < 1.0 else None,
                    "arms": arms,
                    "configured_arms": self.configured,
                    "motor_commands_enabled": False,
                }
            ),
            flush=True,
        )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--identity", required=True)
    args = parser.parse_args()
    identity = yaml.safe_load(Path(args.identity).read_text())
    # rclpy reads the domain at initialisation; changing the GUI's domain is unsafe.
    os.environ["ROS_DOMAIN_ID"] = str(identity["ros_domain_id"])
    os.environ["ROS_LOCALHOST_ONLY"] = "0"
    rclpy.init()
    node = EnvironmentLink(identity)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
