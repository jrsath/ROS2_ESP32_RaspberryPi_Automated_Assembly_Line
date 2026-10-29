#!/usr/bin/env python3
# Show whether the real ESP32 is communicating and which inputs are active.
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import UInt64MultiArray


class IOMonitor(Node):
    def __init__(self):
        super().__init__("project2_io_monitor")
        self.last_seen = None
        self.last_state = None
        self.reported = False
        self.create_subscription(UInt64MultiArray, "/project2/io/state", self.receive, qos_profile_sensor_data)
        self.create_timer(5.0, self.check)
        self.get_logger().info("Waiting for ESP32 /project2/io/state. This connection check sends no actuator commands.")

    def receive(self, message):
        if len(message.data) != 14:
            self.get_logger().error("Unexpected ESP32 state format; upload firmware/project2_io from this project.")
            return
        self.last_seen = time.monotonic()
        values = tuple(message.data)
        state = (values[0], *values[2:8])
        if state != self.last_state or self.reported:
            self.get_logger().info(f"ESP32 connected | outputs verified={bool(values[2])} | Start={values[3]} Pause={values[4]} E-stop healthy={values[5]} selector={values[6]} sensor={values[7]}")
        self.last_state = state
        self.reported = False

    def check(self):
        if self.last_seen is None or time.monotonic() - self.last_seen > 2.0:
            if not self.reported:
                self.get_logger().warning("No ESP32 telemetry. Check flashed serial/Wi-Fi transport and ROS domain. USB: close Serial Monitor and forward the device to WSL. Wi-Fi: check agent IP, firewall and WSL relay.")
            self.reported = True


def main():
    rclpy.init()
    node = IOMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
