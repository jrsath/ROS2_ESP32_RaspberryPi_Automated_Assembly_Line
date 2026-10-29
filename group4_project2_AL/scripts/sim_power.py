#!/usr/bin/env python3
# Simulate the E-stop relay by disabling the Gazebo trajectory controllers.
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from controller_manager_msgs.srv import SwitchController
from std_msgs.msg import Bool, String, UInt64MultiArray


class SimPower(Node):
    def __init__(self):
        super().__init__("project2_sim_power")
        self.switch_clients = {
            arm: self.create_client(
                SwitchController, f"/{arm}/controller_manager/switch_controller"
            )
            for arm in ("assembly_left", "assembly_right", "inspection", "packaging")
        }
        self.power = None
        self.healthy = False
        self.latched = False
        self.seen = False
        self.last_io = 0.0
        self.requested = False
        self.pending = {}
        self.retry_at = 0.0
        self.pub = self.create_publisher(Bool, "/project2/sim/actuator_power", 10)
        self.create_subscription(
            UInt64MultiArray, "/project2/sim/io/state", self.io, qos_profile_sensor_data
        )
        self.create_subscription(String, "/project2/sim/event", self.event, 10)
        self.create_timer(0.05, self.tick)

    def io(self, message):
        if len(message.data) != 14:
            return
        self.last_io = time.monotonic()
        self.healthy = bool(message.data[5])
        if not self.seen:
            # Allow normal startup once a healthy virtual control box is present.
            self.seen = self.healthy
            self.requested = self.healthy
        elif not self.healthy:
            self.latched = True
            self.requested = False

    def event(self, message):
        if message.data == "estop":
            self.latched = True
            self.requested = False
        elif (
            message.data == "reset"
            and self.healthy
            and time.monotonic() - self.last_io < 0.75
        ):
            self.latched = False
            self.requested = True

    def tick(self):
        now = time.monotonic()
        if self.seen and now - self.last_io > 0.75:
            self.latched = True
            self.requested = False
        wanted = self.requested and not self.latched
        if self.pending and all(f.done() for f in self.pending.values()):
            if all(
                f.result() is not None and f.result().ok for f in self.pending.values()
            ):
                self.power = self.switch_target
                self.get_logger().info(
                    "Actuator power " + ("ON" if self.power else "OFF")
                )
            else:
                self.get_logger().warning("Controller power change failed; retrying")
            self.pending.clear()
            self.retry_at = now + 0.5
        if wanted != self.power and not self.pending and now >= self.retry_at:
            if all(c.service_is_ready() for c in self.switch_clients.values()):
                self.switch_target = wanted
                for arm, client in self.switch_clients.items():
                    request = SwitchController.Request()
                    request.strictness = SwitchController.Request.BEST_EFFORT
                    request.activate_asap = True
                    request.timeout.sec = 3
                    names = ["arm_controller", "gripper_controller"]
                    if wanted:
                        request.activate_controllers = names
                    else:
                        request.deactivate_controllers = names
                    self.pending[arm] = client.call_async(request)
        # Never report power available while an off request is pending.
        self.pub.publish(Bool(data=bool(self.power and wanted)))


def main():
    rclpy.init()
    node = SimPower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
