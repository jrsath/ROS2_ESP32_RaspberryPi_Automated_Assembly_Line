#!/usr/bin/env python3
# A range beam drives the virtual proximity input. Poses are used only for the display.
import json
import re
import time
from pathlib import Path
import numpy as np
import rclpy
import yaml
from rclpy.node import Node
from rclpy.clock import Clock, ClockType
from ament_index_python.packages import get_package_share_directory
from std_msgs.msg import Bool, String
from sensor_msgs.msg import LaserScan
from ros_gz_interfaces.srv import SpawnEntity
from tf2_msgs.msg import TFMessage
from group4_project2_al.conveyor import belt_fraction, range_occupied
from group4_project2_al.scene_layout import raised_config


class ConveyorSensor(Node):
    def __init__(self):
        super().__init__("project2_conveyor_sensor")
        share = Path(get_package_share_directory("group4_project2_al"))
        path = self.declare_parameter(
            "config", str(share / "config/production.yaml")
        ).value
        height = self.declare_parameter("table_height", 1.0).value
        config = raised_config(yaml.safe_load(Path(path).read_text()), height)
        self.start = np.array(config["conveyor_in"][:3])
        self.stop = np.array(config["conveyor_out"][:3])
        self.settings = config["conveyor_sensor"]
        self.range_at = 0.0
        self.range_m = float("inf")
        self.spawn = self.create_client(SpawnEntity, "/world/empty/create")
        self.spawn_future = None
        self.sensor_spawned = False
        self.create_subscription(
            LaserScan, "/project2/sim/proximity_scan", self.on_range, 10
        )
        self.positions = {}
        self.last_pose = 0.0
        self.sensor = self.create_publisher(Bool, "/project2/sim/proximity", 10)
        self.display = self.create_publisher(String, "/project2/sim/conveyor", 10)
        self.create_subscription(
            TFMessage, "/project2/sim/entity_poses", self.receive, 10
        )
        # WSL wall-clock corrections must not pause the watchdog heartbeat.
        self.create_timer(
            0.05, self.publish, clock=Clock(clock_type=ClockType.STEADY_TIME)
        )

    def on_range(self, message):
        self.range_at = time.monotonic()
        values = [
            v for v in message.ranges if np.isfinite(v) and v >= message.range_min
        ]
        self.range_m = min(values, default=float("inf"))

    def spawn_sensor(self):
        if self.sensor_spawned or not self.spawn.service_is_ready():
            return
        if self.spawn_future is not None:
            if self.spawn_future.done():
                self.sensor_spawned = bool(self.spawn_future.result().success)
                if not self.sensor_spawned:
                    self.get_logger().error(
                        "Proximity sensor model could not be created"
                    )
            return
        position = self.stop + np.asarray(self.settings["beam_offset"])
        limit = self.settings["maximum_range_m"]
        request = SpawnEntity.Request()
        request.entity_factory.name = "conveyor_proximity_sensor"
        # The spawn service pose overrides the pose written inside the SDF.
        request.entity_factory.pose.position.x = float(position[0])
        request.entity_factory.pose.position.y = float(position[1])
        request.entity_factory.pose.position.z = float(position[2])
        request.entity_factory.pose.orientation.y = 2**-0.5
        request.entity_factory.pose.orientation.w = 2**-0.5
        request.entity_factory.sdf = f"""<sdf version="1.9"><model name="conveyor_proximity_sensor"><static>true</static>
          <link name="sensor">
          <visual name="housing"><pose>-0.012 0 0 0 0 0</pose><geometry><box><size>0.02 0.012 0.012</size></box></geometry><material><ambient>0.12 0.14 0.16 1</ambient></material></visual>
          <sensor name="beam" type="gpu_lidar"><topic>/project2/sim/proximity_scan</topic><update_rate>30</update_rate><always_on>true</always_on>
            <ray><scan><horizontal><samples>3</samples><resolution>1</resolution><min_angle>-0.01</min_angle><max_angle>0.01</max_angle></horizontal></scan>
            <range><min>0.003</min><max>{limit}</max><resolution>0.001</resolution></range></ray>
          </sensor></link></model></sdf>"""
        self.spawn_future = self.spawn.call_async(request)

    def receive(self, message):
        self.last_pose = time.monotonic()
        for item in message.transforms:
            if re.fullmatch(r"toy_\d+_base_1v4", item.child_frame_id):
                v = item.transform.translation
                self.positions[item.child_frame_id] = np.array([v.x, v.y, v.z])

    def publish(self):
        self.spawn_sensor()
        fresh = time.monotonic() - self.range_at < 0.75
        occupied = range_occupied(self.range_m, self.settings["trigger_range_m"])
        # Do not publish stale sensor feedback; the board will stop on its timeout.
        if fresh:
            self.sensor.publish(Bool(data=occupied))
        toys = []
        for name, p in self.positions.items():
            if (
                self.start[0] - 0.04 <= p[0] <= self.stop[0] + 0.04
                and abs(p[1] - self.start[1]) < 0.04
                and abs(p[2] - self.start[2]) < 0.025
            ):
                toys.append(
                    dict(
                        toy=int(name.split("_")[1]),
                        position=belt_fraction(p, self.start, self.stop),
                    )
                )
        self.display.publish(
            String(
                data=json.dumps(
                    dict(
                        fresh=fresh,
                        occupied=occupied,
                        range_m=self.range_m if np.isfinite(self.range_m) else None,
                        toys=toys,
                    )
                )
            )
        )


def main():
    rclpy.init()
    node = ConveyorSensor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
