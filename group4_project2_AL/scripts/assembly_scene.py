#!/usr/bin/env python3
# Spawn each toy's three parts and move them after the measured arm step finishes.
import json
import math
import time
from collections import deque
from pathlib import Path

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from ros_gz_interfaces.msg import Entity
from ros_gz_interfaces.srv import SetEntityPose, SpawnEntity
from std_msgs.msg import Bool, String

from group4_project2_al.kit_trays import tray_sdf as kit_tray_sdf
from group4_project2_al.assembly_run import (
    PARTS,
    STEPS,
    check_count,
    initial_poses,
    step_poses,
    tray_pose,
)


def orientation(rpy):
    roll, pitch, yaw = rpy
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


def fill_pose(message, part_pose):
    message.position.x, message.position.y, message.position.z = map(
        float, part_pose.xyz
    )
    (
        message.orientation.x,
        message.orientation.y,
        message.orientation.z,
        message.orientation.w,
    ) = orientation(part_pose.rpy)


def part_sdf(name, part, contact=False):
    # The three meshes are centred copies of the exported SolidWorks parts.
    geometry = {
        "base_1v4": "<mesh><uri>model://product/1V4_centered.stl</uri></mesh>",
        "insert_3v4": "<mesh><uri>model://product/3V4_centered.stl</uri></mesh>",
        "pin_2v4": "<mesh><uri>model://product/2V4_centered.stl</uri></mesh>",
    }[part]
    color = {
        "base_1v4": "0.18 0.58 0.86 1",
        "insert_3v4": "0.95 0.47 0.16 1",
        "pin_2v4": "0.95 0.78 0.15 1",
    }[part]
    if not contact:
        return (
            f'<sdf version="1.9"><model name="{name}"><static>true</static>'
            f'<link name="{name}__part"><visual name="shape"><geometry>{geometry}</geometry>'
            f"<material><ambient>{color}</ambient><diffuse>{color}</diffuse></material>"
            "</visual></link></model></sdf>"
        )
    return (
        f'<sdf version="1.9"><model name="{name}"><static>false</static>'
        f'<link name="{name}__part"><gravity>false</gravity><visual name="shape"><geometry>{geometry}</geometry>'
        f"<material><ambient>{color}</ambient><diffuse>{color}</diffuse></material>"
        "</visual>"
        "<inertial><mass>0.02</mass><inertia><ixx>0.00001</ixx><iyy>0.00001</iyy>"
        "<izz>0.00001</izz></inertia></inertial></link></model></sdf>"
    )


def tray_sdf(config, contact=False):
    plugin = (
        '<plugin filename="libproject2_al_grasp_system.so" '
        'name="group4_project2_al::GraspSystem"/>'
        if contact
        else ""
    )
    # The university CAD already contains the rack. This invisible anchor only
    # loads the grasp plugin; it adds no bench, rails or collision surfaces.
    return (
        '<sdf version="1.9"><model name="toy_tray"><static>true</static>'
        '<link name="anchor"/>' + plugin + "</model></sdf>"
    )


class AssemblyScene(Node):
    def __init__(self):
        super().__init__("project2_assembly_scene")
        share = Path(get_package_share_directory("group4_project2_al"))
        count = self.declare_parameter("toys", 1).value
        self.qa_trial = self.declare_parameter("qa_trial", False).value
        self.production = self.declare_parameter("production", False).value
        self.count = (
            (6 if self.qa_trial else 5)
            if self.production
            else check_count(count, 6 if self.qa_trial else 5)
        )
        self.height = self.declare_parameter("table_height", 1.0).value
        self.contact = self.declare_parameter("contact_assembly", False).value
        path = Path(
            self.declare_parameter(
                "layout_file", str(share / "config/assembly_run.yaml")
            ).value
        )
        self.config = yaml.safe_load(path.read_text())
        self.spawn = self.create_client(SpawnEntity, "/world/empty/create")
        self.set_pose = self.create_client(SetEntityPose, "/world/empty/set_pose")
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.ready_pub = self.create_publisher(
            Bool, "/project2/sim/assembly_scene/ready", qos
        )
        self.cycle_pub = self.create_publisher(
            String, "/project2/sim/assembly_scene/cycle_ready", 10
        )
        self.step_pub = self.create_publisher(
            String, "/project2/sim/assembly_scene/step_ready", 10
        )
        self.error_pub = self.create_publisher(
            String, "/project2/sim/assembly_scene/error", qos
        )
        self.status_pub = self.create_publisher(
            String, "/project2/sim/assembly_scene/status", qos
        )
        self.create_subscription(String, "/project2/sim/state", self.on_state, 10)
        self.create_subscription(
            String,
            "/project2/sim/motion_ready",
            lambda msg: setattr(self, "motion_key", msg.data),
            10,
        )
        self.state = {}
        self.motion_key = ""
        self.phase = "spawning"
        self.actions = deque()
        self.pending = None
        self.pending_action = None
        self.active_key = None
        self.active_shift = 0
        self.finished_key = None
        self.last_status = None
        self.last_status_at = 0.0
        self.last_service_warning = 0.0
        self.actions.append(
            (
                "spawn",
                "toy_tray",
                tray_sdf(self.config, self.contact),
                tray_pose(self.config, min(5, self.count)),
            )
        )
        for (toy, part), position in initial_poses(
            self.config, self.count, 6 if self.qa_trial else 5
        ).items():
            if self.qa_trial or self.production:
                from group4_project2_al.assembly_run import PartPose

                position = PartPose((toy * 0.1, 0, -3))
            name = f"toy_{toy+1}_{part}"
            self.actions.append(
                ("spawn", name, part_sdf(name, part, self.contact), position)
            )
        carrier = self.config["tray"].get("carrier")
        if carrier:
            from group4_project2_al.assembly_run import PartPose

            rows = 5 if self.production else min(5, self.count)
            for row in range(rows):
                name = f"kit_tray_{row+1}"
                sdf = kit_tray_sdf(name, carrier)
                if self.production:
                    # Reset positions only the selected carriers on the feeder.
                    position = PartPose((-5 - row, 0, 0.2))
                else:
                    values = carrier["first_pose"]
                    xyz = tuple(
                        a + row * b for a, b in zip(values[:3], carrier["row_step"])
                    )
                    position = PartPose(xyz, tuple(values[3:]))
                    sdf = sdf.replace(
                        f'<link name="{name}__part">',
                        f'<static>true</static><link name="{name}__part">',
                    )
                self.actions.append(("spawn", name, sdf, position))
        self.model_count = len(self.actions)
        self.create_timer(0.1, self.tick)
        self.get_logger().info(f"Spawning {self.count} toy kit(s) on the angled tray.")

    def on_state(self, message):
        try:
            self.state = json.loads(message.data)
        except json.JSONDecodeError:
            self.fail("Invalid cell state message")

    def fail(self, reason):
        if self.phase == "failed":
            return
        self.phase = "failed"
        self.actions.clear()
        self.error_pub.publish(String(data=reason))
        self.get_logger().error(reason)

    def send(self, action):
        kind, name, sdf, part_pose = action
        if kind == "spawn":
            request = SpawnEntity.Request()
            request.entity_factory.name = name
            request.entity_factory.sdf = sdf
            fill_pose(request.entity_factory.pose, part_pose)
            request.entity_factory.pose.position.z += self.height
            self.pending = self.spawn.call_async(request)
        else:
            request = SetEntityPose.Request()
            request.entity = Entity(name=name, type=Entity.MODEL)
            fill_pose(request.pose, part_pose)
            request.pose.position.z += self.height
            self.pending = self.set_pose.call_async(request)
        self.pending_action = action

    def publish_status(self):
        status = {
            "toys": self.count,
            "phase": self.phase,
            "spawned": (
                self.model_count - len(self.actions) - int(self.pending is not None)
                if self.phase == "spawning"
                else self.model_count
            ),
            "active": self.active_key,
            "finished": self.finished_key,
        }
        message = json.dumps(status, sort_keys=True)
        now = time.monotonic()
        if message != self.last_status or now - self.last_status_at >= 1.0:
            self.status_pub.publish(String(data=message))
            self.last_status = message
            self.last_status_at = now

    def tick(self):
        if self.phase == "failed":
            return
        if not self.spawn.service_is_ready() or not self.set_pose.service_is_ready():
            now = self.get_clock().now().nanoseconds / 1e9
            if now - self.last_service_warning > 10:
                self.get_logger().info(
                    "Waiting for Gazebo create and set_pose services..."
                )
                self.last_service_warning = now
            return
        if self.pending is not None:
            if not self.pending.done():
                return
            try:
                reply = self.pending.result()
                if reply is None or not reply.success:
                    raise RuntimeError("Gazebo returned success=false")
            except Exception as error:
                self.fail(
                    f"{self.pending_action[0]} {self.pending_action[1]} failed: {error}"
                )
                return
            self.pending = None
            self.pending_action = None
        if self.actions:
            if self.actions[0][0] == "move" and self.state.get("state") != "RUNNING":
                return
            self.send(self.actions.popleft())
            self.publish_status()
            return
        if self.phase == "spawning":
            self.phase = "ready"
            self.ready_pub.publish(Bool(data=True))
            self.get_logger().info("Toy kits and angled tray are ready in Gazebo.")
        if self.contact:
            if self.phase == "shifting":
                self.phase = "ready"
                self.get_logger().info(
                    f"Feeder row {self.active_shift} is at the pickup position."
                )
            cycle = self.state.get("cycle")
            if (
                self.phase == "ready"
                and self.state.get("state") == "RUNNING"
                and self.state.get("step") == "pickup_components"
                and isinstance(cycle, int)
                and 1 <= cycle <= self.count
                and cycle != self.active_shift
            ):
                self.active_shift = cycle
                if cycle > 1:
                    front = initial_poses(self.config, self.count)
                    for part in PARTS:
                        self.actions.append(
                            ("move", f"toy_{cycle}_{part}", None, front[(0, part)])
                        )
                    self.phase = "shifting"
                    self.get_logger().info(f"Advancing feeder row {cycle}.")
                    self.publish_status()
                    return
            if self.phase == "ready" and self.active_shift:
                self.cycle_pub.publish(String(data=str(self.active_shift)))
            self.publish_status()
            return
        if self.phase == "moving":
            self.phase = "ready"
            self.finished_key = self.active_key
        key = f"{self.state.get('cycle')}:{self.state.get('step')}"
        if self.finished_key == key:
            self.step_pub.publish(String(data=key))
        elif (
            self.state.get("mode") == "assembly"
            and self.state.get("state") == "RUNNING"
            and self.motion_key == key
            and self.phase == "ready"
            and self.state.get("step") in STEPS
            and 1 <= self.state.get("cycle", 0) <= self.count
        ):
            toy = self.state["cycle"] - 1
            self.active_key = key
            self.phase = "moving"
            for part, position in step_poses(
                self.config, toy, self.state["step"]
            ).items():
                self.actions.append(("move", f"toy_{toy+1}_{part}", None, position))
        self.publish_status()


def main():
    rclpy.init()
    node = AssemblyScene()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
