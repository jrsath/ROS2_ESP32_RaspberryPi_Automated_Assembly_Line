#!/usr/bin/env python3
# Publish the supplied CAD rig in RViz at its measured mesh positions.
from pathlib import Path
import math
import yaml
import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from visualization_msgs.msg import Marker, MarkerArray


def quat_from_rpy(roll, pitch, yaw):
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


class RigMarkers(Node):
    def __init__(self):
        super().__init__("project2_rig_markers")
        share = Path(get_package_share_directory("group4_assembly_line"))
        with open(share / "config/rig_layout.yaml", encoding="utf-8") as f:
            self.layout = yaml.safe_load(f)
        self.mesh_root = share / "models/project2_rig/meshes"
        product_dir = self.declare_parameter("product_mesh_dir", "").value
        self.product_root = Path(product_dir) if product_dir else None
        self.relocate = self.declare_parameter("relocate_controls", False).value
        self.height = self.declare_parameter("table_height", 0.0).value
        self.show_boxes = self.declare_parameter("show_reference_boxes", True).value
        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.pub = self.create_publisher(MarkerArray, "/project2/rig_markers", qos)
        self.create_timer(1.0, self.publish)
        self.publish()

    def add(self, arr, idx, name, mesh, xyz, rpy, color):
        m = Marker()
        m.header.frame_id = "world"
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = "project2_cad"
        m.id = idx
        m.type = Marker.MESH_RESOURCE
        m.action = Marker.ADD
        m.pose.position.x, m.pose.position.y, m.pose.position.z = [
            float(v) for v in xyz
        ]
        m.pose.position.z += self.height
        q = quat_from_rpy(*[float(v) for v in rpy])
        (
            m.pose.orientation.x,
            m.pose.orientation.y,
            m.pose.orientation.z,
            m.pose.orientation.w,
        ) = q
        m.scale.x = m.scale.y = m.scale.z = 1.0
        m.color.r, m.color.g, m.color.b, m.color.a = color
        m.mesh_resource = "file://" + str(self.mesh_root / mesh)
        m.mesh_use_embedded_materials = False
        m.text = name
        m.lifetime = Duration(seconds=0).to_msg()
        arr.markers.append(m)

    def publish(self):
        arr = MarkerArray(markers=[Marker(action=Marker.DELETEALL)])
        idx = 0
        for item in self.layout["static_parts"]:
            if self.relocate and item["name"] in ("EStop w buttons", "EStop Mount"):
                continue
            self.add(
                arr,
                idx,
                item["name"],
                item["mesh"],
                item["xyz"],
                item["rpy"],
                (0.55, 0.56, 0.58, 1.0),
            )
            idx += 1
        for item in self.layout["boxes"] if self.show_boxes else []:
            self.add(
                arr,
                idx,
                item["name"],
                item["mesh"],
                item["xyz"],
                item["rpy"],
                (0.72, 0.72, 0.70, 1.0),
            )
            idx += 1
        if self.product_root and self.product_root.is_dir():
            for name, mesh, position, colour in (
                (
                    "1V4",
                    "1V4_object1.stl",
                    (-0.55, 0.40, 0.043),
                    (0.18, 0.58, 0.86, 1.0),
                ),
                (
                    "3V4",
                    "3V4_object2.stl",
                    (-0.44, 0.40, 0.042),
                    (0.95, 0.47, 0.16, 1.0),
                ),
            ):
                self.add(
                    arr,
                    idx,
                    name,
                    str(self.product_root / mesh),
                    position,
                    (0, 0, 0),
                    colour,
                )
                idx += 1
            # 2V4 is a labelled proxy until its SolidWorks part has a mesh export.
            proxy = Marker()
            proxy.header.frame_id = "world"
            proxy.header.stamp = self.get_clock().now().to_msg()
            proxy.ns, proxy.id, proxy.type, proxy.action = (
                "project2_product",
                idx,
                Marker.CYLINDER,
                Marker.ADD,
            )
            proxy.pose.position.x, proxy.pose.position.y, proxy.pose.position.z = (
                -0.34,
                0.40,
                0.040,
            )
            proxy.pose.orientation.w = 1.0
            proxy.scale.x, proxy.scale.y, proxy.scale.z = 0.020, 0.020, 0.007
            proxy.color.r, proxy.color.g, proxy.color.b, proxy.color.a = (
                0.95,
                0.78,
                0.15,
                1.0,
            )
            proxy.text = "2V4 proxy"
            arr.markers.append(proxy)
        if self.relocate:
            self.add(
                arr,
                idx,
                "Operator controls",
                "EStop_w_buttons.stl",
                (0, -0.15, -0.07),
                (math.pi / 2, 0, 0),
                (0.3, 0.3, 0.31, 1.0),
            )
            idx += 1
        if self.height > 0:
            pieces = [((0, 0.28, self.height - 0.025), (1.8, 0.85, 0.05))]
            pieces += [
                ((x, y, (self.height - 0.05) / 2), (0.055, 0.055, self.height - 0.05))
                for x in (-0.82, 0.82)
                for y in (-0.08, 0.64)
            ]
            for i, (xyz, size) in enumerate(pieces):
                m = Marker()
                m.header.frame_id = "world"
                m.ns = "work_table"
                m.id = i
                m.type = Marker.CUBE
                m.pose.position.x, m.pose.position.y, m.pose.position.z = map(
                    float, xyz
                )
                m.pose.orientation.w = 1.0
                m.scale.x, m.scale.y, m.scale.z = map(float, size)
                m.color.r, m.color.g, m.color.b, m.color.a = 0.32, 0.34, 0.36, 1.0
                arr.markers.append(m)
        self.pub.publish(arr)


def main(args=None):
    rclpy.init(args=args)
    node = RigMarkers()
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
