#!/usr/bin/env python3
# Draw only the working parts and packages, using measured Gazebo model poses.
from pathlib import Path
from copy import deepcopy
import numpy as np
from scipy.spatial.transform import Rotation
from group4_project2_al.kit_trays import segments
import re
import yaml
import rclpy
from rclpy.node import Node
from tf2_msgs.msg import TFMessage
from visualization_msgs.msg import Marker, MarkerArray
from ament_index_python.packages import get_package_share_directory


class SceneMarkers(Node):
    def __init__(self):
        super().__init__("project2_scene_markers")
        self.share = Path(get_package_share_directory("group4_project2_al"))
        self.config = yaml.safe_load(
            (self.share / "config/production.yaml").read_text()
        )
        self.carrier = yaml.safe_load(
            (self.share / "config/assembly_run.yaml").read_text()
        )["tray"].get("carrier")
        self.pub = self.create_publisher(MarkerArray, "/project2/working_parts", 1)
        self.create_subscription(
            TFMessage, "/project2/sim/entity_poses", self.receive, 1
        )
        self.ids = {}
        self.cache = {}
        self.clear_first = True
        self.create_timer(0.033, self.publish)

    def receive(self, message):
        markers = MarkerArray()
        for pose in message.transforms:
            name = pose.child_frame_id
            if not re.fullmatch(
                r"(toy_[1-6]_(base_1v4|insert_3v4|pin_2v4)|box_[1-6]|lid_[1-6]|kit_tray_[1-5])",
                name,
            ):
                continue
            index = self.ids.setdefault(name, len(self.ids) * 8)
            m = Marker()
            m.header.frame_id = "world"
            m.ns = "working_parts"
            m.id = index
            m.pose.position.x = pose.transform.translation.x
            m.pose.position.y = pose.transform.translation.y
            m.pose.position.z = pose.transform.translation.z
            m.pose.orientation = pose.transform.rotation
            m.action = Marker.ADD
            m.scale.x = m.scale.y = m.scale.z = 1.0
            if name.startswith("kit_tray_"):
                q = pose.transform.rotation
                rotation = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
                origin = np.array(
                    [m.pose.position.x, m.pose.position.y, m.pose.position.z]
                )
                for offset, (xyz, size) in enumerate(segments(self.carrier)):
                    piece = deepcopy(m)
                    piece.id = index + offset
                    piece.type = Marker.CUBE
                    point = origin + rotation @ xyz
                    (
                        piece.pose.position.x,
                        piece.pose.position.y,
                        piece.pose.position.z,
                    ) = map(float, point)
                    piece.scale.x, piece.scale.y, piece.scale.z = map(float, size)
                    piece.color.r, piece.color.g, piece.color.b, piece.color.a = (
                        0.86,
                        0.87,
                        0.81,
                        1.0,
                    )
                    markers.markers.append(piece)
            elif name.startswith("toy_"):
                tag = name.split("_", 2)[2]
                mesh, colour = {
                    "base_1v4": ("1V4", (0.18, 0.58, 0.86)),
                    "insert_3v4": ("3V4", (0.95, 0.47, 0.16)),
                    "pin_2v4": ("2V4", (0.95, 0.78, 0.15)),
                }[tag]
                m.type = Marker.MESH_RESOURCE
                m.mesh_resource = (
                    self.share / "models/product" / f"{mesh}_centered.stl"
                ).as_uri()
                m.color.r, m.color.g, m.color.b = colour
                m.color.a = 1.0
                markers.markers.append(m)
            else:
                mesh = "lid" if name.startswith("lid_") else "box"
                m.type = Marker.MESH_RESOURCE
                m.mesh_resource = (
                    self.share / "models/packaging" / f"{mesh}.stl"
                ).as_uri()
                m.color.r, m.color.g, m.color.b, m.color.a = 0.65, 0.45, 0.24, 1.0
                markers.markers.append(m)
        for marker in markers.markers:
            self.cache[marker.id] = marker

    def publish(self):
        if not self.cache:
            return
        # Replace existing marker poses instead of deleting/reloading every mesh.
        # Queue depth one keeps RViz on the newest measured scene snapshot.
        markers = list(self.cache.values())
        if self.clear_first:
            markers.insert(0, Marker(action=Marker.DELETEALL))
            self.clear_first = False
        self.pub.publish(MarkerArray(markers=markers))


def main():
    rclpy.init()
    node = SceneMarkers()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
