#!/usr/bin/env python3
# Show the camera feed and return one stable inspection result per request.
import json
import time
from pathlib import Path
from group4_project2_al.connection import read_identity
from group4_project2_al.camera_stream import CameraStream
from group4_project2_al.camera_window import CameraWindow
import cv2
import numpy as np
import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from std_msgs.msg import String, Bool
from sensor_msgs.msg import Image
from tf2_msgs.msg import TFMessage
from scipy.spatial.transform import Rotation
from cv_bridge import CvBridge
from rclpy.qos import qos_profile_sensor_data
from group4_project2_al.vision import (
    stl_template,
    inspect_frame,
    triangles,
    render_parts,
)


class CameraInspection(Node):
    def __init__(self):
        super().__init__("project2_camera_inspection")
        share = Path(get_package_share_directory("group4_project2_al"))
        path = self.declare_parameter(
            "config", str(share / "config/production.yaml")
        ).value
        self.config = yaml.safe_load(Path(path).read_text())["camera"]
        self.show = self.declare_parameter("show_window", True).value
        self.window = CameraWindow()
        self.closed_by_user = False
        identity = read_identity(share)
        self.camera_id = identity["camera"]
        self.source = self.camera_id.get("source", "usb")
        self.cad_camera = self.declare_parameter("cad_camera", False).value
        self.poses = {}
        self.pose_at = 0.0
        if self.cad_camera:
            self.source = "cad"
            self.meshes = {
                part: triangles(share / "models/product" / f"{cad}_centered.stl")
                for part, cad in (
                    ("base_1v4", "1V4"),
                    ("pin_2v4", "2V4"),
                    ("insert_3v4", "3V4"),
                )
            }
            self.create_subscription(
                TFMessage, "/project2/sim/entity_poses", self.on_poses, 10
            )
        default_device = (
            self.camera_id.get("url", "")
            if self.source == "url"
            else self.camera_id["device"]
        )
        self.device = self.declare_parameter("device", default_device).value
        self.frame, self.frame_at = None, 0.0
        self.bridge = CvBridge()
        if self.source == "ros":
            self.create_subscription(
                Image, self.camera_id["topic"], self.on_image, qos_profile_sensor_data
            )
        self.references = {
            name: stl_template(share / "models/product/1_assembled.STL", view["rpy"])
            for name, view in self.config["views"].items()
        }
        self.view = next(iter(self.references))
        self.capture = (
            CameraStream(
                self.device, self.source, self.camera_id.get("stale_seconds", 1.0)
            )
            if self.source in ("usb", "url")
            else None
        )
        self.get_logger().info(f"Camera source: {self.source}. Waiting for frames.")
        self.request = None
        self.started = 0.0
        self.good_frames = 0
        self.completed = {}
        self.status = "Waiting..."
        self.result_pub = self.create_publisher(
            String, "/project2/inspection/result", 10
        )
        self.create_subscription(
            String, "/project2/inspection/request", self.on_request, 10
        )
        self.create_subscription(Bool, "/project2/camera/open", self.set_open, 10)
        self.window_state = self.create_publisher(
            Bool, "/project2/camera/window_open", 10
        )
        self.create_timer(
            0.25,
            lambda: self.window_state.publish(
                Bool(data=self.show and self.window.created and not self.closed_by_user)
            ),
        )
        self.create_timer(0.05, self.tick)

    def set_open(self, message):
        if not message.data:
            self.close_camera()
            return
        if self.capture is None and self.source in ("usb", "url"):
            self.capture = CameraStream(
                self.device, self.source, self.camera_id.get("stale_seconds", 1.0)
            )
        self.frame, self.frame_at = None, 0.0
        self.closed_by_user = False
        self.show = True

    def close_camera(self):
        self.show = False
        self.closed_by_user = True
        self.window.close()
        if self.capture is not None:
            self.capture.release()
            self.capture = None
        self.frame, self.frame_at = None, 0.0
        if self.request:
            self.finish(False, False, "Camera closed by operator")
        self.get_logger().info(
            "Camera closed. Use Open camera in the control panel to reconnect."
        )

    def on_poses(self, message):
        self.pose_at = time.monotonic()
        for item in message.transforms:
            if item.child_frame_id.startswith("toy_"):
                t, q = item.transform.translation, item.transform.rotation
                pose = np.eye(4)
                pose[:3, 3] = [t.x, t.y, t.z]
                pose[:3, :3] = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
                self.poses[item.child_frame_id] = pose

    def cad_frame(self):
        if not self.request:
            return True, np.full((480, 800, 3), 30, np.uint8)
        toy = str(self.request["id"]).split(":")[1]
        base = self.poses.get(f"toy_{toy}_base_1v4")
        if base is None or time.monotonic() - self.pose_at > 0.75:
            return False, None
        poses = {
            part: self.poses[f"toy_{toy}_{part}"]
            for part in self.meshes
            if f"toy_{toy}_{part}" in self.poses
        }
        return True, render_parts(
            self.meshes, poses, base, self.config["views"][self.view]["rpy"]
        )

    def on_image(self, message):
        if self.closed_by_user:
            return
        try:
            self.frame = self.bridge.imgmsg_to_cv2(message, desired_encoding="bgr8")
            self.frame_at = time.monotonic()
        except Exception as error:
            self.get_logger().warning(
                "Camera frame could not be decoded: " + str(error)
            )

    def on_request(self, message):
        try:
            request = json.loads(message.data)
            key = str(request["id"])
        except (ValueError, KeyError, TypeError):
            return
        if key in self.completed:
            self.result_pub.publish(String(data=json.dumps(self.completed[key])))
        elif self.closed_by_user:
            # Closing the camera must never turn into an automatic inspection pass.
            self.request = request
            self.finish(False, False, "Camera closed by operator")
        elif not self.request or str(self.request["id"]) != key:
            view = request.get("view", "top")
            if view not in self.references:
                self.get_logger().error("Unknown inspection view: " + str(view))
                return
            self.view = view
            self.request, self.started = request, time.monotonic()
            self.good_frames, self.status = 0, "Verifying..."

    def finish(self, passed, bypass, reason):
        result = {
            "id": self.request["id"],
            "pass": passed,
            "bypassed": bypass,
            "reason": reason,
        }
        self.completed[str(result["id"])] = result
        self.result_pub.publish(String(data=json.dumps(result)))
        self.status = (
            "Pass - no camera bypass" if bypass else ("Pass" if passed else "Fail")
        )
        self.get_logger().info(f"{self.status}: {reason}")
        self.request = None
        self.result_until = time.monotonic() + 2.0

    def tick(self):
        if self.closed_by_user:
            return
        now = time.monotonic()
        if self.request is None and now >= getattr(self, "result_until", 0):
            self.status = "Waiting..."
        if self.source == "cad":
            ok, image = self.cad_frame()
        elif self.source == "ros":
            ok = self.frame is not None and now - self.frame_at < self.camera_id.get(
                "stale_seconds", 1.0
            )
            image = self.frame.copy() if ok else None
        else:
            ok, image = (
                self.capture.read() if self.capture is not None else (False, None)
            )
        if not ok:
            image = np.zeros((480, 800, 3), np.uint8)
            cv2.putText(
                image,
                "No camera connected",
                (170, 230),
                cv2.FONT_HERSHEY_SIMPLEX,
                1,
                (255, 255, 255),
                2,
            )
            if self.request and now - self.started > 1.0:
                bypass = self.config["allow_no_camera"] and not self.cad_camera
                self.finish(bool(bypass), bool(bypass), "No camera connected")
        else:
            view_config = dict(
                self.config, match_limit=self.config["views"][self.view]["match_limit"]
            )
            image, score = inspect_frame(image, self.references[self.view], view_config)
            if self.request:
                self.good_frames = (
                    self.good_frames + 1
                    if score is not None and score <= view_config["match_limit"]
                    else 0
                )
                if self.good_frames >= self.config["stable_frames"]:
                    self.finish(True, False, "STL silhouette matched")
                elif now - self.started > self.config["timeout_s"]:
                    self.finish(False, False, "No stable STL silhouette match")
        colour = (
            (0, 220, 0)
            if self.status.startswith("Pass")
            else ((0, 0, 240) if self.status == "Fail" else (255, 255, 255))
        )
        cv2.putText(
            image, self.status, (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.8, colour, 2
        )
        if self.cad_camera:
            cv2.putText(
                image,
                "SIMULATED CAD CAMERA",
                (20, image.shape[0] - 55),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (90, 90, 90),
                2,
            )
        if self.show:
            if not self.window.draw(image):
                self.close_camera()


def main():
    rclpy.init()
    node = CameraInspection()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.capture is not None:
            node.capture.release()
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
