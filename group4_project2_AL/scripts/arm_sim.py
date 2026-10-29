#!/usr/bin/env python3
# RViz controls for simulated arms; feedback comes from Gazebo when selected.
import json
import math
import time
from pathlib import Path
import numpy as np
import yaml
from scipy.spatial.transform import Rotation
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import String
from std_srvs.srv import SetBool, Trigger
from visualization_msgs.msg import (
    Marker,
    MarkerArray,
    InteractiveMarker,
    InteractiveMarkerControl as Control,
    InteractiveMarkerFeedback as Feedback,
)
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from interactive_markers.menu_handler import MenuHandler
from ament_index_python.packages import get_package_share_directory
from group4_project2_al.arm_model import (
    ArmModel,
    JOINTS,
    HOME,
    BASES,
    transform,
    step_targets,
)
from group4_project2_al.waypoints import load_waypoints


def pose(frame):
    result = Pose()
    result.position.x, result.position.y, result.position.z = map(float, frame[:3, 3])
    q = Rotation.from_matrix(frame[:3, :3]).as_quat()
    (
        result.orientation.x,
        result.orientation.y,
        result.orientation.z,
        result.orientation.w,
    ) = map(float, q)
    return result


class ArmSimulation(Node):
    def __init__(self):
        super().__init__("project2_arm_sim")
        share = Path(get_package_share_directory("group4_project2_al"))
        self.gazebo = self.declare_parameter("backend", "kinematic").value == "gazebo"
        self.model = ArmModel(
            Path(get_package_share_directory("group4_assembly_line"))
            / "urdf/so101.urdf.xacro"
        )
        self.process = self.declare_parameter("control", "manual").value == "process"
        self.external = self.get_parameter("control").value == "external"
        self.require_io = self.declare_parameter("require_io", False).value
        self.follow = (
            self.declare_parameter("master_slave", True).value and not self.process
        )
        self.current = np.tile(HOME, (4, 1))
        self.goals = self.current.copy()
        self.bases = [transform((x, y, 0), (0, 0, yaw)) for x, y, yaw in BASES]
        self.feedback_time = np.zeros(4)
        self.offsets = np.zeros(6)
        self.names = ("assembly_left", "assembly_right", "inspection", "packaging")
        self.trajectory_pubs = []
        self.last_targets = np.full((4, 6), np.nan)
        self.last_trajectory_time = np.zeros(4)
        self.task_key = None
        self.waypoint_index = 0
        self.arrived_at = None
        if self.gazebo:
            rig = Path(get_package_share_directory("group4_assembly_line"))
            self.model = ArmModel(rig / "urdf/so101.urdf.xacro")
            layout = yaml.safe_load((rig / "config/rig_layout.yaml").read_text())
            self.bases = [
                transform(layout["robots"][n]["xyz"], layout["robots"][n]["rpy"])
                for n in self.names
            ]
            mapping = yaml.safe_load((rig / "config/hardware.yaml").read_text())[
                "simulation"
            ]
            self.offsets = np.array([mapping["joint_offsets"][n] for n in JOINTS])
            self.current[:] = 0.0
            self.goals[:] = 0.0
            for arm, name in enumerate(self.names):
                self.create_subscription(
                    JointState,
                    f"/{name}/joint_state_broadcaster/joint_states",
                    lambda msg, i=arm: self.gazebo_state(i, msg),
                    10,
                )
                self.trajectory_pubs.append(
                    [
                        self.create_publisher(
                            JointTrajectory,
                            f"/{name}/{controller}/joint_trajectory",
                            10,
                        )
                        for controller in ("arm_controller", "gripper_controller")
                    ]
                )
        height = self.declare_parameter("table_height", 0.0).value
        for base in self.bases:
            base[2, 3] += height
        self.sent_at = 0.0
        self.received_initial = np.zeros(4, dtype=bool)
        self.server = InteractiveMarkerServer(self, "/project2/sim/controls")
        self.menu = MenuHandler()
        self.menu.insert("Toggle master/slave", callback=self.toggle_follow)
        self.menu.insert("Open / close gripper", callback=self.gripper)
        self.menu.insert("Home all arms", callback=self.home)
        self.joint_pubs = [
            self.create_publisher(
                JointState, f"/project2/sim/arm{i+1}/joint_states", 10
            )
            for i in range(4)
        ]
        for i in range(4):
            self.create_subscription(
                JointState,
                f"/project2/sim/arm{i+1}/command",
                lambda msg, arm=i: self.command(arm, msg),
                10,
            )
        self.status = self.create_publisher(String, "/project2/sim/arm_status", 10)
        self.ready = self.create_publisher(String, "/project2/sim/motion_ready", 10)
        self.scene = self.create_publisher(MarkerArray, "/project2/sim/scene", 10)
        self.create_service(SetBool, "/project2/sim/master_slave", self.set_follow)
        self.create_service(Trigger, "/project2/sim/home", self.home_service)
        self.create_subscription(String, "/project2/sim/state", self.cell_state, 10)
        default_plan = (
            share
            / "config"
            / ("task_waypoints.yaml" if self.gazebo else "kinematic_waypoints.yaml")
        )
        plan_path = self.declare_parameter("waypoints_file", str(default_plan)).value
        self.plan = load_waypoints(
            plan_path,
            self.model.limits,
            "gazebo_native" if self.gazebo else "reference_urdf",
        )
        self.cell = {}
        self.cell_time = 0.0
        self.drag = None
        self.reason = ""
        self.last = time.monotonic()
        self.scene_time = 0.0
        self.controls_time = 0.0
        self.controls_q = np.full((4, 6), np.nan)
        self.controls_dirty = False
        self.build_controls()
        self.create_timer(0.04, self.tick)
        self.get_logger().info(
            "RViz simulation ready. Drag arm1 arrows; right-click for master/slave and gripper."
        )

    def set_follow(self, request, response):
        if self.process or self.external:
            response.success, response.message = (
                False,
                "Use control:=manual for master/slave tests.",
            )
        else:
            self.follow = request.data
            if self.follow:
                self.goals[1:] = self.goals[0]
            else:
                self.goals[1:] = self.current[1:]
            self.build_controls()
            response.success, response.message = True, f"Master/slave: {self.follow}"
        return response

    def toggle_follow(self, feedback):
        self.set_follow(SetBool.Request(data=not self.follow), SetBool.Response())

    def home(self, _=None):
        if not self.process and not self.external:
            self.goals[:] = HOME - self.offsets

    def home_service(self, request, response):
        self.home()
        response.success = not self.process and not self.external
        response.message = (
            "Home requested"
            if response.success
            else "Stop process mode before manual control."
        )
        return response

    def gripper(self, feedback):
        arm = int(feedback.marker_name.split("_")[0][3:]) - 1
        if not self.process and not self.external and (arm == 0 or not self.follow):
            logical = self.goals[arm, 5] + self.offsets[5]
            self.goals[arm, 5] = (0.0 if logical > 0.25 else 0.5) - self.offsets[5]

    def command(self, arm, message):
        if self.process or self.external or (self.follow and arm != 0):
            return
        if message.name != JOINTS or len(message.position) != 6:
            self.reason = (
                "Joint command rejected: expected six named joints in model order."
            )
            return
        q = np.asarray(message.position) - self.offsets
        if (
            not np.isfinite(q).all()
            or np.any(q < self.model.limits[:, 0])
            or np.any(q > self.model.limits[:, 1])
        ):
            self.reason = "Joint command rejected: nonfinite value or joint limit."
            return
        self.goals[arm] = q

    def cell_state(self, message):
        self.cell = json.loads(message.data)
        self.cell_time = time.monotonic()
        key = (self.cell.get("cycle"), self.cell.get("step"))
        if self.process and key != self.task_key:
            self.task_key, self.waypoint_index, self.arrived_at = key, 0, None
            entries = self.plan.get(self.cell["step"])
            if entries:
                self.goals = entries[0][0].copy()

    def gazebo_state(self, arm, message):
        values = dict(zip(message.name, message.position))
        if not all(n in values and math.isfinite(values[n]) for n in JOINTS):
            return
        self.current[arm] = [values[n] for n in JOINTS]
        self.feedback_time[arm] = time.monotonic()
        if not self.received_initial[arm]:
            self.goals[arm] = self.current[arm]
            self.received_initial[arm] = True

    def build_controls(self):
        self.server.clear()
        self.controls_q[:] = np.nan
        for arm in range(4):
            if self.process or self.external or (self.follow and arm != 0):
                continue
            frame, joints = self.model.fk(self.current[arm])
            marker = InteractiveMarker()
            marker.header.frame_id = "world"
            marker.name = f"arm{arm+1}_tool"
            marker.description = f"Arm {arm+1}: drag arrows"
            marker.scale = 0.14
            marker.pose = pose(self.bases[arm] @ frame)
            # RViz controls act along each control's local X axis.
            for name, xyz in [
                ("x", (1.0, 0.0, 0.0)),
                ("y", (0.0, 0.0, 1.0)),
                ("z", (0.0, 1.0, 0.0)),
            ]:
                control = Control(
                    name=name,
                    interaction_mode=Control.MOVE_AXIS,
                    orientation_mode=Control.FIXED,
                )
                control.orientation.w = math.sqrt(0.5)
                control.orientation.x, control.orientation.y, control.orientation.z = [
                    v * math.sqrt(0.5) for v in xyz
                ]
                marker.controls.append(control)
            self.server.insert(marker, feedback_callback=self.feedback)
            self.menu.apply(self.server, marker.name)
            # Small rotation handles control all six joints, including the jaw.
            for j, origin in enumerate(joints):
                joint_marker = InteractiveMarker()
                joint_marker.header.frame_id = "world"
                joint_marker.name = f"arm{arm+1}_joint{j}"
                joint_marker.description = JOINTS[j]
                joint_marker.scale = 0.065
                joint_marker.pose = pose(self.bases[arm] @ origin)
                control = Control(name="rotate", interaction_mode=Control.ROTATE_AXIS)
                axis = self.model.axes[j]
                cross = np.cross([1.0, 0.0, 0.0], axis)
                angle = math.acos(float(np.clip(axis[0], -1.0, 1.0)))
                rotation_axis = (
                    cross / np.linalg.norm(cross)
                    if np.linalg.norm(cross) > 1e-6
                    else np.array([0.0, 1.0, 0.0])
                )
                q = Rotation.from_rotvec(rotation_axis * angle).as_quat()
                (
                    control.orientation.x,
                    control.orientation.y,
                    control.orientation.z,
                    control.orientation.w,
                ) = map(float, q)
                joint_marker.controls.append(control)
                self.server.insert(joint_marker, feedback_callback=self.feedback)
        self.server.applyChanges()

    def feedback(self, message):
        if self.process or self.external:
            return
        arm = int(message.marker_name.split("_")[0][3:]) - 1
        if self.follow and arm != 0:
            return
        if message.event_type == Feedback.MOUSE_DOWN:
            self.drag = message.marker_name
        if message.event_type == Feedback.POSE_UPDATE:
            if "_tool" in message.marker_name:
                p = message.pose.position
                world = np.array([p.x, p.y, p.z, 1.0])
                target = (np.linalg.inv(self.bases[arm]) @ world)[:3]
                result = self.model.solve(target, self.goals[arm])
                if result is None:
                    self.reason = "Target outside reach; keeping the last valid pose."
                else:
                    self.goals[arm] = result
                    self.reason = ""
            else:
                j = int(message.marker_name.rsplit("joint", 1)[1])
                _, frames = self.model.fk(self.current[arm])
                q = message.pose.orientation
                quat = np.array([q.x, q.y, q.z, q.w])
                if not np.isfinite(quat).all() or np.linalg.norm(quat) < 0.1:
                    return
                basis = (self.bases[arm] @ frames[j])[:3, :3]
                rotation = basis.T @ Rotation.from_quat(quat).as_matrix()
                angle = float(
                    Rotation.from_matrix(rotation).as_rotvec() @ self.model.axes[j]
                )
                self.goals[arm, j] = np.clip(angle, *self.model.limits[j])
        if message.event_type == Feedback.MOUSE_UP:
            self.drag = None
            self.controls_dirty = True

    def tick(self):
        now = time.monotonic()
        dt = min(now - self.last, 0.1)
        self.last = now
        if self.follow:
            self.goals[1:] = self.goals[0]
        active = (
            not (self.process or self.require_io) or self.cell.get("state") == "RUNNING"
        )
        if self.require_io and (
            not self.cell.get("io_connected", False) or now - self.cell_time > 0.75
        ):
            active = False
        if not active and not self.process:
            self.goals[:] = self.current
        if active and not self.gazebo:
            self.current += np.clip(self.goals - self.current, -dt, dt)
        if self.follow and not self.gazebo:
            self.current[1:] = self.current[0]
        if self.gazebo and not self.external and now - self.sent_at >= 0.15:
            self.sent_at = now
            # Gazebo feedback is authoritative. Never replace it with desired angles.
            if np.all(now - self.feedback_time < 2.0):
                if self.follow:
                    self.goals[1:] = self.current[0]
                for arm in range(4):
                    target = self.goals[arm] if active else self.current[arm]
                    # Repeating the same trajectory resets its start time.
                    if (
                        np.max(np.abs(target - self.last_targets[arm])) < 0.001
                        and now - self.last_trajectory_time[arm] < 5.0
                    ):
                        continue
                    self.last_targets[arm] = target
                    self.last_trajectory_time[arm] = now
                    seconds = max(
                        0.18, float(np.max(np.abs(target - self.current[arm])))
                    )
                    for pub, sl in zip(
                        self.trajectory_pubs[arm], (slice(0, 5), slice(5, 6))
                    ):
                        trajectory = JointTrajectory()
                        trajectory.joint_names = JOINTS[sl]
                        point = JointTrajectoryPoint(positions=target[sl].tolist())
                        point.time_from_start.sec = int(seconds)
                        point.time_from_start.nanosec = int(
                            (seconds - int(seconds)) * 1e9
                        )
                        trajectory.points = [point]
                        pub.publish(trajectory)
        stamp = self.get_clock().now().to_msg()
        refresh_controls = now - self.controls_time >= 0.2 and (
            self.controls_dirty
            or not np.isfinite(self.controls_q).all()
            or np.any(np.abs(self.current - self.controls_q) > 0.002)
        )
        for arm in range(4):
            msg = JointState()
            msg.header.stamp = stamp
            msg.name = [f"arm{arm+1}_" + n for n in JOINTS]
            msg.position = self.current[arm].tolist()
            if not self.gazebo:
                self.joint_pubs[arm].publish(msg)
            if not refresh_controls:
                continue
            frame, joints = self.model.fk(self.current[arm])
            name = f"arm{arm+1}_tool"
            if name != self.drag:
                self.server.setPose(name, pose(self.bases[arm] @ frame))
            for j, origin in enumerate(joints):
                name = f"arm{arm+1}_joint{j}"
                if name != self.drag:
                    # Include this joint's rotation so the ring reflects its angle.
                    rot = np.eye(4)
                    rot[:3, :3] = Rotation.from_rotvec(
                        self.model.axes[j] * self.current[arm, j]
                    ).as_matrix()
                    self.server.setPose(name, pose(self.bases[arm] @ origin @ rot))
        if refresh_controls:
            # Avoid flooding RViz with unchanged poses; its client tracks update sequence numbers.
            self.server.applyChanges()
            self.controls_q[:] = self.current
            self.controls_time = now
            self.controls_dirty = False
        fresh = not self.gazebo or np.all(now - self.feedback_time < 2.0)
        reached = fresh and np.max(np.abs(self.current - self.goals)) < 0.025
        if self.process and active and reached and self.cell.get("step") in self.plan:
            entries = self.plan[self.cell["step"]]
            if self.arrived_at is None:
                self.arrived_at = now
            if now - self.arrived_at >= entries[self.waypoint_index][1]:
                if self.waypoint_index + 1 < len(entries):
                    self.waypoint_index += 1
                    self.arrived_at = None
                    self.goals = entries[self.waypoint_index][0].copy()
                else:
                    self.ready.publish(
                        String(data=f"{self.cell['cycle']}:{self.cell['step']}")
                    )
        else:
            self.arrived_at = None
        if now - self.scene_time > 0.2:
            self.scene_time = now
            if not self.gazebo:
                self.draw_scene()
            self.status.publish(
                String(
                    data=json.dumps(
                        {
                            "simulation": True,
                            "backend": "gazebo" if self.gazebo else "kinematic",
                            "feedback_ready": bool(fresh),
                            "master_slave": self.follow,
                            "control": (
                                "external"
                                if self.external
                                else ("process" if self.process else "manual")
                            ),
                            "reason": self.reason,
                            "waypoint": self.waypoint_index,
                            "max_joint_error": float(
                                np.max(np.abs(self.current - self.goals))
                            ),
                        }
                    )
                )
            )

    def draw_scene(self):
        array = MarkerArray()

        def add(kind, xyz, size, colour, text=""):
            m = Marker()
            m.header.frame_id = "world"
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns, m.id, m.type = "cell", len(array.markers), kind
            m.pose.orientation.w = 1.0
            m.pose.position.x, m.pose.position.y, m.pose.position.z = map(float, xyz)
            m.scale.x, m.scale.y, m.scale.z = map(float, size)
            m.color.r, m.color.g, m.color.b, m.color.a = map(float, colour)
            m.text = text
            array.markers.append(m)

        add(Marker.CUBE, (0, 0, -0.035), (1.2, 0.6, 0.05), (0.25, 0.28, 0.32, 1))
        add(Marker.CUBE, (0, 0, 0.005), (1.05, 0.10, 0.02), (0.12, 0.14, 0.16, 1))
        for i, (x, y, _) in enumerate(BASES):
            add(
                Marker.TEXT_VIEW_FACING,
                (x, y, 0.42),
                (0, 0, 0.035),
                (0.95, 0.95, 0.95, 1),
                f"Arm {i+1}",
            )
        colour = {
            "red": (1.0, 0.1, 0.1, 1),
            "yellow": (1.0, 0.75, 0.0, 1),
            "green": (0.1, 1.0, 0.25, 1),
        }.get(self.cell.get("tower", "green"))
        add(Marker.SPHERE, (0.55, -0.25, 0.25), (0.04, 0.04, 0.04), colour)
        state = self.cell.get("state", "MANUAL")
        label = "MASTER / SLAVE: arm1 leads" if self.follow else state
        add(Marker.TEXT_VIEW_FACING, (0, 0, 0.55), (0, 0, 0.035), (1, 1, 1, 1), label)
        # Tokens illustrate material flow; these are not contact/grasp physics.
        step = self.cell.get("step", "")
        x = -0.4 if "assembly" in step or "pickup_components" == step else 0.0
        if "pack" in step or "box" in step or "lid" in step:
            x = 0.35
        if step == "reject_product":
            x = 0.1
        add(
            Marker.CUBE,
            (x, 0.16 if step == "reject_product" else 0, 0.05),
            (0.04, 0.06, 0.02),
            (0.95, 0.6, 0.1, 1 if self.process else 0),
        )
        for i in range(5):
            visible = i < self.cell.get("packed", 0)
            add(
                Marker.CUBE,
                (0.48, -0.10, 0.03 + i * 0.025),
                (0.045, 0.09, 0.02),
                (0.45, 0.65, 0.9, 1 if visible else 0),
            )
        self.scene.publish(array)


def main():
    rclpy.init()
    node = ArmSimulation()
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
