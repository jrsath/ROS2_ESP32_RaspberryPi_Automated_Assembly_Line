#!/usr/bin/env python3
# Three cooperating jobs: assembly, conveyor transfer and inspection/packaging.
import json
import os
import random
import time
from concurrent.futures import ThreadPoolExecutor
from threading import Event
from pathlib import Path
import numpy as np
import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from rclpy.clock import Clock, ClockType
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from ros_gz_interfaces.msg import Entity
from ros_gz_interfaces.srv import SetEntityPose, SpawnEntity
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, String, UInt32MultiArray, UInt64MultiArray
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from group4_project2_al.arm_model import ArmModel, JOINTS, HOME, transform
from group4_project2_al.assembly_run import initial_poses, check_count
from group4_project2_al.production_plan import TransferPlanner, frame, make_plans
from group4_project2_al.kit_trays import (
    tray_pose as carrier_pose,
    faces as carrier_faces,
    mesh_sdf,
)
from group4_project2_al.collision_check import CollisionCheck
from group4_project2_al.production_scene import package_sdf
from group4_project2_al.scene_layout import raised_config
from group4_project2_al.grasp_geometry import check_contacts
from group4_project2_al.defects import make_faults

ARMS = ("assembly_left", "assembly_right", "inspection", "packaging")


class ProductionRun(Node):
    def __init__(self):
        super().__init__("project2_production")
        share = Path(get_package_share_directory("group4_project2_al"))
        rig = Path(get_package_share_directory("group4_assembly_line"))
        self.model_folder = share / "models"
        self.qa_trial = self.declare_parameter("qa_trial", False).value
        self.capacity = 6 if self.qa_trial else 5
        self.count = self.declare_parameter("toys", 0).value
        if self.count != 0:
            check_count(self.count, self.capacity)
        self.task_mode = "full"
        self.requested_mode, self.requested_count = self.task_mode, self.count
        self.requested_faults = dict(count=0, placement="random", indices="")
        self.fault_schedule = {}
        if self.qa_trial and self.count != 6:
            raise ValueError("QA trial needs six toys: four good and two flawed")
        self.package_count = 5
        self.qa_cases = ["good"] * 4 + ["missing_gear", "unseated_insert"]
        random.Random(self.declare_parameter("qa_seed", 4).value).shuffle(self.qa_cases)
        self.require_io = self.declare_parameter("require_io", True).value
        self.start_pending = self.declare_parameter("autostart", False).value
        path = self.declare_parameter(
            "config", str(share / "config/production.yaml")
        ).value
        self.height = self.declare_parameter("table_height", 1.0).value
        self.config_path = Path(path)
        self.config = raised_config(
            yaml.safe_load(self.config_path.read_text()), self.height
        )
        layout = yaml.safe_load((rig / "config/rig_layout.yaml").read_text())
        self.original_boxes = []  # Reference boxes are excluded at launch.
        self.spawned_packages = False
        tray = yaml.safe_load((share / "config/assembly_run.yaml").read_text())
        self.tray_config = tray["tray"].get("carrier")
        if self.tray_config:
            from copy import deepcopy

            self.tray_config = deepcopy(self.tray_config)
            for key in ("first_pose", "discard", "work_pose"):
                if key not in self.tray_config:
                    continue
                self.tray_config[key][2] += self.height
            for point in self.tray_config["pull_points"]:
                point[2] += self.height
            self.config["kit_trays"] = self.tray_config
        self.starts = initial_poses(tray, self.capacity, self.capacity)
        from dataclasses import replace

        self.starts = {
            key: replace(
                pose, xyz=(pose.xyz[0], pose.xyz[1], pose.xyz[2] + self.height)
            )
            for key, pose in self.starts.items()
        }
        starts = {
            part: transform(pose.xyz, pose.rpy)
            for (i, part), pose in self.starts.items()
            if i == 0
        }
        alignment = json.loads(
            (share / "models/product/mesh_alignment.json").read_text()
        )["relative_to_1V4_center"]
        bases = {
            name: transform(data["xyz"], data["rpy"])
            for name, data in layout["robots"].items()
        }
        for base in bases.values():
            base[2, 3] += self.height
        seed_file = share / "config/grip_seeds.json"
        seeds = json.loads(seed_file.read_text()) if seed_file.exists() else {}
        self.planner = TransferPlanner(
            ArmModel(rig / "urdf/so101.urdf.xacro"), bases, seeds
        )
        self.collision = CollisionCheck(self.planner.model, bases)
        # Solve paths in the background so ROS can still receive stop commands.
        self.worker = ThreadPoolExecutor(max_workers=1)
        # Slow grip solving must not hold up Reset or manual clearance checks.
        self.plan_worker = ThreadPoolExecutor(max_workers=1)
        self.plan_starts = starts
        self.plan_error = None
        self.plan_preview = {}
        self.preview_paths = None
        self.plan_cancel = Event()
        self.planning = self.plan_worker.submit(
            make_plans,
            self.planner,
            self.config,
            starts,
            alignment,
            min(self.count, self.package_count),
            self.plan_preview,
            self.plan_cancel,
            allow_partial=True,
        )
        self.alignment = alignment
        self.plans = None
        self.ready = False
        self.feedback, self.feedback_at = {}, {}
        self.hold_targets, self.hold_checks, self.hold_retry = {}, {}, {}
        self.sensor_occupied, self.sensor_at = False, 0.0
        self.create_subscription(Bool, "/project2/sim/proximity", self.on_sensor, 10)
        self.outcomes = {}
        self.arm_pub, self.grip_pub = {}, {}
        for arm in ARMS:
            self.create_subscription(
                JointState,
                f"/{arm}/joint_state_broadcaster/joint_states",
                lambda msg, name=arm: self.on_joint(name, msg),
                10,
            )
            self.arm_pub[arm] = self.create_publisher(
                JointTrajectory, f"/{arm}/arm_controller/joint_trajectory", 10
            )
            self.grip_pub[arm] = self.create_publisher(
                JointTrajectory, f"/{arm}/gripper_controller/joint_trajectory", 10
            )
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(
            Bool,
            "/project2/sim/assembly_scene/ready",
            lambda msg: setattr(self, "ready", msg.data),
            qos,
        )
        self.create_subscription(String, "/project2/grasp/reply", self.on_reply, 10)
        self.create_subscription(
            String, "/project2/inspection/result", self.on_inspection, 10
        )
        self.create_subscription(String, "/project2/sim/event", self.on_event, 10)
        self.create_subscription(
            UInt64MultiArray,
            "/project2/sim/io/state",
            self.on_io,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Bool,
            "/project2/sim/command_link",
            lambda msg: setattr(self, "command_link", msg.data),
            10,
        )
        self.request_pub = self.create_publisher(String, "/project2/grasp/request", 10)
        self.camera_pub = self.create_publisher(
            String, "/project2/inspection/request", 10
        )
        self.state_pub = self.create_publisher(String, "/project2/sim/state", 10)
        self.plan_pub = self.create_publisher(String, "/project2/sim/paths", qos)
        self.io_pub = self.create_publisher(
            UInt32MultiArray, "/project2/sim/io/command", 10
        )
        self.pose_client = self.create_client(SetEntityPose, "/world/empty/set_pose")
        self.spawn_client = self.create_client(SpawnEntity, "/world/empty/create")
        self.io, self.io_at, self.last_start, self.sequence = None, 0.0, False, 0
        self.command_link = True
        self.actuator_power = not self.require_io
        self.reset_pending = False
        self.io_seen_healthy = False
        self.create_subscription(
            Bool,
            "/project2/sim/actuator_power",
            lambda msg: setattr(self, "actuator_power", msg.data),
            10,
        )
        self.request_id = 10000
        self.replies, self.inspections = {}, {}
        self.measured_poses = {}
        self.state, self.reason = "READY", "Planning arm transfers"
        self.tasks = {}
        self.setup_done = False
        self.selected_mode = 0
        self.selector_reset_pending = False
        self.selector_revision = 0
        self.run = 0
        self.cycle, self.completed, self.packed, self.rejected = 1, 0, 0, 0
        # These three values prevent loading a moving belt or overwriting its output.
        self.input_toy, self.output_toy, self.moving = None, None, False
        self.bypassed = 0
        self.last_tick = time.monotonic()
        self.log_path = Path.home() / "ROS2/project2_data/results"
        self.log_path.mkdir(parents=True, exist_ok=True)
        self.log_file = (self.log_path / f"production_{time.time_ns()}.jsonl").open("w")
        # WSL wall-clock corrections must not pause the watchdog heartbeat.
        self.create_timer(
            0.05, self.tick, clock=Clock(clock_type=ClockType.STEADY_TIME)
        )

    def record(self, event, **values):
        self.log_file.write(
            json.dumps({"time": time.time(), "event": event, **values}) + "\n"
        )
        self.log_file.flush()

    def on_joint(self, arm, message):
        values = dict(zip(message.name, message.position))
        if all(name in values for name in JOINTS):
            self.feedback[arm] = np.array([values[name] for name in JOINTS])
            self.feedback_at[arm] = time.monotonic()

    def on_sensor(self, message):
        self.sensor_occupied, self.sensor_at = message.data, time.monotonic()

    # Require both the sensor topic and board feedback to be recent and clear.
    def sensor_clear(self):
        fresh = time.monotonic() - self.sensor_at < 0.75
        return (
            fresh
            and not self.sensor_occupied
            and (
                not self.require_io
                or (
                    self.io is not None
                    and time.monotonic() - self.io_at < 0.75
                    and not self.io[7]
                )
            )
        )

    def on_reply(self, message):
        words = message.data.split()
        if len(words) > 2:
            self.replies[words[0]] = words[1:]

    def on_inspection(self, message):
        try:
            result = json.loads(message.data)
            self.inspections[str(result["id"])] = result
        except (ValueError, KeyError):
            pass

    def on_event(self, message):
        event = message.data.strip()
        if event == "teach":
            if not self.setup_done or self.state == "ESTOP":
                self.reason = "Finish Reset and release E-stop before planning a path"
                return
            self.stop(
                "TEACH",
                "Automatic movement stopped. Edit simulation paths; Reset before running.",
            )
            self.tasks.clear()
            self.hold_targets.clear()
            self.moving = False
            return
        if event.startswith("{"):
            try:
                values = json.loads(event)
                if values.get("teach_move"):
                    self.teach_move(values)
                    return
                mode, count = values["mode"], values["products"]
                if mode not in ("full", "assembly", "inspection", "packaging"):
                    raise ValueError("Unknown task")
                if type(count) is not int or not 0 <= count <= 5 or self.qa_trial:
                    raise ValueError(
                        "Select 1–5 products; the QA trial uses six fixed samples"
                    )
                faults = dict(
                    count=values.get("faults", 0),
                    placement=values.get("fault_position", "random"),
                    indices=values.get("fault_indices", ""),
                )
                make_faults(count, **faults)
                changed = (mode, count, faults) != (
                    self.requested_mode,
                    self.requested_count,
                    getattr(
                        self,
                        "requested_faults",
                        dict(count=0, placement="random", indices=""),
                    ),
                )
                self.settings_dirty = getattr(self, "settings_dirty", False) or changed
                self.requested_mode, self.requested_count = mode, count
                self.requested_faults = faults
                if self.state == "RUNNING":
                    self.stop("PAUSED", "Settings changed: press Reset")
                self.reason = "Settings changed: press Reset"
            except (ValueError, KeyError, TypeError) as error:
                self.reason = str(error)
            return
        if event == "start" and not self.require_io:
            self.start_pending = True
        elif event in ("pause", "fault", "estop") and (
            self.state != "ESTOP" or event == "estop"
        ):
            self.stop(
                {"pause": "PAUSED", "fault": "FAULT", "estop": "ESTOP"}[event], event
            )
        elif event == "reset":
            if (
                not self.setup_done
                and not getattr(self, "plan_error", None)
                and self.tasks
            ):
                if self.state == "ESTOP" and self.io is not None and self.io[5]:
                    self.reset_pending = True
                else:
                    self.reason = "Wait for scene setup before Reset"
                return
            if self.io is not None and not self.io[5]:
                return
            self.stop("READY", "Resetting scene")
            # Reload edits on Reset; preserve the raised carrier configuration.
            carrier = self.config.get("kit_trays")
            try:
                self.config = raised_config(
                    yaml.safe_load(self.config_path.read_text()), self.height
                )
            except (OSError, ValueError, KeyError, TypeError, yaml.YAMLError) as error:
                self.stop("FAULT", "Cannot load waypoints: " + str(error))
                return
            if carrier:
                self.config["kit_trays"] = carrier
            self.task_mode, self.count = self.requested_mode, self.requested_count
            # Solve only the stack slots needed by the selected task.
            # A packaging-plan error must not prevent an assembly-only trial.
            if hasattr(self, "plan_starts"):
                slots = (
                    min(self.count, self.package_count)
                    if self.task_mode in ("full", "packaging")
                    else 0
                )
                self.plans, self.plan_error = None, None
                self.plan_preview = {}
                self.preview_paths = None
                self.plan_pub.publish(String(data="{}"))
                # Drop a queued solve if Reset is pressed again before it starts.
                previous = getattr(self, "planning", None)
                cancellation = getattr(self, "plan_cancel", None)
                if cancellation is not None:
                    cancellation.set()
                if previous is not None:
                    previous.cancel()
                self.plan_cancel = Event()
                self.planning = self.plan_worker.submit(
                    make_plans,
                    self.planner,
                    self.config,
                    self.plan_starts,
                    self.alignment,
                    slots,
                    self.plan_preview,
                    self.plan_cancel,
                    allow_partial=True,
                )
            self.run += 1
            self.fault_schedule = make_faults(
                self.count,
                seed=self.run,
                **getattr(self, "requested_faults", dict(count=0)),
            )
            self.settings_dirty = False
            self.inspections.clear()
            self.outcomes.clear()
            self.hold_targets.clear()
            self.hold_checks.clear()
            self.replies.clear()
            self.measured_poses.clear()
            self.cycle = 1
            self.selected_mode = int(self.io[6]) if self.io is not None else 0
            self.tasks = {"setup": {"generator": self.setup(False)}}
            self.setup_done = False
            self.completed = self.packed = self.rejected = self.bypassed = 0
            self.input_toy = self.output_toy = None
            self.moving = False

    def teach_move(self, values):
        # This node drives Gazebo controllers only. No physical torque-release command.
        arm = values["arm"]
        sequence = values.get("sequence")
        if sequence is None:
            sequence = [values["joints"]]
        qs = np.asarray(sequence, dtype=float)
        if self.state != "TEACH" or self.tasks:
            raise ValueError("Enter Plan a path and wait for the previous move")
        if (
            arm not in ARMS
            or qs.ndim != 2
            or qs.shape[1] != 6
            or not 1 <= len(qs) <= 200
            or not np.isfinite(qs).all()
        ):
            raise ValueError("Select an arm and six finite joint angles")
        if np.any(qs < self.planner.model.limits[:, 0]) or np.any(
            qs > self.planner.model.limits[:, 1]
        ):
            raise ValueError("Requested joints are outside arm limits")
        if len(self.feedback) != 4 or any(
            time.monotonic() - t > 0.75 for t in self.feedback_at.values()
        ):
            raise ValueError("Wait for fresh arm feedback")
        if self.require_io and (
            not self.actuator_power or self.io is None or not self.io[5]
        ):
            raise ValueError("Release E-stop and wait for actuator power")
        self.tasks["teach"] = {
            "generator": iter(
                [("stage", "Manual path planning: " + arm)]
                + [("move", arm, q.copy()) for q in qs]
            )
        }
        self.reason = "Checking path clearance"

    def via(self, name, phase):
        # Extra points run in listed order before the normal approach target.
        for point in self.config.get("path_points", {}).get(name, []):
            if point["phase"] == phase:
                yield ("stage", name + ": " + point["label"])
                yield ("move", self.plans[name]["arm"], np.asarray(point["joints"]))

    def on_io(self, message):
        if len(message.data) != 14:
            return
        previous = self.io
        self.io, self.io_at = list(message.data), time.monotonic()
        if previous is None or previous[7] != self.io[7]:
            self.record(
                "conveyor_sensor", occupied=bool(self.io[7]), motor_hz=int(self.io[9])
            )
        if not self.require_io:
            return
        # A switch edge selects a task. Its steady position does not override the GUI.
        if previous is not None and previous[6] != self.io[6] and not self.qa_trial:
            if self.io[6] in (0, 1):
                self.selected_mode = int(self.io[6])
                self.requested_mode = ("full", "packaging")[self.selected_mode]
                self.selector_reset_pending = True
                self.selector_revision = getattr(self, "selector_revision", 0) + 1
                if self.state == "RUNNING":
                    self.stop(
                        "PAUSED", "Selector changed: resetting the selected section"
                    )
        if not self.io[5]:
            if self.io_seen_healthy:
                self.stop("ESTOP", "Emergency stop")
        else:
            self.io_seen_healthy = True
        if not self.io[5]:
            return
        elif self.io[4] and self.state != "ESTOP":
            self.stop("PAUSED", "Pause pressed")
        elif self.io[12] and self.state == "RUNNING":
            self.stop("FAULT", "Micro-ROS watchdog expired")
        if (
            self.state != "ESTOP"
            and self.io[3]
            and not self.last_start
            and self.io[2]
            and self.io[5]
            and not self.io[4]
            and not self.io[12]
        ):
            self.start_pending = True
            if self.state == "PAUSED" and not self.setup_done:
                self.state = "READY"
        self.last_start = bool(self.io[3])

    # Arm joints and gripper have separate controllers but share one move duration.
    def send_move(self, arm, q, duration=None):
        self.hold_targets[arm] = np.asarray(q).copy()
        seconds = duration or max(
            0.4,
            float(np.max(np.abs(q - self.feedback[arm]))) / self.config["speed_rad_s"],
        )
        for pub, names, values in (
            (self.arm_pub[arm], JOINTS[:5], q[:5]),
            (self.grip_pub[arm], JOINTS[5:], q[5:]),
        ):
            point = JointTrajectoryPoint(positions=list(map(float, values)))
            point.time_from_start.sec = int(seconds)
            point.time_from_start.nanosec = int((seconds % 1) * 1e9)
            pub.publish(JointTrajectory(joint_names=names, points=[point]))

    def block_task(self, name, task, reason):
        affected = (
            ("assembly_left", "assembly_right")
            if name == "assembly"
            else (("inspection", "packaging") if name == "packaging" else ())
        )
        for arm in affected:
            if arm in self.feedback:
                self.send_move(arm, self.feedback[arm].copy(), 0.15)
        if name == "conveyor":
            self.moving = False
        task["blocked"] = reason
        self.reason = f"{name} waiting: {reason}. Edit the path, then Reset."
        self.record("task_blocked", task=name, reason=reason)
        self.get_logger().warning(self.reason)

    def stop(self, state, reason):
        if self.state in ("RUNNING", "TEACH") or (
            "setup" in self.tasks and self.state == "READY"
        ):
            for arm, q in self.feedback.items():
                self.send_move(arm, q, 0.15)
            for task in self.tasks.values():
                if task.get("action", ("",))[0] == "move":
                    task["sent"] = False
            self.record("stopped", state=state, reason=reason)
        self.state, self.reason, self.start_pending = state, reason, False

    @staticmethod
    def set_message_pose(message, pose):
        message.position.x, message.position.y, message.position.z = map(
            float, pose[:3, 3]
        )
        q = Rotation.from_matrix(pose[:3, :3]).as_quat()
        (
            message.orientation.x,
            message.orientation.y,
            message.orientation.z,
            message.orientation.w,
        ) = map(float, q)

    def setup(self, first):
        yield ("joint", "clear", "-", "-", None)
        yield ("delay", 0.3)
        # Hide the reference boxes; the working boxes below have movable links.
        for name in self.original_boxes:
            yield ("pose", name, transform((0, 0, -3)))
        # The supplied V6 assembly jig locates in the existing bench holes.
        if self.config.get("assembly_jig"):
            jig = self.config["assembly_jig"]
            pose = frame(self.config["fixture"]) @ frame(jig["relative_to_base"])
            if self.spawned_packages:
                yield ("pose", "assembly_jig", pose)
            else:
                yield (
                    "spawn",
                    "assembly_jig",
                    mesh_sdf("assembly_jig", jig["mesh"], fixed=True),
                    pose,
                )
        for arm in ARMS:
            yield from self.home(arm)
        for (toy, part), pose in self.starts.items():
            position = (
                transform((toy * 0.1, 0, -3))
                if self.qa_trial
                or toy >= self.count
                or self.task_mode in ("inspection", "packaging")
                else transform(pose.xyz, pose.rpy)
            )
            yield ("pose", f"toy_{toy+1}_{part}", position)
        for toy in range(1, self.package_count + 1):
            for name, lid in (("box", False), ("lid", True)):
                pose = frame(self.config[name + "_feed"])
                pose[:3, 3] += np.asarray(self.config["feeder_steps"][name]) * (toy - 1)
                model = f"{name}_{toy}"
                if toy > self.count or self.task_mode in ("assembly", "inspection"):
                    pose = transform((toy * 0.1, int(lid), -3))
                if not self.spawned_packages:
                    yield ("spawn", model, package_sdf(model, lid=lid), pose)
                else:
                    yield ("pose", model, pose)
        if self.tray_config:
            for row in range(5):
                name = f"kit_tray_{row+1}"
                visible = (
                    row < self.count
                    and not self.qa_trial
                    and self.task_mode in ("full", "assembly")
                )
                pose = (
                    carrier_pose(self.tray_config, row)
                    if visible
                    else transform((-5 - row, 0, 0.2))
                )
                yield ("joint", "freeze", name, "-", None)
                yield ("pose", name, pose)
                if visible and self.tray_config.get("work_pose"):
                    # Hold stock at its feeder stop until the tab grip is checked.
                    yield ("joint", "park", name, "project2_rig", pose)
                    for part in ("base_1v4", "insert_3v4", "pin_2v4"):
                        value = self.starts[(row, part)]
                        relative = np.linalg.inv(pose) @ transform(value.xyz, value.rpy)
                        yield ("joint", "fasten", f"toy_{row+1}_{part}", name, relative)
        self.spawned_packages = True
        self.record(
            "scene_reset",
            toys=self.count,
            boxes=self.package_count,
            lids=self.package_count,
        )
        self.setup_done = True
        self.reason = (
            "Scene reset. Path blocked: " + self.plan_error
            if self.plan_error
            else (
                "Ready: press Start"
                if self.count
                else "Select task and No. of Products, then press Reset"
            )
        )

    def home(self, arm, holding=False):
        # Keep a held part clamped while clearing the shared work area.
        q = np.asarray(self.config.get("home", HOME), dtype=float).copy()
        q[5] = self.feedback[arm][5] if holding else self.planner.open
        yield ("move", arm, q)

    def require_path(self, name):
        # An unavailable later task does not stop valid independent work.
        while name not in self.plans:
            reason = self.plans.get("unavailable", {}).get(
                name, "Capture or solve this path"
            )
            yield ("blocked_path", name, reason)

    def pick(self, name, model):
        yield from self.require_path(name)
        plan = self.plans[name]
        arm = plan["arm"]
        yield from self.via(name, "pickup")
        yield ("stage", f"{name}: pickup")
        for q in (plan["approach"][0], plan["q"][0]):
            q = q.copy()
            q[5] = plan.get("open", self.planner.open)
            yield ("move", arm, q)
        yield ("move", arm, plan["q"][0])
        yield ("joint", "pose", model, "-", None)
        yield ("contact", name, model, arm)
        # A part may be resting on a carrier or seated in the bench fixture.
        yield ("joint", "release", model, "-", None)
        yield ("joint", "grasp", model, arm, None)
        yield ("move", arm, plan["approach"][0])

    def place(
        self, name, model, parent="project2_rig", relative=None, defer_open=False
    ):
        plan = self.plans[name]
        yield from self.via(name, "place")
        yield ("stage", f"{name}: placement")
        yield ("move", plan["arm"], plan["approach"][1])
        yield ("move", plan["arm"], plan["q"][1])
        pose = plan["poses"][1] if relative is None else relative
        yield ("joint", "deposit" if relative is None else "seat", model, parent, pose)
        opening = plan.get("open", self.planner.open)
        # Release the insert with a small jaw opening before withdrawing past arm 1.
        release = min(opening, plan["q"][1][5] + 0.10) if defer_open else opening
        q = plan["q"][1].copy()
        q[5] = release
        yield ("move", plan["arm"], q)
        q = plan["approach"][1].copy()
        q[5] = release
        yield ("move", plan["arm"], q)
        if release != opening and not defer_open:
            q = q.copy()
            q[5] = opening
            yield ("move", plan["arm"], q)

    def qa_feed(self):
        # The rubric's separate QA trial supplies prebuilt samples at inspection.
        # Only geometry is changed here; the camera decides pass or fail.
        for toy, defect in enumerate(self.qa_cases, 1):
            yield ("wait", lambda: self.output_toy is None and self.sensor_clear())
            self.cycle = toy
            base = f"toy_{toy}_base_1v4"
            pose = self.plans["belt_out"]
            yield ("pose", base, pose)
            yield ("joint", "park", base, "project2_rig", pose)
            for part, cad in (("pin_2v4", "2V4"), ("insert_3v4", "3V4")):
                model = f"toy_{toy}_{part}"
                if defect == "missing_gear" and cad == "2V4":
                    continue
                values = self.alignment[cad]
                relative = transform(values["xyz_m"], values["rpy_rad"])
                if defect == "unseated_insert" and cad == "3V4":
                    relative[0, 3] += 0.005
                yield ("pose", model, pose @ relative)
                yield ("joint", "fasten", model, base, relative)
            yield ("delay", 0.2)
            self.output_toy = toy
            self.completed += 1
            self.record("qa_sample", toy=toy, expected=defect)
            yield ("wait", lambda: self.output_toy is None)

    def downstream_feed(self):
        # Simulate loading a prebuilt toy for the conveyor/packaging section trial.
        # On the real rig this load is supplied by the operator, not by Gazebo.
        for toy in range(1, self.count + 1):
            # Hold the finished toy until the stopped belt input is available.
            yield ("wait", lambda: self.input_toy is None and not self.moving)
            self.cycle = toy
            base = f"toy_{toy}_base_1v4"
            pose = self.plans["belt_in"]
            yield ("pose", base, pose)
            yield ("joint", "park", base, "project2_rig", pose)
            for part, cad in (("pin_2v4", "2V4"), ("insert_3v4", "3V4")):
                values = self.alignment[cad]
                relative = transform(values["xyz_m"], values["rpy_rad"])
                model = f"toy_{toy}_{part}"
                yield ("pose", model, pose @ relative)
                yield ("joint", "fasten", model, base, relative)
            yield from self.inject_fault(toy)
            self.completed += 1
            self.input_toy = toy
            self.record("prebuilt_loaded", toy=toy)
            yield ("wait", lambda: self.input_toy is None)

    # Arm 1 handles the gear and stages the insert; arm 2 places and loads the base.
    def assembly(self):
        for toy in range(1, self.count + 1):
            self.cycle = toy
            self.record("assembly_started", toy=toy)
            if self.tray_config:
                yield from self.advance_trays(toy)
                if self.tray_config.get("work_pose"):
                    yield from self.pick("load_carrier", f"kit_tray_{toy}")
                    yield from self.place("load_carrier", f"kit_tray_{toy}")
                    yield from self.home("assembly_left")
            else:
                for part in ("base_1v4", "insert_3v4", "pin_2v4"):
                    pose = self.starts[(0, part)]
                    original = self.starts[(toy - 1, part)]
                    yield (
                        "feed",
                        f"toy_{toy}_{part}",
                        transform(original.xyz, original.rpy),
                        transform(pose.xyz, pose.rpy),
                    )
            base, gear, insert = [
                f"toy_{toy}_{part}" for part in ("base_1v4", "pin_2v4", "insert_3v4")
            ]
            # Clear the shared bench before the other arm places the base.
            yield from self.pick("gear", gear)
            yield from self.home("assembly_left", holding=True)
            yield from self.pick("base", base)
            yield from self.place("base", base)
            yield from self.home("assembly_right")
            relative = (
                np.linalg.inv(self.plans["fixture"]) @ self.plans["gear"]["poses"][1]
            )
            yield from self.place("gear", gear, base, relative)
            yield from self.pick("handoff", insert)
            yield from self.place("handoff", insert)
            yield from self.home("assembly_left")
            yield from self.pick("insert", insert)
            # The fixture holds the base while arm 1 stays clear of the insert path.
            if self.config.get("support", {}).get("mode", "fixture") == "arm":
                q = self.plans["hold"].copy()
                yield ("move", "assembly_left", q)
                yield ("joint", "pose", base, "-", None)
                yield ("contact", "support", base, "assembly_left")
            relative = (
                np.linalg.inv(self.plans["fixture"]) @ self.plans["insert"]["poses"][1]
            )
            yield from self.place("insert", insert, base, relative, defer_open=True)
            if self.config.get("support", {}).get("mode", "fixture") == "arm":
                q = q.copy()
                q[5] = min(1.0, q[5] + 0.15)
                yield ("move", "assembly_left", q)
                yield from self.home("assembly_left")
            # Keep the released insert jaws narrow until both arms clear the fixture.
            yield from self.home("assembly_right", holding=True)
            yield from self.home("assembly_right")
            yield ("wait", lambda: self.input_toy is None and not self.moving)
            yield from self.pick("belt", base)
            yield from self.place("belt", base)
            yield from self.home("assembly_right")
            yield from self.inject_fault(toy)
            self.input_toy = toy
            self.completed += 1
            self.record("assembled", toy=toy)
            if self.tray_config:
                yield from self.remove_tray(toy)
            yield ("wait", lambda: self.input_toy is None)

    def inject_fault(self, toy):
        defect = self.fault_schedule.get(toy)
        if defect is None:
            return
        base = f"toy_{toy}_base_1v4"
        yield ("joint", "pose", base, "-", None)
        if defect == "missing_gear":
            model = f"toy_{toy}_pin_2v4"
            yield ("joint", "release", model, "-", None)
            yield (
                "pose",
                model,
                transform((-0.82, -0.10 + toy * 0.03, self.height + 0.0281)),
            )
        else:
            model = f"toy_{toy}_insert_3v4"
            values = self.alignment["3V4"]
            relative = transform(values["xyz_m"], values["rpy_rad"])
            relative[0, 3] += 0.005
            yield ("joint", "release", model, "-", None)
            yield ("pose", model, self.measured_poses[base] @ relative)
            yield ("joint", "fasten", model, base, relative)
        self.record("simulated_fault_injected", toy=toy, defect=defect)

    def advance_trays(self, toy):
        # Advance all remaining kits together after the empty front carrier is removed.
        if toy == 1:
            return
        group = []
        step = np.asarray(self.tray_config["row_step"])
        for number in range(toy, self.count + 1):
            name = f"kit_tray_{number}"
            source = carrier_pose(self.tray_config, number - toy + 1)
            target = carrier_pose(self.tray_config, number - toy)
            yield ("joint", "release", name, "-", None)
            group.append((name, source, target))
            for part in ("base_1v4", "insert_3v4", "pin_2v4"):
                value = self.starts[(number - 1, part)]
                source = transform(value.xyz, value.rpy)
                source[:3, 3] -= step * (toy - 2)
                target = source.copy()
                target[:3, 3] -= step
                # Move the loose stock as a bundle, then reconnect to its plate.
                yield ("joint", "release", f"toy_{number}_{part}", "-", None)
                group.append((f"toy_{number}_{part}", source, target))
        yield ("feed_group", group)
        if self.tray_config.get("work_pose"):
            for number in range(toy, self.count + 1):
                carrier = carrier_pose(self.tray_config, number - toy)
                for part in ("base_1v4", "insert_3v4", "pin_2v4"):
                    value = self.starts[(number - 1, part)]
                    pose = transform(value.xyz, value.rpy)
                    pose[:3, 3] -= step * (toy - 1)
                    yield (
                        "joint",
                        "fasten",
                        f"toy_{number}_{part}",
                        f"kit_tray_{number}",
                        np.linalg.inv(carrier) @ pose,
                    )
        self.record("kit_trays_advanced", front=toy)

    def remove_tray(self, toy):
        # All three components are already clear; arm 2 is home before this move.
        name = f"kit_tray_{toy}"
        yield from self.pick("tray", name)
        plan = self.plans["tray"]
        yield ("stage", "tray: pull clear of feeder")
        for q in plan["pull"]:
            yield ("move", "assembly_left", q)
        yield ("stage", "tray: left-side drop")
        yield ("move", "assembly_left", plan["q"][1])
        yield ("joint", "drop", name, "-", None)
        q = plan["q"][1].copy()
        q[5] = plan["open"]
        yield ("move", "assembly_left", q)
        yield from self.home("assembly_left")
        self.record("empty_tray_removed", toy=toy)

    # Move only after loading finishes and the previous output has been collected.
    def conveyor(self):
        for _ in range(self.count):
            yield (
                "wait",
                lambda: self.input_toy is not None
                and self.output_toy is None
                and self.sensor_clear(),
            )
            toy = self.input_toy
            self.moving = True
            self.input_toy = None
            self.record("belt_started", toy=toy)
            names = [
                f"toy_{toy}_{part}"
                for part in ("base_1v4", "pin_2v4", "insert_3v4")
                if part != "pin_2v4" or self.fault_schedule.get(toy) != "missing_gear"
            ]
            for model in names:
                yield ("joint", "pose", model, "-", None)
            origin = self.measured_poses[names[0]]
            relative = {
                model: np.linalg.inv(origin) @ self.measured_poses[model]
                for model in names
            }
            # Move all three free models together during the animated belt transfer.
            # Teleporting one link of a fixed-joint chain creates large solver impulses.
            for model in names[::-1]:
                yield ("joint", "release", model, "-", None)
            yield ("delay", 0.2)
            target = origin.copy()
            target[:3, 3] += (
                self.plans["belt_out"][:3, 3] - self.plans["belt_in"][:3, 3]
            )
            yield ("bundle", names, origin, target, relative)
            yield ("joint", "park", names[0], "project2_rig", target)
            for model in names[1:]:
                yield ("joint", "fasten", model, names[0], relative[model])
            self.output_toy, self.moving = toy, False
            self.record("belt_stopped", toy=toy)

    def assembly_unload(self):
        # Section test only: a simulated operator clears each finished product.
        for toy in range(1, self.count + 1):
            yield ("wait", lambda: self.output_toy == toy and not self.moving)
            target = frame(self.config["assembly_output"][toy - 1])
            names = [("base_1v4", "1V4"), ("pin_2v4", "2V4"), ("insert_3v4", "3V4")]
            for part, _ in names:
                yield ("joint", "release", f"toy_{toy}_{part}", "-", None)
            for part, cad in names:
                values = self.alignment[cad]
                pose = target @ transform(values["xyz_m"], values["rpy_rad"])
                yield ("pose", f"toy_{toy}_{part}", pose)
                yield ("joint", "park", f"toy_{toy}_{part}", "project2_rig", pose)
            yield ("wait_sensor_clear",)
            self.output_toy = None
            self.outcomes[toy] = "assembled"
            self.record("simulated_operator_unload", toy=toy)

    # Prepare once, then keep that box and lid available if a toy is rejected.
    def packaging(self):
        prepared = False
        for toy in range(1, self.count + 1):
            index = self.packed + 1
            box, lid, base = f"box_{index}", f"lid_{index}", f"toy_{toy}_base_1v4"
            if not prepared and self.task_mode != "inspection":
                if index > self.package_count:
                    raise RuntimeError(
                        "No empty boxes remain; check inspection results and reset"
                    )
                for model, key in ((box, "box"), (lid, "lid")):
                    source = self.plans[key]["poses"][0].copy()
                    source[:3, 3] += np.asarray(self.config["feeder_steps"][key]) * (
                        index - 1
                    )
                    yield ("feed", model, source, self.plans[key]["poses"][0])
                yield from self.pick("box", box)
                yield from self.place("box", box)
                yield from self.home(self.plans["box"]["arm"])
                yield from self.pick("lid", lid)
                yield from self.home("packaging", holding=True)
                prepared = True
                self.record("packaging_prepared", toy=toy)
            yield ("wait", lambda: self.output_toy == toy and not self.moving)
            results = []
            for view, q in self.plans["inspect"]["camera"].items():
                yield ("move", "inspection", q)
                yield (
                    "delay",
                    0.4,
                )  # Let vibration settle before counting stable frames.
                key = f"{self.run}:{toy}:{view}"
                yield ("inspect", key, view)
                results.append(self.inspections[key])
                self.record("inspection_view", toy=toy, view=view, result=results[-1])
            result = {
                "pass": all(r["pass"] for r in results),
                "bypassed": any(r["bypassed"] for r in results),
            }
            self.record("inspection", toy=toy, result=result)
            self.bypassed += int(result["bypassed"])
            if result["pass"]:
                self.outcomes[toy] = "bypass" if result["bypassed"] else "pass"
            yield from self.home("inspection")
            yield from self.pick("inspect", base)
            yield from self.home("inspection", holding=True)
            yield ("wait_sensor_clear",)
            self.output_toy = None
            self.record("output_cleared", toy=toy)
            # Remove a reject without consuming the waiting box or lid.
            if not result["pass"] or self.task_mode == "inspection":
                plan = self.plans["inspect"]
                yield from self.home("inspection", holding=True)
                reject = plan["rejects"][
                    toy - 1 if self.task_mode == "inspection" else self.rejected
                ]
                yield ("move", "inspection", reject["approach"])
                q = reject["q"]
                yield ("move", "inspection", q)
                target = self.planner.tool("inspection", q) @ plan["held"]
                yield ("joint", "deposit", base, "project2_rig", target)
                q = q.copy()
                q[5] = self.planner.open
                yield ("move", "inspection", q)
                self.rejected += int(not result["pass"])
                self.outcomes[toy] = (
                    ("bypass" if result["bypassed"] else "pass")
                    if result["pass"]
                    else "fail"
                )
                self.record(
                    "inspection_sorted" if result["pass"] else "rejected",
                    toy=toy,
                    passed=result["pass"],
                )
                q = reject["approach"].copy()
                q[5] = self.planner.open
                yield ("move", "inspection", q)
                yield from self.home("inspection")
                continue
            relative = (
                np.linalg.inv(self.plans["box"]["poses"][1])
                @ self.plans["inspect"]["poses"][1]
            )
            yield from self.place("inspect", base, box, relative)
            yield from self.home("inspection")
            # Arm 3 is home before arm 4 fits the lid at the same holder.
            relative = (
                np.linalg.inv(self.plans["box"]["poses"][1])
                @ self.plans["lid"]["poses"][1]
            )
            yield from self.place("lid", lid, box, relative)
            yield from self.home("packaging")
            yield from self.pick(f"stack_{index-1}", box)
            yield from self.place(f"stack_{index-1}", box)
            yield from self.home("packaging")
            self.packed += 1
            self.outcomes[toy] = "bypass" if result["bypassed"] else "pass"
            prepared = False
            self.record("packed", toy=toy, stack=index)

    # Advance one action only after its movement, reply or wait condition completes.
    def advance(self, task, dt):
        if task.get("blocked"):
            return False
        if "action" not in task:
            try:
                task["action"] = next(task["generator"])
            except StopIteration:
                return True
            task.update(sent=False, elapsed=0.0, retry=0.0)
        action = task["action"]
        kind = action[0]
        task["elapsed"] += dt
        done = False
        if kind == "blocked_path":
            task["blocked"] = f"{action[1]}: {action[2]}"
            self.reason = "Waiting at " + task["blocked"]
            self.record(
                "task_blocked",
                stage=task.get("stage", action[1]),
                reason=task["blocked"],
            )
            return False
        elif kind == "stage":
            task["stage"] = action[1]
            self.record("stage", name=action[1])
            done = True
        elif kind == "wait":
            done = action[1]()
        elif kind == "wait_sensor_clear":
            done = self.sensor_clear()
            if task["elapsed"] > self.config["conveyor_sensor"]["timeout_s"]:
                raise RuntimeError("Output sensor did not clear after pickup")
        elif kind == "delay":
            done = task["elapsed"] >= action[1]
        elif kind == "move":
            _, arm, q = action
            if not task["sent"]:
                # Check paths off the ROS thread so feedback and watchdogs keep running.
                if "check" not in task:
                    task["snapshot"] = {
                        name: values.copy() for name, values in self.feedback.items()
                    }
                    task["targets"] = {
                        name: values.copy()
                        for name, values in self.hold_targets.items()
                    }
                    task["check"] = self.worker.submit(
                        self.collision.checked_path,
                        arm,
                        self.feedback[arm].copy(),
                        q.copy(),
                        task["snapshot"],
                        task["targets"],
                    )
                    return False
                if not task["check"].done():
                    return False
                hit, relevant = task["check"].result()
                if hit:
                    raise RuntimeError(
                        f"{arm} path overlaps {hit}; capture a clear approach waypoint"
                    )
                targets_changed = any(
                    name != arm
                    and (
                        name not in task["targets"]
                        or np.max(np.abs(values - task["targets"][name])) > 0.03
                    )
                    for name, values in self.hold_targets.items()
                )
                if targets_changed or any(
                    np.max(np.abs(self.feedback[name] - old)) > 0.03
                    for name, old in task["snapshot"].items()
                    if name != arm and name in relevant
                ):
                    del task["check"]
                    return False
                task["elapsed"] = 0.0
                self.send_move(arm, q)
                self.record(
                    "move_started",
                    arm=arm,
                    target=q.tolist(),
                    stage=task.get("stage", "setup"),
                )
                task["sent"] = True
            actual = self.planner.tool(arm, self.feedback[arm])
            target = self.planner.tool(arm, q)
            done = (
                time.monotonic() - self.feedback_at[arm] < 1
                and np.max(np.abs(self.feedback[arm] - q))
                < self.config["motion_completion"]["joint_tolerance"]
                and np.linalg.norm(actual[:3, 3] - target[:3, 3])
                < self.config["motion_completion"]["tool_tolerance_m"]
            )
            if task["elapsed"] > 25:
                raise RuntimeError(
                    f"{task.get('stage', 'move')}: {arm} did not reach target {np.round(q,3).tolist()}; measured {np.round(self.feedback[arm],3).tolist()}"
                )
        elif kind == "contact":
            _, name, model, arm = action
            if "check" not in task:
                cad = {
                    "support": "1V4",
                    "base": "1V4",
                    "belt": "1V4",
                    "inspect": "1V4",
                    "gear": "2V4",
                    "handoff": "3V4",
                    "insert": "3V4",
                }.get(name)
                shape = "box" if name.startswith("stack_") else name
                path = self.model_folder / (
                    f"product/{cad}_centered.stl" if cad else f"packaging/{shape}.stl"
                )
                q = self.feedback[arm]
                task["check"] = self.worker.submit(
                    check_contacts,
                    self.planner.pad_points(
                        self.planner.tool(arm, q),
                        q[5],
                        self.plans[name].get("pad_y", (-0.100, -0.076)),
                        self.plans[name].get("pad_z", (0.0, 0.0)),
                    ),
                    self.measured_poses[model],
                    path,
                    shape,
                    faces=(
                        carrier_faces(self.tray_config)
                        if name in ("tray", "load_carrier")
                        else None
                    ),
                )
            if task["check"].done():
                distances = task["check"].result()
                self.record(
                    "grip_contact", model=model, gap_mm=[v * 1000 for v in distances]
                )
                done = True
        elif kind == "joint":
            if not task["sent"]:
                self.request_id += 1
                task["id"] = str(self.request_id)
                task["sent"] = True
            reply = self.replies.pop(task["id"], None)
            if reply:
                if reply[0] == "ok":
                    done = True
                    self.record(action[1], model=action[2])
                    if action[1] == "pose":
                        values = np.array(list(map(float, reply[2:])))
                        pose = np.eye(4)
                        pose[:3, 3] = values[:3]
                        pose[:3, :3] = Rotation.from_quat(values[3:]).as_matrix()
                        self.measured_poses[action[2]] = pose
                        self.record(
                            "measured_pose", model=action[2], matrix=pose.tolist()
                        )
                elif reply[1] not in (
                    "joint_feedback_pending",
                    "gripper_open",
                    "insertion_not_aligned",
                    "deposit_not_aligned",
                    "out_of_reach",
                ):
                    raise RuntimeError(f'{action[1]} {action[2]}: {" ".join(reply)}')
            if not done and task["elapsed"] >= task["retry"]:
                suffix = ""
                if action[4] is not None:
                    pose = action[4]
                    values = np.r_[
                        pose[:3, 3], Rotation.from_matrix(pose[:3, :3]).as_euler("xyz")
                    ]
                    suffix = " " + " ".join(f"{value:.9f}" for value in values)
                self.request_pub.publish(
                    String(
                        data=f'{task["id"]} {action[1]} {action[2]} {action[3]}{suffix}'
                    )
                )
                task["retry"] = task["elapsed"] + 0.8
            if task["elapsed"] > 8:
                raise RuntimeError(
                    f"Gazebo {action[1]} {action[2]} acknowledgement timed out"
                )
        elif kind == "bundle":
            futures = task.get("futures", [])
            if futures and not all(f.done() for f in futures):
                return False
            if futures and not all(f.result().success for f in futures):
                raise RuntimeError("Conveyor model movement failed")
            arrived = bool(futures) and task.get("last_fraction", 0) >= 1
            sensor = self.io[7] if self.require_io and self.io else self.sensor_occupied
            done = (
                bool(futures)
                and task.get("last_fraction", 0) >= 0.995
                and sensor
                and time.monotonic() - self.sensor_at < 0.75
            )
            if task["elapsed"] > self.config["conveyor_sensor"]["timeout_s"]:
                raise RuntimeError("Conveyor arrival sensor timed out")
            if sensor and task.get("last_fraction", 0) < 0.995:
                raise RuntimeError(
                    "Conveyor sensor occupied before the toy reached the far end"
                )
            if not done and not arrived:
                # Travel follows the returned board output; Pause and sensor stop freeze it.
                powered = not self.require_io or (
                    self.io is not None and self.io[9] > 0
                )
                task["travel"] = task.get("travel", 0.0) + (dt if powered else 0.0)
                fraction = min(1, task["travel"] / self.config["conveyor_seconds"])
                origin = action[2].copy()
                origin[:3, 3] += fraction * (action[3][:3, 3] - origin[:3, 3])
                task["futures"] = []
                for model in action[1]:
                    request = SetEntityPose.Request(
                        entity=Entity(name=model, type=Entity.MODEL)
                    )
                    self.set_message_pose(request.pose, origin @ action[4][model])
                    task["futures"].append(self.pose_client.call_async(request))
                task["last_fraction"] = fraction
        elif kind == "feed_group":
            futures = task.get("futures", [])
            if futures and not all(f.done() for f in futures):
                if task["elapsed"] > 20:
                    raise RuntimeError("Tray advance service timed out")
                return False
            if futures and not all(f.result().success for f in futures):
                raise RuntimeError("Tray advance failed")
            done = bool(futures) and task.get("last_fraction", 0) >= 1
            if not done:
                fraction = min(1.0, task["elapsed"] / 1.2)
                task["futures"] = []
                for name, source, target in action[1]:
                    pose = source.copy()
                    pose[:3, 3] += fraction * (target[:3, 3] - source[:3, 3])
                    request = SetEntityPose.Request(
                        entity=Entity(name=name, type=Entity.MODEL)
                    )
                    self.set_message_pose(request.pose, pose)
                    task["futures"].append(self.pose_client.call_async(request))
                task["last_fraction"] = fraction
        elif kind in ("pose", "spawn", "belt", "feed"):
            if kind == "feed" and np.allclose(action[2], action[3], atol=1e-7):
                generator = task["generator"]
                task.clear()
                task["generator"] = generator
                return False
            future = task.get("future")
            if future is not None:
                if not future.done():
                    if task["elapsed"] > 20:
                        raise RuntimeError("Gazebo pose service timed out")
                    return False
                if not future.result().success:
                    raise RuntimeError(f"Gazebo {kind} failed for {action[1]}")
                task.pop("future")
                if kind not in ("belt", "feed") or task.get("last_fraction", 0) >= 1:
                    done = True
            if not done:
                if kind == "spawn":
                    request = SpawnEntity.Request()
                    request.entity_factory.name, request.entity_factory.sdf = action[
                        1:3
                    ]
                    self.set_message_pose(request.entity_factory.pose, action[3])
                    task["future"] = self.spawn_client.call_async(request)
                else:
                    pose = action[2].copy()
                    if kind in ("belt", "feed"):
                        seconds = (
                            self.config["conveyor_seconds"] if kind == "belt" else 0.8
                        )
                        fraction = min(1, task["elapsed"] / seconds)
                        pose[:3, 3] += fraction * (action[3][:3, 3] - pose[:3, 3])
                        task["last_fraction"] = fraction
                    request = SetEntityPose.Request(
                        entity=Entity(name=action[1], type=Entity.MODEL)
                    )
                    self.set_message_pose(request.pose, pose)
                    task["future"] = self.pose_client.call_async(request)
        elif kind == "inspect":
            done = action[1] in self.inspections
            if task["elapsed"] >= task["retry"]:
                self.camera_pub.publish(
                    String(data=json.dumps({"id": action[1], "view": action[2]}))
                )
                task["retry"] = task["elapsed"] + 0.5
            if task["elapsed"] > self.config["camera"]["timeout_s"] + 5:
                raise RuntimeError("Camera node did not return an inspection result")
        if done:
            # Futures and belt progress belong to this action, not the next cycle.
            generator, stage = task["generator"], task.get("stage")
            task.clear()
            task["generator"] = generator
            if stage is not None:
                task["stage"] = stage
        return False

    # Correct small measured drift; stop instead of chasing a large position error.
    def maintain_position(self, now):
        # The servo controller holds each target. Re-send small measured drift only
        # after checking the correction path; larger errors stop the run.
        if not self.actuator_power or self.state != "RUNNING":
            return
        moving = {
            t["action"][1]
            for t in self.tasks.values()
            if t.get("action", ("",))[0] == "move"
        }
        settings = self.config["position_hold"]
        for arm, target in list(self.hold_targets.items()):
            if arm in moving or arm not in self.feedback:
                continue
            current = self.feedback[arm]
            error = float(np.max(np.abs(target - current)))
            distance = float(
                np.linalg.norm(
                    self.planner.tool(arm, target)[:3, 3]
                    - self.planner.tool(arm, current)[:3, 3]
                )
            )
            if (
                error > settings["maximum_joint_error"]
                or distance > settings["maximum_tool_error_m"]
            ):
                raise RuntimeError(
                    f"{arm}: position drift exceeds the correction limit"
                )
            if (
                error < settings["joint_tolerance"]
                and distance < settings["tool_tolerance_m"]
            ):
                self.hold_checks.pop(arm, None)
                continue
            if now - self.hold_retry.get(arm, 0.0) < settings["correction_interval_s"]:
                continue
            if arm not in self.hold_checks:
                snapshot = {name: q.copy() for name, q in self.feedback.items()}
                self.hold_checks[arm] = (
                    self.worker.submit(
                        self.collision.swept,
                        arm,
                        current.copy(),
                        target.copy(),
                        snapshot,
                    ),
                    snapshot,
                )
                continue
            future, snapshot = self.hold_checks[arm]
            if not future.done():
                continue
            self.hold_checks.pop(arm)
            if any(
                np.max(np.abs(self.feedback[name] - q)) > 0.02
                for name, q in snapshot.items()
            ):
                continue
            if future.result():
                raise RuntimeError(f"{arm}: drift correction blocked by another arm")
            self.send_move(arm, target, 0.5)
            self.hold_retry[arm] = now
            self.record(
                "position_corrected", arm=arm, joint_error=error, tool_error_m=distance
            )

    # A short timer services all three jobs without blocking input handling.
    def tick(self):
        now = time.monotonic()
        dt = min(0.2, now - self.last_tick)
        self.last_tick = now
        fresh = self.io is not None and now - self.io_at < 0.75
        feedback = len(self.feedback) == 4 and all(
            now - t < 0.75 for t in self.feedback_at.values()
        )
        if self.require_io and self.state == "RUNNING" and not fresh:
            self.stop("FAULT", "Micro-ROS feedback lost")
        if self.state == "RUNNING" and not feedback:
            self.stop("FAULT", "Arm feedback lost")
        try:
            preview = getattr(self, "plan_preview", {}).get("paths")
            if preview is not None and preview is not self.preview_paths:
                self.plan_pub.publish(String(data=json.dumps(preview)))
                self.preview_paths = preview
            if (
                getattr(self, "selector_reset_pending", False)
                and self.setup_done
                and fresh
                and self.io[5]
                and not self.io[3]
                and not self.io[4]
                and self.state != "ESTOP"
            ):
                self.selector_reset_pending = False
                self.on_event(String(data="reset"))
            if self.plans is None and self.planning.done() and self.plan_error is None:
                try:
                    self.plans = self.planning.result()
                except Exception as error:
                    self.plan_error = str(error)
                    # Reset restores stock independently of motion planning.
                    # A failed contact path still prevents Start.
                    self.reason = "Path blocked: " + self.plan_error
                    self.get_logger().error(self.reason)
                else:
                    rows = {}
                    for name, plan in self.plans.items():
                        if not isinstance(plan, dict) or "q" not in plan:
                            continue
                        rows[name] = dict(
                            arm=plan["arm"],
                            points={
                                "approach_pickup": plan["approach"][0].tolist(),
                                "pickup": plan["q"][0].tolist(),
                                "approach_place": plan["approach"][1].tolist(),
                                "place": plan["q"][1].tolist(),
                            },
                        )
                    self.plan_pub.publish(String(data=json.dumps(rows)))
                    if not self.setup_done:
                        self.reason = "Transfer plans ready"
            if (
                self.state == "READY"
                and self.ready
                and feedback
                and (self.actuator_power or not self.require_io)
                and not self.setup_done
                and not self.tasks
            ):
                self.tasks["setup"] = {"generator": self.setup(True)}
            if self.reset_pending and self.actuator_power:
                self.state, self.reset_pending = "READY", False
            if (
                "setup" in self.tasks
                and self.state == "READY"
                and (self.actuator_power or not self.require_io)
            ):
                if self.advance(self.tasks["setup"], dt):
                    self.tasks.pop("setup")
            elif (
                self.start_pending
                and (
                    self.count > 0
                    or (
                        getattr(self, "settings_dirty", False)
                        and self.requested_count > 0
                    )
                )
                and self.setup_done
                and feedback
                and (not self.require_io or (fresh and self.actuator_power))
            ):
                if self.state in ("READY", "PAUSED"):
                    if getattr(self, "settings_dirty", False):
                        # Start applies a new product selection; resume keeps existing tasks.
                        self.on_event(String(data="reset"))
                        self.start_pending = self.state == "READY"
                    elif self.plan_error or not self.plans:
                        self.reason = (
                            "Path blocked: " + self.plan_error
                            if self.plan_error
                            else "Wait for path planning"
                        )
                        self.start_pending = not bool(self.plan_error)
                    elif (self.task_mode, self.count) != (
                        self.requested_mode,
                        self.requested_count,
                    ):
                        self.reason = "Settings changed: press Reset before Start"
                    elif self.require_io and (
                        self.io[6] not in (0, 1) or (self.qa_trial and self.io[6] != 0)
                    ):
                        self.reason = (
                            "Select Full line for QA, or a valid production section"
                        )
                    elif not self.require_io or (
                        self.io[5] and self.io[2] and not self.io[12] and not self.io[4]
                    ):
                        if not self.tasks:
                            self.selected_mode = (
                                int(self.io[6]) if self.require_io else 0
                            )
                            if self.qa_trial:
                                jobs = (
                                    ("qa_feed", self.qa_feed),
                                    ("packaging", self.packaging),
                                )
                            elif self.task_mode == "assembly":
                                jobs = (
                                    ("assembly", self.assembly),
                                    ("conveyor", self.conveyor),
                                    ("unload", self.assembly_unload),
                                )
                            else:
                                feed = (
                                    self.assembly
                                    if self.task_mode == "full"
                                    else self.downstream_feed
                                )
                                jobs = (
                                    ("assembly", feed),
                                    ("conveyor", self.conveyor),
                                    ("packaging", self.packaging),
                                )
                            self.tasks = {
                                name: {"generator": generator()}
                                for name, generator in jobs
                            }
                        self.state, self.reason, self.start_pending = (
                            "RUNNING",
                            "",
                            False,
                        )
                        self.record("started", toys=self.count)
            if self.state == "TEACH" and "teach" in self.tasks:
                if not feedback or (
                    self.require_io and (not fresh or not self.actuator_power)
                ):
                    raise RuntimeError(
                        "Manual move stopped: feedback or actuator power lost"
                    )
                if self.advance(self.tasks["teach"], dt):
                    self.tasks.pop("teach")
                    self.reason = (
                        "Position reached. Save coordinate, or enter the next target."
                    )
            if self.state == "RUNNING":
                self.maintain_position(now)
                for name in list(self.tasks):
                    task = self.tasks[name]
                    try:
                        if self.advance(task, dt):
                            self.tasks.pop(name)
                    except (RuntimeError, ValueError) as error:
                        # Hold the affected station at measured joints. Never skip
                        # its failed grasp/place or mark that product complete.
                        self.block_task(name, task, str(error))
                if not self.tasks:
                    self.state = "COMPLETE"
                    self.record(
                        "complete",
                        assembled=self.completed,
                        packed=self.packed,
                        bypassed=self.bypassed,
                    )
                    self.get_logger().info(
                        f"COMPLETE: {self.packed}/{self.count} packed; {self.bypassed} camera bypasses"
                    )
        except Exception as error:
            self.stop("TEACH" if self.state == "TEACH" else "FAULT", str(error))
            self.get_logger().error(str(error))
            # Prevent a failed setup/plan from being retried every timer tick.
            self.tasks.clear()
            self.start_pending = False
            if self.plans is None:
                self.plans = {}
        running = self.state == "RUNNING"
        step = " | ".join(
            f'{name}: {task.get("action", ("waiting",))[0]}'
            for name, task in self.tasks.items()
        )
        state = dict(
            state=self.state,
            mode=("qa" if self.qa_trial else self.task_mode),
            step=step,
            step_label=" | ".join(
                f'{name}: {task.get("stage", task.get("action", ("waiting",))[0])}'
                for name, task in self.tasks.items()
            ),
            cycle=self.cycle,
            cycles=self.count,
            setup_done=self.setup_done,
            plan_error=self.plan_error,
            unavailable_paths=self.plans.get("unavailable", {}) if self.plans else {},
            blocked_tasks={
                name: task["blocked"]
                for name, task in self.tasks.items()
                if task.get("blocked")
            },
            paths_ready=bool(self.plans) and not self.plan_error,
            planning_task=self.plan_preview.get("current", ""),
            start_pending=self.start_pending,
            requested_mode=self.requested_mode,
            requested_count=self.requested_count,
            fault_schedule=self.fault_schedule,
            selector_revision=getattr(self, "selector_revision", 0),
            qa_trial=self.qa_trial,
            io_connected=fresh,
            path_file=str(self.config_path),
            completed=self.completed,
            packed=self.packed,
            rejected=self.rejected,
            reason=self.reason,
            conveyor_running=running and self.moving,
            tower=(
                "yellow"
                if running
                else ("green" if self.state in ("READY", "COMPLETE") else "red")
            ),
            alarm=self.state == "ESTOP",
            inspection_bypassed=self.bypassed,
            outcomes=self.outcomes,
            conveyor_sensor=bool(self.io[7]) if fresh else None,
        )
        self.state_pub.publish(String(data=json.dumps(state)))
        if self.command_link:
            self.sequence = self.sequence % 0xFFFFFFFF + 1
            belt_action = (
                self.tasks.get("conveyor", {}).get("action", ("",))[0] == "bundle"
            )
            enable = (
                running
                and self.moving
                and belt_action
                and (not self.require_io or fresh)
            )
            self.io_pub.publish(
                UInt32MultiArray(
                    data=[
                        self.sequence,
                        int(enable),
                        800 if enable else 0,
                        0,
                        {"red": 0, "yellow": 1, "green": 2}[state["tower"]],
                        int(state["alarm"]),
                    ]
                )
            )


def main():
    rclpy.init()
    node = ProductionRun()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop("PAUSED", "Closing")
        node.worker.shutdown(wait=False)
        node.plan_cancel.set()
        node.plan_worker.shutdown(wait=False)
        node.log_file.close()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
