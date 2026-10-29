# Shared SO101 command interface for simulation and lab hardware.

import math
import threading
import time
from pathlib import Path

import yaml
from action_msgs.msg import GoalStatus
from ament_index_python.packages import get_package_share_directory
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

ARM_JOINTS = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"]
ALL_JOINTS = ARM_JOINTS + ["gripper"]
JOINT_LIMITS = {
    "shoulder_pan": (-1.91986, 1.91986),
    "shoulder_lift": (-1.74533, 1.74533),
    "elbow_flex": (-1.69, 1.69),
    "wrist_flex": (-1.65806, 1.65806),
    "wrist_roll": (-2.74385, 2.84121),
    "gripper": (-0.174533, 1.74533),
}
DEFAULT_LOGICAL_POSE = [0.0, -0.45, 0.70, 0.45, 0.0, 0.70]


def load_hardware_config(path=None):
    if path is None:
        path = (
            Path(get_package_share_directory("group4_assembly_line"))
            / "config/hardware.yaml"
        )
    with open(path, encoding="utf-8") as config_file:
        return yaml.safe_load(config_file)


def _duration_message(seconds):
    seconds = max(0.05, float(seconds))
    whole = int(seconds)
    return whole, int((seconds - whole) * 1_000_000_000)


class SO101Arm:
    # Gazebo treats the supplied CAD pose as q=0. Joint offsets map that pose
    # to the logical angles shared with the lab ROS wrapper.

    def __init__(self, node, robot_name, backend="sim", hardware_config=None):
        self.node = node
        self.robot_name = robot_name
        self.backend = backend
        self.lock = threading.Lock()
        self.current = {}
        self.last_feedback_time = 0.0
        self.last_command = list(DEFAULT_LOGICAL_POSE)
        self.active_goals = []
        self.cancel_requested = threading.Event()
        self.ik_chain = None
        config = hardware_config or load_hardware_config()

        sim = config.get("simulation", {})
        offsets = sim.get("joint_offsets", {})
        signs = sim.get("joint_signs", {})
        self.sim_offsets = {
            name: float(offsets.get(name, DEFAULT_LOGICAL_POSE[i]))
            for i, name in enumerate(ALL_JOINTS)
        }
        self.sim_signs = {name: float(signs.get(name, 1.0)) for name in ALL_JOINTS}
        for name, value in self.sim_signs.items():
            if abs(value) < 1e-12:
                raise ValueError(f"simulation joint sign for {name} cannot be zero")

        if backend == "sim":
            self.namespace = "/" + robot_name.strip("/")
            self.command_type = "sim"
            self.command_units = "radians"
            self.state_units = "radians"
            self.arm_action = ActionClient(
                node,
                FollowJointTrajectory,
                f"{self.namespace}/arm_controller/follow_joint_trajectory",
            )
            self.gripper_action = ActionClient(
                node,
                FollowJointTrajectory,
                f"{self.namespace}/gripper_controller/follow_joint_trajectory",
            )
            node.create_subscription(
                JointState,
                f"{self.namespace}/joint_state_broadcaster/joint_states",
                self._on_joint_state,
                20,
            )
            return

        if backend != "real":
            raise ValueError("backend must be 'sim' or 'real'")

        settings = config["robots"][robot_name]
        self.namespace = str(settings.get("namespace") or f"/{robot_name}")
        self.command_type = str(settings.get("command_type", "joint_state"))
        self.command_units = str(settings.get("command_units", "radians"))
        self.state_units = str(settings.get("state_units", "radians"))
        self.command_topic = str(
            settings.get("command_topic") or f"{self.namespace}/joint_commands"
        )
        self.action_name = str(
            settings.get("action_name")
            or f"{self.namespace}/arm_controller/follow_joint_trajectory"
        )
        self.gripper_action_name = str(
            settings.get("gripper_action_name")
            or f"{self.namespace}/gripper_controller/follow_joint_trajectory"
        )
        self.separate_gripper_action = bool(
            settings.get("separate_gripper_action", True)
        )
        self.state_topic = str(
            settings.get("state_topic") or f"{self.namespace}/joint_states"
        )
        self.state_type = str(settings.get("state_type", "joint_state"))
        self.command_rate_hz = max(5.0, float(settings.get("command_rate_hz", 25.0)))
        self.joint_name_map = {
            name: str(settings.get("joint_name_map", {}).get(name, name))
            for name in ALL_JOINTS
        }
        self.reverse_joint_name_map = {
            wire: logical for logical, wire in self.joint_name_map.items()
        }

        self.real_action = None
        self.real_gripper_action = None
        self.command_publisher = None
        if self.command_type == "follow_joint_trajectory":
            self.real_action = ActionClient(
                node, FollowJointTrajectory, self.action_name
            )
            if self.separate_gripper_action:
                self.real_gripper_action = ActionClient(
                    node, FollowJointTrajectory, self.gripper_action_name
                )
        elif self.command_type == "joint_trajectory":
            self.command_publisher = node.create_publisher(
                JointTrajectory, self.command_topic, 10
            )
        elif self.command_type == "float64_multi_array":
            self.command_publisher = node.create_publisher(
                Float64MultiArray, self.command_topic, 10
            )
        elif self.command_type == "joint_state":
            self.command_publisher = node.create_publisher(
                JointState, self.command_topic, 10
            )
        else:
            raise ValueError(f"Unsupported command_type: {self.command_type}")

        if self.state_type == "joint_state":
            node.create_subscription(
                JointState, self.state_topic, self._on_joint_state, 20
            )
        elif self.state_type == "float64_multi_array":
            node.create_subscription(
                Float64MultiArray, self.state_topic, self._on_float_state, 20
            )
        else:
            raise ValueError(f"Unsupported state_type: {self.state_type}")

    def _to_radians(self, value):
        return math.radians(value) if self.state_units == "degrees" else float(value)

    def _command_value(self, value):
        return math.degrees(value) if self.command_units == "degrees" else float(value)

    def _sim_to_logical(self, name, value):
        return self.sim_signs[name] * float(value) + self.sim_offsets[name]

    def _logical_to_sim(self, name, value):
        return (float(value) - self.sim_offsets[name]) / self.sim_signs[name]

    def _on_joint_state(self, message):
        with self.lock:
            if message.name:
                for wire_name, position in zip(message.name, message.position):
                    name = (
                        wire_name
                        if self.backend == "sim"
                        else self.reverse_joint_name_map.get(wire_name, wire_name)
                    )
                    if name in ALL_JOINTS:
                        value = self._to_radians(position)
                        if self.backend == "sim":
                            value = self._sim_to_logical(name, value)
                        self.current[name] = value
            elif len(message.position) >= len(ALL_JOINTS):
                for name, position in zip(ALL_JOINTS, message.position):
                    value = self._to_radians(position)
                    if self.backend == "sim":
                        value = self._sim_to_logical(name, value)
                    self.current[name] = value
            if len(self.current) >= len(ALL_JOINTS):
                self.last_feedback_time = time.monotonic()

    def _on_float_state(self, message):
        if len(message.data) < len(ALL_JOINTS):
            return
        with self.lock:
            for name, position in zip(ALL_JOINTS, message.data):
                self.current[name] = self._to_radians(position)
            self.last_feedback_time = time.monotonic()

    def positions(self):
        with self.lock:
            return [
                self.current.get(name, self.last_command[i])
                for i, name in enumerate(ALL_JOINTS)
            ]

    def _feedback_ready(self, max_feedback_age=2.0):
        with self.lock:
            complete = len(self.current) >= len(ALL_JOINTS)
            age = (
                time.monotonic() - self.last_feedback_time
                if self.last_feedback_time
                else math.inf
            )
        return complete and age <= max_feedback_age

    def is_ready(self, max_feedback_age=2.0):
        if self.backend == "sim":
            return (
                self.arm_action.server_is_ready()
                and self.gripper_action.server_is_ready()
                and self._feedback_ready(max_feedback_age)
            )
        if self.command_type == "follow_joint_trajectory":
            arm_ready = self.real_action.server_is_ready()
            gripper_ready = (
                not self.separate_gripper_action
                or self.real_gripper_action.server_is_ready()
            )
            return arm_ready and gripper_ready
        return self._feedback_ready(max_feedback_age)

    def wait_ready(self, timeout=15.0):
        end = time.monotonic() + timeout
        if self.backend == "sim":
            remaining = max(0.1, end - time.monotonic())
            if not self.arm_action.wait_for_server(timeout_sec=remaining):
                return False
            remaining = max(0.1, end - time.monotonic())
            if not self.gripper_action.wait_for_server(timeout_sec=remaining):
                return False
            while time.monotonic() < end:
                if self._feedback_ready(5.0):
                    return True
                time.sleep(0.05)
            return False
        if self.command_type == "follow_joint_trajectory":
            if not self.real_action.wait_for_server(
                timeout_sec=max(0.1, end - time.monotonic())
            ):
                return False
            if self.separate_gripper_action:
                return self.real_gripper_action.wait_for_server(
                    timeout_sec=max(0.1, end - time.monotonic())
                )
            return True
        while time.monotonic() < end:
            if self._feedback_ready(1.0):
                return True
            time.sleep(0.05)
        return False

    def cancel_active(self):
        self.cancel_requested.set()
        for goal in list(self.active_goals):
            try:
                goal.cancel_goal_async()
            except Exception:
                pass
        self.active_goals.clear()
        if self.backend == "real" and self.command_type != "follow_joint_trajectory":
            try:
                self._publish_real_topic(self.positions(), 0.1)
            except Exception:
                pass

    def _valid(self, names, values):
        if len(names) != len(values):
            return False
        for name, value in zip(names, values):
            lower, upper = JOINT_LIMITS[name]
            if not math.isfinite(value) or value < lower or value > upper:
                self.node.get_logger().error(
                    f"{self.robot_name}: {name}={value:.3f} outside [{lower:.3f}, {upper:.3f}]"
                )
                return False
        return True

    def _trajectory(self, names, values, duration, wire_names=False):
        message = JointTrajectory()
        message.joint_names = (
            [self.joint_name_map.get(name, name) for name in names]
            if wire_names
            else list(names)
        )
        point = JointTrajectoryPoint()
        point.positions = [float(v) for v in values]
        point.time_from_start.sec, point.time_from_start.nanosec = _duration_message(
            duration
        )
        message.points = [point]
        return message

    def _wait_future(self, future, timeout):
        end = time.monotonic() + timeout
        while time.monotonic() < end and not future.done():
            if self.cancel_requested.is_set():
                return False
            time.sleep(0.02)
        return future.done()

    def _send_action(
        self, client, names, values, duration, timeout=15.0, wire_names=False
    ):
        self.cancel_requested.clear()
        if not client.wait_for_server(timeout_sec=timeout):
            self.node.get_logger().error(
                f"{self.robot_name}: controller action is not available"
            )
            return False
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = self._trajectory(names, values, duration, wire_names)
        sent = client.send_goal_async(goal)
        if not self._wait_future(sent, timeout):
            return False
        handle = sent.result()
        if handle is None or not handle.accepted:
            self.node.get_logger().error(f"{self.robot_name}: trajectory was rejected")
            return False
        self.active_goals.append(handle)
        result = handle.get_result_async()
        if not self._wait_future(result, timeout + duration + 2.0):
            try:
                handle.cancel_goal_async()
            except Exception:
                pass
            return False
        if handle in self.active_goals:
            self.active_goals.remove(handle)
        response = result.result()
        return response is not None and response.status == GoalStatus.STATUS_SUCCEEDED

    def _wait_target(self, target, timeout, tolerance=0.06):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            if self.cancel_requested.is_set():
                return False
            if max(abs(a - b) for a, b in zip(self.positions(), target)) <= tolerance:
                return True
            time.sleep(0.05)
        return False

    def _publish_real_topic(self, values, duration):
        target = [self._command_value(v) for v in values]
        if self.command_type == "joint_trajectory":
            message = self._trajectory(ALL_JOINTS, target, duration, wire_names=True)
        elif self.command_type == "float64_multi_array":
            message = Float64MultiArray(data=target)
        elif self.command_type == "joint_state":
            message = JointState()
            message.header.stamp = self.node.get_clock().now().to_msg()
            message.name = [self.joint_name_map[n] for n in ALL_JOINTS]
            message.position = target
        else:
            return False
        self.command_publisher.publish(message)
        return True

    def _send_real_topic(self, values, duration):
        self.cancel_requested.clear()
        if not self.wait_ready(timeout=3.0):
            self.node.get_logger().error(
                f"{self.robot_name}: no joint feedback from real arm"
            )
            return False
        if self.command_type == "joint_trajectory":
            if not self._publish_real_topic(values, duration):
                return False
        else:
            start = self.positions()
            steps = max(1, int(max(0.05, duration) * self.command_rate_hz))
            for step in range(1, steps + 1):
                if self.cancel_requested.is_set():
                    return False
                ratio = step / steps
                point = [a + (b - a) * ratio for a, b in zip(start, values)]
                if not self._publish_real_topic(point, duration):
                    return False
                time.sleep(1.0 / self.command_rate_hz)
        self.last_command = list(values)
        return self._wait_target(values, max(3.0, duration + 3.0))

    def move_joints(self, positions, duration=2.0, degrees=False):
        if len(positions) != 5:
            raise ValueError("move_joints expects five arm joint values")
        values = (
            [math.radians(v) for v in positions]
            if degrees
            else [float(v) for v in positions]
        )
        if not self._valid(ARM_JOINTS, values):
            return False
        if self.backend == "sim":
            sim_values = [
                self._logical_to_sim(n, v) for n, v in zip(ARM_JOINTS, values)
            ]
            success = self._send_action(
                self.arm_action, ARM_JOINTS, sim_values, duration
            )
            if success:
                self.last_command = values + [self.positions()[5]]
            return success
        target = self.positions()
        target[:5] = values
        if self.command_type == "follow_joint_trajectory":
            names = ARM_JOINTS if self.separate_gripper_action else ALL_JOINTS
            vals = values if self.separate_gripper_action else target
            success = self._send_action(
                self.real_action,
                names,
                [self._command_value(v) for v in vals],
                duration,
                wire_names=True,
            )
            if success:
                self.last_command = list(target)
            return success
        return self._send_real_topic(target, duration)

    def set_gripper(self, position, duration=0.7, degrees=False):
        value = math.radians(position) if degrees else float(position)
        if not self._valid(["gripper"], [value]):
            return False
        if self.backend == "sim":
            sim_value = self._logical_to_sim("gripper", value)
            success = self._send_action(
                self.gripper_action, ["gripper"], [sim_value], duration
            )
            if success:
                current = self.positions()
                current[5] = value
                self.last_command = current
            return success
        target = self.positions()
        target[5] = value
        if self.command_type == "follow_joint_trajectory":
            if self.separate_gripper_action:
                success = self._send_action(
                    self.real_gripper_action,
                    ["gripper"],
                    [self._command_value(value)],
                    duration,
                    wire_names=True,
                )
            else:
                success = self._send_action(
                    self.real_action,
                    ALL_JOINTS,
                    [self._command_value(v) for v in target],
                    duration,
                    wire_names=True,
                )
            if success:
                self.last_command = list(target)
            return success
        return self._send_real_topic(target, duration)

    def move_pose(self, pose, duration=2.0):
        if len(pose) != 6:
            raise ValueError("pose must contain five arm joints and the gripper")
        return self.move_joints(pose[:5], duration) and self.set_gripper(
            pose[5], min(duration, 0.8)
        )

    def move_xyz(self, x, y, z, duration=2.0, residual_limit=0.003):
        try:
            import numpy as np
            from ikpy.chain import Chain
        except ImportError as error:
            raise RuntimeError(
                "Install config/ik_requirements.txt to use move_xyz"
            ) from error
        if self.ik_chain is None:
            model = (
                Path(get_package_share_directory("group4_assembly_line"))
                / "urdf/so101_kinematics.urdf"
            )
            self.ik_chain = Chain.from_urdf_file(
                str(model),
                active_links_mask=[False, True, True, True, True, True, False],
            )
        target = np.array([float(x), float(y), float(z)])
        current = self.positions()[:5]
        initial = np.array([0.0] + current + [0.0])
        angles = self.ik_chain.inverse_kinematics(
            target_position=target, initial_position=initial
        )
        achieved = self.ik_chain.forward_kinematics(angles)[:3, 3]
        residual = float(np.linalg.norm(achieved - target))
        if residual > residual_limit:
            self.node.get_logger().error(
                f"{self.robot_name}: IK residual {residual*1000:.1f} mm is too large"
            )
            return False
        return self.move_joints(list(angles[1:6]), duration=duration)
