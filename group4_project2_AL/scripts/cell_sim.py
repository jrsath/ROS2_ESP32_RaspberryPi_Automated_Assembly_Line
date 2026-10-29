#!/usr/bin/env python3
# Coordinate the simulated cell and command the virtual I/O board.
import csv
from datetime import datetime, timezone
import json
import signal
from pathlib import Path
import time

import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import String, Bool, UInt32MultiArray, UInt64MultiArray
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from group4_project2_al.cell import Cell, MODES


class CellSimulation(Node):
    def __init__(self):
        super().__init__("project2_cell_sim")
        mode = self.declare_parameter("mode", "full").value
        cycles = self.declare_parameter("cycles", 5).value
        seconds = self.declare_parameter("step_seconds", 0.5).value
        seed = self.declare_parameter("seed", 4).value
        self.autostart = self.declare_parameter("autostart", False).value
        self.start_pending = False
        results = Path(
            self.declare_parameter(
                "results_dir", str(Path.home() / "ROS2/project2_data/results")
            ).value
        )
        self.cell = Cell(mode, cycles, seconds, seed)
        self.settings = (cycles, seconds, seed)
        self.require_io = self.declare_parameter("require_io", False).value
        self.manual_mode = self.declare_parameter("manual_mode", False).value
        self.io = None
        self.io_time = 0.0
        self.sequence = 0
        self.last_buttons = (0, 0)
        self.last_boot = None
        self.command_link = True
        self.create_subscription(
            Bool,
            "/project2/sim/command_link",
            lambda msg: setattr(self, "command_link", msg.data),
            10,
        )
        self.command_pub = self.create_publisher(
            UInt32MultiArray, "/project2/sim/io/command", 10
        )
        self.create_subscription(
            UInt64MultiArray,
            "/project2/sim/io/state",
            self.on_io,
            qos_profile_sensor_data,
        )
        self.create_timer(0.1, self.send_io)
        results.mkdir(parents=True, exist_ok=True)
        run_id = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S_%fZ")
        self.log_file = (results / ("simulation_" + run_id + ".csv")).open(
            "x", newline=""
        )
        self.log = csv.DictWriter(
            self.log_file,
            fieldnames=[
                "utc",
                "simulation",
                "mode",
                "event",
                "cycle",
                "step",
                "quality",
                "duration_s",
            ],
        )
        self.log.writeheader()
        self.pub = self.create_publisher(String, "/project2/sim/state", 10)
        self.create_subscription(String, "/project2/sim/event", self.on_event, 10)
        self.last_time = time.monotonic()
        self.last_status = None
        self.wait_motion = self.declare_parameter("wait_motion", False).value
        self.wait_scene = self.declare_parameter("wait_scene", False).value
        self.motion_timeout = self.declare_parameter("motion_timeout", 60.0).value
        self.motion_key = ""
        self.scene_key = ""
        self.scene_ready = not self.wait_scene
        self.scene_status = {}
        self.arm_feedback_ready = not self.wait_motion
        self.motion_started = time.monotonic()
        self.waiting_key = ""
        self.create_subscription(
            String, "/project2/sim/motion_ready", self.motion_ready, 10
        )
        if self.wait_motion:
            self.create_subscription(
                String, "/project2/sim/arm_status", self.arm_status, 10
            )
        if self.wait_scene:
            ready_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self.create_subscription(
                Bool,
                "/project2/sim/assembly_scene/ready",
                lambda msg: setattr(self, "scene_ready", msg.data),
                ready_qos,
            )
            self.create_subscription(
                String, "/project2/sim/assembly_scene/status", self.on_scene_status, 10
            )
            self.create_subscription(
                String,
                "/project2/sim/assembly_scene/step_ready",
                lambda msg: setattr(self, "scene_key", msg.data),
                10,
            )
            self.create_subscription(
                String, "/project2/sim/assembly_scene/error", self.scene_error, 10
            )
        self.create_timer(0.05, self.tick)
        self.get_logger().info("SIMULATION ONLY. Results: " + str(results))

    def on_event(self, message):
        try:
            if message.data == "start" and self.require_io:
                # Start/Pause/E-stop come back through the micro-ROS board.
                return
            if message.data == "start" and not self.can_start():
                self.cell.reason = self.start_wait_reason()
                self.start_pending = True
                return
            if message.data in ("pause", "reset", "estop"):
                self.start_pending = False
            if message.data == "reset" and self.cell.state == "COMPLETE":
                self.cell = Cell(self.cell.mode, *self.settings)
            self.cell.event(message.data.strip())
            self.write_event(
                {"event": message.data.strip(), "cycle": self.cell.index + 1}
            )
        except ValueError as error:
            self.get_logger().warning(str(error))

    def on_io(self, message):
        if len(message.data) != 14:
            return
        data = list(message.data)
        if data[6] > 3 or data[10] > 2:
            return
        now = time.monotonic()
        if self.last_boot is not None and data[0] != self.last_boot:
            self.cell.event("fault")
        self.last_boot = data[0]
        self.io, self.io_time = data, now
        if not self.require_io:
            return
        if data[12] and self.cell.state in ("RUNNING", "WAIT_CLEAR"):
            self.cell.event("fault")
            self.cell.reason = "Micro-ROS command watchdog expired"
        if not data[5]:
            self.cell.event("estop")
            self.start_pending = False
        else:
            self.cell.event("release_estop")
        self.cell.event("occupy" if data[7] else "clear")
        if data[4]:
            self.cell.event("pause")
            self.start_pending = False
        selected = MODES[data[6]]
        if self.wait_scene and selected != "assembly":
            self.cell.event("pause")
            self.cell.reason = "Select Assembly on the control box"
            self.last_buttons = (data[3], data[4])
            return
        if self.wait_scene and self.cell.reason == "Select Assembly on the control box":
            self.cell.reason = ""
        if selected != self.cell.mode and self.cell.state not in ("ESTOP", "FAULT"):
            # Changing the selector stops the old trial and waits for Start.
            self.cell = Cell(selected, *self.settings)
            self.start_pending = False
            self.cell.packaging_occupied = bool(data[7])
            self.write_event({"event": "mode_selected"})
        if (
            data[3]
            and not self.last_buttons[0]
            and data[2]
            and data[5]
            and not data[4]
            and not data[12]
        ):
            if self.can_start():
                self.cell.event("start")
            else:
                self.cell.reason = self.start_wait_reason()
                self.start_pending = True
        self.last_buttons = (data[3], data[4])

    def on_scene_status(self, message):
        try:
            self.scene_status = json.loads(message.data)
        except ValueError:
            self.scene_status = {}

    def can_start(self):
        return self.arm_feedback_ready and self.scene_ready

    def start_wait_reason(self):
        if not self.scene_ready:
            return (
                "Waiting for tray and parts in Gazebo "
                f"({self.scene_status.get('spawned', 0)} spawned)"
            )
        if not self.arm_feedback_ready:
            return "Waiting for all four arm controllers"
        return ""

    def send_io(self):
        if not self.require_io or not self.command_link:
            return
        fresh = self.io is not None and time.monotonic() - self.io_time < 0.75
        if not fresh and self.cell.state in ("RUNNING", "WAIT_CLEAR"):
            self.cell.event("fault")
        state = self.cell.snapshot()
        self.sequence = (self.sequence % 0xFFFFFFFF) + 1
        enable = (
            not self.manual_mode
            and fresh
            and state["conveyor_running"]
            and self.io[2]
            and self.io[5]
            and not self.io[12]
        )
        self.command_pub.publish(
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

    def motion_ready(self, message):
        self.motion_key = message.data

    def arm_status(self, message):
        try:
            self.arm_feedback_ready = bool(json.loads(message.data)["feedback_ready"])
        except (ValueError, KeyError, TypeError):
            self.arm_feedback_ready = False

    def scene_error(self, message):
        self.cell.event("fault")
        self.cell.reason = message.data

    def write_event(self, event):
        self.log.writerow(
            dict(
                utc=datetime.now(timezone.utc).isoformat(),
                simulation=True,
                mode=self.cell.mode,
                **event,
            )
        )
        self.log_file.flush()

    def tick(self):
        now = time.monotonic()
        if self.start_pending and self.can_start() and self.cell.state == "READY":
            self.cell.reason = ""
            self.cell.event("start")
            self.start_pending = False
        if (
            self.autostart
            and not self.require_io
            and self.cell.state == "READY"
            and self.arm_feedback_ready
            and self.scene_ready
        ):
            self.cell.event("start")
            self.autostart = False
        key = f"{self.cell.index+1}:{self.cell.step}"
        # Only advance an animated step after its arm targets have been reached.
        ready = (not self.wait_motion or self.motion_key == key) and (
            not self.wait_scene or (self.scene_ready and self.scene_key == key)
        )
        if (
            key != self.waiting_key
            or self.cell.state != "RUNNING"
            or not self.arm_feedback_ready
        ):
            self.motion_started, self.waiting_key = now, key
        if (
            not self.manual_mode
            and not ready
            and self.arm_feedback_ready
            and (not self.wait_scene or self.scene_ready)
            and now - self.motion_started > self.motion_timeout
        ):
            self.cell.event("fault")
            self.cell.reason = f"Arm target not reached within {self.motion_timeout:g}s"
        if not self.manual_mode:
            self.cell.tick(now - self.last_time, motion_ready=ready)
        self.last_time = now
        for event in self.cell.events:
            self.write_event(event)
        self.cell.events.clear()
        state = self.cell.snapshot()
        if self.cell.state == "READY" and not self.can_start():
            state["reason"] = self.start_wait_reason()
        state["assembly_scene_ready"] = self.scene_ready
        if self.manual_mode:
            state["step"] = "manual arm control"
            state["step_label"] = "Manual arm control"
            state["conveyor_running"] = False
        state["io_required"] = self.require_io
        state["io_connected"] = self.io is not None and now - self.io_time < 0.75
        self.pub.publish(String(data=json.dumps(state)))
        summary = (state["state"], state["step"], state["cycle"])
        if summary != self.last_status:
            self.get_logger().info(
                f"{state['state']} | cycle {state['cycle']}/{state['cycles']} | {state['step']}"
            )
            self.last_status = summary


def main():
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = None
    try:
        node = CellSimulation()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # ros2 launch may forward a second Ctrl+C while cleanup is running.
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        if node:
            node.log_file.close()
            node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
