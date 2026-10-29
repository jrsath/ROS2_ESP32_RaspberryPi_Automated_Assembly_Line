#!/usr/bin/env python3
# Run against a manual Gazebo launch with panel:=false, on an isolated ROS domain.
import importlib.util
import json
import os
from pathlib import Path
import time
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from visualization_msgs.msg import InteractiveMarkerFeedback as Feedback
from std_msgs.msg import Bool
from PyQt5.QtCore import Qt, QPoint
from PyQt5.QtTest import QTest
from PyQt5.QtWidgets import QApplication, QPushButton, QCheckBox
from ament_index_python.packages import get_package_share_directory
from group4_project2_al.arm_model import ArmModel, JOINTS, transform
import yaml

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
spec = importlib.util.spec_from_file_location(
    "panel", Path(__file__).parents[1] / "scripts/control_panel.py"
)
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)
rclpy.init()
app = QApplication([])
node = Node("project2_panel_test")
panel = module.Panel(node)
panel.show()
states = {}
names = ("assembly_left", "assembly_right", "inspection", "packaging")
for name in names:
    node.create_subscription(
        JointState,
        f"/{name}/joint_state_broadcaster/joint_states",
        lambda msg, n=name: states.update({n: np.array(msg.position)}),
        10,
    )
command = node.create_publisher(JointState, "/project2/sim/arm1/command", 10)
feedback = node.create_publisher(Feedback, "/project2/sim/controls/feedback", 10)
checks = []


def wait_for(predicate, title, timeout=25):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        QTest.qWait(30)
        if predicate():
            checks.append(title)
            print("PASS:", title, flush=True)
            return
    raise AssertionError(
        title
        + "\n"
        + json.dumps({"cell": panel.cell, "arm": panel.arm, "io": panel.io})
    )


def click(text):
    button = next(w for w in panel.findChildren(QPushButton) if w.text() == text)
    QTest.mouseClick(button, Qt.LeftButton, pos=button.rect().center())
    QTest.qWait(400)


try:
    wait_for(
        lambda: panel.io and not panel.io[12] and panel.io[5] and len(states) == 4,
        "micro-ROS state, command acknowledgement and four Gazebo arms",
    )
    QTest.qWait(1500)
    click("RESET")
    wait_for(lambda: panel.cell.get("state") == "READY", "reset leaves cell ready")
    click("START")
    wait_for(
        lambda: panel.cell.get("state") == "RUNNING",
        "Start click returns through micro-ROS",
    )
    panel.set_follow(True)
    wait_for(lambda: panel.arm.get("master_slave"), "master/slave enabled")
    target = np.array([0.25, 0.0, 0.0, 0.0, 0.0, 0.0])
    command.publish(
        JointState(
            name=JOINTS, position=(target + [0, -0.45, 0.7, 0.45, 0, 0.7]).tolist()
        )
    )
    wait_for(
        lambda: all(np.max(np.abs(states[n] - target)) < 0.035 for n in names),
        "Gazebo arm moves and all three followers copy it",
        40,
    )
    # Send the same feedback produced by dragging the RViz tool marker.
    rig = Path(get_package_share_directory("group4_assembly_line"))
    model = ArmModel(rig / "urdf/so101.urdf.xacro")
    layout = yaml.safe_load((rig / "config/rig_layout.yaml").read_text())["robots"][
        names[0]
    ]
    base = transform(layout["xyz"], layout["rpy"])
    target[0] = 0.35
    tcp = (base @ model.fk(target)[0])[:3, 3]
    msg = Feedback(
        marker_name="arm1_tool",
        event_type=Feedback.POSE_UPDATE,
        client_id="integration_test",
    )
    msg.header.frame_id = "world"
    msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = map(float, tcp)
    msg.pose.orientation.w = 1.0
    feedback.publish(msg)
    wait_for(
        lambda: np.linalg.norm((base @ model.fk(states[names[0]])[0])[:3, 3] - tcp)
        < 0.004,
        "RViz tool feedback drives Gazebo through IK",
        40,
    )
    follow = next(
        w for w in panel.findChildren(QCheckBox) if w.text().startswith("Master/slave")
    )
    QTest.mouseClick(follow, Qt.LeftButton, pos=QPoint(10, follow.height() // 2))
    wait_for(
        lambda: not panel.arm.get("master_slave", True), "master/slave can be disabled"
    )
    QTest.qWait(1000)
    held = states[names[1]].copy()
    target[0] = 0.15
    command.publish(
        JointState(
            name=JOINTS, position=(target + [0, -0.45, 0.7, 0.45, 0, 0.7]).tolist()
        )
    )
    wait_for(
        lambda: abs(states[names[0]][0] - 0.15) < 0.03, "independent leader command", 40
    )
    assert np.max(np.abs(states[names[1]] - held)) < 0.04
    checks.append("followers hold when master/slave is disabled")
    click("PAUSE")
    wait_for(
        lambda: panel.cell.get("state") == "PAUSED" and panel.io[10] == 0,
        "Pause stops cell and sets red",
    )
    click("START")
    click("EMERGENCY STOP")
    wait_for(
        lambda: panel.cell.get("state") == "ESTOP"
        and panel.io[11] == 1
        and panel.io[9] == 0,
        "E-stop inhibits output and sounds alarm",
    )
    click("RELEASE E-STOP")
    assert panel.cell["state"] == "ESTOP"
    click("RESET")
    wait_for(
        lambda: panel.cell.get("state") == "READY"
        and panel.io[10] == 2
        and not panel.io[11],
        "release plus Reset clears latch without restarting",
    )
    panel.mode.setCurrentIndex(2)
    wait_for(
        lambda: panel.cell.get("mode") == "inspection",
        "mode selector is received through micro-ROS",
    )
    click("START")
    lost = next(
        w for w in panel.findChildren(QCheckBox) if w.text().startswith("Interrupt")
    )
    QTest.mouseClick(lost, Qt.LeftButton, pos=QPoint(10, lost.height() // 2))
    wait_for(
        lambda: panel.io[12] == 1
        and panel.cell.get("state") == "FAULT"
        and panel.io[10] == 0,
        "missing command heartbeat trips real client watchdog",
    )
    QTest.mouseClick(lost, Qt.LeftButton, pos=QPoint(10, lost.height() // 2))
    wait_for(lambda: panel.io[12] == 0, "command link recovers")
    click("RESET")
    QTest.qWait(400)
    out = Path(__file__).parents[1] / "docs/validation"
    out.mkdir(exist_ok=True)
    panel.grab().save(str(out / "control_panel.png"))
    (out / "integration_panel.json").write_text(
        json.dumps({"checks": checks, "result": "passed"}, indent=2) + "\n"
    )
finally:
    panel.close()
    node.destroy_node()
    rclpy.shutdown()
