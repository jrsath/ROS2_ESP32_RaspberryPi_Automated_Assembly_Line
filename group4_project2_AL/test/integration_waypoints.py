#!/usr/bin/env python3
# Exercise recording or ordered playback against a running Gazebo launch.
import argparse
import copy
import importlib.util
import json
import os
from pathlib import Path
import subprocess
import time
import yaml
import rclpy
from rclpy.node import Node
from PyQt5.QtTest import QTest
from PyQt5.QtWidgets import QApplication

parser = argparse.ArgumentParser()
parser.add_argument("phase", choices=["capture", "playback"])
args = parser.parse_args()
root = Path(__file__).parents[1]
os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
spec = importlib.util.spec_from_file_location(
    "panel", root / "scripts/control_panel.py"
)
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)
rclpy.init()
app = QApplication([])
node = Node("project2_waypoint_test")
panel = module.Panel(node)
output = Path("/tmp/project2_waypoint_test.yaml")
seen = []


def wait_for(predicate, timeout=30):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        QTest.qWait(30)
        if panel.cell.get("step") == "assembly_task_2":
            index = panel.arm.get("waypoint")
            if index is not None and (not seen or seen[-1] != index):
                seen.append(index)
        if predicate():
            return
    raise AssertionError(
        json.dumps({"cell": panel.cell, "arm": panel.arm, "io": panel.io})
    )


try:
    wait_for(
        lambda: panel.io
        and panel.io[5]
        and not panel.io[12]
        and panel.arm.get("feedback_ready"),
        120,
    )
    QTest.qWait(1500)
    panel.send_event("reset")
    wait_for(lambda: panel.cell.get("state") == "READY")
    panel.pulse(0)
    wait_for(lambda: panel.cell.get("state") == "RUNNING")
    if args.phase == "capture":
        panel.pulse(1)
        wait_for(lambda: panel.cell.get("state") == "PAUSED")
        QTest.qWait(1000)
        output.unlink(missing_ok=True)
        process = subprocess.Popen(
            [
                "ros2",
                "run",
                "group4_project2_al",
                "capture_waypoint.py",
                "assembly_task_2",
                "--slot",
                "approach",
                "--output",
                str(output),
            ],
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
        )
        wait_for(lambda: process.poll() is not None, 15)
        message = process.stdout.read()
        assert process.returncode == 0, message
        data = yaml.safe_load(output.read_text())
        entries = data["steps"]["assembly_task_2"]["waypoints"]
        assert [entry["label"] for entry in entries] == ["approach", "work"]
        assert len(entries[0]["arms"]) == 4
        result = {"phase": "capture", "result": "passed", "message": message.strip()}
        # Use nearby demo poses for the separate ordered-playback check.
        data = yaml.safe_load((root / "config/task_waypoints.yaml").read_text())
        work = data["steps"]["assembly_task_2"]["waypoints"][0]
        entries = []
        for label, delta in [("approach", -0.08), ("work", 0.0), ("retreat", 0.08)]:
            entry = copy.deepcopy(work)
            entry["label"], entry["dwell_s"] = label, 0.4
            entry["arms"]["assembly_left"][0] += delta
            entries.append(entry)
        data["steps"]["assembly_task_2"]["waypoints"] = entries
        output.write_text(yaml.safe_dump(data, sort_keys=False))
    else:
        wait_for(lambda: panel.cell.get("state") == "COMPLETE", 180)
        assert seen == [0, 1, 2], seen
        assert panel.cell["completed"] == 1 and panel.cell["packed"] == 1
        result = {
            "phase": "playback",
            "result": "passed",
            "waypoint_order": seen,
            "completed": panel.cell["completed"],
            "packed": panel.cell["packed"],
        }
    (root / "docs/validation" / f"waypoint_{args.phase}.json").write_text(
        json.dumps(result, indent=2) + "\n"
    )
    print(result, flush=True)
finally:
    panel.close()
    node.destroy_node()
    rclpy.shutdown()
