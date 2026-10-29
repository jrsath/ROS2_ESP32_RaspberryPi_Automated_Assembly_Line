#!/usr/bin/env python3
# Run against a production launch with panel:=false on the simulation ROS domain.
import argparse
import importlib.util
import json
import os
from pathlib import Path
import time
import rclpy
from rclpy.node import Node
from PyQt5.QtTest import QTest
from PyQt5.QtWidgets import QApplication

parser = argparse.ArgumentParser()
parser.add_argument(
    "--mode", choices=("full", "assembly", "inspection", "packaging"), default="full"
)
parser.add_argument("--products", type=int, choices=range(1, 6), default=5)
parser.add_argument("--faults", type=int, default=0)
parser.add_argument("--timeout", type=float, default=900)
parser.add_argument(
    "--diagnostic",
    action="store_true",
    help="Check Start/Pause and wait at the first blocked task",
)
args = parser.parse_args()
if not 0 <= args.faults <= args.products:
    parser.error("Fault count must be between zero and the product count")
os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
root = Path(__file__).resolve().parents[1]
import yaml

identity = yaml.safe_load((root / "ID.yaml").read_text())
if int(os.environ.get("ROS_DOMAIN_ID", 0)) != int(identity["simulation_ros_domain_id"]):
    raise SystemExit("Run inside bash run.sh shell (simulation domain)")
spec = importlib.util.spec_from_file_location(
    "panel", root / "scripts/control_panel.py"
)
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)
rclpy.init()
app = QApplication([])
node = Node("project2_process_test")
panel = module.Panel(node)
panel.show()
started = time.monotonic()
result = dict(mode=args.mode, products=args.products, faults=args.faults, passed=False)


def wait_for(predicate, seconds=30, check_fault=True):
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        QTest.qWait(30)
        if predicate():
            return
        if check_fault and (
            panel.cell.get("state") == "FAULT" or panel.cell.get("plan_error")
        ):
            raise AssertionError(
                panel.cell.get("plan_error") or panel.cell.get("reason")
            )
    raise AssertionError("Timed out: " + json.dumps(panel.cell))


try:
    wait_for(
        lambda: panel.io
        and panel.io[5]
        and not panel.io[12]
        and panel.arm.get("feedback_ready")
        and panel.cell.get("setup_done"),
        seconds=90,
        check_fault=False,
    )
    panel.task_choice.setCurrentIndex(panel.task_choice.findData(args.mode))
    panel.product_choice.setCurrentIndex(args.products)
    panel.fault_count.setCurrentIndex(args.faults)
    wait_for(
        lambda: panel.cell.get("requested_count") == args.products, check_fault=False
    )
    panel.send_event("reset")
    wait_for(
        lambda: panel.cell.get("cycles") == args.products
        and not panel.cell.get("setup_done"),
        check_fault=False,
    )
    panel.pulse(0)
    wait_for(lambda: panel.cell.get("state") == "RUNNING", seconds=300)
    panel.pulse(1)
    wait_for(lambda: panel.cell.get("state") == "PAUSED")
    QTest.qWait(400)
    panel.pulse(0)
    wait_for(lambda: panel.cell.get("state") == "RUNNING")
    wait_for(
        lambda: panel.cell.get("state") == "COMPLETE"
        or (args.diagnostic and panel.cell.get("blocked_tasks")),
        seconds=args.timeout,
    )
    result["completed_cycle"] = panel.cell.get("state") == "COMPLETE"
    result["scope"] = (
        "Start/Pause and blocked-task hold"
        if args.diagnostic
        else "Complete production batch"
    )
    if not args.diagnostic:
        assert panel.cell["completed"] == args.products
        if args.mode in ("full", "packaging"):
            assert panel.cell["packed"] == args.products - args.faults
            assert panel.cell["rejected"] == args.faults
    elif panel.cell.get("blocked_tasks"):
        assert panel.cell["state"] == "RUNNING"
        assert panel.cell["completed"] < args.products
    result["passed"] = True
    print("PASS:", panel.cell, flush=True)
except Exception as error:
    result["error"] = str(error)
    raise
finally:
    result.update(seconds=round(time.monotonic() - started, 2), cell=panel.cell)
    output = root / (
        "docs/validation/latest_diagnostic.json"
        if args.diagnostic
        else "docs/validation/latest_process.json"
    )
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(result, indent=2) + "\n")
    panel.close()
    node.destroy_node()
    rclpy.try_shutdown()
