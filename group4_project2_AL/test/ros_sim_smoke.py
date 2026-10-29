#!/usr/bin/env python3
# Exercise the installed launch file over ROS without connecting to hardware.
import json
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


def main():
    if (
        os.environ.get("ROS_DOMAIN_ID") != "93"
        or os.environ.get("ROS_LOCALHOST_ONLY") != "1"
    ):
        raise SystemExit("Use ROS_DOMAIN_ID=93 ROS_LOCALHOST_ONLY=1")
    rclpy.init()
    node = Node("project2_test_probe")
    latest = {}
    pub = node.create_publisher(String, "/project2/sim/event", 10)

    def receive(message):
        latest.clear()
        latest.update(json.loads(message.data))

    node.create_subscription(String, "/project2/sim/state", receive, 10)

    def wait_for(predicate, process, seconds=15):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.05)
            if predicate():
                return
            if process.poll() is not None:
                raise AssertionError("Simulation exited unexpectedly")
        raise AssertionError("Timed out: " + repr(latest))

    try:
        for mode in ("full", "inspection"):
            latest.clear()
            with tempfile.TemporaryFile(mode="w+t") as output:
                process = subprocess.Popen(
                    [
                        "ros2",
                        "launch",
                        "group4_project2_al",
                        "simulation.launch.py",
                        "mode:=" + mode,
                        "step_seconds:=0.05",
                    ],
                    stdout=output,
                    stderr=subprocess.STDOUT,
                    start_new_session=True,
                )
                try:
                    wait_for(
                        lambda: latest.get("state") == "READY"
                        and pub.get_subscription_count() > 0,
                        process,
                    )
                    pub.publish(String(data="estop"))
                    wait_for(lambda: latest.get("state") == "ESTOP", process)
                    pub.publish(String(data="release_estop"))
                    pub.publish(String(data="reset"))
                    wait_for(lambda: latest.get("state") == "READY", process)
                    pub.publish(String(data="start"))
                    wait_for(lambda: latest.get("state") == "COMPLETE", process)
                    assert latest["simulation"] is True
                    if mode == "full":
                        assert latest["packed"] == latest["completed"] == 5
                    else:
                        assert latest["completed"] == 6 and latest["rejected"] == 2
                    print("PASS:", mode, latest)
                finally:
                    if process.poll() is None:
                        os.killpg(process.pid, signal.SIGINT)
                        try:
                            process.wait(timeout=8)
                        except subprocess.TimeoutExpired:
                            os.killpg(process.pid, signal.SIGKILL)
                            process.wait()
                    output.seek(0)
                    transcript = output.read()
                    if process.returncode != 0 or "Traceback" in transcript:
                        raise AssertionError(transcript)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
