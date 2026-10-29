#!/usr/bin/env python3
# Read the running lab ROS graph. This tool never sends motor commands.
import argparse
import json
import os
from pathlib import Path
import re
import socket
import subprocess
import time
import rclpy
from rclpy.node import Node


def main():
    parser = argparse.ArgumentParser(
        description="Record lab driver interfaces without moving the rig"
    )
    parser.add_argument("--seconds", type=float, default=8.0)
    parser.add_argument("--output", type=Path, default=Path("rig_interfaces.json"))
    args = parser.parse_args()
    if not 1 <= args.seconds <= 60:
        parser.error("--seconds must be between 1 and 60")
    rclpy.init()
    node = Node("project2_interface_discovery")
    try:
        end = time.monotonic() + args.seconds
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.1)
        topics = node.get_topic_names_and_types()
        relevant = [
            (name, types)
            for name, types in topics
            if re.search(
                r"joint|motor|servo|arm|follow|leader|camera|image|torque|conveyor|project2",
                name,
                re.I,
            )
        ]
        report = dict(
            host=socket.gethostname(),
            environment={
                key: os.environ.get(key, "")
                for key in ("ROS_DOMAIN_ID", "RMW_IMPLEMENTATION", "ROS_LOCALHOST_ONLY")
            },
            nodes=node.get_node_names_and_namespaces(),
            topics=[],
            services=node.get_service_names_and_types(),
            interfaces={},
        )
        for name, types in sorted(relevant):
            entries = {}
            for kind, getter in (
                ("publishers", node.get_publishers_info_by_topic),
                ("subscribers", node.get_subscriptions_info_by_topic),
            ):
                entries[kind] = [
                    dict(
                        node=i.node_namespace.rstrip("/") + "/" + i.node_name,
                        reliability=str(i.qos_profile.reliability),
                        durability=str(i.qos_profile.durability),
                    )
                    for i in getter(name)
                ]
            report["topics"].append(dict(name=name, types=types, **entries))
        types = sorted({t for _, ts in relevant for t in ts})
        for kind in types:
            try:
                result = subprocess.run(
                    ["ros2", "interface", "show", kind],
                    text=True,
                    capture_output=True,
                    timeout=5,
                )
                report["interfaces"][kind] = (
                    result.stdout if result.returncode == 0 else result.stderr
                )
            except (OSError, subprocess.TimeoutExpired) as error:
                report["interfaces"][kind] = str(error)
        args.output.write_text(json.dumps(report, indent=2) + "\n")
        print(f"Saved {len(relevant)} relevant topics to {args.output.resolve()}")
        for topic in report["topics"]:
            print(topic["name"] + " : " + ", ".join(topic["types"]))
        if not relevant:
            print(
                "No rig topics found. Start the university drivers in this ROS domain and retry."
            )
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
