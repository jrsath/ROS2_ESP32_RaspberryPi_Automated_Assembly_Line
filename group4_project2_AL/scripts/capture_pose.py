#!/usr/bin/env python3
# Save a measured arm pose into the production waypoint file.
import argparse
import json
import math
import time
from pathlib import Path
import yaml
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from ament_index_python.packages import get_package_share_directory
from group4_project2_al.arm_model import JOINTS


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "transfer",
        help="base, gear, handoff, insert, belt, box, inspect, lid, stack_0 ... stack_4",
    )
    parser.add_argument(
        "point", choices=["pickup", "place", "approach_pickup", "approach_place"]
    )
    parser.add_argument(
        "--arm",
        required=True,
        choices=["assembly_left", "assembly_right", "inspection", "packaging"],
    )
    parser.add_argument(
        "--hardware",
        action="store_true",
        help="Read stationary lab feedback through the hardware monitor",
    )
    parser.add_argument(
        "--file",
        type=Path,
        default=Path(get_package_share_directory("group4_project2_al"))
        / "config/production.yaml",
    )
    args = parser.parse_args()
    rclpy.init()
    node = Node("capture_production_pose")
    data = {}

    def receive(message):
        values = dict(zip(message.name, message.position))
        if all(j in values and math.isfinite(values[j]) for j in JOINTS):
            q = [float(values[j]) for j in JOINTS]
            now = time.monotonic()
            if (
                "anchor" not in data
                or max(abs(a - b) for a, b in zip(q, data["anchor"])) > 0.003
            ):
                data["anchor"], data["stable_since"] = q, now
            data["q"], data["at"] = q, now

    node.create_subscription(
        JointState,
        f"/{args.arm}/joint_state_broadcaster/joint_states",
        receive,
        qos_profile_sensor_data,
    )
    node.create_subscription(
        String,
        "/project2/sim/state",
        lambda msg: data.update(state=json.loads(msg.data)["state"]),
        10,
    )
    end = time.monotonic() + 8
    try:
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.1)
            now = time.monotonic()
            settled = now - data.get("stable_since", now) >= 0.5
            allowed = args.hardware or data.get("state") in ("READY", "PAUSED")
            if "q" in data and allowed and settled and now - data["at"] < 0.5:
                text = args.file.read_text()
                config = yaml.safe_load(text)
                config.setdefault("joint_overrides", {}).setdefault(args.transfer, {})[
                    args.point
                ] = data["q"]
                # Replace only captured joints so calibration keeps the other comments.
                block = yaml.safe_dump(
                    {"joint_overrides": config["joint_overrides"]}, sort_keys=False
                )
                document = yaml.compose(text)
                for key, value in document.value:
                    if key.value == "joint_overrides":
                        end = value.end_mark.index
                        lines = text[:end].splitlines(keepends=True)
                        while lines and (
                            not lines[-1].strip() or lines[-1].lstrip().startswith("#")
                        ):
                            end -= len(lines.pop())
                        text = text[: key.start_mark.index] + block + text[end:]
                        break
                else:
                    text = (
                        text.rstrip()
                        + "\n\n# Captured joint angles, in radians.\n"
                        + block
                    )
                args.file.write_text(text)
                print(
                    f"Saved {args.transfer}/{args.point} to {args.file}. Restart production to load it."
                )
                return
        raise RuntimeError(
            "Need stationary, fresh joint feedback. Pause simulation, or use --hardware with the lab monitor."
        )
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
