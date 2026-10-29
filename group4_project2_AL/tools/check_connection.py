#!/usr/bin/env python3
# Check local settings before starting the agent; this does not move the rig.
import argparse
import importlib.util
from pathlib import Path
import re
import sys
import yaml

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location("connection", ROOT / "project/connection.py")
connection = importlib.util.module_from_spec(spec)
spec.loader.exec_module(connection)


def check(root=ROOT):
    data = yaml.safe_load((root / "ID.yaml").read_text())
    args = connection.agent_args(data)
    header = (root / "firmware/project2_io/transport_config.h").read_text()
    expected = {"PROJECT2_WIFI": int(data["transport"] == "wifi"), "ROS_DOMAIN": int(data["ros_domain_id"])}
    for key, value in expected.items():
        match = re.search(r"\b" + key + r"\s*(?:=\s*)?(\d+)", header)
        if not match or int(match.group(1)) != value:
            raise ValueError("Arduino settings differ from ID.yaml. Run python3 tools/apply_id.py and upload the complete firmware/project2_io sketch again.")
    print(f"Transport: {data['transport']} | ROS domain: {data['ros_domain_id']}")
    print("Agent: " + " ".join(args))
    print("Close Arduino Serial Monitor before USB operation.")
    if data['transport'] == 'wifi':
        print(f"ESP32 must target agent {data['network']['agent_ip']}:{data['network']['agent_port']}; USB flashing alone does not change transport.")
    print("Local checks passed. Board connection is confirmed only when /project2/io/state arrives.")
    return args


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.parse_args()
    try:
        check()
    except (ValueError, OSError, KeyError) as error:
        print(str(error), file=sys.stderr)
        raise SystemExit(2)
