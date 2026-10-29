#!/usr/bin/env python3
# Select the ESP32 explicitly when arm adapters are also plugged in.
import argparse
import os
from pathlib import Path
import yaml


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--identity", required=True)
    parser.add_argument("--transport", choices=("usb", "wifi"), required=True)
    parser.add_argument("--agent", required=True)
    args = parser.parse_args()
    identity = yaml.safe_load(Path(args.identity).read_text())
    if args.transport == "wifi":
        port = int(identity["network"]["agent_port"])
        if not 1 <= port <= 65535:
            raise SystemExit("agent_port must be 1–65535")
        command = [args.agent, "udp4", "--port", str(port)]
    else:
        setting = identity.get("serial_device", "auto")
        if setting == "auto":
            devices = {}
            for pattern in ("serial/by-id/*", "ttyUSB*", "ttyACM*"):
                for path in sorted(Path("/dev").glob(pattern)):
                    if path.exists():
                        devices.setdefault(path.resolve(), path)
            if len(devices) != 1:
                raise SystemExit(
                    "Set serial_device to the ESP32 /dev/serial/by-id path in ID.yaml. Found: "
                    + ", ".join(str(p) for p in devices.values())
                )
            setting = str(next(iter(devices.values())))
        if not os.access(setting, os.R_OK | os.W_OK):
            raise SystemExit(
                f"Cannot open {setting}. Check USB attachment and dialout membership; close Serial Monitor."
            )
        command = [args.agent, "serial", "--dev", setting, "-b", "115200"]
    print(
        f"Agent: {args.transport} · hardware domain {os.environ['ROS_DOMAIN_ID']}",
        flush=True,
    )
    os.execv(command[0], command)


if __name__ == "__main__":
    main()
