# Connection settings shared by hardware launch and the connection checker.
import os
from pathlib import Path
import yaml


def identity_path(share):
    # run.sh points to the editable source ID.yaml, even with a copied installation.
    return Path(os.environ.get("PROJECT2_ID_FILE", str(Path(share) / "ID.yaml")))


def read_identity(share):
    return yaml.safe_load(identity_path(share).read_text())


def serial_device(setting="auto", dev_root=Path("/dev"), access=os.access):
    dev_root = Path(dev_root)
    if setting and setting != "auto":
        path = Path(setting)
        if not path.exists():
            raise ValueError(f"ESP32 USB device not found: {path}. Set serial_device: auto or a /dev/serial/by-id path in ID.yaml. On WSL, attach the USB device with usbipd first.")
    else:
        # Stable names first; deduplicate links to the same device.
        candidates = {}
        for pattern in ("serial/by-id/*", "ttyUSB*", "ttyACM*"):
            for item in sorted(dev_root.glob(pattern)):
                if item.exists():
                    candidates.setdefault(item.resolve(), item)
        if not candidates:
            raise ValueError("No USB serial device found. Use a data cable. On WSL run usbipd attach --wsl --busid YOUR_BUSID in Windows PowerShell, then retry.")
        if len(candidates) != 1:
            names = ", ".join(map(str, candidates.values()))
            raise ValueError(f"Several USB serial devices found: {names}. Set the ESP32 serial_device explicitly in ID.yaml; do not select an arm adapter.")
        path = next(iter(candidates.values()))
    if not access(path, os.R_OK | os.W_OK):
        raise ValueError(f"No read/write permission for {path}. Run sudo usermod -aG dialout $USER, then log out and back in.")
    return str(path)


def agent_args(data):
    transport = data.get("transport", "serial")
    if transport == "serial":
        return ["serial", "--dev", serial_device(data.get("serial_device", "auto")), "-b", "115200"]
    if transport == "wifi":
        port = int(data["network"]["agent_port"])
        if not 1 <= port <= 65535:
            raise ValueError("network.agent_port must be between 1 and 65535")
        return ["udp4", "--port", str(port)]
    raise ValueError("transport must be serial or wifi in ID.yaml")
