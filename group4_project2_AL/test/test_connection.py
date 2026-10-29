# USB selection must not silently choose one of several motor adapters.
import os
from pathlib import Path
import pytest
from group4_project2_al.connection import serial_device, read_identity


def test_no_device_explains_wsl_forwarding(tmp_path):
    with pytest.raises(ValueError, match="usbipd attach"):
        serial_device("auto", tmp_path)


def test_stable_link_and_tty_are_one_device(tmp_path):
    device = tmp_path / "ttyUSB0"
    device.touch()
    stable = tmp_path / "serial/by-id/esp32"
    stable.parent.mkdir(parents=True)
    stable.symlink_to(device)
    assert serial_device("auto", tmp_path) == str(stable)


def test_multiple_devices_require_explicit_selection(tmp_path):
    for name in ("ttyUSB0", "ttyUSB1"):
        (tmp_path / name).touch()
    with pytest.raises(ValueError, match="Several USB"):
        serial_device("auto", tmp_path)
    assert serial_device(str(tmp_path / "ttyUSB1")) == str(tmp_path / "ttyUSB1")


def test_permission_error_is_actionable(tmp_path):
    device = tmp_path / "ttyUSB0"
    device.touch()
    with pytest.raises(ValueError, match="dialout"):
        serial_device(str(device), access=lambda *_: False)


def test_source_identity_takes_precedence(tmp_path, monkeypatch):
    editable = tmp_path / "laptop.yaml"
    editable.write_text("ros_domain_id: 7")
    monkeypatch.setenv("PROJECT2_ID_FILE", str(editable))
    assert read_identity(tmp_path)["ros_domain_id"] == 7
