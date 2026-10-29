# A connection choice must not move simulation commands into the hardware domain.
import importlib.util
from pathlib import Path
import pytest

spec = importlib.util.spec_from_file_location(
    "environment_profile",
    Path(__file__).resolve().parents[1] / "project/environment_profile.py",
)
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


def test_usb_uses_remote_agent_and_wifi_uses_laptop():
    identity = {
        "ros_domain_id": 0,
        "simulation_ros_domain_id": 42,
        "transport": "serial",
        "network": {"agent_port": 8888},
    }
    wifi, local = module.profile_settings(identity, "wifi_laptop")
    assert local and wifi["transport"] == "wifi"
    usb, local = module.profile_settings(identity, "usb_lab")
    assert not local and usb["transport"] == "serial"
    assert identity["transport"] == "serial"


def test_profile_rejects_shared_domain_and_unknown_choice():
    with pytest.raises(ValueError, match="different ROS domains"):
        module.profile_settings({"ros_domain_id": 42}, "usb_lab")
    with pytest.raises(ValueError, match="Choose"):
        module.profile_settings({}, "other")
