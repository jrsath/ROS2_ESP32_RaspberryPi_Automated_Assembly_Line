# These profiles select the agent host, not the university motor-driver protocol.
import copy

PROFILES = {
    "wifi_laptop": ("Wi-Fi / laptop ROS", "wifi", True),
    "usb_lab": ("USB / lab PC ROS", "serial", False),
}


def profile_settings(identity, name):
    if name not in PROFILES:
        raise ValueError("Choose Wi-Fi / laptop ROS or USB / lab PC ROS")
    result = copy.deepcopy(identity)
    result["transport"] = PROFILES[name][1]
    domain = int(result["ros_domain_id"])
    if not 0 <= domain <= 232:
        raise ValueError("ros_domain_id must be between 0 and 232")
    if domain == int(result.get("simulation_ros_domain_id", 42)):
        raise ValueError("Hardware and simulation must use different ROS domains")
    if name == "wifi_laptop" and not 1 <= int(result["network"]["agent_port"]) <= 65535:
        raise ValueError("network.agent_port must be between 1 and 65535")
    return result, PROFILES[name][2]
