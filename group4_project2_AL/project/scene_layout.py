# Keep waypoint files relative to the table; shift into Gazebo world once.
from copy import deepcopy


def raised_config(config, height):
    result = deepcopy(config)
    for key in (
        "fixture",
        "handoff",
        "conveyor_in",
        "conveyor_out",
        "reject",
        "box_feed",
        "lid_feed",
        "box_nest",
    ):
        result[key][2] += height
    for pose in result.get("reject_slots", []):
        pose[2] += height
    for pose in result.get("assembly_output", []):
        pose[2] += height
    for pose in result["stack"]:
        pose[2] += height
    return result
