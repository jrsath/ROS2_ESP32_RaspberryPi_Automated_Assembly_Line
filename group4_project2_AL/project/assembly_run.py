# Positions for the five-step Gazebo assembly trial.
from dataclasses import dataclass
import math
import json
from pathlib import Path
from ament_index_python.packages import get_package_share_directory


PARTS = ("base_1v4", "insert_3v4", "pin_2v4")
STEPS = (
    "pickup_components",
    "assembly_task_1",
    "assembly_task_2",
    "assembly_task_3",
    "place_on_conveyor",
)


@dataclass(frozen=True)
class PartPose:
    xyz: tuple
    rpy: tuple = (0.0, 0.0, 0.0)


def check_count(count, maximum=5):
    if type(count) is not int or not 1 <= count <= maximum:
        raise ValueError(f"Toy count must be an integer from 1 to {maximum}")
    return count


def initial_poses(config, count, maximum=5):
    check_count(count, maximum)
    if "parts_local" in config["tray"]:
        from .kit_trays import tray_pose
        from .arm_model import transform
        from scipy.spatial.transform import Rotation

        result = {}
        for toy in range(count):
            carrier = tray_pose(config["tray"]["carrier"], toy)
            for part, values in config["tray"]["parts_local"].items():
                pose = carrier @ transform(values[:3], values[3:])
                result[(toy, part)] = PartPose(
                    tuple(pose[:3, 3]), tuple(Rotation.from_matrix(pose[:3, :3]).as_euler("xyz"))
                )
        return result
    if "parts" in config["tray"]:
        tray = config["tray"]
        return {
            (toy, part): PartPose(
                tuple(
                    value + toy * shift
                    for value, shift in zip(
                        tray["parts"][part]["xyz"], tray["row_step"]
                    )
                ),
                tuple(tray["parts"][part]["rpy"]),
            )
            for toy in range(count)
            for part in PARTS
        }
    x, y, z = config["tray"]["first_part"]
    offsets = config["tray"].get("part_offsets_x")
    if offsets is None:
        dx = config["tray"]["part_spacing_x"]
        offsets = [index * dx for index in range(len(PARTS))]
    dy = config["tray"]["toy_spacing_y"]
    dz = config["tray"]["toy_rise_z"]
    return {
        (toy, part): PartPose((x + offsets[part_index], y + toy * dy, z + toy * dz))
        for toy in range(count)
        for part_index, part in enumerate(PARTS)
    }


def step_poses(config, toy, step):
    # Return the models moved by this step in the pose-only trial.
    if step not in STEPS or not 0 <= toy < 5:
        raise ValueError("Unknown assembly step or toy index")
    wx, wy, wz = config["work_center"]
    # Use the active CAD alignment for both the trial and the production run.
    folder = Path(get_package_share_directory("group4_project2_al")) / "models/product"
    alignment = json.loads((folder / "mesh_alignment.json").read_text())[
        "relative_to_1V4_center"
    ]
    insert = alignment["3V4"]["xyz_m"]
    top = alignment["2V4"]["xyz_m"]
    top_yaw = alignment["2V4"]["rpy_rad"][2]
    if step == "pickup_components":
        return {"base_1v4": PartPose((wx, wy, wz))}
    if step == "assembly_task_1":
        return {
            "pin_2v4": PartPose(
                (wx + top[0], wy + top[1], wz + top[2] + 0.02), (0, 0, top_yaw)
            )
        }
    if step == "assembly_task_2":
        return {
            "pin_2v4": PartPose(
                (wx + top[0], wy + top[1], wz + top[2]), (0, 0, top_yaw)
            )
        }
    if step == "assembly_task_3":
        return {
            "insert_3v4": PartPose((wx + insert[0], wy + insert[1], wz + insert[2]))
        }
    cx, cy, cz = config["conveyor_center"]
    cx += toy * config["conveyor_spacing_x"]
    return {
        "base_1v4": PartPose((cx, cy, cz)),
        "insert_3v4": PartPose((cx + insert[0], cy + insert[1], cz + insert[2])),
        "pin_2v4": PartPose((cx + top[0], cy + top[1], cz + top[2]), (0, 0, top_yaw)),
    }


def tray_pose(config, count):
    check_count(count)
    tray = config["tray"]
    x, y, z = tray["first_part"]
    offsets = tray.get(
        "part_offsets_x", [0, tray["part_spacing_x"], 2 * tray["part_spacing_x"]]
    )
    return PartPose(
        (
            x + (min(offsets) + max(offsets)) / 2,
            y + 2 * tray["toy_spacing_y"],
            z + 2 * tray["toy_rise_z"] - 0.015,
        ),
        (math.atan2(tray["toy_rise_z"], tray["toy_spacing_y"]), 0, 0),
    )
