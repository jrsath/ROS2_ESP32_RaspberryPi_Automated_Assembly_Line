# Small URDF kinematics helper shared by the simulator and tests.
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation

JOINTS = [
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_roll",
    "gripper",
]
HOME = np.array([0.0, -0.8, 0.0, 0.0, 0.0, 0.5])
BASES = [
    (-0.42, -0.22, 0.0),
    (-0.14, 0.22, np.pi),
    (0.16, -0.22, 0.0),
    (0.44, 0.22, np.pi),
]


def transform(xyz=(0, 0, 0), rpy=(0, 0, 0)):
    result = np.eye(4)
    result[:3, :3] = Rotation.from_euler("xyz", rpy).as_matrix()
    result[:3, 3] = xyz
    return result


class ArmModel:
    def __init__(self, path):
        self.path = Path(path)
        root = ET.parse(path).getroot()
        by_name = {j.attrib["name"]: j for j in root.findall("joint")}
        self.origins, self.axes, self.limits = [], [], []
        for name in JOINTS:
            j = by_name[name]
            origin = j.find("origin")
            self.origins.append(
                transform(
                    [float(x) for x in origin.get("xyz", "0 0 0").split()],
                    [float(x) for x in origin.get("rpy", "0 0 0").split()],
                )
            )
            self.axes.append(
                np.array([float(x) for x in j.find("axis").get("xyz").split()])
            )
            limit = j.find("limit")
            self.limits.append([float(limit.get("lower")), float(limit.get("upper"))])
        self.limits = np.array(self.limits)
        tip = by_name["gripper_frame_joint"].find("origin")
        self.tip = transform(
            [float(x) for x in tip.get("xyz").split()],
            [float(x) for x in tip.get("rpy").split()],
        )

    def fk(self, angles):
        # The moving jaw is a separate branch; it does not move the tool frame.
        frame = np.eye(4)
        joints = []
        for i in range(5):
            frame = frame @ self.origins[i]
            joints.append(frame.copy())
            rot = np.eye(4)
            rot[:3, :3] = Rotation.from_rotvec(self.axes[i] * angles[i]).as_matrix()
            frame = frame @ rot
        joints.append(frame @ self.origins[5])
        return frame @ self.tip, joints

    def solve(self, xyz, seed):
        xyz, seed = np.asarray(xyz), np.asarray(seed)
        if xyz.shape != (3,) or not np.isfinite(xyz).all() or np.linalg.norm(xyz) > 0.6:
            return None
        # Position-only IK suits this five-axis arm. Wrist rotation has its own handle.
        fit = least_squares(
            lambda q: self.fk(np.r_[q, seed[5]])[0][:3, 3] - xyz,
            seed[:5],
            bounds=(self.limits[:5, 0], self.limits[:5, 1]),
            max_nfev=65,
            ftol=1e-7,
            xtol=1e-7,
            gtol=1e-7,
        )
        angles = np.r_[fit.x, seed[5]]
        return (
            angles if np.linalg.norm(self.fk(angles)[0][:3, 3] - xyz) < 0.003 else None
        )


def description(path, prefix, base):
    robot = ET.parse(path).getroot()
    for mesh in robot.iter("mesh"):
        name = mesh.get("filename").removeprefix("package://")
        if name.startswith("assets/"):
            mesh.set("filename", (Path(path).parent / name).resolve().as_uri())
    for link in robot.findall("link"):
        link.set("name", prefix + link.get("name"))
    for joint in robot.findall("joint"):
        joint.set("name", prefix + joint.get("name"))
        for end in ("parent", "child"):
            joint.find(end).set("link", prefix + joint.find(end).get("link"))
    for transmission in robot.findall("transmission"):
        robot.remove(transmission)
    ET.SubElement(robot, "link", name="world")
    fixed = ET.SubElement(robot, "joint", name=prefix + "mount", type="fixed")
    ET.SubElement(fixed, "parent", link="world")
    ET.SubElement(fixed, "child", link=prefix + "base_link")
    ET.SubElement(fixed, "origin", xyz=f"{base[0]} {base[1]} 0", rpy=f"0 0 {base[2]}")
    return ET.tostring(robot, encoding="unicode")


def step_targets(step):
    # Illustrative poses, not calibrated product assembly trajectories.
    goals = np.tile(HOME, (4, 1))
    assembly = {
        "pickup_components": (0.55, -0.6, 0.9, 0.35, 0.0, 1.0),
        "assembly_task_1": (0.1, -0.1, 0.8, 0.5, 0.0, 0.15),
        "assembly_task_2": (-0.2, 0.1, 0.5, 0.65, 0.45, 0.15),
        "assembly_task_3": (0.1, 0.2, 0.4, 0.4, -0.45, 0.15),
        "place_on_conveyor": (-0.45, -0.3, 0.9, 0.7, 0.0, 1.0),
    }
    if step in assembly:
        goals[0] = assembly[step]
        goals[1] = goals[0]
        goals[1, 0] *= -1
    elif step in ("inspect", "reject_product", "transfer_to_inspection"):
        goals[2] = [
            0.65 if step == "reject_product" else -0.25,
            -0.2,
            0.9,
            0.4,
            0.0,
            0.5,
        ]
    elif step in (
        "pickup_box",
        "pickup_assembly",
        "place_in_box",
        "pickup_lid",
        "place_lid",
        "pickup_package",
        "stack_package",
    ):
        index = [
            "pickup_box",
            "pickup_assembly",
            "place_in_box",
            "pickup_lid",
            "place_lid",
            "pickup_package",
            "stack_package",
        ].index(step)
        goals[3] = [
            (-0.55, 0.1, 0.25, -0.7, 0.25, 0.25, 0.7)[index],
            -0.25,
            0.7,
            0.35 + 0.08 * (index % 3),
            0.0,
            0.9 if index % 2 else 0.15,
        ]
    return goals
