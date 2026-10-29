#!/usr/bin/env python3
# Check planned contacts after editing station poses or grip offsets.
import argparse
import json
from pathlib import Path
import numpy as np
import yaml
from ament_index_python.packages import get_package_share_directory
from scipy.spatial.transform import Rotation
from group4_project2_al.arm_model import ArmModel, transform
from group4_project2_al.assembly_run import initial_poses
from group4_project2_al.collision_check import CollisionCheck, overlap
from group4_project2_al.production_plan import TransferPlanner, frame, make_plans
from group4_project2_al.vision import triangles
from group4_project2_al.grasp_geometry import check_contacts, triangles_in_box
from group4_project2_al.kit_trays import faces as carrier_faces, segments, tray_pose

parser = argparse.ArgumentParser(
    description="Check planned STL contacts and clearances"
)
parser.add_argument("--section", choices=("all", "assembly"), default="all")
args = parser.parse_args()
stack_count = 0 if args.section == "assembly" else 5
root = Path(__file__).resolve().parents[1]
rig = Path(get_package_share_directory("group4_assembly_line"))
config = yaml.safe_load((root / "config/production.yaml").read_text())
layout = yaml.safe_load((rig / "config/rig_layout.yaml").read_text())
model = ArmModel(rig / "urdf/so101.urdf.xacro")
planner = TransferPlanner(
    model, {n: transform(v["xyz"], v["rpy"]) for n, v in layout["robots"].items()}
)
alignment = json.loads((root / "models/product/mesh_alignment.json").read_text())[
    "relative_to_1V4_center"
]
assembly_config = yaml.safe_load((root / "config/assembly_run.yaml").read_text())
config["kit_trays"] = assembly_config["tray"].get("carrier")
stock = initial_poses(assembly_config, 1)
starts = {part: transform(v.xyz, v.rpy) for (_, part), v in stock.items()}
seed_file = root / "config/grip_seeds.json"
planner.contact_seeds = json.loads(seed_file.read_text()) if seed_file.exists() else {}
plans = make_plans(planner, config, starts, alignment, stack_count)
box = plans["box"]["poses"][1]
size = np.asarray(config["box_size"])
wall = config["wall"]

# Every product vertex must fit above the floor and inside the open box.
points = []
for part in ("1V4", "2V4", "3V4"):
    offset = alignment[part]
    pose = (
        np.linalg.inv(box)
        @ plans["inspect"]["poses"][1]
        @ transform(offset["xyz_m"], offset["rpy_rad"])
    )
    mesh = triangles(root / f"models/product/{part}_centered.stl").reshape(-1, 3)
    points.append(mesh @ pose[:3, :3].T + pose[:3, 3])
points = np.concatenate(points)
low = np.r_[-size[:2] / 2 + wall, -size[2] / 2 + wall]
high = np.r_[size[:2] / 2 - wall, size[2] / 2]
assert np.all(points.min(0) >= low - 1e-6), "Product intersects box floor or wall"
assert np.all(points.max(0) <= high + 1e-6), "Product protrudes from box"
print(
    "Product clearance, lower/upper (mm):",
    np.round((points.min(0) - low) * 1000, 2),
    np.round((high - points.max(0)) * 1000, 2),
)

# The completed-box pickup must clear the lid roof, including its open approach.
lid = plans["lid"]["poses"][1]
roof = (
    (lid @ np.array([0, 0, 0.001, 1]))[:3],
    lid[:3, :3],
    np.array([0.02745, 0.05025, 0.001]),
)
collision = CollisionCheck(model, planner.bases)


def hand_intersects(arm, q, obstacle):
    pose = np.eye(4)
    pose[:3, 3], pose[:3, :3] = obstacle[0], obstacle[1]
    grip = planner.tool(arm, q) @ np.linalg.inv(model.tip)
    jaw = grip @ model.origins[5] @ transform(rpy=[0, 0, q[5]])
    for filename, hand_pose in (("Fixed_Gripper", grip), ("Moving_Jaw", jaw)):
        faces = triangles(rig / f"models/project2_rig/meshes/{filename}.stl")
        faces = faces @ hand_pose[:3, :3].T + hand_pose[:3, 3]
        if triangles_in_box(faces, pose, obstacle[2]):
            return True
    return False


for name, plan in plans.items():
    if not isinstance(plan, dict) or "contact" not in plan:
        continue
    cad = {
        "support": "1V4",
        "base": "1V4",
        "belt": "1V4",
        "inspect": "1V4",
        "gear": "2V4",
        "handoff": "3V4",
        "insert": "3V4",
    }.get(name)
    kind = "box" if name.startswith("stack_") else name
    path = root / (
        f"models/product/{cad}_centered.stl" if cad else f"models/packaging/{kind}.stl"
    )
    q = plan["q"][0]
    pads = planner.pad_points(
        planner.tool(plan["arm"], q), q[5], plan["pad_y"], plan["pad_z"]
    )
    check_contacts(
        pads,
        plan["poses"][0],
        path,
        kind,
        faces=carrier_faces(config["kit_trays"]) if name == "tray" else None,
    )

# Check the actual finger meshes against all five carriers before gripping.
carrier = config.get("kit_trays")
if carrier:
    obstacles = []
    for row in range(5):
        pose = tray_pose(carrier, row)
        for xyz, size in segments(carrier):
            local = pose @ transform(xyz)
            obstacles.append((local[:3, 3], local[:3, :3], np.asarray(size) / 2))
    for name in ("base", "gear", "handoff", "tray"):
        plan = plans[name]
        for endpoint in (plan["q"][0], plan["approach"][0]):
            for opening in (plan["q"][0][5], plan["open"]):
                q = endpoint.copy()
                q[5] = opening
                assert not any(
                    hand_intersects(plan["arm"], q, obstacle) for obstacle in obstacles
                ), f"{name}: fingers intersect a kit carrier"
    # The whole empty tray must clear the table edge at release.
    plan = plans["tray"]
    pose = planner.tool(plan["arm"], plan["q"][1]) @ plan["held"]
    points = carrier_faces(carrier).reshape(-1, 3) @ pose[:3, :3].T + pose[:3, 3]
    assert points[:, 0].max() < -0.90, "Empty tray still overlaps the table at drop-off"

for index in range(stack_count):
    plan = plans[f"stack_{index}"]
    for q in (plan["q"][0], plan["approach"][0]):
        for opening in (q[5], plan["open"]):
            joints = q.copy()
            joints[5] = opening
            assert not hand_intersects(
                plan["arm"], joints, roof
            ), f"{index}: box grip crosses lid roof"

# Check the gripper against previously stacked packages, including lid handles.
for index in range(1, stack_count):
    plan = plans[f"stack_{index}"]
    for earlier in range(index):
        pose = frame(config["stack"][earlier])
        obstacles = [(pose[:3, 3], pose[:3, :3], np.array([0.0255, 0.0483, 0.012]))]
        for height, half in (
            (0.0135, [0.02745, 0.05025, 0.001]),
            (0.0215, [0.002, 0.025, 0.004]),
        ):
            obstacles.append(
                ((pose @ np.array([0, 0, height, 1]))[:3], pose[:3, :3], np.array(half))
            )
        for endpoint in ("q", "approach"):
            for angle in (plan["q"][1][5], plan["open"]):
                q = plan[endpoint][1].copy()
                q[5] = angle
                assert not any(
                    hand_intersects(plan["arm"], q, o) for o in obstacles
                ), f"{index}: gripper overlaps stacked package {earlier}"

# Final approach orientations should remain close to their contact orientations.
for name in ("insert", "lid", "inspect", "box"):
    plan = plans[name]
    actual = planner.tool(plan["arm"], plan["approach"][1]) @ plan["held"]
    angle = Rotation.from_matrix(
        plan["poses"][1][:3, :3].T @ actual[:3, :3]
    ).magnitude()
    assert angle < 0.03, name + ": approach rotates the part"
# Check the insert release/withdrawal beside the stationary support arm.
plan = plans["insert"]
contact = plan["q"][1].copy()
release = contact.copy()
release[5] = min(plan["open"], contact[5] + 0.1)
retreat = plan["approach"][1].copy()
retreat[5] = release[5]
home = np.asarray(config["home"], dtype=float)
arm_support = config.get("support", {}).get("mode", "fixture") == "arm"
support_pose = plans["hold"] if arm_support else home
for start, end in ((contact, release), (release, retreat)):
    assert (
        collision.swept(plan["arm"], start, end, {"assembly_left": support_pose})
        is None
    ), "Insert withdrawal overlaps support arm"
# Clear the supporting arm, then bring the narrow jaws home before opening fully.
if arm_support:
    support = plans["hold"].copy()
    support_open = support.copy()
    support_open[5] = min(1.0, support[5] + 0.15)
    home = np.asarray(config["home"], dtype=float)
    for start, end in ((support, support_open), (support_open, home)):
        assert (
            collision.swept("assembly_left", start, end, {"assembly_right": retreat})
            is None
        ), "Support withdrawal overlaps arm 2"
right_home = home.copy()
right_home[5] = release[5]
for start, end in ((retreat, right_home), (right_home, home)):
    assert (
        collision.swept("assembly_right", start, end, {"assembly_left": home}) is None
    ), "Arm 2 cannot clear the fixture before opening"
print(
    f"Contact, approach and insert withdrawal checks passed; {stack_count} stack targets checked."
)
print("These geometric checks do not validate friction or physical grip force.")
