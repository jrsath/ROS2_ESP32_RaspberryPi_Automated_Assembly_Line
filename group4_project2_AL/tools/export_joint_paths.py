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
parser.add_argument("--products", type=int, choices=range(6), default=5)
parser.add_argument("--seconds", type=int, default=120)
args = parser.parse_args()
stack_count = args.products
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

# Export candidates even if a later path fails, with the failure recorded beside them.
import signal
from group4_project2_al.path_store import ACTIONS, geometry_signature

original = planner.contact_transfer


def checked(arm, poses, contact, **options):
    plan = original(arm, poses, contact, **options)
    key = next(
        (name for name, value in config["contacts"].items() if value == contact), "tray"
    )
    cad = {
        "base": "1V4",
        "belt": "1V4",
        "inspect": "1V4",
        "gear": "2V4",
        "handoff": "3V4",
        "insert": "3V4",
    }.get(key)
    kind = "box" if key == "stack" else key
    mesh_path = root / (
        f"models/product/{cad}_centered.stl" if cad else f"models/packaging/{kind}.stl"
    )
    q = plan["q"][0]
    pads = planner.pad_points(planner.tool(arm, q), q[5], plan["pad_y"], plan["pad_z"])
    check_contacts(
        pads,
        plan["poses"][0],
        mesh_path,
        kind,
        faces=carrier_faces(config["kit_trays"]) if key == "tray" else None,
    )
    return plan


planner.contact_transfer = checked
preview, error = {}, None


def expired(*unused):
    raise TimeoutError(f"Planning exceeded {args.seconds}s")


signal.signal(signal.SIGALRM, expired)
signal.alarm(args.seconds)
try:
    plans = make_plans(planner, config, starts, alignment, stack_count, preview)
except (ValueError, TimeoutError) as failure:
    error = str(failure)
finally:
    signal.alarm(0)
paths = preview.get("paths", {})
missing = [name for name in ACTIONS if name not in paths]
output = dict(
    units="radians",
    joint_order=[
        "shoulder_pan",
        "shoulder_lift",
        "elbow_flex",
        "wrist_flex",
        "wrist_roll",
        "gripper",
    ],
    verified_for_hardware=False,
    full_cycle_validated=False,
    geometry_signature=geometry_signature(root),
    error=error,
    missing_paths=missing,
    paths=paths,
)
path = root / "config/solved_joint_paths.yaml"
path.write_text(
    "# Generated joint candidates. Capture and calibrate physical poses before use.\n"
    + yaml.safe_dump(output, sort_keys=False)
)
seed_file.write_text(json.dumps(planner.contact_seeds, indent=2) + "\n")
print(
    f"Saved {len(paths)} tasks / {sum(len(v['points']) for v in paths.values())} movements to {path}"
)
if error:
    print(error)
if missing:
    print("Still needed: " + ", ".join(missing))
raise SystemExit(2 if error or missing else 0)
