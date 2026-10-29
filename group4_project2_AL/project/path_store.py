# Named joint paths use radians and the same six joints as the arm controller.
import math
import os
from pathlib import Path
import tempfile
import yaml

ACTIONS = {
    "load_carrier": ("assembly_left", "Loaded tray: rack to bench"),
    "base": ("assembly_right", "Base: feeder to assembly fixture"),
    "gear": ("assembly_left", "Gear: outside grip to base pin"),
    "handoff": ("assembly_left", "Insert: feeder to handoff"),
    "insert": ("assembly_right", "Insert: handoff to base"),
    "belt": ("assembly_right", "Finished toy: fixture to conveyor"),
    "tray": ("assembly_left", "Empty tray: bench to left discard"),
    "box": ("packaging", "Box: shelf to packaging holder"),
    "inspect": ("inspection", "Toy: conveyor to box"),
    "lid": ("packaging", "Lid: shelf to box"),
    **{
        f"stack_{i}": ("packaging", f"Closed box: holder to output slot {i+1}")
        for i in range(5)
    },
}
POINTS = ("approach_pickup", "pickup", "approach_place", "place")


def angles(values):
    q = [float(v) for v in values]
    if len(q) != 6 or not all(math.isfinite(v) for v in q):
        raise ValueError("Expected six finite joint angles in radians")
    return q


def write_sections(path, sections):
    # Replace only the edited YAML sections, preserving the surrounding comments.
    path = Path(path)
    text = path.read_text()
    for name, data in sections.items():
        block = yaml.safe_dump({name: data}, sort_keys=False)
        document = yaml.compose(text)
        for key, value in document.value:
            if key.value == name:
                end = value.end_mark.index
                lines = text[:end].splitlines(keepends=True)
                while lines and (
                    not lines[-1].strip() or lines[-1].lstrip().startswith("#")
                ):
                    end -= len(lines.pop())
                text = text[: key.start_mark.index] + block + text[end:]
                break
        else:
            text = text.rstrip() + "\n\n" + block
    # A failed write must not leave half a waypoint file behind.
    fd, temporary = tempfile.mkstemp(
        dir=path.parent, prefix=".waypoints-", suffix=".yaml"
    )
    try:
        with os.fdopen(fd, "w") as stream:
            stream.write(text)
        os.replace(temporary, path)
    finally:
        if os.path.exists(temporary):
            os.unlink(temporary)


def save_point(path, action, point, values, label="", phase="pickup"):
    if action not in ACTIONS:
        raise ValueError("Unknown arm action")
    q = angles(values)
    data = yaml.safe_load(Path(path).read_text())
    if point in POINTS:
        overrides = data.get("joint_overrides", {})
        overrides.setdefault(action, {})[point] = q
        write_sections(path, {"joint_overrides": overrides})
    else:
        if phase not in ("pickup", "place") or not label.strip():
            raise ValueError(
                "Give the intermediate point a name and pickup/place phase"
            )
        paths = data.get("path_points", {})
        paths.setdefault(action, []).append(
            dict(label=label.strip(), phase=phase, joints=q)
        )
        write_sections(path, {"path_points": paths})


def remove_point(path, action, point):
    data = yaml.safe_load(Path(path).read_text())
    if point in POINTS:
        values = data.get("joint_overrides", {})
        values.get(action, {}).pop(point, None)
        if not values.get(action):
            values.pop(action, None)
        write_sections(path, {"joint_overrides": values})
    else:
        values = data.get("path_points", {})
        del values[action][int(point)]
        write_sections(path, {"path_points": values})


def geometry_signature(root):
    # Invalidate saved solutions whenever the CAD, robot or station poses change.
    import hashlib

    root = Path(root)
    digest = hashlib.sha256()
    inputs = [root / "config/production.yaml", root / "config/assembly_run.yaml"]
    inputs += sorted((root / "models/product").glob("*"))
    inputs += sorted((root / "models/v6").glob("*.stl"))
    inputs += [
        root / "vendor/group4_assembly_line/urdf/so101.urdf.xacro",
        root / "vendor/group4_assembly_line/config/rig_layout.yaml",
    ]
    for path in inputs:
        if path.is_file():
            digest.update(str(path.relative_to(root)).encode())
            digest.update(path.read_bytes())
    return digest.hexdigest()


def load_solved_paths(root):
    # A saved solution is a simulation reference, never a hardware calibration.
    root = Path(root)
    path = root / "config/solved_joint_paths.yaml"
    if not path.exists():
        return {}
    try:
        data = yaml.safe_load(path.read_text())
        if data["geometry_signature"] != geometry_signature(root):
            return {}
        for row in data["paths"].values():
            for values in row["points"].values():
                angles(values)
        return data["paths"]
    except (KeyError, TypeError, ValueError, yaml.YAMLError):
        return {}
