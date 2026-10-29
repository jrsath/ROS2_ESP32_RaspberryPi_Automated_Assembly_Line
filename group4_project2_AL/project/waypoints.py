# Load task poses before allowing the process to start.
from pathlib import Path
import numpy as np
import yaml
from .arm_model import JOINTS
from .cell import ASSEMBLY, INSPECTION, PACKAGING

ARMS = ("assembly_left", "assembly_right", "inspection", "packaging")
STEPS = tuple(
    dict.fromkeys(
        ASSEMBLY + INSPECTION + ["transfer_to_packaging", "reject_product"] + PACKAGING
    )
)


def load_waypoints(path, limits, space):
    data = yaml.safe_load(Path(path).read_text())
    if (
        data.get("joint_names") != JOINTS
        or data.get("units") != "radians"
        or data.get("space") != space
    ):
        raise ValueError(
            "Waypoint joint order, units or coordinate convention does not match this backend"
        )
    result = {}
    for step in STEPS:
        entries = data.get("steps", {}).get(step, {}).get("waypoints", [])
        if not entries:
            raise ValueError("Missing waypoints for " + step)
        result[step] = []
        for entry in entries:
            poses = entry.get("arms", {})
            if set(poses) != set(ARMS):
                raise ValueError(step + ": specify all four arms")
            q = np.array([poses[name] for name in ARMS], dtype=float)
            if (
                q.shape != (4, 6)
                or not np.isfinite(q).all()
                or np.any(q < limits[:, 0])
                or np.any(q > limits[:, 1])
            ):
                raise ValueError(step + ": nonfinite angles or joint limit exceeded")
            dwell = float(entry.get("dwell_s", 0.0))
            if not np.isfinite(dwell) or not 0 <= dwell <= 60:
                raise ValueError(step + ": dwell_s must be between 0 and 60")
            result[step].append((q, dwell, str(entry.get("label", "work"))))
    return result
