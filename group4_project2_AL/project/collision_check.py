# Swept box checks use the same link shapes as Gazebo.
import xml.etree.ElementTree as ET
import yaml
import numpy as np
from scipy.spatial.transform import Rotation
from .arm_model import JOINTS, transform


def overlap(a, b, margin=0.001):
    # Separating-axis test for two oriented boxes, including edge cross-products.
    pa, ra, ha = a
    pb, rb, hb = b
    if np.any(np.abs(pb - pa) > np.abs(ra) @ ha + np.abs(rb) @ hb):
        return False
    axes = [*ra.T, *rb.T]
    axes += [np.cross(x, y) for x in ra.T for y in rb.T]
    for axis in axes:
        length = np.linalg.norm(axis)
        if length < 1e-8:
            continue
        axis = axis / length
        reach = np.abs(ra.T @ axis) @ ha + np.abs(rb.T @ axis) @ hb
        if abs((pb - pa) @ axis) >= reach - margin:
            return False
    return True


class CollisionCheck:
    def __init__(self, model, bases):
        self.model, self.bases = model, bases
        root = ET.parse(model.path).getroot()
        self.links = [
            "base_link",
            "shoulder_link",
            "upper_arm_link",
            "lower_arm_link",
            "wrist_link",
            "gripper_link",
            "moving_jaw_link",
        ]
        self.shapes = {}
        bounds_file = model.path.parent.parent / "config/gripper_clearance.yaml"
        gripper_bounds = (
            yaml.safe_load(bounds_file.read_text()) if bounds_file.exists() else {}
        )
        for link in root.findall("link"):
            shapes = []
            for collision in link.findall("collision"):
                box = collision.find("geometry/box")
                if box is None:
                    continue
                origin = collision.find("origin")
                xyz = (
                    [float(v) for v in origin.get("xyz", "0 0 0").split()]
                    if origin is not None
                    else [0] * 3
                )
                rpy = (
                    [float(v) for v in origin.get("rpy", "0 0 0").split()]
                    if origin is not None
                    else [0] * 3
                )
                shapes.append(
                    (
                        transform(xyz, rpy),
                        np.array([float(v) for v in box.get("size").split()]) / 2,
                    )
                )
            for box in gripper_bounds.get(link.get("name"), []):
                shapes.append((transform(box["xyz"]), np.array(box["size"]) / 2))
            self.shapes[link.get("name")] = shapes

    def boxes(self, arm, q):
        frames = [self.bases[arm]]
        current = frames[0]
        for i in range(6):
            turn = np.eye(4)
            turn[:3, :3] = Rotation.from_rotvec(self.model.axes[i] * q[i]).as_matrix()
            current = current @ self.model.origins[i] @ turn
            frames.append(current)
        boxes = []
        for link, frame in zip(self.links, frames):
            for origin, half in self.shapes.get(link, []):
                pose = frame @ origin
                boxes.append((pose[:3, 3], pose[:3, :3], half))
        return boxes

    def path_bounds(self, arm, start, target):
        samples = max(2, int(np.max(np.abs(target - start)) / 0.04) + 1)
        lows, highs = [], []
        for fraction in np.linspace(0, 1, samples):
            for center, rotation, half in self.boxes(
                arm, start + (target - start) * fraction
            ):
                reach = np.abs(rotation) @ half
                lows.append(center - reach)
                highs.append(center + reach)
        return np.min(lows, axis=0) - 0.003, np.max(highs, axis=0) + 0.003

    def relevant_arms(self, arm, start, target, feedback, targets):
        # Compare swept envelopes, not arbitrary changes at the other end of the line.
        low, high = self.path_bounds(arm, start, target)
        relevant = {arm}
        for name, q in feedback.items():
            if name == arm:
                continue
            other_low, other_high = self.path_bounds(name, q, targets.get(name, q))
            if np.all(high >= other_low) and np.all(other_high >= low):
                relevant.add(name)
        return relevant

    def checked_path(self, arm, start, target, feedback, targets):
        relevant = self.relevant_arms(arm, start, target, feedback, targets)
        others = {name: q for name, q in feedback.items() if name in relevant}
        return self.swept(arm, start, target, others), relevant

    def swept(self, arm, start, target, others):
        stationary = {
            name: self.boxes(name, q) for name, q in others.items() if name != arm
        }
        samples = max(2, int(np.max(np.abs(target - start)) / 0.04) + 1)
        for fraction in np.linspace(0, 1, samples):
            boxes = self.boxes(arm, start + (target - start) * fraction)
            for name, other in stationary.items():
                if any(overlap(a, b) for a in boxes for b in other):
                    return name
        return None
