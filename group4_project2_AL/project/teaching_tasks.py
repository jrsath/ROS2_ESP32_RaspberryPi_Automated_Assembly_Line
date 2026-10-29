# Task order is for taught-path replay; production dependencies stay in the sequencer.
import copy
import uuid
from pathlib import Path
import yaml
from .path_store import ACTIONS, POINTS, angles, write_sections

ARMS = ("assembly_left", "assembly_right", "inspection", "packaging")
MOVEMENTS = {
    "approach_pickup": "Approach pickup",
    "pickup": "Grip position",
    "approach_place": "Approach destination",
    "place": "Place position",
}


def load_tasks(path):
    data = yaml.safe_load(Path(path).read_text()) or {}
    if "teaching_tasks" in data:
        return copy.deepcopy(data["teaching_tasks"])
    tasks = {arm: [] for arm in ARMS}
    for action, (arm, label) in ACTIONS.items():
        saved = data.get("joint_overrides", {}).get(action, {})
        movements = []
        for key in POINTS:
            phase = (
                "pickup"
                if key == "approach_pickup"
                else "place" if key == "approach_place" else None
            )
            if phase:
                for point in data.get("path_points", {}).get(action, []):
                    if point["phase"] == phase:
                        movements.append(
                            dict(
                                id=uuid.uuid4().hex,
                                label=point["label"],
                                joints=point["joints"],
                            )
                        )
            movements.append(
                dict(id=key, label=MOVEMENTS[key], point=key, joints=saved.get(key))
            )
        tasks[arm].append(
            dict(id=action, action=action, label=label, movements=movements)
        )
    return tasks


def save_tasks(path, tasks):
    if set(tasks) != set(ARMS):
        raise ValueError("The path catalogue needs four arm columns")
    seen = set()
    for arm in ARMS:
        for task in tasks[arm]:
            if not task["label"].strip() or task["id"] in seen:
                raise ValueError("Tasks need a name and unique ID")
            seen.add(task["id"])
            if task.get("action") and ACTIONS[task["action"]][0] != arm:
                raise ValueError("A production task must stay with its assigned arm")
            ids = set()
            for movement in task["movements"]:
                if movement["id"] in ids or not movement["label"].strip():
                    raise ValueError("Movements need a name and unique ID")
                ids.add(movement["id"])
                if movement.get("joints") is not None:
                    angles(movement["joints"])
    write_sections(path, {"teaching_tasks": tasks})


def resolve(task, solved):
    result = []
    for movement in task["movements"]:
        q = movement.get("joints")
        if q is None:
            q = (
                solved.get(task.get("action"), {})
                .get("points", {})
                .get(movement.get("point"))
            )
        result.append(None if q is None else angles(q))
    return result
