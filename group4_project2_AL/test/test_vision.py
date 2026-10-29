# Check the CAD reference and the two visible defect examples.
import json
from pathlib import Path
import cv2
import numpy as np
from scipy.spatial.transform import Rotation
from group4_project2_al.vision import (
    triangles,
    stl_template,
    outline_score,
    inspect_frame,
)

ROOT = Path(__file__).resolve().parents[1]


def render(mesh, angle=0):
    xy = (mesh @ Rotation.from_euler("xyz", [angle, 0, 0]).as_matrix().T)[:, :, :2]
    low, high = xy.min((0, 1)), xy.max((0, 1))
    xy = (xy - low) * 460 / max(high - low) + 20
    mask = np.zeros((512, 512), np.uint8)
    for triangle in xy.astype(np.int32):
        cv2.fillConvexPoly(mask, triangle, 255)
    return mask


def test_visible_defects():
    folder = ROOT / "models/product"
    alignment = json.loads((folder / "mesh_alignment.json").read_text())[
        "relative_to_1V4_center"
    ]
    parts = {}
    for name in ("1V4", "2V4", "3V4"):
        pose = alignment[name]
        parts[name] = (
            triangles(folder / (name + "_centered.stl"))
            @ Rotation.from_euler("xyz", pose["rpy_rad"]).as_matrix().T
            + pose["xyz_m"]
        )
    reference = stl_template(folder / "1_assembled.STL", [0, 0, 0])
    good = render(np.concatenate(list(parts.values())))
    missing = render(np.concatenate([parts["1V4"], parts["3V4"]]))
    unseated = render(
        np.concatenate([parts["1V4"], parts["2V4"], parts["3V4"] + [0.005, 0, 0]])
    )
    assert outline_score(good, reference) < 0.04
    assert outline_score(missing, reference) > 0.04
    # The end fault is hidden in the overhead silhouette; use the angled view.
    angled = stl_template(folder / "1_assembled.STL", [1.0, 0, 0])
    good_end = render(np.concatenate(list(parts.values())), 1.0)
    bad_end = render(
        np.concatenate([parts["1V4"], parts["2V4"], parts["3V4"] + [0.005, 0, 0]]), 1.0
    )
    assert outline_score(good_end, angled) < 0.01
    assert outline_score(bad_end, angled) > 0.01


def test_blank_frame_is_not_a_pass():
    config = dict(roi=[0, 0, 1, 1], min_area=1500, match_limit=0.02)
    image = np.zeros((480, 640, 3), np.uint8)
    _, score = inspect_frame(image, np.zeros((256, 256), np.uint8), config)
    assert score is None


def test_simulated_camera_runs_the_image_classifier():
    from group4_project2_al.vision import render_parts
    from group4_project2_al.arm_model import transform

    folder = ROOT / "models/product"
    alignment = json.loads((folder / "mesh_alignment.json").read_text())[
        "relative_to_1V4_center"
    ]
    meshes = {
        part: triangles(folder / (cad + "_centered.stl"))
        for part, cad in (("base", "1V4"), ("gear", "2V4"), ("insert", "3V4"))
    }
    poses = {
        part: transform(alignment[cad]["xyz_m"], alignment[cad]["rpy_rad"])
        for part, cad in (("base", "1V4"), ("gear", "2V4"), ("insert", "3V4"))
    }
    config = dict(roi=[0.15, 0.15, 0.7, 0.7], min_area=1500, match_limit=0.04)
    for case in ("good", "missing_gear", "unseated_insert"):
        actual = {name: pose.copy() for name, pose in poses.items()}
        if case == "missing_gear":
            actual.pop("gear")
        if case == "unseated_insert":
            actual["insert"][0, 3] += 0.005
        passed = []
        for view, limit in (([0, 0, 0], 0.04), ([1.35, 0, 0], 0.025)):
            image = render_parts(meshes, actual, np.eye(4), view)
            reference = stl_template(folder / "1_assembled.STL", view)
            _, score = inspect_frame(image, reference, dict(config, match_limit=limit))
            passed.append(score is not None and score <= limit)
        assert all(passed) == (case == "good"), (case, passed)
