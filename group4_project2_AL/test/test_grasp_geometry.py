# Check actual triangle distances, including the gaps rejected before attachment.
import numpy as np
import pytest
from group4_project2_al.grasp_geometry import surface_distance, check_contacts


def test_triangle_distance_uses_faces_edges_and_vertices():
    triangle = np.array([[[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]]])
    assert surface_distance(np.array([0.2, 0.2, 0.003]), triangle) == pytest.approx(
        0.003
    )
    assert surface_distance(np.array([0.5, -0.2, 0.0]), triangle) == pytest.approx(0.2)
    assert surface_distance(np.array([-1.0, 0.0, 0.0]), triangle) == pytest.approx(1.0)


def test_contact_rejects_air_grips_and_off_centre_boxes(monkeypatch):
    import group4_project2_al.grasp_geometry as geometry

    # Two flat outer box walls, wide enough to include the test contacts.
    faces = np.array(
        [[[x, -0.1, -0.1], [x, 0.1, -0.1], [x, 0.0, 0.1]] for x in (-0.0255, 0.0255)]
    )
    monkeypatch.setattr(geometry, "mesh", lambda _: faces)
    pads = np.array([[-0.0255, 0.0, 0.0], [0.0255, 0.0, 0.0]])
    assert max(check_contacts(pads, np.eye(4), "box.stl", "box")) < 1e-9
    with pytest.raises(ValueError, match="maximum is 1mm"):
        check_contacts(pads + [0.003, 0.0, 0.0], np.eye(4), "box.stl", "box")
    with pytest.raises(ValueError, match="centred"):
        check_contacts(pads + [0.0, 0.01, 0.0], np.eye(4), "box.stl", "box")


def test_gear_bore_is_not_a_valid_grip(monkeypatch):
    import group4_project2_al.grasp_geometry as geometry

    monkeypatch.setattr(geometry, "mesh", lambda _: None)
    monkeypatch.setattr(geometry, "surface_distance", lambda *args: 0.0)
    with pytest.raises(ValueError, match="outside circumference"):
        check_contacts([[0.006, 0, 0], [-0.006, 0, 0]], np.eye(4), "gear.stl", "gear")


def test_v6_insert_grip_stays_on_straight_outer_faces():
    from pathlib import Path
    import yaml
    from itertools import product

    root = Path(__file__).resolve().parents[1]
    contact = yaml.safe_load((root / "config/production.yaml").read_text())["contacts"][
        "handoff"
    ]
    for point in contact["points"]:
        # Check the whole permitted slide/height range, not just its centre.
        for slide, skew, height in product((-1, 1), (-1, 1), contact["height_range"]):
            p = np.array(point)
            p[0] += slide * contact["slide_m"] + skew * contact["skew_m"]
            p[2] += height
            from group4_project2_al.grasp_geometry import mesh

            assert (
                surface_distance(p, mesh(str(root / "models/product/3V4_centered.stl")))
                < 0.001
            )
