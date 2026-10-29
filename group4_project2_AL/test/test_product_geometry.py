# V6 retains the floor, gear shoulder and slider guide datums used by the fixture.
import json
from pathlib import Path

import numpy as np
import yaml
from scipy.spatial.transform import Rotation

from group4_project2_al.vision import triangles


ROOT = Path(__file__).resolve().parents[1]
FOLDER = ROOT / "models/product"
DATA = json.loads((FOLDER / "mesh_alignment.json").read_text())
PARTS = DATA["relative_to_1V4_center"]


def test_v6_dimensions_and_seated_datums():
    assert DATA["revision"] == "V6"
    bounds = {}
    for part, size in (
        ("1V4", [76.88757, 28, 18]),
        ("2V4", [28.1981, 28.08373, 10]),
        ("3V4", [50.24739, 25, 13.8]),
    ):
        mesh = triangles(FOLDER / (part + "_centered.stl"))
        bounds[part] = np.array([mesh.min((0, 1)), mesh.max((0, 1))])
        np.testing.assert_allclose(
            np.diff(bounds[part], axis=0)[0] * 1000, size, atol=0.002
        )
    floor = bounds["1V4"][0, 2]
    # These are the unchanged mating levels measured upward from the base floor.
    gear_bottom = PARTS["2V4"]["xyz_m"][2] + bounds["2V4"][0, 2]
    slider_bottom = PARTS["3V4"]["xyz_m"][2] + bounds["3V4"][0, 2]
    assert abs(gear_bottom - floor - 0.0028) < 0.000002
    assert abs(slider_bottom - floor - 0.0018) < 0.000002


def test_v6_reference_contains_the_same_seated_parts():
    assembled = []
    for part, pose in PARTS.items():
        mesh = triangles(FOLDER / (part + "_centered.stl"))
        assembled.append(
            mesh @ Rotation.from_euler("xyz", pose["rpy_rad"]).as_matrix().T
            + pose["xyz_m"]
        )
    # Camera and Gazebo must use the same revision and seated transforms.
    reference = triangles(FOLDER / "1_assembled.STL") * 0.001
    np.testing.assert_allclose(reference, np.concatenate(assembled), atol=1e-8)


def test_longer_insert_finger_clears_the_gear_body():
    # The bore/chamfers end at radius 7.75mm; tooth roots start beyond 13mm.
    # Between those radii the gear is solid, so the insert cannot occupy it.
    mesh = triangles(FOLDER / "3V4_centered.stl")
    samples = np.concatenate([mesh.reshape(-1, 3), mesh.mean(axis=1)])
    gear, insert = PARTS["2V4"], PARTS["3V4"]
    placed = (
        samples @ Rotation.from_euler("xyz", insert["rpy_rad"]).as_matrix().T
        + insert["xyz_m"]
    )

    def in_gear_core(shift):
        local = (placed + [shift, 0, 0] - gear["xyz_m"]) @ Rotation.from_euler(
            "xyz", gear["rpy_rad"]
        ).as_matrix()
        bore = np.asarray(gear["bore_center_local_m"])
        radius = np.linalg.norm(local[:, :2] - bore, axis=1)
        return (radius > 0.0078) & (radius < 0.0128) & (np.abs(local[:, 2]) < 0.0044)

    assert not in_gear_core(0).any()
    # Reusing the old insertion depth would put the longer finger through the gear.
    assert in_gear_core(-0.006).any()


def test_v6_base_keeps_its_support_planes():
    config = yaml.safe_load((ROOT / "config/production.yaml").read_text())
    tray = yaml.safe_load((ROOT / "config/assembly_run.yaml").read_text())["tray"]
    bottom = triangles(FOLDER / "1V4_centered.stl").min((0, 1))[2]
    assert abs(config["fixture"][2] + bottom - 0.0231) < 1e-7
    assert abs(config["conveyor_in"][2] + bottom - 0.0700) < 1e-7
    assert abs(config["conveyor_out"][2] + bottom - 0.0700) < 1e-7
    # The V6 base pocket is 3.2mm above the carrier's bottom datum.
    base = tray["parts_local"]["base_1v4"]
    assert abs(base[2] + bottom - 0.0032 - 0.00005) < 1e-7
    # Sensor stays above the same sidewall, despite the new base origin.
    assert (
        abs(
            config["conveyor_out"][2]
            + config["conveyor_sensor"]["beam_offset"][2]
            - 0.14
        )
        < 1e-7
    )
