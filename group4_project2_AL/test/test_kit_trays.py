# Check that the carrier layout fits the CAD ramp and moves each kit together.
from pathlib import Path
import numpy as np
import yaml
from group4_project2_al.arm_model import transform
from group4_project2_al.assembly_run import initial_poses
from group4_project2_al.kit_trays import tray_pose, faces, tray_sdf
from group4_project2_al.vision import triangles

ROOT = Path(__file__).resolve().parents[1]
CONFIG = yaml.safe_load((ROOT / "config/assembly_run.yaml").read_text())
CARRIER = CONFIG["tray"]["carrier"]


def test_five_carriers_fit_ramp_without_lane_dividers():
    mesh = faces(CARRIER).reshape(-1, 3)
    # The raised pull tab projects forward; the bottom footprint sits on the ramp.
    mesh = mesh[np.abs(mesh[:, 2]) < 1e-8]
    previous = None
    for row in range(5):
        pose = tray_pose(CARRIER, row)
        floor = mesh @ pose[:3, :3].T + pose[:3, 3]
        assert floor[:, 0].min() > -0.534
        assert floor[:, 0].max() < -0.350
        assert floor[:, 1].min() > 0.326
        assert floor[:, 1].max() < 0.575
        if previous is not None:
            assert np.allclose(pose[:3, 3] - previous, CARRIER["row_step"])
        previous = pose[:3, 3]


def test_components_fit_one_carrier_in_requested_order():
    poses = initial_poses(CONFIG, 1)
    xs = [poses[(0, name)].xyz[0] for name in ("insert_3v4", "pin_2v4", "base_1v4")]
    assert xs == sorted(xs)
    half = np.array(CARRIER["size"][:2]) / 2 - CARRIER["wall"]
    for name, cad in (("insert_3v4", "3V4"), ("pin_2v4", "2V4"), ("base_1v4", "1V4")):
        value = poses[(0, name)]
        pose = np.linalg.inv(tray_pose(CARRIER)) @ transform(value.xyz, value.rpy)
        points = triangles(ROOT / f"models/product/{cad}_centered.stl").reshape(-1, 3)
        points = points @ pose[:3, :3].T + pose[:3, 3]
        assert np.all(np.abs(points[:, :2]) < half + 1e-6), name
        floor = 0.0032 if name == "base_1v4" else 0.005
        assert abs(points[:, 2].min() - floor - 0.00005) < 1e-7, name


def test_carrier_is_one_connected_mesh_model():
    import xml.etree.ElementTree as ET

    model = ET.fromstring(tray_sdf("kit_tray_1", CARRIER)).find("model")
    assert len(model.findall("link")) == 1
    assert model.find("link/visual/geometry/mesh/uri").text == "model://v6/jig.stl"
    assert model.find("link/collision/geometry/mesh/uri").text == "model://v6/jig.stl"
