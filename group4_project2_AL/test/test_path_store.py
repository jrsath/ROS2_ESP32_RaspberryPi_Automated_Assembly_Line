# Saving a path must preserve comments and other settings.
from pathlib import Path
import importlib.util
import pytest
import yaml

spec = importlib.util.spec_from_file_location(
    "path_store", Path(__file__).resolve().parents[1] / "project/path_store.py"
)
m = importlib.util.module_from_spec(spec)
spec.loader.exec_module(m)


def test_points_preserve_settings_and_order(tmp_path):
    path = tmp_path / "production.yaml"
    path.write_text("# Speed stays documented\nspeed: 0.7\n# End note\n")
    m.save_point(path, "lid", "pickup", [0.1] * 6)
    m.save_point(path, "lid", "extra", [0.2] * 6, "Clear shelf", "pickup")
    m.save_point(path, "lid", "extra", [0.3] * 6, "Above box", "place")
    m.save_point(path, "lid", "place", [0.4] * 6)
    data = yaml.safe_load(path.read_text())
    assert data["speed"] == 0.7
    assert "# Speed stays documented" in path.read_text()
    assert [p["label"] for p in data["path_points"]["lid"]] == [
        "Clear shelf",
        "Above box",
    ]
    m.remove_point(path, "lid", "pickup")
    m.remove_point(path, "lid", "0")
    data = yaml.safe_load(path.read_text())
    assert data["joint_overrides"]["lid"] == {"place": [0.4] * 6}
    assert data["path_points"]["lid"][0]["label"] == "Above box"


@pytest.mark.parametrize("q", [[0] * 5, [float("nan")] * 6, [float("inf")] * 6])
def test_invalid_capture_does_not_change_file(tmp_path, q):
    path = tmp_path / "production.yaml"
    path.write_text("speed: 0.7\n")
    with pytest.raises(ValueError):
        m.save_point(path, "lid", "pickup", q)
    assert path.read_text() == "speed: 0.7\n"


def test_captured_pair_preserves_carry_grip_and_checks_destination():
    import numpy as np
    from types import SimpleNamespace
    from group4_project2_al.production_plan import captured_transfer

    def tool(arm, q):
        pose = np.eye(4)
        pose[0, 3] = q[0]
        return pose

    planner = SimpleNamespace(
        tool=tool, model=SimpleNamespace(limits=np.array([[-2, 2]] * 6))
    )
    source, target = np.eye(4), np.eye(4)
    target[0, 3] = 0.1
    capture = {"pickup": [0] * 6, "place": [0.1, 0, 0, 0, 0, 0]}
    plan = captured_transfer(planner, "packaging", [source, target], capture)
    assert np.allclose(plan["held"], np.eye(4))
    capture["place"][0] = 0.105
    with pytest.raises(ValueError, match="1mm"):
        captured_transfer(planner, "packaging", [source, target], capture)
    capture["place"][0] = 0.1
    capture["place"][5] = 0.2
    with pytest.raises(ValueError, match="gripper"):
        captured_transfer(planner, "packaging", [source, target], capture)


def test_saved_solutions_are_invalidated_when_geometry_changes(tmp_path):
    config = tmp_path / "config"
    config.mkdir()
    (config / "production.yaml").write_text("fixture: [0, 0, 0]\n")
    data = dict(
        geometry_signature=m.geometry_signature(tmp_path),
        paths={"base": {"points": {"pickup": [0.1] * 6}}},
    )
    (config / "solved_joint_paths.yaml").write_text(yaml.safe_dump(data))
    assert m.load_solved_paths(tmp_path)["base"]["points"]["pickup"] == [0.1] * 6
    (config / "production.yaml").write_text("fixture: [0.1, 0, 0]\n")
    assert m.load_solved_paths(tmp_path) == {}
