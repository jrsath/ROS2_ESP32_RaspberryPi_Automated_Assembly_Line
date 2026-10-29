from pathlib import Path
import copy
import pytest
import yaml
from group4_project2_al.arm_model import ArmModel
from group4_project2_al.waypoints import load_waypoints, STEPS

ROOT = Path(__file__).parents[1]
RIG = ROOT / "vendor/group4_assembly_line"
LIMITS = ArmModel(RIG / "urdf/so101.urdf.xacro").limits


def test_every_task_has_a_valid_demo_pose():
    plan = load_waypoints(ROOT / "config/task_waypoints.yaml", LIMITS, "gazebo_native")
    assert set(plan) == set(STEPS)


@pytest.mark.parametrize("bad", ["missing_arm", "nan", "limit", "units", "empty"])
def test_invalid_plan_stops_before_motion(tmp_path, bad):
    data = yaml.safe_load((ROOT / "config/task_waypoints.yaml").read_text())
    step = data["steps"]["assembly_task_2"]
    pose = step["waypoints"][0]["arms"]
    if bad == "missing_arm":
        del pose["packaging"]
    elif bad == "nan":
        pose["assembly_left"][0] = float("nan")
    elif bad == "limit":
        pose["assembly_left"][0] = 20.0
    elif bad == "units":
        data["units"] = "degrees"
    else:
        step["waypoints"] = []
    path = tmp_path / "invalid.yaml"
    path.write_text(yaml.safe_dump(data))
    with pytest.raises(ValueError):
        load_waypoints(path, LIMITS, "gazebo_native")


def test_multiple_poses_preserve_order_and_dwell(tmp_path):
    data = yaml.safe_load((ROOT / "config/task_waypoints.yaml").read_text())
    entries = data["steps"]["assembly_task_2"]["waypoints"]
    approach = copy.deepcopy(entries[0])
    approach["label"], approach["dwell_s"] = "approach", 0.4
    entries.insert(0, approach)
    path = tmp_path / "plan.yaml"
    path.write_text(yaml.safe_dump(data))
    plan = load_waypoints(path, LIMITS, "gazebo_native")["assembly_task_2"]
    assert [p[2] for p in plan] == ["approach", "work"]
    assert plan[0][1] == 0.4
