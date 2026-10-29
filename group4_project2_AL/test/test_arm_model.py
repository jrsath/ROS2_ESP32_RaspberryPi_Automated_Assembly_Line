# Check actual URDF reachability and limits rather than matching solver internals.
from pathlib import Path
import numpy as np
from group4_project2_al.arm_model import ArmModel, HOME, step_targets

MODEL = (
    Path(__file__).resolve().parents[1] / "vendor/group4_assembly_line/urdf/so101.urdf.xacro"
)


def test_reachable_target_and_unreachable_rejection():
    model = ArmModel(MODEL)
    target = model.fk(HOME + np.array([0.15, 0.1, -0.1, 0, 0, 0]))[0][:3, 3]
    result = model.solve(target, HOME)
    assert result is not None
    assert np.linalg.norm(model.fk(result)[0][:3, 3] - target) < 0.003
    assert np.all(result >= model.limits[:, 0])
    assert np.all(result <= model.limits[:, 1])
    assert model.solve([10, 0, 0], HOME) is None
    assert model.solve([float("nan"), 0, 0], HOME) is None


def test_gripper_does_not_move_tool_frame():
    model = ArmModel(MODEL)
    closed = HOME.copy()
    closed[5] = 0
    np.testing.assert_allclose(model.fk(HOME)[0], model.fk(closed)[0])


def test_demo_targets_are_within_model_limits():
    from group4_project2_al.cell import ASSEMBLY, INSPECTION, PACKAGING

    model = ArmModel(MODEL)
    for step in ASSEMBLY + INSPECTION + PACKAGING + ["reject_product"]:
        q = step_targets(step)
        assert np.all(q >= model.limits[:, 0]) and np.all(q <= model.limits[:, 1])


def test_pan_axis_is_vertical_in_the_cad_frame():
    model = ArmModel(MODEL)
    axis = model.origins[0][:3, :3] @ model.axes[0]
    np.testing.assert_allclose(axis, [0, 0, 1], atol=1e-8)
