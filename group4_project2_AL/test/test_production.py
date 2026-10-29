# Exercise the reject branch without moving Gazebo or sending ROS commands.
import importlib.util
from pathlib import Path
from types import SimpleNamespace
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location(
    "production_run_test", ROOT / "scripts/production_run.py"
)
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


def test_replaced_plan_stops_before_solving_more_grips():
    from concurrent.futures import CancelledError
    from threading import Event
    import pytest
    from group4_project2_al.production_plan import TransferPlanner, make_plans

    planner = object.__new__(TransferPlanner)
    cancel = Event()
    cancel.set()
    with pytest.raises(CancelledError):
        make_plans(planner, {}, {}, {}, cancel=cancel)


def test_reject_keeps_the_prepared_box():
    events = []
    pose = np.eye(4)
    plan = dict(
        arm="inspection",
        poses=[pose, pose],
        held=pose,
        camera={"top": np.zeros(6)},
        rejects=[dict(q=np.zeros(6), approach=np.zeros(6)) for _ in range(2)],
    )
    rig = SimpleNamespace(
        count=2,
        task_mode="full",
        package_count=2,
        packed=0,
        rejected=0,
        bypassed=0,
        run=1,
        config={"feeder_steps": {"box": [0, 0, 0], "lid": [0, 0, 0]}},
        plans={k: plan for k in ("box", "lid", "inspect")},
        inspections={
            "1:1:top": {"pass": False, "bypassed": False},
            "1:2:top": {"pass": True, "bypassed": False},
        },
        planner=SimpleNamespace(open=0.5, tool=lambda *_: pose),
        outcomes={},
    )
    rig.record = lambda event, **values: events.append((event, values))
    rig.pick = lambda name, model: iter([("pick", name, model)])
    rig.place = lambda name, model, *args: iter([("place", name, model)])
    rig.home = lambda arm, **args: iter([("home", arm)])
    actions = list(module.ProductionRun.packaging(rig))
    assert rig.rejected == 1 and rig.packed == 1
    assert rig.outcomes == {1: "fail", 2: "pass"}
    assert [a for a in actions if a[:2] == ("pick", "box")] == [
        ("pick", "box", "box_1")
    ]
    assert [a for a in actions if a[:2] == ("place", "inspect")] == [
        ("place", "inspect", "toy_2_base_1v4")
    ]
    toy_placed = actions.index(("place", "inspect", "toy_2_base_1v4"))
    lid = actions.index(("place", "lid", "lid_1"))
    assert toy_placed < lid
    assert ("home", "inspection") in actions[toy_placed:lid]
    assert not any(
        isinstance(value, str) and value == "box_slide"
        for action in actions
        for value in action
    )
    assert any(event == "rejected" for event, _ in events)


def test_hold_corrects_small_drift_and_stops_large_error():
    import pytest
    from concurrent.futures import Future

    calls = []

    def tool(arm, q):
        pose = np.eye(4)
        pose[0, 3] = q[0] * 0.1
        return pose

    def check(*args):
        future = Future()
        future.set_result(None)
        return future

    rig = SimpleNamespace(
        actuator_power=True,
        state="RUNNING",
        tasks={},
        hold_targets={"inspection": np.zeros(6)},
        feedback={"inspection": np.array([0.03, 0, 0, 0, 0, 0])},
        hold_checks={},
        hold_retry={},
        planner=SimpleNamespace(tool=tool),
        collision=SimpleNamespace(swept=lambda *args: None),
        worker=SimpleNamespace(submit=check),
        config={
            "position_hold": dict(
                joint_tolerance=0.02,
                tool_tolerance_m=0.003,
                maximum_joint_error=0.15,
                maximum_tool_error_m=0.015,
                correction_interval_s=1.0,
            )
        },
        send_move=lambda *args: calls.append(args),
        record=lambda *args, **kwargs: None,
    )
    module.ProductionRun.maintain_position(rig, 10.0)
    module.ProductionRun.maintain_position(rig, 10.1)
    assert len(calls) == 1
    assert np.array_equal(calls[0][1], np.zeros(6))
    rig.feedback["inspection"][0] = 0.2
    with pytest.raises(RuntimeError, match="drift exceeds"):
        module.ProductionRun.maintain_position(rig, 12.0)


def test_insert_can_withdraw_before_opening_fully():
    # V5's insertion position needs narrow jaws until the support arm is clear.
    q = np.zeros(6)
    plan = dict(
        arm="assembly_right", q=[q, q], approach=[q, q], poses=[np.eye(4)] * 2, open=0.5
    )
    rig = SimpleNamespace(
        plans={"insert": plan},
        planner=SimpleNamespace(open=0.5),
        config={"gripper_angles": {"insert": {"release": 0.1}}},
    )
    rig.via = lambda *args: iter([])
    narrow = list(module.ProductionRun.place(rig, "insert", "toy", defer_open=True))
    normal = list(module.ProductionRun.place(rig, "insert", "toy"))
    assert narrow[-1][0] == "move" and narrow[-1][2][5] == 0.1
    assert normal[-1][0] == "move" and normal[-1][2][5] == 0.5


def test_conveyor_waits_for_sensor_after_reaching_end():
    import time
    from concurrent.futures import Future

    future = Future()
    future.set_result(SimpleNamespace(success=True))
    task = dict(
        action=("bundle", [], np.eye(4), np.eye(4), {}),
        elapsed=4.0,
        futures=[future],
        last_fraction=1.0,
        generator=iter([]),
    )
    rig = SimpleNamespace(
        require_io=True,
        io=[0] * 14,
        sensor_at=time.monotonic(),
        config={"conveyor_sensor": {"timeout_s": 12.0}},
    )
    module.ProductionRun.advance(rig, task, 0.05)
    assert "action" in task
    rig.io[7] = 1
    module.ProductionRun.advance(rig, task, 0.05)
    assert list(task) == ["generator"]


def test_unexpected_occupied_sensor_stops_transfer():
    import time
    import pytest
    from concurrent.futures import Future

    future = Future()
    future.set_result(SimpleNamespace(success=True))
    task = dict(
        action=("bundle", [], np.eye(4), np.eye(4), {}),
        elapsed=2.0,
        futures=[future],
        last_fraction=0.5,
        generator=iter([]),
    )
    rig = SimpleNamespace(
        require_io=True,
        io=[0] * 14,
        sensor_at=time.monotonic(),
        config={"conveyor_sensor": {"timeout_s": 12.0}},
    )
    rig.io[7] = 1
    with pytest.raises(RuntimeError, match="before the toy reached"):
        module.ProductionRun.advance(rig, task, 0.05)


def test_task_change_pauses_and_waits_for_reset():
    import json

    rig = SimpleNamespace(
        state="RUNNING", qa_trial=False, requested_mode="full", requested_count=1
    )
    stops = []
    rig.stop = lambda state, reason: stops.append((state, reason))
    module.ProductionRun.on_event(
        rig, SimpleNamespace(data=json.dumps({"mode": "assembly", "products": 5}))
    )
    assert (rig.requested_mode, rig.requested_count) == ("assembly", 5)
    assert stops == [("PAUSED", "Settings changed: press Reset")]
    # A malformed selection cannot change the accepted configuration.
    module.ProductionRun.on_event(
        rig, SimpleNamespace(data=json.dumps({"mode": "full", "products": True}))
    )
    assert (rig.requested_mode, rig.requested_count) == ("assembly", 5)


def test_stage_survives_completed_action_for_fault_diagnostics():
    events = []
    rig = SimpleNamespace(record=lambda *args, **kwargs: events.append((args, kwargs)))
    task = {"generator": iter([("stage", "insert: placement"), ("delay", 0.1)])}
    module.ProductionRun.advance(rig, task, 0.05)
    assert task["stage"] == "insert: placement"
    module.ProductionRun.advance(rig, task, 0.2)
    assert task["stage"] == "insert: placement" and "action" not in task


def test_reset_can_leave_failed_packaging_plan_for_assembly(tmp_path):
    # A failed stack plan must not lock the operator out of another section.
    submitted = []
    config_file = tmp_path / "production.yaml"
    config_file.write_text((ROOT / "config/production.yaml").read_text())
    rig = SimpleNamespace(
        config_path=config_file,
        height=0.0,
        setup_done=False,
        plan_error="stack_0: lid clearance",
        state="FAULT",
        io=None,
        requested_mode="assembly",
        requested_count=2,
        run=0,
        inspections={},
        outcomes={},
        hold_targets={},
        hold_checks={},
        replies={},
        measured_poses={},
        plan_starts={},
        alignment={},
        config={},
        package_count=5,
        planner=object(),
        spawned_packages=True,
        plan_worker=SimpleNamespace(
            submit=lambda *args, **kwargs: submitted.append(args)
        ),
        plan_pub=SimpleNamespace(publish=lambda message: None),
        stop=lambda *args: None,
        setup=lambda first: iter([]),
    )
    module.ProductionRun.on_event(rig, SimpleNamespace(data="reset"))
    assert (rig.task_mode, rig.count) == ("assembly", 2)
    assert submitted[0][5] == 0
    assert rig.plan_error is None and rig.plans is None
    assert "setup" in rig.tasks


def test_scene_reset_does_not_need_motion_plans():
    from scipy.spatial.transform import Rotation

    config = {
        "box_feed": [0.1, 0.2, 0.3, 0, 0, 0],
        "lid_feed": [0.4, 0.5, 0.6, 0, 0, 0],
        "feeder_steps": {"box": [0, 0.1, 0], "lid": [0, 0.1, 0]},
    }
    rig = SimpleNamespace(
        original_boxes=[],
        starts={},
        plans=None,
        config=config,
        spawned_packages=True,
        package_count=2,
        count=2,
        task_mode="full",
        tray_config=None,
        plan_error="stack_0 clearance",
        record=lambda *args, **kwargs: None,
        home=lambda arm: iter([]),
    )
    actions = list(module.ProductionRun.setup(rig, False))
    assert rig.setup_done
    poses = {a[1]: a[2] for a in actions if a[0] == "pose"}
    assert set(poses) == {"box_1", "box_2", "lid_1", "lid_2"}
    assert np.allclose(poses["box_2"][:3, 3], [0.1, 0.3, 0.3])
    assert "Path blocked" in rig.reason


def test_blocked_assembly_holds_measured_joints_without_stopping_other_jobs():
    sent = []
    events = []
    task = {"action": ("move", "assembly_right", np.ones(6))}
    rig = SimpleNamespace(
        state="RUNNING",
        feedback={"assembly_left": np.zeros(6), "assembly_right": np.ones(6) * 0.1},
        send_move=lambda arm, q, duration: sent.append((arm, q.copy())),
        record=lambda *a, **k: events.append((a, k)),
        get_logger=lambda: SimpleNamespace(warning=lambda text: None),
    )
    module.ProductionRun.block_task(rig, "assembly", task, "Contact not reached")
    assert rig.state == "RUNNING"
    assert task["blocked"] == "Contact not reached"
    assert len(sent) == 2
    assert np.allclose(sent[1][1], rig.feedback["assembly_right"])
    assert module.ProductionRun.advance(rig, task, 0.05) is False


def test_missing_stack_path_waits_without_picking_up_the_box():
    rig = SimpleNamespace(plans={"unavailable": {"stack_0": "Lid clearance"}})
    pending = module.ProductionRun.require_path(rig, "stack_0")
    assert next(pending) == ("blocked_path", "stack_0", "Lid clearance")
    assert next(pending)[0] == "blocked_path"
