# Test controls and ordering before adding hardware-specific motion adapters.
import pytest
from group4_project2_al.cell import Cell


def finish(cell):
    for _ in range(1000):
        cell.tick(0.1)
        if cell.state == "COMPLETE":
            return
    raise AssertionError(cell.snapshot())


def test_five_full_cycles_include_all_packaging_steps():
    cell = Cell(step_seconds=0.1)
    cell.event("start")
    finish(cell)
    assert cell.completed == cell.packed == 5
    assert cell.rejected == 0
    steps = [e["step"] for e in cell.events if e["event"] == "step_complete"]
    assert steps.count("assembly_task_3") == steps.count("place_lid") == 5
    assert (
        steps.index("inspect")
        < steps.index("pickup_box")
        < steps.index("stack_package")
    )


def test_qa_trial_has_four_good_and_two_distinct_flaws():
    cell = Cell(mode="inspection", step_seconds=0.1)
    cell.event("start")
    finish(cell)
    assert cell.completed == 6 and cell.rejected == 2 and cell.packed == 0
    assert sorted(cell.quality) == ["flaw_1", "flaw_2"] + ["good"] * 4
    assert (
        sum(
            e["step"] == "reject_product" and e["event"] == "step_complete"
            for e in cell.events
        )
        == 2
    )


def test_pause_freezes_work_and_requires_start():
    cell = Cell()
    cell.event("start")
    cell.tick(0.1)
    cell.event("pause")
    cell.tick(10)
    assert cell.elapsed == 0.1 and not cell.snapshot()["conveyor_running"]
    cell.event("start")
    assert cell.state == "RUNNING"


def test_estop_release_does_not_restart():
    cell = Cell()
    cell.event("start")
    cell.event("estop")
    cell.event("fault")
    cell.event("start")
    cell.event("reset")
    assert cell.state == "ESTOP" and cell.snapshot()["alarm"]
    cell.event("release_estop")
    assert cell.state == "ESTOP"
    cell.event("reset")
    assert cell.state == "READY"
    cell.event("start")
    assert cell.state == "RUNNING"


def test_backpressure_holds_conveyor_and_assembly_deposit():
    cell = Cell(step_seconds=0.1)
    cell.event("start")
    for _ in range(4):
        cell.tick(0.1)
    assert cell.step == "place_on_conveyor"
    cell.event("occupy")
    cell.tick(5)
    assert cell.state == "WAIT_CLEAR"
    assert cell.step == "place_on_conveyor"
    assert not cell.snapshot()["conveyor_running"]
    cell.event("clear")
    cell.tick(0.1)
    assert cell.step == "transfer_to_inspection"


def test_fault_reset_does_not_count_interrupted_product():
    cell = Cell()
    cell.event("start")
    cell.tick(0.2)
    cell.event("fault")
    cell.event("reset")
    assert cell.state == "READY" and cell.completed == 0
    assert cell.step == "pickup_components" and cell.elapsed == 0


@pytest.mark.parametrize("mode", ["assembly", "packaging"])
def test_section_modes_finish_five_cycles(mode):
    cell = Cell(mode=mode, step_seconds=0.1)
    cell.event("start")
    finish(cell)
    assert cell.completed == 5
    assert all(e["step"] != "inspect" for e in cell.events)


def test_invalid_mode_rejected():
    with pytest.raises(ValueError):
        Cell(mode="hardware")


def test_unknown_event_rejected():
    with pytest.raises(ValueError):
        Cell().event("move_real_robot")
