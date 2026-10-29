# Fault schedules must select whole products without duplicates or overflow.
import pytest
from group4_project2_al.defects import make_faults


def test_fault_positions_and_types():
    assert make_faults(5) == {}
    assert list(make_faults(5, 2, "first")) == [1, 2]
    assert list(make_faults(5, 2, "last")) == [4, 5]
    assert make_faults(5, 2, "custom", "2,4") == {
        2: "missing_gear",
        4: "unseated_insert",
    }
    schedule = make_faults(5, 3, seed=19)
    assert schedule == make_faults(5, 3, seed=19)
    assert len(schedule) == 3 and set(schedule) <= set(range(1, 6))
    assert len(make_faults(5, 5)) == 5


@pytest.mark.parametrize(
    "count,placement,indices",
    [
        (6, "random", ""),
        (-1, "random", ""),
        (True, "random", ""),
        (2, "custom", "1,1"),
        (2, "custom", "1,6"),
        (2, "custom", "2"),
        (1, "custom", "no"),
        (1, "unknown", ""),
    ],
)
def test_invalid_fault_selection(count, placement, indices):
    with pytest.raises(ValueError):
        make_faults(5, count, placement, indices)
