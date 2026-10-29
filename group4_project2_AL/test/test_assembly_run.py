from pathlib import Path
import yaml
import pytest

from group4_project2_al.assembly_run import (
    PARTS,
    STEPS,
    check_count,
    initial_poses,
    step_poses,
    tray_pose,
)


CONFIG = yaml.safe_load(
    (Path(__file__).parents[1] / "config/assembly_run.yaml").read_text()
)


@pytest.mark.parametrize("count", [1, 2, 3, 4, 5])
def test_one_kit_and_one_run_per_count(count):
    positions = initial_poses(CONFIG, count)
    assert len(positions) == 3 * count
    assert all((toy, part) in positions for toy in range(count) for part in PARTS)
    assert len({pose.xyz for pose in positions.values()}) == 3 * count
    # Every row keeps the same left-to-right kit order and feeder pitch.
    order = ("insert_3v4", "pin_2v4", "base_1v4")
    for row in range(count):
        xs = [positions[(row, part)].xyz[0] for part in order]
        assert xs == sorted(xs)
        for part in PARTS:
            expected = tuple(
                a + row * b
                for a, b in zip(positions[(0, part)].xyz, CONFIG["tray"]["row_step"])
            )
            assert positions[(row, part)].xyz == pytest.approx(expected)


@pytest.mark.parametrize("count", [0, 6, -1, 1.5, True])
def test_invalid_count_rejected(count):
    with pytest.raises(ValueError):
        check_count(count)


def test_parts_change_at_the_expected_stage():
    assert list(step_poses(CONFIG, 0, STEPS[0])) == ["base_1v4"]
    assert list(step_poses(CONFIG, 0, STEPS[1])) == ["pin_2v4"]
    assert list(step_poses(CONFIG, 0, STEPS[2])) == ["pin_2v4"]
    assert list(step_poses(CONFIG, 0, STEPS[3])) == ["insert_3v4"]
    assert set(step_poses(CONFIG, 0, STEPS[4])) == set(PARTS)
