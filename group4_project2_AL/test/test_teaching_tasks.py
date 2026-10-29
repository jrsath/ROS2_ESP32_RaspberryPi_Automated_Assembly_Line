# Named paths survive edits without changing unrelated production settings.
from group4_project2_al.teaching_tasks import load_tasks, save_tasks, resolve, ARMS
import pytest
import yaml


def test_reorder_and_resolve_saved_feedback(tmp_path):
    path = tmp_path / "production.yaml"
    path.write_text("# Keep this comment\nspeed_rad_s: 0.7\n")
    tasks = load_tasks(path)
    task = tasks["packaging"][0]
    task["movements"][0]["joints"] = [0.2] * 6
    task["movements"].reverse()
    tasks["packaging"].reverse()
    save_tasks(path, tasks)
    assert load_tasks(path) == tasks
    assert "# Keep this comment" in path.read_text()
    assert resolve(task, {})[-1] == [0.2] * 6
    assert resolve(task, {})[0] is None
    assert yaml.safe_load(path.read_text())["speed_rad_s"] == 0.7


def test_invalid_sequence_cannot_replace_file(tmp_path):
    path = tmp_path / "production.yaml"
    path.write_text("speed_rad_s: 0.7\n")
    tasks = load_tasks(path)
    tasks[ARMS[0]][0]["movements"][0]["joints"] = [float("nan")] * 6
    with pytest.raises(ValueError, match="finite"):
        save_tasks(path, tasks)
    assert path.read_text() == "speed_rad_s: 0.7\n"
