# Check restart behaviour with disposable processes, without launching any motors.
import importlib.util
import os
from pathlib import Path
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location(
    "project_session", ROOT / "tools/session.py"
)
session = importlib.util.module_from_spec(spec)
spec.loader.exec_module(session)


def test_stale_pid_is_not_signalled():
    assert session.alive({os.getpid(): "not-the-start-time"}) == {}


def test_restart_stops_orphans_and_keeps_unrelated_process():
    token = "test-" + str(os.getpid())
    unrelated = subprocess.Popen([sys.executable, "-c", "import time; time.sleep(40)"])
    child = subprocess.Popen(
        [sys.executable, "-c", "import time; time.sleep(40)"],
        env=dict(os.environ, GROUP4_PROJECT2_SESSION=token),
        start_new_session=True,
    )
    try:
        time.sleep(0.1)
        session.stop(token)
        child.wait(timeout=3)
        assert unrelated.poll() is None
        assert not session.old_processes(session.processes(), token)
    finally:
        if child.poll() is None:
            child.kill()
        child.wait()
        unrelated.terminate()
        unrelated.wait()


def test_descendants_selected_but_ancestors_excluded():
    table = {
        101: {
            "parent": 1,
            "start": "1",
            "args": ["ros2", "launch", "group4_project2_al", "production.launch.py"],
            "env": [],
        },
        102: {"parent": 101, "start": "2", "args": ["gazebo"], "env": []},
        103: {
            "parent": 1,
            "start": "3",
            "args": ["ros2", "launch", "another_project", "demo.launch.py"],
            "env": [],
        },
    }
    assert session.old_processes(table) == {101: "1", 102: "2"}
