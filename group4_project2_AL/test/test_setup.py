# Check that setup isolates ROS settings and preserves existing workspace contents.
import os
from pathlib import Path
import subprocess
import pytest

ROOT = Path(__file__).resolve().parents[1]


def test_clean_environment_keeps_display_but_drops_old_overlays():
    environment = dict(
        os.environ,
        AMENT_PREFIX_PATH="/old/install",
        PYTHONPATH="/old/python",
        LD_LIBRARY_PATH="/old/lib",
        DISPLAY=":99",
        RMW_IMPLEMENTATION="old_rmw",
    )
    result = subprocess.run(
        ["bash", str(ROOT / "tools/clean_env.sh"), "env"],
        env=environment,
        capture_output=True,
        text=True,
        check=True,
    )
    values = dict(line.split("=", 1) for line in result.stdout.splitlines())
    assert values["DISPLAY"] == ":99"
    assert (
        not {"AMENT_PREFIX_PATH", "PYTHONPATH", "LD_LIBRARY_PATH", "RMW_IMPLEMENTATION"}
        & values.keys()
    )


def test_existing_workspace_is_left_untouched(tmp_path):
    sentinel = tmp_path / "my_existing_work.txt"
    sentinel.write_text("keep this work")
    result = subprocess.run(
        [
            "bash",
            str(ROOT / "install_build.sh"),
            "--skip-system",
            "--workspace",
            str(tmp_path),
        ],
        capture_output=True,
        text=True,
    )
    assert result.returncode != 0
    assert "Existing workspace left untouched" in result.stderr
    assert sentinel.read_text() == "keep this work"
    assert list(tmp_path.iterdir()) == [sentinel]


@pytest.mark.parametrize("script", ["install_build.sh", "run.sh", "tools/clean_env.sh"])
def test_shell_syntax(script):
    subprocess.run(["bash", "-n", str(ROOT / script)], check=True)


def test_copied_scripts_regain_permissions_and_linux_shebang(tmp_path):
    import importlib.util

    spec = importlib.util.spec_from_file_location(
        "check_install", ROOT / "tools/check_install.py"
    )
    check_install = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(check_install)
    root = tmp_path / "folder with spaces" / "group4_project2_AL"
    rig = root / "vendor/group4_assembly_line"
    for folder in (root, rig):
        (folder / "scripts").mkdir(parents=True)
        (folder / "package.xml").write_text("<package><name>example</name></package>")
        (folder / "CMakeLists.txt").write_text(
            "install(PROGRAMS scripts/rig_markers.py DESTINATION lib/example)"
        )
        script = folder / "scripts/rig_markers.py"
        script.write_bytes(b"#!/usr/bin/env python3\r\nprint('ok')\r\n")
        script.chmod(0o644)
    check_install.repair(root)
    for folder in (root, rig):
        script = folder / "scripts/rig_markers.py"
        assert os.access(script, os.X_OK)
        result = subprocess.run(
            [str(script)], capture_output=True, text=True, check=True
        )
        assert result.stdout == "ok\n"
