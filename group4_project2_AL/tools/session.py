#!/usr/bin/env python3
# Replace this user's Project 2 launches, including their orphaned children.
import argparse
import fcntl
import os
from pathlib import Path
import signal
import socket
import subprocess
import time
import uuid

TAG = "GROUP4_PROJECT2_SESSION"
PACKAGES = {"group4_project2_al", "group4_project2"}


def processes():
    found = {}
    for folder in Path("/proc").iterdir():
        if not folder.name.isdigit():
            continue
        try:
            if folder.stat().st_uid != os.getuid():
                continue
            fields = (folder / "stat").read_text().rsplit(")", 1)[1].split()
            if fields[0] == "Z":
                continue
            found[int(folder.name)] = {
                "start": fields[19],
                "parent": int(fields[1]),
                "args": (folder / "cmdline")
                .read_bytes()
                .decode(errors="replace")
                .split("\0")[:-1],
                "env": (folder / "environ").read_bytes().split(b"\0"),
            }
        except (OSError, ValueError, IndexError):
            continue
    return found


def old_processes(table, token=None):
    selected = set()
    for pid, item in table.items():
        tags = [v for v in item["env"] if v.startswith((TAG + "=").encode())]
        if tags and (token is None or (TAG + "=" + token).encode() in tags):
            selected.add(pid)
        if token is not None:
            continue
        # Also recognise launch commands from releases before session tracking.
        args = item["args"]
        if any(
            args[i] == "launch" and args[i + 1] in PACKAGES
            for i in range(len(args) - 1)
        ):
            selected.add(pid)
        # Old run.sh passed its lock into children; these can outlive ros2 launch.
        try:
            for fd in Path(f"/proc/{pid}/fd").iterdir():
                target = Path(os.readlink(fd))
                if (
                    target.name == "run.lock"
                    and target.parent.name == ".project2"
                    and (target.parent.parent / ".project2-managed").is_file()
                ):
                    selected.add(pid)
                    break
        except OSError:
            pass
    # Include descendants even if a tool cleared its environment.
    while True:
        children = {pid for pid, item in table.items() if item["parent"] in selected}
        if children <= selected:
            break
        selected |= children
    # Never signal the launcher, its calling terminal or its ancestors.
    ancestor = os.getpid()
    while ancestor in table:
        selected.discard(ancestor)
        ancestor = table[ancestor]["parent"]
    return {pid: table[pid]["start"] for pid in selected}


def alive(saved):
    table = processes()
    return {
        pid: start
        for pid, start in saved.items()
        if pid in table and table[pid]["start"] == start
    }


def send(saved, sig):
    for pid in alive(saved):
        try:
            os.kill(pid, sig)
        except ProcessLookupError:
            pass


def stop(token=None):
    targets = old_processes(processes(), token)
    if not targets:
        return
    print(f"Closing {len(targets)} previous Project 2 processes...", flush=True)
    # Give ROS time to stop controllers and close Gazebo before escalation.
    for sig, seconds in ((signal.SIGINT, 8), (signal.SIGTERM, 3), (signal.SIGKILL, 2)):
        send(targets, sig)
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            targets = alive(targets)
            targets.update(old_processes(processes(), token))
            if not targets:
                return
            time.sleep(0.2)
    if alive(targets):
        raise RuntimeError(
            "Project processes did not stop; no new session was started."
        )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", type=int)
    parser.add_argument("command", nargs=argparse.REMAINDER)
    args = parser.parse_args()
    command = args.command[1:] if args.command[:1] == ["--"] else args.command
    if not command:
        parser.error("a launch command is required")
    state = Path.home() / ".local/state/group4_project2"
    state.mkdir(parents=True, exist_ok=True)
    token = uuid.uuid4().hex
    child = None

    def interrupted(signum, frame):
        raise KeyboardInterrupt

    signal.signal(signal.SIGTERM, interrupted)
    try:
        # Serialise competing starts across copies/workspaces. Children do not inherit this lock.
        with (state / "start.lock").open("w") as lock:
            fcntl.flock(lock, fcntl.LOCK_EX)
            stop()
            if args.port:
                with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
                    try:
                        sock.bind(("0.0.0.0", args.port))
                    except OSError:
                        raise SystemExit(
                            f"UDP {args.port} is used by another application/standalone agent. Close it or change ID.yaml; unrelated processes were left running."
                        )
            environment = dict(os.environ, **{TAG: token})
            child = subprocess.Popen(command, env=environment, start_new_session=True)
        return child.wait()
    except KeyboardInterrupt:
        return 130
    finally:
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        signal.signal(signal.SIGTERM, signal.SIG_IGN)
        if child is not None:
            stop(token)
            child.wait()


if __name__ == "__main__":
    raise SystemExit(main())
