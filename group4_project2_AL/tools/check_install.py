#!/usr/bin/env python3
# Repair copied script permissions, then check what ROS will actually execute.
import argparse
import os
from pathlib import Path
import re
import shlex
import xml.etree.ElementTree as ET


def programs(root):
    for folder in (root, root / "vendor/group4_assembly_line"):
        package = ET.parse(folder / "package.xml").getroot().findtext("name")
        cmake = (folder / "CMakeLists.txt").read_text()
        for block in re.findall(
            r"install\s*\(PROGRAMS\s+(.*?)\s+DESTINATION", cmake, re.S
        ):
            for name in shlex.split(block):
                yield package, folder / name


def repair(root):
    # Windows extraction and Git checkout can remove +x or introduce CRLF.
    paths = {path for _, path in programs(root)}
    paths.update(root.rglob("*.sh"))
    for path in sorted(paths):
        data = path.read_bytes()
        clean = data.replace(b"\r\n", b"\n")
        if clean != data:
            path.write_bytes(clean)
        path.chmod(path.stat().st_mode | 0o111)


def check(root, workspace):
    from ament_index_python.packages import get_package_prefix

    errors = []
    for package, source in programs(root):
        expected = workspace / "install" / package
        try:
            prefix = Path(get_package_prefix(package))
        except LookupError:
            errors.append(f"{package}: not in this ROS environment")
            continue
        installed = prefix / "lib" / package / source.name
        if prefix.resolve() != expected.resolve():
            errors.append(f"{package}: wrong overlay {prefix}")
        if not installed.is_file():
            errors.append(f"Missing installed executable: {installed}")
        elif not os.access(installed, os.X_OK):
            errors.append(f"Not executable: {installed} (source: {source})")
        elif b"\r" in installed.read_bytes().split(b"\n", 1)[0]:
            errors.append(f"Windows line ending in shebang: {installed}")
        elif installed.is_symlink() and installed.resolve() != source.resolve():
            errors.append(f"Old source link: {installed} -> {installed.resolve()}")
    if errors:
        raise SystemExit(
            "\n".join(errors) + "\nRebuild from this copy: bash install_build.sh"
        )
    print("ROS executables checked, including rig_markers.py.", flush=True)


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--repair", action="store_true")
    parser.add_argument("--workspace", type=Path)
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[1]
    if args.repair:
        repair(root)
    if args.workspace:
        check(root, args.workspace)
