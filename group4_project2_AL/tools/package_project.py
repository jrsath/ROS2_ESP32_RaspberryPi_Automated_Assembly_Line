#!/usr/bin/env python3
# Make a portable source ZIP without local passwords, caches or build output.
import argparse
from pathlib import Path
from zipfile import ZipFile, ZIP_DEFLATED


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("output", type=Path)
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[1]
    output = args.output.resolve()
    if output.is_relative_to(root):
        parser.error("Save the ZIP outside the project folder")
    excluded = {
        "ID.yaml",
        ".project2_workspace",
        "firmware/project2_io/io_config.h",
        "firmware/project2_io/transport_config.h",
    }
    directories = {
        ".git",
        "build",
        "install",
        "log",
        "__pycache__",
        ".pytest_cache",
        ".project2",
    }
    output.parent.mkdir(parents=True, exist_ok=True)
    with ZipFile(output, "w", ZIP_DEFLATED) as archive:
        for path in sorted(root.rglob("*")):
            if not path.is_file():
                continue
            relative = path.relative_to(root)
            if (
                path.name == "ID.yaml"
                or relative.as_posix() in excluded
                or directories.intersection(relative.parts)
            ):
                continue
            if (
                path.suffix in (".pyc", ".zip", ".aux", ".log")
                or path.name.startswith("review-")
                or "secret" in path.name.lower()
            ):
                continue
            archive.write(path, str(Path("group4_project2_AL") / relative))
    print(output)


if __name__ == "__main__":
    main()
