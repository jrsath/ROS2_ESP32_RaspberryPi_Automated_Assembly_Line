#!/usr/bin/env python3
# Update the runtime parts and camera reference from the supplied V6 exports.
import json
from pathlib import Path
import shutil
import numpy as np
from scipy.spatial.transform import Rotation
from import_v6 import read_stl, write_stl


def main():
    root = Path(__file__).resolve().parents[1]
    folder = root / "models/product"
    source = root / "models/v6"
    manifest = json.loads((source / "manifest.json").read_text())
    current = json.loads((folder / "mesh_alignment.json").read_text())
    # Keep a recoverable V5 source outside the distributable code folder.
    backup = Path.home() / "ROS2/project2_docs_archive/product_before_v6"
    if current["revision"] != "V6" and not backup.exists():
        shutil.copytree(folder, backup)
    parts = current["relative_to_1V4_center"]
    reference = []
    for part, filename in (("1V4", "1"), ("2V4", "2"), ("3V4", "3")):
        record = manifest["files"][filename]
        mesh = read_stl(source / (filename + ".stl"))
        old_bounds = np.asarray(parts[part]["bounds_m"])
        if not np.allclose(np.ptp(mesh, axis=(0, 1)), old_bounds[1]-old_bounds[0], atol=0.00001):
            raise ValueError(part + ": dimensions changed; remeasure mating datums first")
        if part == "2V4" and "bore_center_local_m" not in parts[part]:
            parts[part]["bore_center_local_m"] = (-np.asarray(parts[part]["source_center_mm"][:2]) * 0.001).tolist()
        shutil.copy2(source / (filename + ".stl"), folder / (part + "_centered.stl"))
        parts[part].update(
            bounds_m=record["bounds_m"], source="models/v6/" + filename + ".stl",
            source_sha256=record["sha256"], source_center_mm=record["source_origin_mm"],
        )
        reference.append(mesh @ Rotation.from_euler("xyz", parts[part]["rpy_rad"]).as_matrix().T + parts[part]["xyz_m"])
    # The camera reference reader expects millimetres, while individual meshes use metres.
    write_stl(folder / "1_assembled.STL", np.concatenate(reference) * 1000, unit="millimetres")
    current.update(
        revision="V6", source="project2_v6_cad STL exports",
        assembly_reference="Retained measured V5 floor/gear/insert datums; V6 assembled STEP/STL was not supplied. Validate the physical snap-fit before use.",
    )
    (folder / "mesh_alignment.json").write_text(json.dumps(current, indent=2) + "\n")
    print("V6 runtime parts and camera reference updated; run geometry checks before production.")


if __name__ == "__main__":
    main()
