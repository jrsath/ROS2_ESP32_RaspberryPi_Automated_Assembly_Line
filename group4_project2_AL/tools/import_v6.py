#!/usr/bin/env python3
# Import the supplied STL exports without changing their source files.
import argparse
import hashlib
import importlib.util
import json
from pathlib import Path
import shutil
import numpy as np
from scipy.spatial.transform import Rotation

ROOT = Path(__file__).resolve().parents[1]

def read_stl(path):
    data = path.read_bytes()
    count = int.from_bytes(data[80:84], "little")
    if not count or len(data) != 84 + 50 * count:
        raise ValueError(f"Expected a binary STL: {path}")
    records = np.frombuffer(data, offset=84, dtype=[("normal", "<f4", 3), ("points", "<f4", (3, 3)), ("attribute", "<u2")])
    mesh = records["points"].astype(float)
    if not np.isfinite(mesh).all():
        raise ValueError(f"Non-finite vertices: {path}")
    return mesh


def write_stl(path, mesh, unit="metres"):
    records = np.zeros(len(mesh), dtype=[("normal", "<f4", 3), ("points", "<f4", (3, 3)), ("attribute", "<u2")])
    normals = np.cross(mesh[:, 1] - mesh[:, 0], mesh[:, 2] - mesh[:, 0])
    records["normal"] = normals / np.maximum(np.linalg.norm(normals, axis=1)[:, None], 1e-30)
    records["points"] = mesh
    path.parent.mkdir(parents=True, exist_ok=True)
    header = ("Project 2 V6 - " + unit).encode().ljust(80, b" ")
    path.write_bytes(header + len(mesh).to_bytes(4, "little") + records.tobytes())


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("source", type=Path)
    args = parser.parse_args()
    output = ROOT / "models/v6"
    output.mkdir(parents=True, exist_ok=True)
    manifest = {"revision": "V6", "source_unit": "mm", "mesh_unit": "m", "files": {}}
    for name in ("1", "1A", "2", "3", "jig", "assembly_jig", "base", "fork", "gear"):
        source = args.source / (name + ".STL")
        mesh = read_stl(source)
        low, high = mesh.min((0, 1)), mesh.max((0, 1))
        centre = (low + high) / 2
        # Carrier: long direction is X, bottom plane is Z=0.
        # Other exports keep their CAD axes and use the bounding-box centre.
        rotation = Rotation.from_euler("z", -np.pi / 2).as_matrix() if name == "jig" else np.eye(3)
        if name == "jig":
            centre[2] = low[2]
        converted = (mesh - centre) @ rotation.T * 0.001
        write_stl(output / (name + ".stl"), converted)
        manifest["files"][name] = {"source": source.name, "sha256": hashlib.sha256(source.read_bytes()).hexdigest(), "triangles": len(mesh), "size_mm": (high-low).tolist(), "source_origin_mm": centre.tolist(), "rotation": rotation.tolist(), "bounds_m": [converted.min((0,1)).tolist(), converted.max((0,1)).tolist()]}
    # Separate fixture names from the three toy parts: base.STL is a jig section.
    manifest["roles"] = {"toy_base": "1", "toy_gear": "2", "toy_insert": "3", "loaded_carrier_without_parts": "jig", "bench_fixture": "assembly_jig", "carrier_sections": ["fork", "gear", "base"]}
    manifest["notes"] = ["1.STL and 1A.STL are individual base parts, not a complete assembled toy.", "STL has no mating transforms. Validate the three seated part transforms before using a new camera reference.", "Imported assets do not by themselves change the active production sequence."]
    (output / "manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")
    print("Imported V6 assets:", output)


if __name__ == "__main__":
    main()
