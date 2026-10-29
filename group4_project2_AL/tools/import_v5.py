#!/usr/bin/env python3
# Convert the supplied V5 part exports and rebuild the camera reference.
import argparse
import hashlib
import io
import json
from pathlib import Path
import xml.etree.ElementTree as ET
import zipfile

import numpy as np
from scipy.spatial.transform import Rotation


def read_3mf(data):
    # Apply component/build transforms before centring the part. Units stay in mm.
    with zipfile.ZipFile(io.BytesIO(data)) as archive:
        root = ET.fromstring(archive.read("3D/3dmodel.model"))
    if root.get("unit", "millimeter") != "millimeter":
        raise ValueError("Expected the millimetre V5 exports")
    objects = {item.get("id"): item for item in root.findall(".//{*}object")}

    def apply(points, text):
        if not text:
            return points
        matrix = np.array([float(v) for v in text.split()]).reshape(4, 3)
        return points @ matrix[:3] + matrix[3]

    def mesh(object_id, visited=()):
        if object_id in visited:
            raise ValueError("Cyclic 3MF component reference")
        obj = objects[object_id]
        vertices = obj.findall("./{*}mesh/{*}vertices/{*}vertex")
        if vertices:
            points = np.array(
                [[float(v.get(k)) for k in ("x", "y", "z")] for v in vertices]
            )
            faces = np.array(
                [
                    [int(t.get(k)) for k in ("v1", "v2", "v3")]
                    for t in obj.findall("./{*}mesh/{*}triangles/{*}triangle")
                ]
            )
            return points[faces]
        return np.concatenate(
            [
                apply(
                    mesh(c.get("objectid"), (*visited, object_id)), c.get("transform")
                )
                for c in obj.findall("./{*}components/{*}component")
            ]
        )

    return np.concatenate(
        [
            apply(mesh(item.get("objectid")), item.get("transform"))
            for item in root.findall("./{*}build/{*}item")
        ]
    )


def write_stl(path, mesh):
    # Binary STL has no unit field; individual meshes use metres, camera STL uses mm.
    dtype = np.dtype(
        [("normal", "<f4", 3), ("points", "<f4", (3, 3)), ("attribute", "<u2")]
    )
    records = np.zeros(len(mesh), dtype=dtype)
    normals = np.cross(mesh[:, 1] - mesh[:, 0], mesh[:, 2] - mesh[:, 0])
    lengths = np.linalg.norm(normals, axis=1)
    normals /= np.maximum(lengths[:, None], 1e-30)
    records["normal"], records["points"] = normals, mesh
    path.write_bytes(
        b"V5 fidget toy".ljust(80, b" ")
        + len(mesh).to_bytes(4, "little")
        + records.tobytes()
    )


def main():
    parser = argparse.ArgumentParser(
        description="Import V5.zip; station offsets are maintained in config."
    )
    parser.add_argument("archive", type=Path)
    parser.add_argument(
        "--output",
        type=Path,
        default=Path(__file__).resolve().parents[1] / "models/product",
    )
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    # V5 keeps the floor, gear shoulder and slider guide levels from V4.
    # Its retaining feature moves 6mm toward the open end; the longer insert
    # finger needs the same 6mm shift to clear the gear at the seated position.
    # Its taller pin shifts the base bounding-box centre upward by 1.1mm.
    alignment = {
        "1V4": {"xyz_m": [0.0, 0.0, 0.0], "rpy_rad": [0.0, 0.0, 0.0]},
        "2V4": {
            "xyz_m": [-0.028566618, 0.000109956, -0.001199999],
            "rpy_rad": [0.0, 0.0, 0.100355656],
        },
        "3V4": {"xyz_m": [0.00232931, 0.0, -0.000299999], "rpy_rad": [0.0, 0.0, 0.0]},
    }
    expected = {
        "1V4": [76.88757, 28, 18],
        "2V4": [28.1981, 28.08373, 10],
        "3V4": [50.24739, 25, 13.8],
    }
    assembled = []
    with zipfile.ZipFile(args.archive) as archive:
        for part, pose in alignment.items():
            member = "V5/3mf/" + part + "V5.3MF"
            data = archive.read(member)
            mesh = read_3mf(data)
            low, high = mesh.min((0, 1)), mesh.max((0, 1))
            if not np.allclose(high - low, expected[part], atol=0.002):
                raise ValueError(
                    part
                    + ": unexpected dimensions; review mating positions before importing"
                )
            centred = (mesh - (low + high) / 2) * 0.001
            write_stl(args.output / (part + "_centered.stl"), centred)
            pose["bounds_m"] = [
                centred.min((0, 1)).tolist(),
                centred.max((0, 1)).tolist(),
            ]
            pose["source"] = member
            pose["source_sha256"] = hashlib.sha256(data).hexdigest()
            pose["source_center_mm"] = ((low + high) / 2).tolist()
            assembled.append(
                centred @ Rotation.from_euler("xyz", pose["rpy_rad"]).as_matrix().T
                + pose["xyz_m"]
            )
    assembled = np.concatenate(assembled)
    write_stl(args.output / "1_assembled.STL", assembled * 1000)
    metadata = {
        "revision": "V5",
        "source": "V5.zip part 3MF exports",
        "source_unit": "mm",
        "mesh_unit": "m",
        "assembly_reference": "V4 floor/gear datums; V5 retainer moves the insert 6mm outward. No assembled V5 mesh supplied",
        "relative_to_1V4_center": alignment,
        "assembled_bounds_m": [
            assembled.min((0, 1)).tolist(),
            assembled.max((0, 1)).tolist(),
        ],
    }
    (args.output / "mesh_alignment.json").write_text(
        json.dumps(metadata, indent=2) + "\n"
    )
    print("Imported V5 meshes and rebuilt the assembled camera reference:", args.output)


if __name__ == "__main__":
    main()
