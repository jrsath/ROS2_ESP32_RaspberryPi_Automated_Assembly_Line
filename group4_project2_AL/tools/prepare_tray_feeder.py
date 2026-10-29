#!/usr/bin/env python3
# Remove the two internal lane dividers and their brackets from the supplied rack.
import argparse
from pathlib import Path
import struct
import numpy as np
from scipy.sparse import coo_matrix
from scipy.sparse.csgraph import connected_components


def open_feeder(source, destination):
    data = source.read_bytes()
    count = struct.unpack_from("<I", data, 80)[0]
    dtype = np.dtype(
        [("normal", "<f4", 3), ("points", "<f4", (3, 3)), ("attribute", "<u2")]
    )
    records = np.frombuffer(data, dtype=dtype, count=count, offset=84)
    points, indices = np.unique(
        np.round(records["points"].reshape(-1, 3), 7), axis=0, return_inverse=True
    )
    faces = indices.reshape(-1, 3)
    edges = np.concatenate([faces[:, [0, 1]], faces[:, [1, 2]], faces[:, [2, 0]]])
    graph = coo_matrix(
        (np.ones(len(edges)), (edges[:, 0], edges[:, 1])),
        shape=(len(points), len(points)),
    )
    components, labels = connected_components(graph, directed=False)
    removed = []
    for index in range(components):
        cloud = points[labels == index]
        # Assembly plate local X is reversed by its pi-radian mounting rotation.
        centre_x = -0.62 - (cloud[:, 0].min() + cloud[:, 0].max()) / 2
        if min(abs(centre_x - x) for x in (-0.4355, -0.4855)) < 0.001:
            removed.append(index)
    if len(removed) != 4:
        raise ValueError(
            "Rack mesh changed: expected two dividers and two rear brackets"
        )
    keep = ~np.isin(labels[faces[:, 0]], removed)
    header = b"University assembly plate: internal dividers removed".ljust(80, b" ")
    destination.write_bytes(
        header + struct.pack("<I", int(keep.sum())) + records[keep].tobytes()
    )
    print(
        f"Removed {len(removed)} divider/bracket bodies; retained both outer guides and the bench"
    )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--rig",
        type=Path,
        default=Path(__file__).resolve().parents[1] / "vendor/group4_assembly_line",
    )
    args = parser.parse_args()
    folder = args.rig / "models/project2_rig/meshes"
    open_feeder(folder / "Assembly_Plate.stl", folder / "Assembly_Plate_Open.stl")


if __name__ == "__main__":
    main()
