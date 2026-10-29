# Measure fingertip contact against the rendered STL surfaces, in metres.
from functools import lru_cache
from pathlib import Path
import numpy as np
from .vision import triangles


@lru_cache(maxsize=16)
def mesh(path):
    return triangles(Path(path)).astype(float)


def surface_distance(point, faces):
    # Closest point on triangle interiors or on their three edges.
    a, b, c = faces[:, 0], faces[:, 1], faces[:, 2]
    ab, ac = b - a, c - a
    normal = np.cross(ab, ac)
    length = np.einsum("ij,ij->i", normal, normal)
    delta = point - a
    projected = (
        point
        - normal
        * (np.einsum("ij,ij->i", delta, normal) / np.maximum(length, 1e-30))[:, None]
    )
    v = projected - a
    aa = np.einsum("ij,ij->i", ab, ab)
    bb = np.einsum("ij,ij->i", ac, ac)
    cross = np.einsum("ij,ij->i", ab, ac)
    va = np.einsum("ij,ij->i", v, ab)
    vb = np.einsum("ij,ij->i", v, ac)
    denominator = np.maximum(aa * bb - cross * cross, 1e-30)
    u = (bb * va - cross * vb) / denominator
    w = (aa * vb - cross * va) / denominator
    inside = (u >= 0) & (w >= 0) & (u + w <= 1) & (length > 1e-24)
    best = (
        np.min(np.linalg.norm(projected[inside] - point, axis=1))
        if inside.any()
        else np.inf
    )
    for start, end in ((a, b), (b, c), (c, a)):
        edge = end - start
        fraction = np.clip(
            np.einsum("ij,ij->i", point - start, edge)
            / np.maximum(np.einsum("ij,ij->i", edge, edge), 1e-30),
            0,
            1,
        )
        best = min(
            best,
            np.min(np.linalg.norm(start + fraction[:, None] * edge - point, axis=1)),
        )
    return float(best)


def check_contacts(pads_world, object_pose, path, kind, tolerance=0.001, faces=None):
    pads = (np.asarray(pads_world) - object_pose[:3, 3]) @ object_pose[:3, :3]
    faces = mesh(str(path)) if faces is None else faces
    distances = [surface_distance(point, faces) for point in pads]
    if max(distances) > tolerance:
        raise ValueError(
            f"{kind}: fingertips are {np.round(np.asarray(distances)*1000, 2).tolist()}mm from the mesh; maximum is 1mm"
        )
    if kind == "gear":
        # Both fingertips must touch the outside teeth, never the centre bore.
        if (
            min(np.linalg.norm(pads[:, :2], axis=1)) < 0.012
            or np.dot(pads[0, :2], pads[1, :2]) >= 0
        ):
            raise ValueError("Gear grip must straddle the outside circumference")
    if kind == "box" and (
        abs(pads[:, 1].mean()) > tolerance or pads[0, 0] * pads[1, 0] >= 0
    ):
        raise ValueError(
            "Box grip must be centred along its length and touch opposite sides"
        )
    return distances


def triangles_in_box(faces, pose, half, margin=0.0005):
    # Triangle/box separating-axis test in the box frame.
    triangles = (faces - pose[:3, 3]) @ pose[:3, :3]
    half = np.asarray(half) - margin
    possible = np.ones(len(triangles), dtype=bool)
    edges = np.roll(triangles, -1, axis=1) - triangles
    axes = [np.broadcast_to(axis, (len(triangles), 3)) for axis in np.eye(3)]
    axes.append(np.cross(edges[:, 0], edges[:, 1]))
    axes.extend(
        np.cross(edges[:, edge], axis) for edge in range(3) for axis in np.eye(3)
    )
    for axis in axes:
        projection = np.einsum("nij,nj->ni", triangles, axis)
        radius = np.abs(axis) @ half
        possible &= (projection.min(1) <= radius) & (projection.max(1) >= -radius)
    return bool(possible.any())
