# STL silhouette matching for a fixed camera and a plain contrasting background.
from pathlib import Path
import cv2
import numpy as np
from scipy.spatial.transform import Rotation


def triangles(path):
    data = Path(path).read_bytes()
    dtype = np.dtype(
        [("normal", "<f4", 3), ("points", "<f4", (3, 3)), ("attribute", "<u2")]
    )
    count = int.from_bytes(data[80:84], "little")
    if len(data) != 84 + 50 * count:
        raise ValueError("Reference must be a binary STL")
    return np.frombuffer(data, dtype=dtype, count=count, offset=84)["points"].astype(
        float
    )


def normalise(mask, size=256):
    points = cv2.findNonZero(mask)
    if points is None:
        return np.zeros((size, size), np.uint8)
    centre, axes = cv2.PCACompute(points[:, 0].astype(np.float32), mean=None)
    angle = np.degrees(np.arctan2(axes[0, 1], axes[0, 0]))
    matrix = cv2.getRotationMatrix2D(tuple(centre[0]), angle, 1)
    # Pad before rotating so tall or diagonal objects are not clipped.
    side = int(np.hypot(*mask.shape)) + 4
    matrix[:, 2] += np.array([side / 2, side / 2]) - centre[0]
    rotated = cv2.warpAffine(mask, matrix, (side, side), flags=cv2.INTER_NEAREST)
    x, y, w, h = cv2.boundingRect(cv2.findNonZero(rotated))
    crop = rotated[y : y + h, x : x + w]
    scale = (size - 20) / max(w, h)
    crop = cv2.resize(
        crop,
        (max(1, round(w * scale)), max(1, round(h * scale))),
        interpolation=cv2.INTER_NEAREST,
    )
    result = np.zeros((size, size), np.uint8)
    y, x = (size - crop.shape[0]) // 2, (size - crop.shape[1]) // 2
    result[y : y + crop.shape[0], x : x + crop.shape[1]] = crop
    return result


def stl_template(path, view_rpy=(np.pi / 2, 0, 0)):
    mesh = triangles(path) @ Rotation.from_euler("xyz", view_rpy).as_matrix().T
    xy = mesh[:, :, :2]
    low, high = xy.min(axis=(0, 1)), xy.max(axis=(0, 1))
    xy = (xy - low) * 460 / max(high - low) + 20
    mask = np.zeros((512, 512), np.uint8)
    for triangle in xy.astype(np.int32):
        cv2.fillConvexPoly(mask, triangle, 255)
    return normalise(mask)


def outline_score(mask, reference):
    def outer_outline(image):
        contours, _ = cv2.findContours(
            image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )
        filled = np.zeros_like(image)
        cv2.drawContours(filled, contours, -1, 255, cv2.FILLED)
        return normalise(filled) > 0

    candidate = outer_outline(mask)
    reference = outer_outline(reference)
    # Both ends of the toy are allowed to face either direction.
    return min(
        1
        - np.count_nonzero(candidate & ref) / max(1, np.count_nonzero(candidate | ref))
        for ref in (reference, np.flip(reference, (0, 1)))
    )


def inspect_frame(frame, reference, config):
    h, w = frame.shape[:2]
    rx, ry, rw, rh = config["roi"]
    x, y, cw, ch = int(rx * w), int(ry * h), int(rw * w), int(rh * h)
    crop = frame[y : y + ch, x : x + cw]
    gray = cv2.cvtColor(crop, cv2.COLOR_BGR2GRAY)
    gray = cv2.GaussianBlur(gray, (5, 5), 0)
    _, mask = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)
    # Use the minority intensity as foreground; keep the background outside the ROI.
    if np.count_nonzero(mask) > mask.size * 0.55:
        mask = cv2.bitwise_not(mask)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    contours = [c for c in contours if cv2.contourArea(c) >= config["min_area"]]
    output = frame.copy()
    cv2.rectangle(output, (x, y), (x + cw, y + ch), (180, 180, 180), 1)
    if not contours:
        return output, None
    contour = max(contours, key=cv2.contourArea)
    target = np.zeros_like(mask)
    cv2.drawContours(target, [contour], -1, 255, cv2.FILLED)
    score = outline_score(target, reference)
    colour = (0, 220, 0) if score <= config["match_limit"] else (0, 0, 240)
    cv2.drawContours(output, [contour + np.array([[[x, y]]])], -1, colour, 2)
    label = "Fidget toy" if score <= config["match_limit"] else "Unverified object"
    cv2.putText(output, label, (20, 75), cv2.FONT_HERSHEY_SIMPLEX, 0.7, colour, 2)
    cv2.putText(
        output,
        f"Toy outline mismatch: {score:.3f}",
        (20, h - 25),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.6,
        colour,
        2,
    )
    return output, score


def render_parts(meshes, poses, base, view_rpy):
    # Orthographic CAD camera for repeatable simulation tests, not real-camera evidence.
    rotation = Rotation.from_euler("xyz", view_rpy).as_matrix()
    triangles_in_view = []
    for part, mesh in meshes.items():
        if part not in poses:
            continue
        relative = np.linalg.inv(base) @ poses[part]
        if np.linalg.norm(relative[:3, 3]) > 0.12:
            continue
        local = mesh @ relative[:3, :3].T + relative[:3, 3]
        triangles_in_view.append(local @ rotation.T)
    image = np.full((720, 960, 3), 255, np.uint8)
    if not triangles_in_view:
        return image
    xy = np.concatenate(triangles_in_view)[:, :, :2]
    low, high = xy.min((0, 1)), xy.max((0, 1))
    xy = (xy - (low + high) / 2) * 500 / max(high - low) + [480, 360]
    for triangle in xy.astype(np.int32):
        cv2.fillConvexPoly(image, triangle, (35, 35, 35))
    return image
