# Plan matched pickup and placement poses for the five-axis SO101 arms.
import hashlib
import json
from concurrent.futures import CancelledError
import numpy as np
from scipy.optimize import least_squares
from .vision import triangles
from .grasp_geometry import triangles_in_box
from scipy.spatial.transform import Rotation
from .arm_model import transform


def frame(values):
    return transform(values[:3], values[3:])


class TransferPlanner:
    def __init__(self, model, bases, contact_seeds=None):
        self.model, self.bases = model, bases
        self.contact_seeds = contact_seeds if contact_seeds is not None else {}
        self.limits = model.limits[:5].copy()
        self.limits[:, 0] += 0.05
        self.limits[:, 1] -= 0.05
        self.closed, self.open = 0.0, 0.5
        self.cancel_event = None

    def check_cancelled(self):
        if self.cancel_event is not None and self.cancel_event.is_set():
            raise CancelledError("Replaced by a newer Reset")

    def tool(self, arm, q):
        return self.bases[arm] @ self.model.fk(q)[0]

    def transfer(self, arm, poses, offset=(0, 0, 0.002), free_yaw=False, grip_rpy=None):
        # Choose the grasp orientation and the destination joints together.
        # This avoids demanding six independent axes from a five-axis arm.
        count = len(poses)
        poses = [pose.copy() for pose in poses]
        low, high = np.tile(self.limits[:, 0], count), np.tile(self.limits[:, 1], count)
        if free_yaw:
            low, high = np.r_[low, -np.pi], np.r_[high, np.pi]
        pickup_point = (poses[0] @ np.r_[offset, 1])[:3]

        def residual(q):
            self.check_cancelled()
            tools = [
                self.tool(arm, np.r_[j, self.closed])
                for j in q[: count * 5].reshape(-1, 5)
            ]
            held = np.linalg.inv(tools[0]) @ poses[0]
            errors = [(tools[0][:3, 3] - pickup_point) / 0.003]
            if grip_rpy is not None:
                desired = (
                    poses[0][:3, :3] @ Rotation.from_euler("xyz", grip_rpy).as_matrix()
                )
                errors.append(
                    Rotation.from_matrix(desired.T @ tools[0][:3, :3]).as_rotvec()
                    / 0.03
                )
            targets = [pose.copy() for pose in poses[1:]]
            if free_yaw:
                targets[-1][:3, :3] = (
                    Rotation.from_euler("z", q[-1]).as_matrix() @ targets[-1][:3, :3]
                )
            for tool, target in zip(tools[1:], targets):
                actual = tool @ held
                errors += [
                    (actual[:3, 3] - target[:3, 3]) / 0.003,
                    Rotation.from_matrix(target[:3, :3].T @ actual[:3, :3]).as_rotvec()
                    / 0.12,
                ]
            return np.concatenate(errors)

        seeds = []
        for pose in poses:
            local = (np.linalg.inv(self.bases[arm]) @ np.r_[pose[:3, 3], 1])[:3]
            seed = self.model.solve(local, np.array([0, -0.6, 0.2, 0, 0, self.closed]))
            seeds.extend(seed[:5] if seed is not None else [0, -0.6, 0.2, 0, 0])
        best = None
        for index in range(32):
            rng = np.random.default_rng(index)
            start = np.asarray(seeds) + rng.normal(0, 0.8 if index else 0, count * 5)
            if free_yaw:
                start = np.r_[start, 0.0]
            result = least_squares(
                residual,
                np.clip(start, low + 0.001, high - 0.001),
                bounds=(low, high),
                max_nfev=240,
            )
            score = np.linalg.norm(residual(result.x))
            if best is None or score < best[0]:
                best = score, result.x
            if score < 0.75:
                break
        qs = [np.r_[q, self.closed] for q in best[1][: count * 5].reshape(-1, 5)]
        if free_yaw:
            poses[-1][:3, :3] = (
                Rotation.from_euler("z", best[1][-1]).as_matrix() @ poses[-1][:3, :3]
            )
        if np.linalg.norm(self.tool(arm, qs[0])[:3, 3] - pickup_point) > 0.003:
            raise ValueError(f"{arm}: grasp point is too far from the part")
        held = np.linalg.inv(self.tool(arm, qs[0])) @ poses[0]
        for q, target in zip(qs[1:], poses[1:]):
            actual = self.tool(arm, q) @ held
            distance = np.linalg.norm(actual[:3, 3] - target[:3, 3])
            angle = Rotation.from_matrix(target[:3, :3].T @ actual[:3, :3]).magnitude()
            if distance > 0.004 or angle > 0.12:
                raise ValueError(
                    f"{arm}: placement error {distance:.4f}m / {angle:.3f}rad"
                )
        return {"q": qs, "held": held, "poses": poses, "score": best[0]}

    def roof_clearance(self, tool, jaw, object_pose):
        # Keep the fingers outside the lid skirt when lifting the completed box.
        if not hasattr(self, "finger_edges"):
            self.finger_edges = []
            for name in ("Fixed_Gripper", "Moving_Jaw"):
                points, indices = np.unique(
                    triangles(
                        self.model.path.parent.parent
                        / f"models/project2_rig/meshes/{name}.stl"
                    ).reshape(-1, 3),
                    axis=0,
                    return_inverse=True,
                )
                faces = indices.reshape(-1, 3)
                edges = np.concatenate(
                    [faces[:, [0, 1]], faces[:, [1, 2]], faces[:, [2, 0]]]
                )
                edges = np.unique(np.sort(edges, axis=1), axis=0)
                self.finger_edges.append(points[edges])
        grip = np.linalg.inv(object_pose) @ tool @ np.linalg.inv(self.model.tip)
        poses = [grip, grip @ self.model.origins[5] @ transform(rpy=[0, 0, jaw])]
        errors = []
        for edges, pose in zip(self.finger_edges, poses):
            edges = edges @ pose[:3, :3].T + pose[:3, 3]
            start, end = edges[:, 0], edges[:, 1]
            dz = end[:, 2] - start[:, 2]
            cuts = (
                (np.minimum(start[:, 2], end[:, 2]) <= 0.0135)
                & (np.maximum(start[:, 2], end[:, 2]) >= 0.0135)
                & (np.abs(dz) > 1e-10)
            )
            if cuts.any():
                fraction = (0.0135 - start[cuts, 2]) / dz[cuts]
                xy = start[cuts, :2] + fraction[:, None] * (
                    end[cuts, :2] - start[cuts, :2]
                )
                penetration = np.min(np.array([0.02795, 0.05075]) - np.abs(xy), axis=1)
                errors.append(max(0.0, float(penetration.max())))
            else:
                errors.append(0.0)
        return np.asarray(errors)

    def lid_collision(self, arm, q, box_pose):
        # Check whole triangles: a wide face can cross the lid even when its edges miss.
        if not hasattr(self, "finger_faces"):
            folder = self.model.path.parent.parent / "models/project2_rig/meshes"
            self.finger_faces = [
                triangles(folder / f"{name}.stl")
                for name in ("Fixed_Gripper", "Moving_Jaw")
            ]
        grip = self.tool(arm, q) @ np.linalg.inv(self.model.tip)
        moving = grip @ self.model.origins[5] @ transform(rpy=[0, 0, q[5]])
        roof = box_pose @ transform([0, 0, 0.0135])
        return any(
            triangles_in_box(
                faces @ pose[:3, :3].T + pose[:3, 3], roof, [0.02745, 0.05025, 0.001]
            )
            for faces, pose in zip(self.finger_faces, (grip, moving))
        )

    def contact_transfer(self, arm, poses, contact, free_yaw=False, attempt_limit=None):
        # Solve both fingertips on opposite outside faces, together with the place pose.
        poses = [p.copy() for p in poses]
        count = len(poses)
        low = list(np.tile(self.limits[:, 0], count))
        high = list(np.tile(self.limits[:, 1], count))
        low += [
            -0.18,
            contact["height_range"][0],
            contact["height_range"][0],
            -contact.get("slide_m", 0.001),
        ]
        high += [
            0.95,
            contact["height_range"][1],
            contact["height_range"][1],
            contact.get("slide_m", 0.001),
        ]
        low += [-contact.get("skew_m", 0.006), -0.104, -0.080]
        high += [contact.get("skew_m", 0.006), -0.097, -0.073]
        low += [-0.8, -0.8]
        high += [0.8, 0.8]
        if contact.get("round"):
            low += [-np.pi]
            high += [np.pi]
        if free_yaw:
            low += [-np.pi]
            high += [np.pi]
        low, high = np.asarray(low), np.asarray(high)
        points = np.asarray(contact["points"], dtype=float)
        seeds = []
        for pose in poses:
            local = (np.linalg.inv(self.bases[arm]) @ np.r_[pose[:3, 3], 1])[:3]
            seed = self.model.solve(local, np.array([0, -0.6, 0.2, 0, 0, 0]))
            seeds.extend(seed[:5] if seed is not None else [0, -0.6, 0.2, 0, 0])

        def residual(values):
            self.check_cancelled()
            joints = values[: count * 5].reshape(-1, 5)
            tools = [self.tool(arm, np.r_[q, 0]) for q in joints]
            jaw, height_a, height_b, slide = values[count * 5 : count * 5 + 4]
            desired = points.copy()
            if contact.get("round"):
                desired = (
                    desired
                    @ Rotation.from_euler("z", values[count * 5 + 9]).as_matrix().T
                )
            desired[:, 2] += [height_a, height_b]
            desired[:, contact["slide_axis"]] += (
                slide + np.array([1, -1]) * values[count * 5 + 4]
            )
            desired = desired @ poses[0][:3, :3].T + poses[0][:3, 3]
            pads = self.pad_points(
                tools[0],
                jaw,
                values[count * 5 + 5 : count * 5 + 7],
                values[count * 5 + 7 : count * 5 + 9],
            )
            errors = [(pads - desired).ravel() / 0.0005]
            held = np.linalg.inv(tools[0]) @ poses[0]
            target = poses[-1].copy()
            if free_yaw:
                target[:3, :3] = (
                    Rotation.from_euler("z", values[-1]).as_matrix() @ target[:3, :3]
                )
            actual = tools[-1] @ held
            errors += [
                (actual[:3, 3] - target[:3, 3]) / 0.0005,
                Rotation.from_matrix(target[:3, :3].T @ actual[:3, :3]).as_rotvec()
                / 0.01,
                np.array([height_a / 0.05, height_b / 0.05, slide / 0.05]),
            ]
            if contact.get("normal_alignment"):
                # A thin tray wall needs the pad faces square to its two sides.
                grip = tools[0] @ np.linalg.inv(self.model.tip)
                moving = grip @ self.model.origins[5] @ transform(rpy=[0, 0, jaw])
                direction = poses[0][:3, :3] @ (points[1] - points[0])
                direction /= np.linalg.norm(direction)
                alignment = np.array(
                    [-grip[:3, 0] @ direction, moving[:3, 0] @ direction]
                )
                errors.append(
                    np.maximum(0.0, contact["normal_alignment"] - alignment) / 0.02
                )
            if contact.get("closed_box"):
                errors.append(self.roof_clearance(tools[0], jaw, poses[0]) / 0.0005)
            return np.concatenate(errors)

        # Reuse solved seeds only after checking them against the current geometry.
        signature = json.dumps(
            {
                "version": 2,
                "arm": arm,
                "contact": contact,
                "free_yaw": free_yaw,
                "poses": [
                    np.round(np.linalg.inv(self.bases[arm]) @ pose, 9).tolist()
                    for pose in poses
                ],
                "limits": self.limits.tolist(),
                "origins": [v.tolist() for v in self.model.origins],
            },
            sort_keys=True,
        )
        key = hashlib.sha256(signature.encode()).hexdigest()
        cached = np.asarray(self.contact_seeds.get(key, []), dtype=float)
        best = None
        if (
            cached.shape == low.shape
            and np.isfinite(cached).all()
            and np.all(cached >= low)
            and np.all(cached <= high)
        ):
            error = residual(cached)[:12]
            if (
                (
                    not (contact.get("closed_box") or contact.get("normal_alignment"))
                    or max(residual(cached)[-2:]) < 0.1
                )
                and np.max(np.linalg.norm(error[:6].reshape(2, 3) * 0.0005, axis=1))
                <= 0.0009
                and np.linalg.norm(error[6:9] * 0.0005)
                + 0.06 * np.linalg.norm(error[9:12] * 0.01)
                <= 0.0009
            ):
                best = np.linalg.norm(error), cached
        for index in range(
            0
            if best is not None
            else min(
                contact.get("attempts", 12 if contact.get("closed_box") else 36),
                attempt_limit if attempt_limit is not None else 1000,
            )
        ):
            values = np.r_[
                np.asarray(seeds)
                + np.random.default_rng(index).normal(
                    0, 0.8 if index else 0, count * 5
                ),
                0.2,
                0.0,
                0.0,
                0.0,
                0.0,
                -0.100,
                -0.076,
                0.0,
                0.0,
            ]
            if contact.get("round"):
                values = np.r_[values, 0.0]
            if free_yaw:
                values = np.r_[values, 0.0]
            fit = least_squares(
                residual,
                np.clip(values, low + 1e-7, high - 1e-7),
                bounds=(low, high),
                max_nfev=350,
            )
            score = np.linalg.norm(residual(fit.x)[:12]) + (
                np.linalg.norm(residual(fit.x)[-2:])
                if contact.get("closed_box") or contact.get("normal_alignment")
                else 0.0
            )
            if best is None or score < best[0]:
                best = score, fit.x
            error = residual(fit.x)[:12]
            if (
                (
                    not (contact.get("closed_box") or contact.get("normal_alignment"))
                    or max(residual(fit.x)[-2:]) < 0.1
                )
                and np.max(np.linalg.norm(error[:6].reshape(2, 3) * 0.0005, axis=1))
                <= 0.0009
                and np.linalg.norm(error[6:9] * 0.0005)
                + 0.06 * np.linalg.norm(error[9:12] * 0.01)
                <= 0.0009
            ):
                best = score, fit.x
                break
        score, values = best
        errors = residual(values)[:12]
        contact_error = np.max(
            np.linalg.norm(errors[:6].reshape(2, 3) * 0.0005, axis=1)
        )
        placement_error = np.linalg.norm(errors[6:9] * 0.0005) + 0.06 * np.linalg.norm(
            errors[9:12] * 0.01
        )
        if (
            contact_error > 0.0009
            or placement_error > 0.0009
            or (
                (contact.get("closed_box") or contact.get("normal_alignment"))
                and max(residual(values)[-2:]) >= 0.1
            )
        ):
            lid_error = (
                max(residual(values)[-2:]) * 0.0005
                if contact.get("closed_box")
                else 0.0
            )
            raise ValueError(
                f"{arm}: grip clearance failed (contact {contact_error * 1000:.2f}mm, "
                f"placement {placement_error * 1000:.2f}mm, lid intrusion {lid_error * 1000:.2f}mm); "
                "adjust the contact/fixture waypoints"
            )
        jaw = values[count * 5]
        qs = [np.r_[q, jaw] for q in values[: count * 5].reshape(-1, 5)]
        if contact.get("closed_box") and self.lid_collision(arm, qs[0], poses[0]):
            raise ValueError(
                f"{arm}: completed-box grip intersects the lid; adjust the pickup pose"
            )
        self.contact_seeds[key] = values.tolist()
        if free_yaw:
            poses[-1][:3, :3] = (
                Rotation.from_euler("z", values[-1]).as_matrix() @ poses[-1][:3, :3]
            )
        return {
            "q": qs,
            "held": np.linalg.inv(self.tool(arm, qs[0])) @ poses[0],
            "poses": poses,
            "score": score,
            "contact": contact,
            "pad_y": values[count * 5 + 5 : count * 5 + 7],
            "pad_z": values[count * 5 + 7 : count * 5 + 9],
        }

    def pad_points(self, tool, jaw, pad_y=(-0.100, -0.076), pad_z=(0.0, 0.0)):
        # These points lie on the flat inside pads in the university gripper meshes.
        grip = tool @ np.linalg.inv(self.model.tip)
        moving = grip @ self.model.origins[5] @ transform(rpy=[0, 0, jaw])
        widths = np.array(
            [0.001 + 0.35 * (pad_y[0] + 0.104), 0.001 + 0.35 * (pad_y[1] + 0.080)]
        )
        z = np.asarray(pad_z) * widths
        return np.array(
            [
                (grip @ [0.0079, pad_y[0], z[0], 1])[:3],
                (moving @ [-0.0123, pad_y[1], 0.0001 + z[1], 1])[:3],
            ]
        )

    def tool_pose(self, arm, target, seed):
        # Preserve orientation during short lifts and insertions; position-only IK
        # can turn a held lid or insert over while reaching the same tool point.
        def error(joints):
            pose = self.tool(arm, np.r_[joints, seed[5]])
            return np.r_[
                (pose[:3, 3] - target[:3, 3]) / 0.002,
                Rotation.from_matrix(target[:3, :3].T @ pose[:3, :3]).as_rotvec()
                / 0.04,
            ]

        fit = least_squares(
            error,
            seed[:5],
            bounds=(self.limits[:, 0], self.limits[:, 1]),
            max_nfev=300,
            ftol=1e-10,
            xtol=1e-10,
            gtol=1e-10,
        )
        if np.linalg.norm(error(fit.x)) > 1.5:
            raise ValueError(
                f"{arm}: approach cannot retain orientation ({np.linalg.norm(error(fit.x)):.2f})"
            )
        return np.r_[fit.x, seed[5]]

    def approach(self, arm, transfer, index, dz=0.045):
        q = transfer["q"][index].copy()
        target = self.tool(arm, q).copy()
        target[2, 3] += dz
        return self.tool_pose(arm, target, q)

    def camera_view(self, arm, target, config, view, seed):
        # The camera stays on arm 3 and looks at the toy on the stopped belt.
        # Replace this mounting estimate with the measured tool-to-camera pose.
        mount = frame(config["tool_to_camera"])
        direction = (
            -target[:3, :3]
            @ Rotation.from_euler("xyz", view["rpy"]).as_matrix().T[:, 2]
        )
        eye = target[:3, 3] - direction * view.get("distance_m", config["height_m"])

        def residual(joints):
            camera = self.tool(arm, np.r_[joints, self.open]) @ mount
            return np.r_[
                (camera[:3, 3] - eye) / 0.003, (camera[:3, 2] - direction) / 0.04
            ]

        best = None
        for index in range(20):
            start = seed[:5] + np.random.default_rng(index).normal(0, 0.7, 5)
            fit = least_squares(
                residual,
                np.clip(start, self.limits[:, 0] + 0.001, self.limits[:, 1] - 0.001),
                bounds=(self.limits[:, 0], self.limits[:, 1]),
                max_nfev=250,
            )
            score = np.linalg.norm(residual(fit.x))
            if best is None or score < best[0]:
                best = score, np.r_[fit.x, self.open]
            if score < 0.1:
                break
        if best[0] > 1.0:
            raise ValueError(
                "Camera view is outside arm 3 workspace; calibrate camera.mount"
            )
        return best[1]

    def held_flat(self, arm, held, position, seed):
        # Five axes can put the toy flat while leaving its yaw unconstrained.
        def error(joints):
            pose = self.tool(arm, np.r_[joints, self.closed]) @ held
            return np.r_[
                (pose[:3, 3] - position) / 0.003, (pose[:3, 2] - [0, 0, 1]) / 0.04
            ]

        best = None
        for index in range(24):
            start = seed[:5] + np.random.default_rng(index).normal(
                0, 0.7 if index else 0, 5
            )
            fit = least_squares(
                error,
                np.clip(start, self.limits[:, 0] + 0.001, self.limits[:, 1] - 0.001),
                bounds=(self.limits[:, 0], self.limits[:, 1]),
                max_nfev=220,
            )
            score = np.linalg.norm(error(fit.x))
            if best is None or score < best[0]:
                best = score, np.r_[fit.x, self.closed]
            if score < 0.1:
                break
        if best[0] > 0.5:
            raise ValueError("Reject position is outside arm 3 workspace")
        return best[1]

    def discard_carrier(self, loaded, carrier):
        # Regrasp the same front tab. An empty plate may tilt outside the table.
        from .kit_trays import faces

        arm = loaded["arm"]
        pickup = loaded["q"][1].copy()
        held = loaded["held"]
        vertices = faces(carrier).reshape(-1, 3)
        target = np.asarray(carrier["discard"][:3])
        edge = carrier.get("table_left_edge", -0.9) - 0.002

        def error(joints):
            self.check_cancelled()
            pose = self.tool(arm, np.r_[joints, pickup[5]]) @ held
            points = vertices @ pose[:3, :3].T + pose[:3, 3]
            return np.r_[
                (pose[:3, 3] - target) / 0.0003,
                max(0.0, points[:, 0].max() - edge) / 0.0003,
                (joints - pickup[:5]) * 0.002,
            ]

        for attempt in range(12):
            seed = pickup[:5] + np.random.default_rng(attempt).normal(
                0, 0.8 if attempt else 0, 5
            )
            fit = least_squares(
                error,
                np.clip(seed, self.limits[:, 0] + 0.001, self.limits[:, 1] - 0.001),
                bounds=(self.limits[:, 0], self.limits[:, 1]),
                max_nfev=300,
            )
            if np.linalg.norm(error(fit.x)[:3]) < 1 and error(fit.x)[3] < 1:
                break
        else:
            raise ValueError("Empty carrier cannot clear the left table edge")
        destination = np.r_[fit.x, pickup[5]]
        return dict(
            arm=arm,
            q=[pickup, destination],
            held=held,
            poses=[loaded["poses"][1], self.tool(arm, destination) @ held],
            approach=[loaded["approach"][1].copy(), destination.copy()],
            open=loaded["open"],
            contact=loaded["contact"],
            pad_y=loaded["pad_y"],
            pad_z=loaded["pad_z"],
            score=0.0,
        )

    def held_position(self, arm, held, position, seed):
        target = self.tool(arm, seed) @ held
        target[:3, 3] = position
        return self.tool_pose(arm, target @ np.linalg.inv(held), seed)


def captured_transfer(planner, arm, poses, overrides, contact=None):
    # A fully taught pickup/place pair does not need the numerical grip solver.
    # Contact and collision checks still run before the simulated grasp.
    qs = []
    for name in ("pickup", "place"):
        q = np.asarray(overrides[name], dtype=float)
        if (
            q.shape != (6,)
            or not np.isfinite(q).all()
            or np.any(q < planner.model.limits[:, 0])
            or np.any(q > planner.model.limits[:, 1])
        ):
            raise ValueError("Captured " + name + " is outside joint limits")
        qs.append(q)
    if abs(qs[0][5] - qs[1][5]) > 0.001:
        raise ValueError("Keep the same gripper opening while carrying the part")
    held = np.linalg.inv(planner.tool(arm, qs[0])) @ poses[0]
    actual = planner.tool(arm, qs[1]) @ held
    error = np.linalg.norm(actual[:3, 3] - poses[1][:3, 3])
    turn = np.linalg.norm(
        Rotation.from_matrix(poses[1][:3, :3].T @ actual[:3, :3]).as_rotvec()
    )
    if error + 0.06 * turn > 0.001:
        raise ValueError(
            "Captured placement differs from the configured target by more than 1mm; edit the target or recapture"
        )
    result = dict(q=qs, held=held, poses=[p.copy() for p in poses], score=error)
    if contact is not None:
        result.update(
            contact=contact, pad_y=np.array([-0.100, -0.076]), pad_z=np.zeros(2)
        )
    return result


def make_plans(
    planner,
    config,
    starts,
    alignment,
    count=5,
    preview=None,
    cancel=None,
    allow_partial=False,
):
    # Each transfer owns one arm; the runtime schedules independent transfers together.
    left, right, inspect, pack = (
        "assembly_left",
        "assembly_right",
        "inspection",
        "packaging",
    )
    result = {}
    planner.cancel_event = cancel
    planner.check_cancelled()

    def add(name, arm, source, target, offset=(0, 0, 0.002), free=False, feeder=False):
        planner.check_cancelled()
        if preview is not None:
            preview["current"] = name
        grip_key = (
            "stack"
            if name.startswith("stack_")
            else ("tray" if name == "load_carrier" else name)
        )
        offset = config["grips"].get(name, config["grips"].get(grip_key, offset))
        overrides = config.get("joint_overrides", {}).get(name, {})
        if "pickup" in overrides and "place" in overrides:
            plan = captured_transfer(
                planner,
                arm,
                [source, target],
                overrides,
                config.get("contacts", {}).get(grip_key),
            )
        elif feeder:
            reverse = planner.transfer(arm, [target, source], offset, free_yaw=True)
            source = planner.tool(arm, reverse["q"][1]) @ reverse["held"]
            plan = {
                "q": reverse["q"][::-1],
                "poses": [source, target],
                "held": reverse["held"],
                "score": reverse["score"],
            }
        elif grip_key in config.get("contacts", {}):
            try:
                plan = planner.contact_transfer(
                    arm,
                    [source, target],
                    config["contacts"][grip_key],
                    free_yaw=free,
                    attempt_limit=(
                        2 if allow_partial and name.startswith("stack_") else None
                    ),
                )
            except ValueError as error:
                raise ValueError(f"{name}: {error}") from error
        else:
            plan = planner.transfer(
                arm,
                [source, target],
                offset,
                free_yaw=free,
                grip_rpy=config.get("grip_orientation", {}).get(grip_key),
            )
        plan["arm"] = arm
        plan["approach"] = [
            (
                np.asarray(overrides["approach_pickup"], dtype=float)
                if "approach_pickup" in overrides
                else planner.approach(
                    arm, plan, 0, 0.04 if grip_key == "tray" else config["approach_m"]
                )
            ),
            (
                np.asarray(overrides["approach_place"], dtype=float)
                if "approach_place" in overrides
                else (
                    plan["q"][1].copy()
                    if name == "tray"
                    else planner.approach(arm, plan, 1, config["approach_m"])
                )
            ),
        ]
        if name in ("box", "lid") and "approach_pickup" not in overrides:
            target = planner.tool(arm, plan["q"][0]).copy()
            target[:3, 3] += plan["poses"][0][:3, 2] * config["feeder_clearance_m"]
            plan["approach"][0] = planner.tool_pose(arm, target, plan["q"][0])
        if name == "load_carrier" and "approach_pickup" not in overrides:
            # Leave the inclined rack along its normal before turning the plate.
            target = planner.tool(arm, plan["q"][0]).copy()
            target[:3, 3] += source[:3, 2] * 0.08
            plan["approach"][0] = planner.tool_pose(arm, target, plan["q"][0])
        jaw = config.get("gripper_angles", {}).get(grip_key, {})
        plan["open"] = (
            min(1.0, plan["q"][0][5] + 0.30)
            if "contact" in plan
            else jaw.get("open", planner.open)
        )
        if grip_key == "tray":
            plan["open"] = plan["q"][0][5] + config["kit_trays"]["jaw_opening_rad"]
        for q in plan["q"] + plan["approach"]:
            if "contact" not in plan:
                q[5] = jaw.get("closed", planner.closed)
        if config.get("contacts", {}).get(grip_key, {}).get("closed_box"):
            for q in (plan["q"][0], plan["approach"][0]):
                for opening in (q[5], plan["open"]):
                    check = q.copy()
                    check[5] = opening
                    if planner.lid_collision(arm, check, plan["poses"][0]):
                        raise ValueError(f"{name}: gripper approach intersects the lid")
        result[name] = plan
        if preview is not None:
            # Show solved transfers even if a later task fails. These are teaching
            # candidates; the full production run still requires all checks.
            preview["paths"] = {
                key: dict(
                    arm=value["arm"],
                    points=dict(
                        approach_pickup=value["approach"][0].tolist(),
                        pickup=value["q"][0].tolist(),
                        approach_place=value["approach"][1].tolist(),
                        place=value["q"][1].tolist(),
                    ),
                )
                for key, value in result.items()
                if isinstance(value, dict) and "q" in value
            }
        return planner.tool(arm, plan["q"][-1]) @ plan["held"]

    carrier = config.get("kit_trays")
    if carrier and carrier.get("work_pose"):
        config["contacts"]["tray"] = carrier["contact"]
        rack = frame(carrier["first_pose"])
        work = add("load_carrier", left, rack, frame(carrier["work_pose"]), free=True)
        # Parts stay at their CAD offsets while the complete carrier is moved.
        starts = {
            name: work @ np.linalg.inv(rack) @ pose for name, pose in starts.items()
        }

    fixture = add(
        "base",
        right,
        starts["base_1v4"],
        frame(config["fixture"]),
        offset=(0.025, 0, 0.007),
        free=False,
    )
    gear = transform(alignment["2V4"]["xyz_m"], alignment["2V4"]["rpy_rad"])
    insert = transform(alignment["3V4"]["xyz_m"], alignment["3V4"]["rpy_rad"])
    add("gear", left, starts["pin_2v4"], fixture @ gear, free=True)
    handoff = add(
        "handoff",
        left,
        starts["insert_3v4"],
        fixture @ frame(config["handoff_offset"]),
        free=False,
    )
    add("insert", right, handoff, fixture @ insert)
    # Approach the open end horizontally; the base is not turned over.
    insert_target = result["insert"]["poses"][1][:3, 3] + fixture[:3, 0] * 0.040
    result["insert"]["approach"][1] = planner.held_position(
        right, result["insert"]["held"], insert_target, result["insert"]["q"][1]
    )
    belt_in = add(
        "belt", right, fixture, frame(config["conveyor_in"]), offset=(0.025, 0, 0.007)
    )
    belt_out = belt_in.copy()
    belt_out[:3, 3] += np.array(config["conveyor_out"][:3]) - np.array(
        config["conveyor_in"][:3]
    )
    nest = add(
        "box",
        pack,
        frame(config["box_feed"]),
        frame(config["box_nest"]),
        offset=(0.023, 0, 0.009),
    )
    # Use the imported base bounds so CAD height changes cannot sink it into the box.
    toy_z = (
        -config["box_size"][2] / 2
        + config["wall"]
        - alignment["1V4"]["bounds_m"][0][2]
        + config["packaging_clearance"]
    )
    toy_target = nest @ transform((0, 0, toy_z), (0, 0, np.pi / 2))
    # The toy's long axis is X; the box's long axis is Y.
    placed_toy = add("inspect", inspect, belt_out, toy_target, offset=(0.025, 0, 0.007))
    nest[:3, :3] = placed_toy[:3, :3] @ Rotation.from_euler("z", -np.pi / 2).as_matrix()
    nest = add("box", pack, frame(config["box_feed"]), nest, offset=(0.023, 0, 0.009))
    result["inspect"]["camera"] = {
        name: planner.camera_view(
            inspect,
            belt_out,
            config["camera"]["mount"],
            view,
            result["inspect"]["q"][0],
        )
        for name, view in config["camera"]["views"].items()
    }
    result["inspect"]["rejects"] = []
    for position in config["reject_slots"]:
        q = planner.held_flat(
            inspect,
            result["inspect"]["held"],
            np.asarray(position),
            result["inspect"]["q"][0],
        )
        approach = planner.held_position(
            inspect, result["inspect"]["held"], np.asarray(position) + [0, 0, 0.04], q
        )
        result["inspect"]["rejects"].append({"q": q, "approach": approach})
    # Fill, close and collect from the original holder; no intermediate box slide.
    lid_target = nest @ transform(
        (0, 0, config["box_size"][2] / 2 + config["packaging_clearance"])
    )
    add("lid", pack, frame(config["lid_feed"]), lid_target)
    if config.get("kit_trays"):
        carrier = config["kit_trays"]
        width, length, floor = carrier["size"]
        wall = carrier["wall"]
        side_x = -(width - wall) / 2
        clearance = carrier["grip_clearance_m"]
        # Grasp the left rim. Keep both pads within 1mm of its actual faces.
        config["contacts"]["tray"] = carrier.get("contact") or {
            "points": [
                [
                    side_x - wall / 2 - clearance,
                    0,
                    floor / 2 + carrier["rim_height"] - 0.0002,
                ],
                [
                    side_x + wall / 2 + clearance,
                    0,
                    floor / 2 + carrier["rim_height"] - 0.0002,
                ],
            ],
            "height_range": [-0.00005, 0.00005],
            "slide_axis": 1,
            "slide_m": 0.006,
            "skew_m": 0.0005,
            "attempts": 12,
            "normal_alignment": 0.99,
        }
        if "load_carrier" in result:
            result["tray"] = planner.discard_carrier(result["load_carrier"], carrier)
        else:
            add(
                "tray",
                left,
                frame(carrier["first_pose"]),
                frame(carrier["discard"]),
                free=True,
            )
        plan = result["tray"]
        plan["open"] = plan["q"][0][5] + carrier["jaw_opening_rad"]
        seed = plan["approach"][0]
        plan["pull"] = []
        for position in carrier["pull_points"]:
            # An empty carrier may turn while being pulled clear of the feeder.
            normal = plan["poses"][0][:3, 2]

            def error(joints):
                actual = planner.tool(left, np.r_[joints, seed[5]]) @ plan["held"]
                return np.r_[
                    (actual[:3, 3] - position) / 0.0003,
                    (actual[:3, 2] - normal) / 0.8,
                    (joints - seed[:5]) * 0.01,
                ]

            fit = least_squares(
                error,
                seed[:5],
                bounds=(planner.limits[:, 0], planner.limits[:, 1]),
                max_nfev=500,
            )
            seed = np.r_[fit.x, seed[5]]
            actual = planner.tool(left, seed) @ plan["held"]
            if np.linalg.norm(actual[:3, 3] - position) > 0.0009:
                raise ValueError("Tray pull waypoint is outside arm 1 workspace")
            plan["pull"].append(seed)
    if preview is not None:
        preview["paths"] = {
            name: dict(
                arm=plan["arm"],
                points=dict(
                    approach_pickup=plan["approach"][0].tolist(),
                    pickup=plan["q"][0].tolist(),
                    approach_place=plan["approach"][1].tolist(),
                    place=plan["q"][1].tolist(),
                ),
            )
            for name, plan in result.items()
            if isinstance(plan, dict) and "q" in plan
        }
    result["unavailable"] = {}
    for index, target in enumerate(config["stack"][:count]):
        name = f"stack_{index}"
        try:
            add(name, pack, nest, frame(target))
        except ValueError as error:
            if not allow_partial:
                raise
            # Test the available sequence. Stop before any unsolved box transfer.
            for pending in range(index, count):
                result["unavailable"][f"stack_{pending}"] = str(error)
            break
    # Plan arm support only when a calibrated, collision-free hold is selected.
    if config.get("support", {}).get("mode", "fixture") == "arm":
        support = planner.contact_transfer(
            left, [fixture, fixture], config["contacts"]["support"]
        )
        support["arm"] = left
        result["support"] = support
        result["hold"] = support["q"][0]
    # Captured joint poses override individual pickup/place/approach targets.
    # Recompute held transforms and reject inconsistent destination poses.
    for name, overrides in config.get("joint_overrides", {}).items():
        if name not in result or "q" not in result[name]:
            if name in {f"stack_{i}" for i in range(count, len(config["stack"]))}:
                continue
            raise ValueError("Unknown transfer override: " + name)
        plan = result[name]
        for key, values in overrides.items():
            group, index = {
                "pickup": ("q", 0),
                "place": ("q", 1),
                "approach_pickup": ("approach", 0),
                "approach_place": ("approach", 1),
            }[key]
            q = np.asarray(values, dtype=float)
            if (
                q.shape != (6,)
                or not np.isfinite(q).all()
                or np.any(q < planner.model.limits[:, 0])
                or np.any(q > planner.model.limits[:, 1])
            ):
                raise ValueError(name + ": joint override is outside limits")
            plan[group][index] = q
        plan["held"] = (
            np.linalg.inv(planner.tool(plan["arm"], plan["q"][0])) @ plan["poses"][0]
        )
        actual = planner.tool(plan["arm"], plan["q"][1]) @ plan["held"]
        if np.linalg.norm(actual[:3, 3] - plan["poses"][1][:3, 3]) > 0.001:
            raise ValueError(
                name + ": captured pickup and place disagree by more than 1 mm"
            )
    # Validate intermediate waypoints before any production movement starts.
    for name, points in config.get("path_points", {}).items():
        if name not in result or "q" not in result[name]:
            # Output slots not used in this run may retain their saved paths.
            if name.startswith("stack_"):
                continue
            raise ValueError("Unknown path: " + name)
        for point in points:
            q = np.asarray(point["joints"], dtype=float)
            if point["phase"] not in ("pickup", "place") or not point["label"].strip():
                raise ValueError(name + ": name the point and select pickup/place")
            if (
                q.shape != (6,)
                or not np.isfinite(q).all()
                or np.any(q < planner.model.limits[:, 0])
                or np.any(q > planner.model.limits[:, 1])
            ):
                raise ValueError(name + ": intermediate joints outside limits")
    result["fixture"] = fixture
    result["belt_in"], result["belt_out"] = belt_in, belt_out
    return result
