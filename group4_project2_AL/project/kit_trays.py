# One removable carrier holds the insert, gear and base for a single product.
import numpy as np
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from .arm_model import transform
from .vision import triangles


def tray_pose(settings, row=0, height=0.0):
    values = settings["first_pose"]
    pose = transform(values[:3], values[3:])
    pose[:3, 3] += np.asarray(settings["row_step"]) * row + [0, 0, height]
    return pose


def segments(settings):
    # Floor and outer rim only; there are no dividers between the components.
    width, length, floor = settings["size"]
    wall, rim = settings["wall"], settings["rim_height"]
    z = (floor + rim) / 2
    return [
        ([0, 0, 0], [width, length, floor]),
        ([-(width - wall) / 2, 0, z], [wall, length, rim]),
        ([(width - wall) / 2, 0, z], [wall, length, rim]),
        ([0, -(length - wall) / 2, z], [width - 2 * wall, wall, rim]),
        ([0, (length - wall) / 2, z], [width - 2 * wall, wall, rim]),
    ]


def faces(settings):
    if settings.get("mesh"):
        root = Path(get_package_share_directory("group4_project2_al"))
        return triangles(root / "models" / settings["mesh"])
    # The contact test uses the same boxes as the visible tray and its collisions.
    vertices = (
        np.array(
            [
                [-1, -1, -1],
                [1, -1, -1],
                [1, 1, -1],
                [-1, 1, -1],
                [-1, -1, 1],
                [1, -1, 1],
                [1, 1, 1],
                [-1, 1, 1],
            ]
        )
        / 2
    )
    indices = np.array(
        [
            [0, 2, 1],
            [0, 3, 2],
            [4, 5, 6],
            [4, 6, 7],
            [0, 1, 5],
            [0, 5, 4],
            [1, 2, 6],
            [1, 6, 5],
            [2, 3, 7],
            [2, 7, 6],
            [3, 0, 4],
            [3, 4, 7],
        ]
    )
    return np.concatenate(
        [
            (vertices * np.asarray(size) + xyz)[indices]
            for xyz, size in segments(settings)
        ]
    )


def tray_sdf(name, settings):
    if settings.get("mesh"):
        # The V6 fork, gear and base supports form one rigid carrier.
        return mesh_sdf(name, settings["mesh"])
    shapes = []
    for index, (xyz, size) in enumerate(segments(settings)):
        pose = " ".join(map(str, xyz)) + " 0 0 0"
        geometry = (
            "<geometry><box><size>"
            + " ".join(map(str, size))
            + "</size></box></geometry>"
        )
        shapes.append(
            f'<visual name="rim_{index}"><pose>{pose}</pose>{geometry}'
            "<material><ambient>0.86 0.87 0.81 1</ambient><diffuse>0.86 0.87 0.81 1</diffuse></material></visual>"
            f'<collision name="solid_{index}"><pose>{pose}</pose>{geometry}</collision>'
        )
    return (
        f'<sdf version="1.9"><model name="{name}"><link name="{name}__part"><gravity>false</gravity>'
        "<inertial><mass>0.04</mass><inertia><ixx>0.00002</ixx><iyy>0.00012</iyy><izz>0.00014</izz></inertia></inertial>"
        + "".join(shapes)
        + "</link></model></sdf>"
    )


def mesh_sdf(name, mesh, fixed=False):
    geometry = f"<geometry><mesh><uri>model://{mesh}</uri></mesh></geometry>"
    return (
        f'<sdf version="1.9"><model name="{name}">'
        f'<static>{str(fixed).lower()}</static><link name="{name}__part"><gravity>false</gravity>'
        "<inertial><mass>0.06</mass><inertia><ixx>0.00004</ixx>"
        "<iyy>0.00015</iyy><izz>0.00018</izz></inertia></inertial>"
        f'<visual name="shape">{geometry}<material>'
        "<ambient>0.90 0.91 0.86 1</ambient><diffuse>0.90 0.91 0.86 1</diffuse>"
        f'</material></visual><collision name="shape">{geometry}</collision>'
        "</link></model></sdf>"
    )
