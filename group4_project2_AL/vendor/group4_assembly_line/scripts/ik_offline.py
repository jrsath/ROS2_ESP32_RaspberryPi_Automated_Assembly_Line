#!/usr/bin/env python3
import argparse
from pathlib import Path

import numpy as np
from ament_index_python.packages import get_package_share_directory
from ikpy.chain import Chain


def main():
    parser = argparse.ArgumentParser(description="Offline SO101 position IK check.")
    default_urdf = (
        Path(get_package_share_directory("group4_assembly_line"))
        / "urdf/so101_kinematics.urdf"
    )
    parser.add_argument("--urdf", type=Path, default=default_urdf)
    parser.add_argument(
        "--target", type=float, nargs=3, help="XYZ metres in the SO101 base frame"
    )
    parser.add_argument(
        "--output", type=Path, default=Path.home() / "ROS2/project2_data/ik_preview.png"
    )
    args = parser.parse_args()

    chain = Chain.from_urdf_file(
        str(args.urdf),
        active_links_mask=[False, True, True, True, True, True, False],
    )
    seed = np.array([0.0, 0.0, -0.45, 0.70, 0.45, 0.0, 0.0])
    target = (
        np.array(args.target) if args.target else chain.forward_kinematics(seed)[:3, 3]
    )
    angles = chain.inverse_kinematics(target_position=target, initial_position=seed)
    achieved = chain.forward_kinematics(angles)[:3, 3]
    error = float(np.linalg.norm(achieved - target))

    print("Target XYZ (m):", target)
    print("Reached XYZ (m):", achieved)
    print(f"Position residual: {error * 1000:.3f} mm")
    for link, angle, active in zip(chain.links, angles, chain.active_links_mask):
        if active:
            print(f"{link.name}: {np.degrees(angle):.2f} degrees")
    if error > 0.003:
        raise SystemExit("IK rejected: residual exceeds 3 mm")

    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    figure = plt.figure(figsize=(7, 6))
    axis = figure.add_subplot(111, projection="3d")
    chain.plot(angles, axis, target=target)
    axis.set_title("SO101 offline IK")
    axis.set_xlabel("X (m)")
    axis.set_ylabel("Y (m)")
    axis.set_zlabel("Z (m)")
    args.output.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(args.output, dpi=150, bbox_inches="tight")
    plt.close(figure)
    print("Plot:", args.output)


if __name__ == "__main__":
    main()
