#!/usr/bin/env python3
# Offline only: calculate/plot a pose; never open serial or publish ROS commands.
import argparse
from pathlib import Path
import sys
import numpy as np
from ikpy.chain import Chain


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--urdf",
        type=Path,
        default=Path.home()
        / "ROS2/project2_assets/reference/ik/MT558_SO101_IK/SO101/so101_new_calib.urdf",
    )
    parser.add_argument(
        "--target", type=float, nargs=3, help="XYZ metres in model base frame"
    )
    parser.add_argument(
        "--output", type=Path, default=Path.home() / "ROS2/project2_data/ik_preview.png"
    )
    args = parser.parse_args()
    chain = Chain.from_urdf_file(
        str(args.urdf), active_links_mask=[False, True, True, True, True, True, False]
    )
    # Generate a reachable target for the default smoke test.
    seed = np.array([0.0, 0.2, -0.2, 0.3, 0.15, -0.1, 0.0])
    target = (
        np.array(args.target) if args.target else chain.forward_kinematics(seed)[:3, 3]
    )
    if not np.all(np.isfinite(target)):
        raise SystemExit("Target must be finite")
    angles = chain.inverse_kinematics(
        target_position=target, initial_position=np.zeros(7)
    )
    achieved = chain.forward_kinematics(angles)[:3, 3]
    error = np.linalg.norm(achieved - target)
    limits_ok = all(
        link.bounds[0] - 1e-6 <= angle <= link.bounds[1] + 1e-6
        for link, angle, active in zip(chain.links, angles, chain.active_links_mask)
        if active
    )
    print("OFFLINE MODEL ONLY; no physical robot commands.")
    print("Target XYZ (m):", target)
    print("Reached XYZ (m):", achieved)
    print(f"Position residual: {error * 1000:.4f} mm")
    for link, angle, active in zip(chain.links, angles, chain.active_links_mask):
        if active:
            print(f"{link.name}: {np.degrees(angle):.3f} degrees")
    if error > 0.001 or not limits_ok:
        raise SystemExit("IK rejected: residual exceeds 1 mm or joint limit exceeded")
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig = plt.figure(figsize=(7, 6))
    axis = fig.add_subplot(111, projection="3d")
    chain.plot(angles, axis, target=target)
    axis.set_title("SO101 offline IK - position only")
    axis.set_xlabel("X (m)")
    axis.set_ylabel("Y (m)")
    axis.set_zlabel("Z (m)")
    args.output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(args.output, dpi=150, bbox_inches="tight")
    plt.close(fig)
    print("Plot:", args.output)
    print("No collision, calibration or physical reachability guarantee.")


if __name__ == "__main__":
    main()
