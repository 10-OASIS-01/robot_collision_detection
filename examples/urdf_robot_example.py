"""
URDF robot model example for the robot_collision_detection package.

This example demonstrates how to load a robot from a URDF file,
visualize it, and perform collision detection between two URDF-loaded robots.

Run after installing the package: pip install -e .
Usage: python examples/urdf_robot_example.py /path/to/robot.urdf
"""

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import argparse

from robot_collision_detection import URDFRobot, min_distance_between_robots
from robot_collision_detection.visualization.plotting import plot_robot


def main():
    parser = argparse.ArgumentParser(description="Load and visualize a URDF robot model")
    parser.add_argument("urdf_path", help="Path to the URDF file")
    parser.add_argument("--capsule-radius", type=float, default=0.05,
                        help="Radius for collision capsules (meters, default: 0.05)")
    args = parser.parse_args()

    robot = URDFRobot(args.urdf_path, capsule_radius=args.capsule_radius)
    print(f"Robot: {robot.name}")
    print(f"Joints: {robot.num_joints}")
    for i, (name, limits) in enumerate(zip(robot.joint_names, robot.joint_limits)):
        lo, hi = limits
        print(f"  [{i}] {name}: [{np.degrees(lo):.1f}, {np.degrees(hi):.1f}] deg")

    # Zero configuration
    zero_angles = np.zeros(robot.num_joints)
    transforms = robot.forward_kinematics(zero_angles)
    robot.update_geometric_model(transforms)

    print(f"\nGeometric model: {len(robot.spheres)} spheres, {len(robot.capsules)} capsules")
    print("\nJoint positions at zero configuration:")
    for i, T in enumerate(transforms):
        pos = T[:3, 3]
        print(f"  Frame {i}: [{pos[0]:.4f}, {pos[1]:.4f}, {pos[2]:.4f}] m")

    # Visualization
    fig = plt.figure(figsize=(16, 6))
    fig.suptitle(f"URDF Robot: {robot.name}", fontsize=14)

    # Plot 1: Zero configuration
    ax1 = fig.add_subplot(131, projection='3d')
    plot_robot(ax1, robot, zero_angles, 'b')
    ax1.set_title("Zero Configuration")
    _set_axes(ax1, transforms)

    # Plot 2: A non-zero configuration
    mid_angles = np.zeros(robot.num_joints)
    for i, (lo, hi) in enumerate(robot.joint_limits):
        if lo is not None and hi is not None:
            mid_angles[i] = (lo + hi) / 2
    ax2 = fig.add_subplot(132, projection='3d')
    plot_robot(ax2, robot, mid_angles, 'r')
    ax2.set_title("Mid-Range Configuration")
    transforms_mid = robot.forward_kinematics(mid_angles)
    _set_axes(ax2, transforms_mid)

    # Plot 3: Dual robot collision check
    T_base_2 = np.eye(4)
    T_base_2[1, 3] = 0.8
    robot2 = URDFRobot(args.urdf_path, base_transform=T_base_2,
                       name=f"{robot.name}_2", capsule_radius=args.capsule_radius)

    angles1 = np.zeros(robot.num_joints)
    angles2 = mid_angles.copy()

    transforms1 = robot.forward_kinematics(angles1)
    transforms2 = robot2.forward_kinematics(angles2)
    robot.update_geometric_model(transforms1)
    robot2.update_geometric_model(transforms2)

    dist, coll_type, coll_elems = min_distance_between_robots(robot, robot2)

    ax3 = fig.add_subplot(133, projection='3d')
    plot_robot(ax3, robot, angles1, 'b')
    plot_robot(ax3, robot2, angles2, 'r')
    status = "COLLISION!" if dist < 0 else f"Safe (d={dist:.4f}m)"
    ax3.set_title(f"Dual Robot: {status}")
    all_transforms = transforms1 + transforms2
    _set_axes(ax3, all_transforms)

    print(f"\nDual robot distance: {dist:.4f} m")
    print(f"Closest pair: {coll_type} — {coll_elems}")

    from matplotlib.lines import Line2D
    ax3.legend(
        [Line2D([0], [0], color='b', lw=4), Line2D([0], [0], color='r', lw=4)],
        [robot.name, robot2.name],
    )

    plt.tight_layout()
    plt.show()


def _set_axes(ax, transforms):
    positions = np.array([T[:3, 3] for T in transforms])
    center = positions.mean(axis=0)
    span = max(positions.max(axis=0) - positions.min(axis=0)) * 0.7
    if span < 0.1:
        span = 0.5
    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.set_zlabel("Z (m)")
    ax.set_xlim(center[0] - span, center[0] + span)
    ax.set_ylim(center[1] - span, center[1] + span)
    ax.set_zlim(center[2] - span, center[2] + span)


if __name__ == "__main__":
    main()
