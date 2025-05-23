"""
Base robot model example for the robot_collision_detection package.

This example demonstrates the basic usage of the Robot class, showing how to:
- Create robots with different base positions
- Set joint angles and calculate forward kinematics
- Update the geometric models based on joint configurations
- Calculate the minimum distance between two robots
- Visualize the robots in 3D space
"""

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import sys
import os

# Add the parent directory to the Python path for imports from the package
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from robot_collision_detection import (
    Robot, 
    min_distance_between_robots, 
    plot_robot
)

# Define DH parameters [a, alpha, d]
dh_params = [
    [0, 0, 1100],        # Joint 1
    [600, np.pi/2, 0],   # Joint 2
    [1400, 0, 0]         # Joint 3
]

# Create two robots with different base positions
T_base_r1 = np.eye(4)  # Identity matrix for the first robot
T_base_r2 = np.array([
    [1, 0, 0, 0],
    [0, 1, 0, 2000],  # 2000mm offset along Y-axis
    [0, 0, 1, 0],
    [0, 0, 0, 1]
])

robot1 = Robot(dh_params, T_base_r1, name="Robot1")
robot2 = Robot(dh_params, T_base_r2, name="Robot2")

# Set joint angles (in radians)
joint_angles_r1 = np.radians([0, -45, -20])
joint_angles_r2 = np.radians([0, 45, 20])

# Calculate forward kinematics
transforms_r1 = robot1.forward_kinematics(joint_angles_r1)
transforms_r2 = robot2.forward_kinematics(joint_angles_r2)

# Update geometric models
robot1.update_geometric_model(transforms_r1)
robot2.update_geometric_model(transforms_r2)

# Calculate minimum distance between robots
min_dist, collision_type, collision_elements = min_distance_between_robots(robot1, robot2)
print(f"Minimum distance: {min_dist:.2f}mm")

if min_dist < 0:
    print(f"Collision detected! Type: {collision_type}")
    print(f"Collision elements: {' and '.join(collision_elements)}")
else:
    print("No collision detected")
    if collision_type:
        print(f"Closest elements: {' and '.join(collision_elements)} ({collision_type})")

# Create a figure for visualization
fig = plt.figure(figsize=(12, 10))
ax = fig.add_subplot(111, projection='3d')
plt.title('Basic Robot Model Visualization')

# Plot the robots
plot_robot(ax, robot1, joint_angles_r1, 'b')
plot_robot(ax, robot2, joint_angles_r2, 'r')

# Add a legend
from matplotlib.lines import Line2D
custom_lines = [
    Line2D([0], [0], color='b', lw=4),
    Line2D([0], [0], color='r', lw=4)
]
ax.legend(custom_lines, ['Robot1', 'Robot2'])

# Set axis properties
ax.set_xlabel('X (mm)')
ax.set_ylabel('Y (mm)')
ax.set_zlabel('Z (mm)')
ax.set_xlim(-500, 2500)
ax.set_ylim(-500, 2500)
ax.set_zlim(0, 2500)

# Add grid for better visualization
ax.grid(True)

# Print geometric model details
print("\nGeometric Model Details:")
print(f"Robot1: {len(robot1.spheres)} spheres, {len(robot1.capsules)} capsules")
print(f"Robot2: {len(robot2.spheres)} spheres, {len(robot2.capsules)} capsules")
print("\nSphere positions (Robot1):")
for i, (center, radius) in enumerate(robot1.spheres):
    print(f"  S{i+1}: Center = {center}, Radius = {radius}mm")

plt.tight_layout()
plt.show()