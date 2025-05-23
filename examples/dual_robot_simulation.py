"""
Dual robot collision simulation example for the robot_collision_detection package.

This example simulates two robots moving along predefined paths and
detects if and when they collide with each other.
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

# Set DH parameters [a, alpha, d]
dh_params = [
    [0, 0, 1100],         # Joint 1
    [600, np.pi/2, 0],    # Joint 2
    [1400, 0, 0]          # Joint 3
]

# Set base transformation matrices for two robots
T_base_r1 = np.array([
    [1, 0, 0, 0],
    [0, 1, 0, 1000],
    [0, 0, 1, 0],
    [0, 0, 0, 1]
])

T_base_r2 = np.array([
    [1, 0, 0, 0],
    [0, 1, 0, 2000],
    [0, 0, 1, 0],
    [0, 0, 0, 1]
])

# Create two robots
robot1 = Robot(dh_params, T_base_r1, "R1")
robot2 = Robot(dh_params, T_base_r2, "R2")

# Set initial and final joint angles (degrees to radians)
joint_angles_start_r1 = np.radians([0, -45, -20])
joint_angles_end_r1 = np.radians([0, 45, 20])

joint_angles_start_r2 = np.radians([0, 45, 20])
joint_angles_end_r2 = np.radians([0, -45, -20])

# Set joint speeds (degrees/second)
joint_speeds = np.radians([0, 45, 20])  # Angular velocity

# Calculate total simulation time (seconds)
joint_speeds_safe = joint_speeds.copy()
joint_speeds_safe[joint_speeds_safe == 0] = 1e-6  # Replace zero with a very small number
max_time_joints = np.max(np.abs(joint_angles_end_r1 - joint_angles_start_r1) / joint_speeds_safe)
total_time = max_time_joints

# Create time points (100 samples)
num_samples = 100
time_points = np.linspace(0, total_time, num_samples)

# Calculate joint angles for each time point
joint_angles_r1 = []
joint_angles_r2 = []

for t in time_points:
    # Linear interpolation to calculate joint angles
    ratio = t / total_time
    angles_r1 = joint_angles_start_r1 + ratio * (joint_angles_end_r1 - joint_angles_start_r1)
    angles_r2 = joint_angles_start_r2 + ratio * (joint_angles_end_r2 - joint_angles_start_r2)
    
    joint_angles_r1.append(angles_r1)
    joint_angles_r2.append(angles_r2)

# Calculate minimum distance for each time point
distances = []
collision_types = []
collision_elements_list = []
collision_index = -1

for i in range(num_samples):
    # Update robots with current joint angles
    transforms_r1 = robot1.forward_kinematics(joint_angles_r1[i])
    transforms_r2 = robot2.forward_kinematics(joint_angles_r2[i])
    
    robot1.update_geometric_model(transforms_r1)
    robot2.update_geometric_model(transforms_r2)
    
    # Calculate minimum distance
    dist, coll_type, coll_elements = min_distance_between_robots(robot1, robot2)
    
    distances.append(dist)
    collision_types.append(coll_type)
    collision_elements_list.append(coll_elements)
    
    # Print information at key time points
    if i % 10 == 0 or (1.0 <= time_points[i] <= 1.25):  # Print every 10 samples, and every point within 1-1.25 seconds
        print(f"Time: {time_points[i]:.2f}s, Distance: {dist:.2f}mm")
    
    # Detect collision
    if dist < 0 and collision_index == -1:
        collision_index = i
        print(f"Collision detected! Time: {time_points[i]:.2f}s, Distance: {dist:.2f}mm")

# Create figure window
plt.figure(figsize=(14, 6))

# Plot distance-time curve
plt.subplot(121)
plt.plot(time_points, distances, 'b-')
plt.axhline(y=0, color='r', linestyle='--')
plt.grid(True)
plt.xlabel('Time (seconds)')
plt.ylabel('Minimum Distance (mm)')
plt.title('Minimum Distance Between Two Robots Over Time')

# Mark collision point
if collision_index != -1:
    collision_time = time_points[collision_index]
    collision_dist = distances[collision_index]
    plt.plot(collision_time, collision_dist, 'ro', markersize=8)
    plt.annotate(f'Collision Point ({collision_time:.2f}s, {collision_dist:.2f}mm)',
                 xy=(collision_time, collision_dist),
                 xytext=(collision_time+0.1, collision_dist-50),
                 arrowprops=dict(facecolor='black', shrink=0.05))
    
    # Add collision information
    collision_info = f"Collision Type: {collision_types[collision_index]}\n"
    collision_info += f"Collision Elements: {' and '.join(collision_elements_list[collision_index])}"
    plt.annotate(collision_info, xy=(0.05, 0.05), xycoords='axes fraction', 
                 bbox=dict(boxstyle="round,pad=0.3", fc="yellow", alpha=0.3))

# Create 3D visualization
ax = plt.subplot(122, projection='3d')

# If there is a collision, display the configuration at the time of collision
if collision_index != -1:
    plot_robot(ax, robot1, joint_angles_r1[collision_index], 'b')
    plot_robot(ax, robot2, joint_angles_r2[collision_index], 'r')
    plt.title(f'Robot Configuration at Collision (t={time_points[collision_index]:.2f}s)')
else:
    # Otherwise, display the initial configuration
    plot_robot(ax, robot1, joint_angles_r1[0], 'b')
    plot_robot(ax, robot2, joint_angles_r2[0], 'r')
    plt.title('Initial Robot Configuration')

if collision_index != -1:
    print(f"Collision occurred! Time: {time_points[collision_index]:.2f}s")
    print(f"Collision Distance: {distances[collision_index]:.2f}mm")
    print(f"Collision Type: {collision_types[collision_index]}")
    print(f"Collision Elements: {' and '.join(collision_elements_list[collision_index])}")
else:
    print("No collision detected")
    print(f"Minimum Distance: {min(distances):.2f}mm")

# Set 3D plot view and labels
ax.set_xlabel('X (mm)')
ax.set_ylabel('Y (mm)')
ax.set_zlabel('Z (mm)')
ax.set_xlim(-1000, 3000)
ax.set_ylim(0, 5000)
ax.set_zlim(0, 4000)

# Add legend
from matplotlib.lines import Line2D
custom_lines = [
    Line2D([0], [0], color='b', lw=4),
    Line2D([0], [0], color='r', lw=4)
]
ax.legend(custom_lines, ['Robot R1', 'Robot R2'])

plt.tight_layout()
plt.show()