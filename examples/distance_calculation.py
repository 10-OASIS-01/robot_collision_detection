"""
Distance calculation example for the robot_collision_detection package.

This example demonstrates how to calculate distances between different geometric
primitives (spheres and capsules) using the provided functions.
"""

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import sys
import os

# Add the parent directory to the Python path for imports from the package
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from robot_collision_detection.distance.primitives import (
    dist_sphere_sphere,
    dist_sphere_capsule,
    dist_capsule_capsule
)
from robot_collision_detection.visualization.plotting import (
    plot_sphere,
    plot_capsule
)

# Create a figure for plotting
fig = plt.figure(figsize=(18, 6))
fig.suptitle('Distance Calculation Between Geometric Primitives', fontsize=16)

# ==========================================================================
# Example 1: Distance between two spheres
# ==========================================================================
ax1 = fig.add_subplot(131, projection='3d')
ax1.set_title('Sphere-Sphere Distance')

# Define two spheres: [center, radius]
sphere1 = [np.array([0, 0, 0]), 1.0]
sphere2 = [np.array([3, 0, 0]), 1.5]

# Calculate distance between spheres
distance = dist_sphere_sphere(sphere1, sphere2)

# Plot spheres
plot_sphere(ax1, sphere1[0], sphere1[1], 'b')
plot_sphere(ax1, sphere2[0], sphere2[1], 'r')

# Plot a line between sphere centers
ax1.plot([sphere1[0][0], sphere2[0][0]], 
         [sphere1[0][1], sphere2[0][1]], 
         [sphere1[0][2], sphere2[0][2]], 'k--')

# Add distance annotation
midpoint = (sphere1[0] + sphere2[0]) / 2
ax1.text(midpoint[0], midpoint[1], midpoint[2] + 0.5, 
         f"Distance = {distance:.2f}", 
         ha='center', fontsize=10)

# Set axis limits and labels
ax1.set_xlabel('X')
ax1.set_ylabel('Y')
ax1.set_zlabel('Z')
ax1.set_xlim(-2, 5)
ax1.set_ylim(-2, 2)
ax1.set_zlim(-2, 2)

# Print distance information
print("Example 1: Sphere-Sphere Distance")
print(f"Sphere 1: Center = {sphere1[0]}, Radius = {sphere1[1]}")
print(f"Sphere 2: Center = {sphere2[0]}, Radius = {sphere2[1]}")
print(f"Distance between spheres = {distance:.4f}")
print()

# ==========================================================================
# Example 2: Distance between sphere and capsule
# ==========================================================================
ax2 = fig.add_subplot(132, projection='3d')
ax2.set_title('Sphere-Capsule Distance')

# Define a sphere: [center, radius]
sphere = [np.array([0, 2, 1]), 0.8]

# Define a capsule: [start_point, end_point, radius]
capsule = [np.array([0, -1, 0]), np.array([0, 1, 0]), 0.5]

# Calculate distance
distance = dist_sphere_capsule(sphere, capsule)

# Plot sphere and capsule
plot_sphere(ax2, sphere[0], sphere[1], 'b')
plot_capsule(ax2, capsule[0], capsule[1], capsule[2], 'r')

# Calculate closest point on capsule axis to sphere center
capsule_vec = capsule[1] - capsule[0]
capsule_len = np.linalg.norm(capsule_vec)
capsule_dir = capsule_vec / capsule_len

# Project sphere center onto capsule axis
v = sphere[0] - capsule[0]
proj_len = np.dot(v, capsule_dir)
proj_len = max(0, min(capsule_len, proj_len))  # Clamp to capsule length
closest_point = capsule[0] + proj_len * capsule_dir

# Plot line from sphere center to closest point on capsule axis
ax2.plot([sphere[0][0], closest_point[0]], 
         [sphere[0][1], closest_point[1]], 
         [sphere[0][2], closest_point[2]], 'k--')

# Add distance annotation
midpoint = (sphere[0] + closest_point) / 2
ax2.text(midpoint[0], midpoint[1], midpoint[2] + 0.5, 
         f"Distance = {distance:.2f}", 
         ha='center', fontsize=10)

# Set axis limits and labels
ax2.set_xlabel('X')
ax2.set_ylabel('Y')
ax2.set_zlabel('Z')
ax2.set_xlim(-2, 2)
ax2.set_ylim(-2, 4)
ax2.set_zlim(-2, 3)

# Print distance information
print("Example 2: Sphere-Capsule Distance")
print(f"Sphere: Center = {sphere[0]}, Radius = {sphere[1]}")
print(f"Capsule: Start = {capsule[0]}, End = {capsule[1]}, Radius = {capsule[2]}")
print(f"Distance between sphere and capsule = {distance:.4f}")
print()

# ==========================================================================
# Example 3: Distance between two capsules
# ==========================================================================
ax3 = fig.add_subplot(133, projection='3d')
ax3.set_title('Capsule-Capsule Distance')

# Define two capsules: [start_point, end_point, radius]
capsule1 = [np.array([-1, -1, 0]), np.array([1, -1, 0]), 0.4]
capsule2 = [np.array([-1, 1, 1]), np.array([1, 1, 1]), 0.3]

# Calculate distance
distance = dist_capsule_capsule(capsule1, capsule2)

# Plot capsules
plot_capsule(ax3, capsule1[0], capsule1[1], capsule1[2], 'b')
plot_capsule(ax3, capsule2[0], capsule2[1], capsule2[2], 'r')

# Compute closest points between the two capsule centerlines
# (This is a simplified approach - the actual algorithm is more complex)
# For detailed calculation, check the dist_segment_segment function
midpoint1 = (capsule1[0] + capsule1[1]) / 2
midpoint2 = (capsule2[0] + capsule2[1]) / 2

# Plot a line between midpoints (as a simplified visualization)
ax3.plot([midpoint1[0], midpoint2[0]], 
         [midpoint1[1], midpoint2[1]], 
         [midpoint1[2], midpoint2[2]], 'k--')

# Add distance annotation
annotation_point = (midpoint1 + midpoint2) / 2
ax3.text(annotation_point[0], annotation_point[1], annotation_point[2] + 0.5, 
         f"Distance = {distance:.2f}", 
         ha='center', fontsize=10)

# Set axis limits and labels
ax3.set_xlabel('X')
ax3.set_ylabel('Y')
ax3.set_zlabel('Z')
ax3.set_xlim(-2, 2)
ax3.set_ylim(-2, 2)
ax3.set_zlim(-1, 3)

# Print distance information
print("Example 3: Capsule-Capsule Distance")
print(f"Capsule 1: Start = {capsule1[0]}, End = {capsule1[1]}, Radius = {capsule1[2]}")
print(f"Capsule 2: Start = {capsule2[0]}, End = {capsule2[1]}, Radius = {capsule2[2]}")
print(f"Distance between capsules = {distance:.4f}")

# ==========================================================================
# Show the plots
# ==========================================================================
plt.tight_layout()
plt.show()

# ==========================================================================
# Additional Examples: Collision Cases
# ==========================================================================
print("\n=== Collision Examples ===")

# Colliding spheres
sphere1 = [np.array([0, 0, 0]), 1.0]
sphere2 = [np.array([1.5, 0, 0]), 1.0]
distance = dist_sphere_sphere(sphere1, sphere2)
print(f"Colliding spheres distance: {distance:.4f} (negative means collision)")

# Colliding sphere and capsule
sphere = [np.array([0, 0, 0]), 1.0]
capsule = [np.array([-1, 0, 0]), np.array([1, 0, 0]), 0.5]
distance = dist_sphere_capsule(sphere, capsule)
print(f"Colliding sphere-capsule distance: {distance:.4f} (negative means collision)")

# Colliding capsules
capsule1 = [np.array([0, -0.5, 0]), np.array([0, 0.5, 0]), 0.4]
capsule2 = [np.array([-0.5, 0, 0]), np.array([0.5, 0, 0]), 0.3]
distance = dist_capsule_capsule(capsule1, capsule2)
print(f"Colliding capsules distance: {distance:.4f} (negative means collision)")