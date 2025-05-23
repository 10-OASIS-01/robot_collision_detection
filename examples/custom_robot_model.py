"""
Custom robot model example for the robot_collision_detection package.

This example demonstrates how to define a custom robot geometric model by
subclassing the Robot class and overriding the init_geometric_model method.
"""

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import sys
import os
import time

# Add the parent directory to the Python path for imports from the package
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from robot_collision_detection import (
    Robot,
    min_distance_between_robots,
    plot_robot
)

class CustomRobot(Robot):
    """
    Custom robot class with a user-defined geometric model.
    """
    def __init__(self, dh_params, base_transform=np.eye(4), name="CustomRobot", robot_type="standard"):
        """
        Initialize the custom robot.
        
        Args:
            dh_params: DH parameters
            base_transform: Base transformation matrix
            name: Robot name
            robot_type: Type of geometric model to use ("standard", "precise", "simplified")
        """
        self.robot_type = robot_type
        super().__init__(dh_params, base_transform, name)
    
    def init_geometric_model(self):
        """
        Initialize the robot's geometric model based on the robot type.
        
        Different models can be used for different purposes:
        - "standard": Default model with balanced precision and computation time
        - "precise": More detailed model with more geometric primitives
        - "simplified": Simplified model for faster computation
        """
        if self.robot_type == "precise":
            # A more detailed model with more spheres and capsules
            # More primitives = higher precision but slower computation
            self.spheres = [
                [[0, 0, 0], 400],          # S1: Base (larger sphere)
                [[0, 0, 550], 330],         # S1.5: Mid-base (additional sphere)
                [[0, 0, 1100], 370],        # S2: Joint 1
                [[300, 0, 1100], 250],      # S2.5: Mid-arm1 (additional sphere)
                [[600, 0, 1100], 350],      # S3: Joint 2
                [[1000, 0, 1100], 300],     # S3.5: Mid-arm2 (additional sphere)
                [[1400, 0, 1100], 320],     # S4: Joint 3
                [[2000, 0, 1100], 250]      # S5: End effector (additional sphere)
            ]
            
            self.capsules = [
                [[0, 0, 0], [0, 0, 1100], 320],          # C1: Base to Joint 1
                [[0, 0, 1100], [600, 0, 1100], 270],      # C2: Joint 1 to Joint 2
                [[600, 0, 1100], [1400, 0, 1100], 280],   # C3: Joint 2 to Joint 3
                [[1400, 0, 1100], [2000, 0, 1100], 220]   # C4: Joint 3 to End effector
            ]
            
        elif self.robot_type == "simplified":
            # A simplified model with fewer primitives
            # Fewer primitives = lower precision but faster computation
            self.spheres = [
                [[0, 0, 0], 450],             # S1: Base (larger to cover more area)
                [[600, 0, 1100], 400],        # S2: Middle joint (encompasses joint 1 & 2)
                [[2000, 0, 1100], 380]        # S3: End effector (larger to cover more area)
            ]
            
            self.capsules = [
                [[0, 0, 0], [600, 0, 1100], 380],         # C1: Base to Middle
                [[600, 0, 1100], [2000, 0, 1100], 330]    # C2: Middle to End
            ]
            
        else:  # "standard" - the default model
            # The standard model from the original implementation
            self.spheres = [
                [[0, 0, 0], 400],             # S1: Base
                [[0, 0, 1100], 370],          # S2: Joint 1
                [[600, 0, 1100], 350],        # S3: Joint 2
                [[600+1400, 0, 1100], 350]    # S4: Joint 3
            ]
            
            self.capsules = [
                [[0, 0, 0], [0, 0, 1100], 350],                # C1: Base to Joint 1
                [[0, 0, 1100], [600, 0, 1100], 300],           # C2: Joint 1 to Joint 2
                [[600, 0, 1100], [600+1400, 0, 1100], 300]     # C3: Joint 2 to Joint 3
            ]
    
    def update_geometric_model(self, transforms):
        """
        Update the position of the geometric model based on joint positions.
        
        Override for the custom robot to handle different number of spheres and capsules.
        
        Args:
            transforms: List of transformation matrices
        """
        if self.robot_type == "precise":
            # Update spheres for the precise model
            sphere_centers = [
                transforms[0][:3, 3],                          # S1: Base
                transforms[0][:3, 3] + np.array([0, 0, 550]),  # S1.5: Mid-base
                transforms[1][:3, 3],                          # S2: Joint 1
                transforms[1][:3, 3] + np.array([300, 0, 0]),  # S2.5: Mid-arm1
                transforms[2][:3, 3],                          # S3: Joint 2
                transforms[2][:3, 3] + np.array([400, 0, 0]),  # S3.5: Mid-arm2
                transforms[3][:3, 3],                          # S4: Joint 3
                transforms[3][:3, 3] + np.array([600, 0, 0])   # S5: End effector
            ]
            
            for i, center in enumerate(sphere_centers):
                self.spheres[i][0] = center
            
            # Update capsules
            self.capsules[0][0] = transforms[0][:3, 3]  # C1 start
            self.capsules[0][1] = transforms[1][:3, 3]  # C1 end
            
            self.capsules[1][0] = transforms[1][:3, 3]  # C2 start
            self.capsules[1][1] = transforms[2][:3, 3]  # C2 end
            
            self.capsules[2][0] = transforms[2][:3, 3]  # C3 start
            self.capsules[2][1] = transforms[3][:3, 3]  # C3 end
            
            self.capsules[3][0] = transforms[3][:3, 3]  # C4 start
            self.capsules[3][1] = transforms[3][:3, 3] + np.array([600, 0, 0])  # C4 end
        
        elif self.robot_type == "simplified":
            # Update spheres for the simplified model
            sphere_centers = [
                transforms[0][:3, 3],                          # S1: Base
                transforms[2][:3, 3],                          # S2: Middle joint
                transforms[3][:3, 3] + np.array([600, 0, 0])   # S3: End effector
            ]
            
            for i, center in enumerate(sphere_centers):
                self.spheres[i][0] = center
            
            # Update capsules
            self.capsules[0][0] = transforms[0][:3, 3]  # C1 start
            self.capsules[0][1] = transforms[2][:3, 3]  # C1 end
            
            self.capsules[1][0] = transforms[2][:3, 3]  # C2 start
            self.capsules[1][1] = transforms[3][:3, 3] + np.array([600, 0, 0])  # C2 end
        
        else:  # "standard"
            # Use the standard update method for the default model
            super().update_geometric_model(transforms)

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
    [0, 1, 0, 3000],
    [0, 0, 1, 0],
    [0, 0, 0, 1]
])

# Create the robots with different model types
robot1 = CustomRobot(dh_params, T_base_r1, "Robot1", "standard")
robot2 = CustomRobot(dh_params, T_base_r2, "Robot2", "precise")
robot3 = CustomRobot(dh_params, T_base_r2, "Robot3", "simplified")

# Set joint angles (in radians)
joint_angles = np.radians([0, -30, 45])

# Create a figure for visualization
fig = plt.figure(figsize=(18, 6))
fig.suptitle('Comparison of Different Robot Geometric Models', fontsize=16)

# Plot standard model
ax1 = fig.add_subplot(131, projection='3d')
plot_robot(ax1, robot1, joint_angles, 'b')
ax1.set_title('Standard Model')
ax1.set_xlabel('X (mm)')
ax1.set_ylabel('Y (mm)')
ax1.set_zlabel('Z (mm)')
ax1.set_xlim(-500, 2500)
ax1.set_ylim(0, 2000)
ax1.set_zlim(0, 2000)

# Plot precise model
ax2 = fig.add_subplot(132, projection='3d')
plot_robot(ax2, robot2, joint_angles, 'r')
ax2.set_title('Precise Model')
ax2.set_xlabel('X (mm)')
ax2.set_ylabel('Y (mm)')
ax2.set_zlabel('Z (mm)')
ax2.set_xlim(-500, 2500)
ax2.set_ylim(2000, 4000)
ax2.set_zlim(0, 2000)

# Plot simplified model
ax3 = fig.add_subplot(133, projection='3d')
plot_robot(ax3, robot3, joint_angles, 'g')
ax3.set_title('Simplified Model')
ax3.set_xlabel('X (mm)')
ax3.set_ylabel('Y (mm)')
ax3.set_zlabel('Z (mm)')
ax3.set_xlim(-500, 2500)
ax3.set_ylim(2000, 4000)
ax3.set_zlim(0, 2000)

plt.tight_layout()
plt.show()

# =====================================================================
# Compare model characteristics and collision detection performance
# =====================================================================
print("Model Characteristics:")
print(f"Standard model: {len(robot1.spheres)} spheres, {len(robot1.capsules)} capsules")
print(f"Precise model:  {len(robot2.spheres)} spheres, {len(robot2.capsules)} capsules")
print(f"Simplified model: {len(robot3.spheres)} spheres, {len(robot3.capsules)} capsules")
print()

# Create another robot for collision testing
test_robot = Robot(dh_params, np.eye(4), "TestRobot")
test_angles = np.radians([0, 0, 0])

# Time the collision detection for each model

# Standard model
transforms1 = robot1.forward_kinematics(joint_angles)
robot1.update_geometric_model(transforms1)
test_transforms = test_robot.forward_kinematics(test_angles)
test_robot.update_geometric_model(test_transforms)

start_time = time.time()
num_iterations = 1000
for _ in range(num_iterations):
    dist1, type1, elements1 = min_distance_between_robots(robot1, test_robot)
standard_time = (time.time() - start_time) / num_iterations * 1000  # ms

# Precise model
transforms2 = robot2.forward_kinematics(joint_angles)
robot2.update_geometric_model(transforms2)

start_time = time.time()
for _ in range(num_iterations):
    dist2, type2, elements2 = min_distance_between_robots(robot2, test_robot)
precise_time = (time.time() - start_time) / num_iterations * 1000  # ms

# Simplified model
transforms3 = robot3.forward_kinematics(joint_angles)
robot3.update_geometric_model(transforms3)

start_time = time.time()
for _ in range(num_iterations):
    dist3, type3, elements3 = min_distance_between_robots(robot3, test_robot)
simplified_time = (time.time() - start_time) / num_iterations * 1000  # ms

print(f"Performance Comparison ({num_iterations} iterations):")
print(f"Standard model: {standard_time:.3f} ms per check, distance: {dist1:.2f} mm")
print(f"Precise model:  {precise_time:.3f} ms per check, distance: {dist2:.2f} mm")
print(f"Simplified model: {simplified_time:.3f} ms per check, distance: {dist3:.2f} mm")
print()

# Check if distances are similar
print("Distance Error Analysis:")
print(f"Standard vs Precise: {abs(dist1 - dist2):.2f} mm difference")
print(f"Standard vs Simplified: {abs(dist1 - dist3):.2f} mm difference")
print(f"Precise vs Simplified: {abs(dist2 - dist3):.2f} mm difference")

# Summary
print("\nSummary:")
print("The precise model offers better accuracy at the cost of computation time.")
print("The simplified model is faster but may be less accurate.")
print("The standard model provides a balance between accuracy and speed.")
print("\nChoose the appropriate model based on your specific requirements:")
print("- Use the precise model for offline planning or when accuracy is critical")
print("- Use the simplified model for real-time applications requiring fast computation")
print("- Use the standard model for general-purpose collision detection")