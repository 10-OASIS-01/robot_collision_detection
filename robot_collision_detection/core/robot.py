import numpy as np
from .kinematics import dh_transform

class Robot:
    """
    Robot class representing a robotic arm with geometric primitives for collision detection.
    """
    def __init__(self, dh_params, base_transform=np.eye(4), name="Robot"):
        """
        Initialize the robot.
        
        Args:
            dh_params: List of DH parameters, each element is [a, alpha, d]
            base_transform: Transformation of the base coordinate system relative to the reference system
            name: Robot name
        """
        self.dh_params = dh_params
        self.base_transform = base_transform
        self.name = name
        self.init_geometric_model()
    
    def init_geometric_model(self):
        """
        Initialize the robot's geometric model (spheres and capsules).
        
        Default model is based on KUKA KR1000_TITAN dimensions.
        Override this method to customize the geometric model.
        """
        # Sphere parameters: [center coordinates, radius]
        self.spheres = [
            [[0, 0, 0], 400],        # S1: Base
            [[0, 0, 1100], 370],     # S2: Joint 1
            [[600, 0, 1100], 350],   # S3: Joint 2
            [[600+1400, 0, 1100], 350]  # S4: Joint 3
        ]
        
        # Capsule parameters: [start point, end point, radius]
        self.capsules = [
            [[0, 0, 0], [0, 0, 1100], 350],           # C1
            [[0, 0, 1100], [600, 0, 1100], 300],      # C2
            [[600, 0, 1100], [600+1400, 0, 1100], 300]  # C3
        ]
    
    def forward_kinematics(self, joint_angles):
        """
        Calculate forward kinematics, returning the position of each joint in the world coordinate system.
        
        Args:
            joint_angles: List of joint angles in radians
            
        Returns:
            List of 4x4 transformation matrices
        """
        transforms = [self.base_transform]
        
        for i, (a, alpha, d) in enumerate(self.dh_params):
            T = dh_transform(a, alpha, d, joint_angles[i])
            transforms.append(transforms[-1] @ T)
        
        return transforms
    
    def update_geometric_model(self, transforms):
        """
        Update the position of the geometric model based on joint positions.
        
        Args:
            transforms: List of 4x4 transformation matrices from forward_kinematics
        """
        # Update sphere positions
        sphere_centers = [
            transforms[0][:3, 3],  # S1 at base
            transforms[1][:3, 3],  # S2 at joint 1
            transforms[2][:3, 3],  # S3 at joint 2
            transforms[3][:3, 3]   # S4 at joint 3
        ]
        
        for i, center in enumerate(sphere_centers):
            self.spheres[i][0] = center
        
        # Update capsule positions
        self.capsules[0][0] = transforms[0][:3, 3]  # C1 start
        self.capsules[0][1] = transforms[1][:3, 3]  # C1 end
        
        self.capsules[1][0] = transforms[1][:3, 3]  # C2 start
        self.capsules[1][1] = transforms[2][:3, 3]  # C2 end
        
        self.capsules[2][0] = transforms[2][:3, 3]  # C3 start
        self.capsules[2][1] = transforms[3][:3, 3]  # C3 end