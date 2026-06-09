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
        if len(transforms) < len(self.spheres):
            raise ValueError(
                f"Need at least {len(self.spheres)} transforms for {len(self.spheres)} spheres, "
                f"got {len(transforms)}"
            )
        if len(transforms) - 1 < len(self.capsules):
            raise ValueError(
                f"Need at least {len(self.capsules) + 1} transforms for {len(self.capsules)} capsules, "
                f"got {len(transforms)}"
            )

        for i in range(len(self.spheres)):
            self.spheres[i][0] = transforms[i][:3, 3]

        for j in range(len(self.capsules)):
            self.capsules[j][0] = transforms[j][:3, 3]
            self.capsules[j][1] = transforms[j + 1][:3, 3]