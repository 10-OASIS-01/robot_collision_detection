from .core.robot import Robot
from .core.kinematics import dh_transform
from .distance.primitives import (
    dist_point_segment, 
    dist_sphere_sphere, 
    dist_sphere_capsule, 
    dist_segment_segment, 
    dist_capsule_capsule
)
from .distance.collision import min_distance_between_robots
from .visualization.plotting import plot_sphere, plot_capsule, plot_robot

__version__ = '0.1.0'