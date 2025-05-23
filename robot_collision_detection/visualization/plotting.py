import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

def plot_sphere(ax, center, radius, color='b', alpha=0.2):
    """
    Plot a sphere in a 3D coordinate system.
    
    Args:
        ax: Matplotlib 3D axis
        center: Sphere center coordinates
        radius: Sphere radius
        color: Sphere color
        alpha: Transparency (0 to 1)
    """
    u, v = np.mgrid[0:2*np.pi:20j, 0:np.pi:10j]
    x = center[0] + radius * np.cos(u) * np.sin(v)
    y = center[1] + radius * np.sin(u) * np.sin(v)
    z = center[2] + radius * np.cos(v)
    ax.plot_surface(x, y, z, color=color, alpha=alpha)

def plot_capsule(ax, start, end, radius, color='b', alpha=0.2):
    """
    Plot a capsule in a 3D coordinate system.
    
    Args:
        ax: Matplotlib 3D axis
        start: Start point of the capsule centerline
        end: End point of the capsule centerline
        radius: Capsule radius
        color: Capsule color
        alpha: Transparency (0 to 1)
    """
    start = np.array(start)
    end = np.array(end)
    
    # Calculate capsule direction vector
    v = end - start
    length = np.linalg.norm(v)
    if length < 1e-10:
        return  # Too short to plot
    
    v = v / length
    
    # Create a vector not parallel to v
    not_v = np.array([1, 0, 0])
    if abs(np.dot(v, not_v)) > 0.9:
        not_v = np.array([0, 1, 0])
    
    # Calculate two orthogonal vectors to form an orthogonal basis with v
    n1 = np.cross(v, not_v)
    n1 = n1 / np.linalg.norm(n1)
    n2 = np.cross(v, n1)
    
    # Create cylinder
    t = np.linspace(0, length, 20)
    theta = np.linspace(0, 2 * np.pi, 20)
    t_grid, theta_grid = np.meshgrid(t, theta)
    
    x_cyl = start[0] + v[0] * t_grid + radius * np.cos(theta_grid) * n1[0] + radius * np.sin(theta_grid) * n2[0]
    y_cyl = start[1] + v[1] * t_grid + radius * np.cos(theta_grid) * n1[1] + radius * np.sin(theta_grid) * n2[1]
    z_cyl = start[2] + v[2] * t_grid + radius * np.cos(theta_grid) * n1[2] + radius * np.sin(theta_grid) * n2[2]
    
    ax.plot_surface(x_cyl, y_cyl, z_cyl, color=color, alpha=alpha)
    
    # Create two hemispheres
    u, v_sphere = np.mgrid[0:2*np.pi:20j, 0:np.pi:10j]
    
    # Start hemisphere
    x_sphere1 = start[0] + radius * np.cos(u) * np.sin(v_sphere)
    y_sphere1 = start[1] + radius * np.sin(u) * np.sin(v_sphere)
    z_sphere1 = start[2] + radius * np.cos(v_sphere)
    
    # End hemisphere
    x_sphere2 = end[0] + radius * np.cos(u) * np.sin(v_sphere)
    y_sphere2 = end[1] + radius * np.sin(u) * np.sin(v_sphere)
    z_sphere2 = end[2] + radius * np.cos(v_sphere)
    
    # Create mask and set unwanted points to NaN
    if length > 0:
        direction = v / np.linalg.norm(v)
        
        # Copy arrays to avoid modifying original data
        x_sphere1_masked = x_sphere1.copy()
        y_sphere1_masked = y_sphere1.copy()
        z_sphere1_masked = z_sphere1.copy()
        
        x_sphere2_masked = x_sphere2.copy()
        y_sphere2_masked = y_sphere2.copy()
        z_sphere2_masked = z_sphere2.copy()
        
        # Apply mask to each point
        for i in range(x_sphere1.shape[0]):
            for j in range(x_sphere1.shape[1]):
                p1 = np.array([x_sphere1[i, j], y_sphere1[i, j], z_sphere1[i, j]]) - start
                if np.dot(p1, direction) >= 0:  # If the point is facing inside the cylinder
                    x_sphere1_masked[i, j] = np.nan
                    y_sphere1_masked[i, j] = np.nan
                    z_sphere1_masked[i, j] = np.nan
                
                p2 = np.array([x_sphere2[i, j], y_sphere2[i, j], z_sphere2[i, j]]) - end
                if np.dot(p2, -direction) >= 0:  # If the point is facing inside the cylinder
                    x_sphere2_masked[i, j] = np.nan
                    y_sphere2_masked[i, j] = np.nan
                    z_sphere2_masked[i, j] = np.nan
        
        # Plot hemispheres (using NaN mask)
        ax.plot_surface(x_sphere1_masked, y_sphere1_masked, z_sphere1_masked, color=color, alpha=alpha)
        ax.plot_surface(x_sphere2_masked, y_sphere2_masked, z_sphere2_masked, color=color, alpha=alpha)

def plot_robot(ax, robot, joint_angles, color='b'):
    """
    Plot the 3D model of the robot.
    
    Args:
        ax: Matplotlib 3D axis
        robot: Robot object
        joint_angles: Joint angles in radians
        color: Robot color
    """
    # Calculate robot joint positions
    transforms = robot.forward_kinematics(joint_angles)
    
    # Update geometric model position
    robot.update_geometric_model(transforms)
    
    # Plot spheres
    for center, radius in robot.spheres:
        plot_sphere(ax, center, radius, color=color)
    
    # Plot capsules
    for start, end, radius in robot.capsules:
        plot_capsule(ax, start, end, radius, color=color)
    
    # Plot kinematic chain skeleton
    points = [t[:3, 3] for t in transforms]
    xs = [p[0] for p in points]
    ys = [p[1] for p in points]
    zs = [p[2] for p in points]
    ax.plot(xs, ys, zs, 'k-', linewidth=2)