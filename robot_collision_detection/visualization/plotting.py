import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

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

    # Vectorized masking of hemisphere points
    if length > 0:
        direction = v / np.linalg.norm(v)

        # Start hemisphere: mask points facing inside the cylinder
        points1 = np.stack([x_sphere1 - start[0], y_sphere1 - start[1], z_sphere1 - start[2]], axis=-1)
        mask1 = np.einsum('...k,k->...', points1, direction) >= 0
        x_sphere1_masked = np.where(mask1, np.nan, x_sphere1)
        y_sphere1_masked = np.where(mask1, np.nan, y_sphere1)
        z_sphere1_masked = np.where(mask1, np.nan, z_sphere1)

        # End hemisphere: mask points facing inside the cylinder
        points2 = np.stack([x_sphere2 - end[0], y_sphere2 - end[1], z_sphere2 - end[2]], axis=-1)
        mask2 = np.einsum('...k,k->...', points2, -direction) >= 0
        x_sphere2_masked = np.where(mask2, np.nan, x_sphere2)
        y_sphere2_masked = np.where(mask2, np.nan, y_sphere2)
        z_sphere2_masked = np.where(mask2, np.nan, z_sphere2)

        ax.plot_surface(x_sphere1_masked, y_sphere1_masked, z_sphere1_masked, color=color, alpha=alpha)
        ax.plot_surface(x_sphere2_masked, y_sphere2_masked, z_sphere2_masked, color=color, alpha=alpha)

def _plot_cylinder(ax, start, end, radius, color='b', alpha=0.4):
    start = np.array(start)
    end = np.array(end)
    v = end - start
    length = np.linalg.norm(v)
    if length < 1e-10:
        return
    v = v / length

    not_v = np.array([1, 0, 0])
    if abs(np.dot(v, not_v)) > 0.9:
        not_v = np.array([0, 1, 0])
    n1 = np.cross(v, not_v)
    n1 = n1 / np.linalg.norm(n1)
    n2 = np.cross(v, n1)

    t = np.linspace(0, length, 2)
    theta = np.linspace(0, 2 * np.pi, 16)
    t_grid, theta_grid = np.meshgrid(t, theta)

    x = start[0] + v[0] * t_grid + radius * np.cos(theta_grid) * n1[0] + radius * np.sin(theta_grid) * n2[0]
    y = start[1] + v[1] * t_grid + radius * np.cos(theta_grid) * n1[1] + radius * np.sin(theta_grid) * n2[1]
    z = start[2] + v[2] * t_grid + radius * np.cos(theta_grid) * n1[2] + radius * np.sin(theta_grid) * n2[2]
    ax.plot_surface(x, y, z, color=color, alpha=alpha)


def _plot_frame(ax, T, length=0.06):
    origin = T[:3, 3]
    colors = ['r', 'g', 'b']
    for i, c in enumerate(colors):
        endpoint = origin + T[:3, i] * length
        ax.plot([origin[0], endpoint[0]],
                [origin[1], endpoint[1]],
                [origin[2], endpoint[2]],
                c, linewidth=1.5)


def plot_robot(ax, robot, joint_angles, color='b', show_collision=True):
    """
    Plot the 3D model of the robot.

    Args:
        ax: Matplotlib 3D axis
        robot: Robot object
        joint_angles: Joint angles in radians
        color: Robot color
        show_collision: Whether to show collision geometry (spheres/capsules)
    """
    transforms = robot.forward_kinematics(joint_angles)
    robot.update_geometric_model(transforms)

    if show_collision:
        for center, radius in robot.spheres:
            plot_sphere(ax, center, radius, color=color)
        for start, end, radius in robot.capsules:
            plot_capsule(ax, start, end, radius, color=color)

    points = [t[:3, 3] for t in transforms]
    xs = [p[0] for p in points]
    ys = [p[1] for p in points]
    zs = [p[2] for p in points]
    ax.plot(xs, ys, zs, 'k-', linewidth=2)


def _plot_mesh(ax, vertices, faces, color='steelblue', alpha=0.6, edgecolor='gray', linewidth=0.1):
    triangles = vertices[faces]
    poly = Poly3DCollection(triangles, alpha=alpha, linewidth=linewidth, edgecolor=edgecolor)
    poly.set_facecolor(color)
    ax.add_collection3d(poly)


def plot_urdf_robot(ax, robot, joint_angles, color='steelblue',
                    show_frames=True, show_collision=False, show_mesh="collision",
                    link_radius=None, joint_radius=None):
    """
    Plot a URDF robot with mesh rendering, joint markers, and coordinate frames.

    Args:
        ax: Matplotlib 3D axis
        robot: URDFRobot object
        joint_angles: Joint angles in radians
        color: Link/mesh color
        show_frames: Whether to show coordinate frames at each joint
        show_collision: Whether to overlay collision capsules/spheres (transparent)
        show_mesh: Which mesh to render - "collision", "visual", or None
        link_radius: Radius for link cylinders when no mesh (auto-scaled if None)
        joint_radius: Radius for joint spheres (auto-scaled if None)
    """
    transforms = robot.forward_kinematics(joint_angles)
    robot.update_geometric_model(transforms)

    positions = np.array([T[:3, 3] for T in transforms])
    link_lengths = np.linalg.norm(np.diff(positions, axis=0), axis=1)
    avg_link = np.mean(link_lengths[link_lengths > 1e-6]) if np.any(link_lengths > 1e-6) else 0.1

    if link_radius is None:
        link_radius = avg_link * 0.08
    if joint_radius is None:
        joint_radius = avg_link * 0.14

    # Try to render STL meshes
    has_meshes = False
    if show_mesh and hasattr(robot, 'get_transformed_meshes'):
        mesh_data = robot.get_transformed_meshes(transforms, mesh_type=show_mesh)
        if mesh_data:
            has_meshes = True
            for verts, faces in mesh_data:
                _plot_mesh(ax, verts, faces, color=color, alpha=0.6,
                           edgecolor='dimgray', linewidth=0.15)

    # Fallback: draw link cylinders if no mesh
    if not has_meshes:
        for i in range(len(transforms) - 1):
            p_start = transforms[i][:3, 3]
            p_end = transforms[i + 1][:3, 3]
            seg_len = np.linalg.norm(p_end - p_start)
            if seg_len > 1e-6:
                _plot_cylinder(ax, p_start, p_end, link_radius, color=color, alpha=0.5)

    # Draw skeleton line
    xs = [p[0] for p in positions]
    ys = [p[1] for p in positions]
    zs = [p[2] for p in positions]
    ax.plot(xs, ys, zs, '-', color='dimgray', linewidth=1.5, zorder=5)

    # Draw joints
    active_idx = set()
    for i, joint in enumerate(robot._urdf_joints):
        if joint["type"] in ("revolute", "continuous", "prismatic"):
            active_idx.add(i + 1)

    for i, T in enumerate(transforms):
        pos = T[:3, 3]
        if i in active_idx:
            plot_sphere(ax, pos, joint_radius, color='orangered', alpha=0.7)
        elif i == 0:
            plot_sphere(ax, pos, joint_radius * 1.3, color='dimgray', alpha=0.6)

    # Draw coordinate frames
    if show_frames:
        frame_len = avg_link * 0.25
        for T in transforms:
            _plot_frame(ax, T, length=frame_len)

    # Optionally overlay collision geometry
    if show_collision:
        for center, radius in robot.spheres:
            plot_sphere(ax, center, radius, color=color, alpha=0.08)
        for start, end, radius in robot.capsules:
            if np.linalg.norm(np.array(end) - np.array(start)) > 1e-6:
                plot_capsule(ax, start, end, radius, color=color, alpha=0.08)