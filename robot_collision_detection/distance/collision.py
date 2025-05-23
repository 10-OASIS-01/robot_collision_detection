import numpy as np

def min_distance_between_robots(robot1, robot2):
    """
    Calculate the minimum distance between two robots.
    
    Args:
        robot1: First robot object
        robot2: Second robot object
    
    Returns:
        Tuple of (minimum_distance, collision_type, collision_elements)
    """
    from .primitives import dist_sphere_sphere, dist_sphere_capsule, dist_capsule_capsule

    min_dist = float('inf')
    collision_type = ""
    collision_elements = []
    
    # Sphere-to-sphere distance
    for i, s1 in enumerate(robot1.spheres):
        for j, s2 in enumerate(robot2.spheres):
            dist = dist_sphere_sphere(s1, s2)
            if dist < min_dist:
                min_dist = dist
                collision_type = "sphere-sphere"
                collision_elements = [f"{robot1.name}-S{i+1}", f"{robot2.name}-S{j+1}"]
    
    # Sphere-to-capsule distance
    for i, s1 in enumerate(robot1.spheres):
        for j, c2 in enumerate(robot2.capsules):
            dist = dist_sphere_capsule(s1, c2)
            if dist < min_dist:
                min_dist = dist
                collision_type = "sphere-capsule"
                collision_elements = [f"{robot1.name}-S{i+1}", f"{robot2.name}-C{j+1}"]
    
    for i, s2 in enumerate(robot2.spheres):
        for j, c1 in enumerate(robot1.capsules):
            dist = dist_sphere_capsule(s2, c1)
            if dist < min_dist:
                min_dist = dist
                collision_type = "capsule-sphere"
                collision_elements = [f"{robot1.name}-C{j+1}", f"{robot2.name}-S{i+1}"]
    
    # Capsule-to-capsule distance
    for i, c1 in enumerate(robot1.capsules):
        for j, c2 in enumerate(robot2.capsules):
            dist = dist_capsule_capsule(c1, c2)
            if dist < min_dist:
                min_dist = dist
                collision_type = "capsule-capsule"
                collision_elements = [f"{robot1.name}-C{i+1}", f"{robot2.name}-C{j+1}"]
    
    return min_dist, collision_type, collision_elements