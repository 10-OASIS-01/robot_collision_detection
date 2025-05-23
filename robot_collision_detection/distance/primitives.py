import numpy as np

def dist_point_segment(p, seg_start, seg_end):
    """
    Calculate the shortest distance from a point to a line segment.
    
    Args:
        p: Point coordinates as a numpy array
        seg_start: Start point of the line segment as a numpy array
        seg_end: End point of the line segment as a numpy array
    
    Returns:
        Shortest distance from the point to the line segment
    """
    v = seg_end - seg_start
    w = p - seg_start
    
    c1 = np.dot(w, v)
    if c1 <= 0:
        return np.linalg.norm(p - seg_start)
    
    c2 = np.dot(v, v)
    if c2 <= c1:
        return np.linalg.norm(p - seg_end)
    
    b = c1 / c2
    pb = seg_start + b * v
    return np.linalg.norm(p - pb)

def dist_sphere_sphere(s1, s2):
    """
    Calculate the distance between two spheres.
    
    Args:
        s1: First sphere as [center, radius]
        s2: Second sphere as [center, radius]
    
    Returns:
        Distance between the surfaces of the spheres (negative if overlapping)
    """
    center1, r1 = s1
    center2, r2 = s2
    
    dist = np.linalg.norm(np.array(center1) - np.array(center2))
    return dist - r1 - r2

def dist_sphere_capsule(sphere, capsule):
    """
    Calculate the distance between a sphere and a capsule.
    
    Args:
        sphere: Sphere as [center, radius]
        capsule: Capsule as [start_point, end_point, radius]
    
    Returns:
        Distance between the surfaces (negative if overlapping)
    """
    center, r_sphere = sphere
    seg_start, seg_end, r_capsule = capsule
    
    # Calculate the distance from the sphere center to the capsule's centerline segment
    dist = dist_point_segment(np.array(center), np.array(seg_start), np.array(seg_end))
    return dist - r_sphere - r_capsule

def dist_segment_segment(seg1_start, seg1_end, seg2_start, seg2_end):
    """
    Calculate the shortest distance between two line segments.
    
    Args:
        seg1_start: Start point of the first line segment
        seg1_end: End point of the first line segment
        seg2_start: Start point of the second line segment
        seg2_end: End point of the second line segment
    
    Returns:
        Shortest distance between the line segments
    """
    u = seg1_end - seg1_start  # Direction vector of the first line segment
    v = seg2_end - seg2_start  # Direction vector of the second line segment
    w = seg1_start - seg2_start  # Vector connecting the start points of the two line segments
    
    a = np.dot(u, u)  # 'a' in the paper
    b = np.dot(u, v)  # 'b' in the paper
    c = np.dot(v, v)  # 'c' in the paper
    d = np.dot(u, w)  # 'd' in the paper
    e = np.dot(v, w)  # 'e' in the paper
    f = np.dot(w, w)  # 'f' in the paper
    
    # Calculate the denominator
    denom = a*c - b*b
    
    # Initialize parameters
    sc = 0.0
    tc = 0.0
    
    if denom < 1e-8:  # Parallel or almost parallel case
        # Set sc to an arbitrary value, e.g., 0
        sc = 0.0
        # Calculate tc to minimize distance
        if b > c:  # Use point-to-segment distance
            tc = d/b
        else:
            tc = e/c
    else:
        # Calculate sc and tc
        sc = (b*e - c*d) / denom
        tc = (a*e - b*d) / denom
    
    # Ensure sc and tc are within [0,1] range
    if sc < 0.0:
        sc = 0.0
        tc = e/c if e/c >= 0 and e/c <= 1 else (0.0 if e/c < 0 else 1.0)
    elif sc > 1.0:
        sc = 1.0
        tc = (e+b)/c if (e+b)/c >= 0 and (e+b)/c <= 1 else (0.0 if (e+b)/c < 0 else 1.0)
    
    if tc < 0.0:
        tc = 0.0
        sc = -d/a if -d/a >= 0 and -d/a <= 1 else (0.0 if -d/a < 0 else 1.0)
    elif tc > 1.0:
        tc = 1.0
        sc = (b-d)/a if (b-d)/a >= 0 and (b-d)/a <= 1 else (0.0 if (b-d)/a < 0 else 1.0)
    
    # Calculate the closest points
    closest1 = seg1_start + sc * u
    closest2 = seg2_start + tc * v
    
    return np.linalg.norm(closest1 - closest2)

def dist_capsule_capsule(c1, c2):
    """
    Calculate the distance between two capsules.
    
    Args:
        c1: First capsule as [start_point, end_point, radius]
        c2: Second capsule as [start_point, end_point, radius]
    
    Returns:
        Distance between the surfaces (negative if overlapping)
    """
    seg1_start, seg1_end, r1 = c1
    seg2_start, seg2_end, r2 = c2
    
    # Calculate the distance between the line segments
    dist = dist_segment_segment(
        np.array(seg1_start), np.array(seg1_end), 
        np.array(seg2_start), np.array(seg2_end)
    )
    
    return dist - r1 - r2