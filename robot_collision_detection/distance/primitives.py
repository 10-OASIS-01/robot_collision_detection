import numpy as np


def _to_3d(arr, name="array"):
    arr = np.asarray(arr, dtype=float)
    if arr.shape != (3,):
        raise ValueError(f"{name} must be a 3D vector, got shape {arr.shape}")
    return arr


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
    p = _to_3d(p, "p")
    seg_start = _to_3d(seg_start, "seg_start")
    seg_end = _to_3d(seg_end, "seg_end")
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

    dist = np.linalg.norm(_to_3d(center1, "center1") - _to_3d(center2, "center2"))
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

    dist = dist_point_segment(_to_3d(center, "center"), _to_3d(seg_start, "seg_start"), _to_3d(seg_end, "seg_end"))
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
    u = seg1_end - seg1_start
    v = seg2_end - seg2_start
    w = seg1_start - seg2_start

    a = np.dot(u, u)
    b = np.dot(u, v)
    c = np.dot(v, v)
    d = np.dot(u, w)
    e = np.dot(v, w)

    # Handle degenerate segments (zero length)
    if a < 1e-12 and c < 1e-12:
        return np.linalg.norm(seg1_start - seg2_start)
    if a < 1e-12:
        return dist_point_segment(seg1_start, seg2_start, seg2_end)
    if c < 1e-12:
        return dist_point_segment(seg2_start, seg1_start, seg1_end)

    denom = a * c - b * b

    if denom < 1e-8:
        sc = 0.0
        tc = np.clip(e / c, 0.0, 1.0)
    else:
        sc = (b * e - c * d) / denom
        tc = (a * e - b * d) / denom

    # Two-pass clamping to ensure both parameters are optimal within [0,1]
    sc = np.clip(sc, 0.0, 1.0)
    tc = np.clip((e + b * sc) / c, 0.0, 1.0)
    sc = np.clip((-d + b * tc) / a, 0.0, 1.0)

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

    dist = dist_segment_segment(
        _to_3d(seg1_start, "seg1_start"), _to_3d(seg1_end, "seg1_end"),
        _to_3d(seg2_start, "seg2_start"), _to_3d(seg2_end, "seg2_end"),
    )

    return dist - r1 - r2


# --- Fast-path functions (skip input validation, for internal use) ---

def _dist_point_segment_fast(p, seg_start, seg_end):
    v = seg_end - seg_start
    w = p - seg_start
    c1 = np.dot(w, v)
    if c1 <= 0:
        return np.linalg.norm(p - seg_start)
    c2 = np.dot(v, v)
    if c2 <= c1:
        return np.linalg.norm(p - seg_end)
    b = c1 / c2
    return np.linalg.norm(p - (seg_start + b * v))


def _dist_segment_segment_fast(seg1_start, seg1_end, seg2_start, seg2_end):
    u = seg1_end - seg1_start
    v = seg2_end - seg2_start
    w = seg1_start - seg2_start
    a = np.dot(u, u)
    b = np.dot(u, v)
    c = np.dot(v, v)
    d = np.dot(u, w)
    e = np.dot(v, w)
    if a < 1e-12 and c < 1e-12:
        return np.linalg.norm(seg1_start - seg2_start)
    if a < 1e-12:
        return _dist_point_segment_fast(seg1_start, seg2_start, seg2_end)
    if c < 1e-12:
        return _dist_point_segment_fast(seg2_start, seg1_start, seg1_end)
    denom = a * c - b * b
    if denom < 1e-8:
        sc = 0.0
        tc = np.clip(e / c, 0.0, 1.0)
    else:
        sc = (b * e - c * d) / denom
        tc = (a * e - b * d) / denom
    sc = np.clip(sc, 0.0, 1.0)
    tc = np.clip((e + b * sc) / c, 0.0, 1.0)
    sc = np.clip((-d + b * tc) / a, 0.0, 1.0)
    return np.linalg.norm((seg1_start + sc * u) - (seg2_start + tc * v))