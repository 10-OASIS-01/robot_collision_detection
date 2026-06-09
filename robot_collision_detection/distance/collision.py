import numpy as np
from .primitives import dist_sphere_sphere, dist_sphere_capsule, dist_capsule_capsule


def _vectorized_sphere_sphere(centers1, radii1, centers2, radii2):
    """Compute (n, m) sphere-sphere surface distance matrix."""
    diff = centers1[:, np.newaxis, :] - centers2[np.newaxis, :, :]
    return np.linalg.norm(diff, axis=2) - radii1[:, np.newaxis] - radii2[np.newaxis, :]


def _vectorized_point_segment(points, seg_starts, seg_ends):
    """Compute (n, m) point-to-segment distance matrix.

    Args:
        points: (n, 3)
        seg_starts: (m, 3)
        seg_ends: (m, 3)

    Returns:
        (n, m) distance matrix
    """
    v = seg_ends - seg_starts                                         # (m, 3)
    c2 = np.einsum('mk,mk->m', v, v)                                 # (m,)
    w = points[:, np.newaxis, :] - seg_starts[np.newaxis, :, :]       # (n, m, 3)
    c1 = np.einsum('nmk,mk->nm', w, v)                               # (n, m)

    safe_c2 = np.maximum(c2, 1e-12)
    b = np.clip(c1 / safe_c2[np.newaxis, :], 0.0, 1.0)

    closest = seg_starts[np.newaxis, :, :] + b[:, :, np.newaxis] * v[np.newaxis, :, :]
    return np.linalg.norm(points[:, np.newaxis, :] - closest, axis=2)


def _vectorized_segment_segment(starts1, ends1, starts2, ends2):
    """Compute (n, m) segment-segment distance matrix.

    Args:
        starts1, ends1: (n, 3)
        starts2, ends2: (m, 3)

    Returns:
        (n, m) distance matrix
    """
    n = len(starts1)
    m = len(starts2)

    u = ends1 - starts1                                               # (n, 3)
    v = ends2 - starts2                                               # (m, 3)
    w = starts1[:, np.newaxis, :] - starts2[np.newaxis, :, :]         # (n, m, 3)

    a = np.einsum('nk,nk->n', u, u)                                   # (n,)
    c = np.einsum('mk,mk->m', v, v)                                   # (m,)
    b_val = np.einsum('nk,mk->nm', u, v)                              # (n, m)
    d_val = np.einsum('nmk,nk->nm', w, u)                             # (n, m)
    e_val = np.einsum('nmk,mk->nm', w, v)                             # (n, m)

    safe_a = np.maximum(a, 1e-12)
    safe_c = np.maximum(c, 1e-12)

    denom = a[:, np.newaxis] * c[np.newaxis, :] - b_val ** 2

    sc = np.where(denom < 1e-8, 0.0,
                  (b_val * e_val - c[np.newaxis, :] * d_val) / np.maximum(denom, 1e-12))
    tc = np.where(denom < 1e-8,
                  np.clip(e_val / safe_c[np.newaxis, :], 0.0, 1.0),
                  (a[:, np.newaxis] * e_val - b_val * d_val) / np.maximum(denom, 1e-12))

    # Two-pass clamping
    sc = np.clip(sc, 0.0, 1.0)
    tc = np.clip((e_val + b_val * sc) / safe_c[np.newaxis, :], 0.0, 1.0)
    sc = np.clip((-d_val + b_val * tc) / safe_a[:, np.newaxis], 0.0, 1.0)

    closest1 = starts1[:, np.newaxis, :] + sc[:, :, np.newaxis] * u[:, np.newaxis, :]
    closest2 = starts2[np.newaxis, :, :] + tc[:, :, np.newaxis] * v[np.newaxis, :, :]

    result = np.linalg.norm(closest1 - closest2, axis=2)

    # Scalar fallback for degenerate segments (zero length)
    degen_mask = (a[:, np.newaxis] < 1e-12) | (c[np.newaxis, :] < 1e-12)
    if np.any(degen_mask):
        from .primitives import _dist_point_segment_fast
        for i, j in zip(*np.where(degen_mask)):
            ai, cj = a[i], c[j]
            if ai < 1e-12 and cj < 1e-12:
                result[i, j] = np.linalg.norm(starts1[i] - starts2[j])
            elif ai < 1e-12:
                result[i, j] = _dist_point_segment_fast(starts1[i], starts2[j], ends2[j])
            else:
                result[i, j] = _dist_point_segment_fast(starts2[j], starts1[i], ends1[i])

    return result


def _extract_spheres(robot):
    centers = np.array([s[0] for s in robot.spheres])
    radii = np.array([s[1] for s in robot.spheres])
    return centers, radii


def _extract_capsules(robot):
    starts = np.array([c[0] for c in robot.capsules])
    ends = np.array([c[1] for c in robot.capsules])
    radii = np.array([c[2] for c in robot.capsules])
    return starts, ends, radii


def min_distance_between_robots(robot1, robot2, threshold=None):
    """
    Calculate the minimum distance between two robots.

    Args:
        robot1: First robot object
        robot2: Second robot object
        threshold: Optional early termination threshold. When any pair distance
                   falls below this value, return immediately.

    Returns:
        Tuple of (minimum_distance, collision_type, collision_elements)
    """
    min_dist = float('inf')
    collision_type = ""
    collision_elements = []

    c1, r1 = _extract_spheres(robot1)
    c2, r2 = _extract_spheres(robot2)
    cs1, ce1, cr1 = _extract_capsules(robot1)
    cs2, ce2, cr2 = _extract_capsules(robot2)

    # --- Sphere-Sphere (vectorized) ---
    if len(c1) > 0 and len(c2) > 0:
        dists_ss = _vectorized_sphere_sphere(c1, r1, c2, r2)
        flat_idx = np.argmin(dists_ss)
        d = float(dists_ss.flat[flat_idx])
        if d < min_dist:
            i, j = np.unravel_index(flat_idx, dists_ss.shape)
            min_dist = d
            collision_type = "sphere-sphere"
            collision_elements = [f"{robot1.name}-S{i+1}", f"{robot2.name}-S{j+1}"]
        if threshold is not None and min_dist < threshold:
            return min_dist, collision_type, collision_elements

    # --- Sphere(r1)-Capsule(r2) (vectorized) ---
    if len(c1) > 0 and len(cs2) > 0:
        pt_dists = _vectorized_point_segment(c1, cs2, ce2)
        dists_sc = pt_dists - r1[:, np.newaxis] - cr2[np.newaxis, :]
        flat_idx = np.argmin(dists_sc)
        d = float(dists_sc.flat[flat_idx])
        if d < min_dist:
            i, j = np.unravel_index(flat_idx, dists_sc.shape)
            min_dist = d
            collision_type = "sphere-capsule"
            collision_elements = [f"{robot1.name}-S{i+1}", f"{robot2.name}-C{j+1}"]
        if threshold is not None and min_dist < threshold:
            return min_dist, collision_type, collision_elements

    # --- Sphere(r2)-Capsule(r1) (vectorized) ---
    if len(c2) > 0 and len(cs1) > 0:
        pt_dists = _vectorized_point_segment(c2, cs1, ce1)
        dists_cs = pt_dists - r2[:, np.newaxis] - cr1[np.newaxis, :]
        flat_idx = np.argmin(dists_cs)
        d = float(dists_cs.flat[flat_idx])
        if d < min_dist:
            i, j = np.unravel_index(flat_idx, dists_cs.shape)
            min_dist = d
            collision_type = "capsule-sphere"
            collision_elements = [f"{robot1.name}-C{j+1}", f"{robot2.name}-S{i+1}"]
        if threshold is not None and min_dist < threshold:
            return min_dist, collision_type, collision_elements

    # --- Capsule-Capsule (vectorized) ---
    if len(cs1) > 0 and len(cs2) > 0:
        seg_dists = _vectorized_segment_segment(cs1, ce1, cs2, ce2)
        dists_cc = seg_dists - cr1[:, np.newaxis] - cr2[np.newaxis, :]
        flat_idx = np.argmin(dists_cc)
        d = float(dists_cc.flat[flat_idx])
        if d < min_dist:
            i, j = np.unravel_index(flat_idx, dists_cc.shape)
            min_dist = d
            collision_type = "capsule-capsule"
            collision_elements = [f"{robot1.name}-C{i+1}", f"{robot2.name}-C{j+1}"]

    return min_dist, collision_type, collision_elements


def min_distance_within_robot(robot, skip_adjacent=1, threshold=None):
    """
    Calculate the minimum distance between non-adjacent primitives within a single robot.

    Args:
        robot: Robot object
        skip_adjacent: Number of adjacent links to skip (default 1).
                       Primitives on links connected by <= skip_adjacent joints
                       are not checked.
        threshold: Optional early termination threshold.

    Returns:
        Tuple of (minimum_distance, collision_type, collision_elements)
    """
    min_dist = float('inf')
    collision_type = ""
    collision_elements = []

    centers, radii = _extract_spheres(robot)
    cap_starts, cap_ends, cap_radii = _extract_capsules(robot)
    ns = len(centers)
    nc = len(cap_starts)

    # --- Self Sphere-Sphere ---
    if ns > 0:
        dists_ss = _vectorized_sphere_sphere(centers, radii, centers, radii)
        # Mask: self + adjacent
        idx_s = np.arange(ns)
        mask = np.abs(idx_s[:, None] - idx_s[None, :]) <= skip_adjacent
        dists_ss[mask] = np.inf

        flat_idx = np.argmin(dists_ss)
        d = float(dists_ss.flat[flat_idx])
        if d < min_dist:
            i, j = np.unravel_index(flat_idx, dists_ss.shape)
            min_dist = d
            collision_type = "sphere-sphere"
            collision_elements = [f"{robot.name}-S{i+1}", f"{robot.name}-S{j+1}"]
        if threshold is not None and min_dist < threshold:
            return min_dist, collision_type, collision_elements

    # --- Self Sphere-Capsule ---
    if ns > 0 and nc > 0:
        pt_dists = _vectorized_point_segment(centers, cap_starts, cap_ends)
        dists_sc = pt_dists - radii[:, np.newaxis] - cap_radii[np.newaxis, :]

        # Mask: sphere i is adjacent to capsule j if |i - j| <= skip_adjacent or |i - (j+1)| <= skip_adjacent
        idx_s = np.arange(ns)
        idx_c = np.arange(nc)
        adj1 = np.abs(idx_s[:, None] - idx_c[None, :])
        adj2 = np.abs(idx_s[:, None] - (idx_c[None, :] + 1))
        mask = (np.minimum(adj1, adj2) <= skip_adjacent)
        dists_sc[mask] = np.inf

        flat_idx = np.argmin(dists_sc)
        d = float(dists_sc.flat[flat_idx])
        if d < min_dist:
            i, j = np.unravel_index(flat_idx, dists_sc.shape)
            min_dist = d
            collision_type = "sphere-capsule"
            collision_elements = [f"{robot.name}-S{i+1}", f"{robot.name}-C{j+1}"]
        if threshold is not None and min_dist < threshold:
            return min_dist, collision_type, collision_elements

    # --- Self Capsule-Capsule ---
    if nc > 0:
        seg_dists = _vectorized_segment_segment(cap_starts, cap_ends, cap_starts, cap_ends)
        dists_cc = seg_dists - cap_radii[:, np.newaxis] - cap_radii[np.newaxis, :]

        # Mask: capsule i adjacent to capsule j if |i - j| <= skip_adjacent
        idx_c = np.arange(nc)
        mask = np.abs(idx_c[:, None] - idx_c[None, :]) <= skip_adjacent
        dists_cc[mask] = np.inf

        flat_idx = np.argmin(dists_cc)
        d = float(dists_cc.flat[flat_idx])
        if d < min_dist:
            i, j = np.unravel_index(flat_idx, dists_cc.shape)
            min_dist = d
            collision_type = "capsule-capsule"
            collision_elements = [f"{robot.name}-C{i+1}", f"{robot.name}-C{j+1}"]

    return min_dist, collision_type, collision_elements
