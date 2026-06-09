import numpy as np


class IKResult:
    """Result of an inverse kinematics solve."""

    __slots__ = ("joint_angles", "success", "position_error", "orientation_error",
                 "iterations", "collision_free")

    def __init__(self, joint_angles, success, position_error, orientation_error,
                 iterations, collision_free):
        self.joint_angles = joint_angles
        self.success = success
        self.position_error = position_error
        self.orientation_error = orientation_error
        self.iterations = iterations
        self.collision_free = collision_free

    def __repr__(self):
        status = "OK" if self.success else "FAIL"
        coll = ", collision_free" if self.collision_free else ""
        return (f"IKResult({status}, pos_err={self.position_error:.6f}, "
                f"ori_err={self.orientation_error:.6f}, iters={self.iterations}{coll})")


def _rotation_error(R_current, R_target):
    """Compute orientation error as a 3-vector (angle-axis form)."""
    R_err = R_target @ R_current.T
    trace_val = np.clip((np.trace(R_err) - 1) / 2, -1.0, 1.0)
    angle = np.arccos(trace_val)
    if abs(angle) < 1e-10:
        return np.zeros(3)
    skew = (R_err - R_err.T) / (2 * np.sin(angle))
    axis = np.array([skew[2, 1], skew[0, 2], skew[1, 0]])
    return axis * angle


def _numerical_jacobian(robot, joint_angles, ee_index=-1, delta=1e-7):
    """Compute the 6xN Jacobian numerically via finite differences.

    Rows 0-2: position (dx, dy, dz) / dq
    Rows 3-5: orientation (angular velocity) / dq
    """
    n = len(joint_angles)
    q = np.array(joint_angles, dtype=float)

    T0 = robot.forward_kinematics(q)
    T_ee = T0[ee_index]
    p0 = T_ee[:3, 3]
    R0 = T_ee[:3, :3]

    J = np.zeros((6, n))
    for i in range(n):
        q_plus = q.copy()
        q_plus[i] += delta
        T_plus = robot.forward_kinematics(q_plus)
        T_ee_plus = T_plus[ee_index]

        J[:3, i] = (T_ee_plus[:3, 3] - p0) / delta

        R_plus = T_ee_plus[:3, :3]
        dR = R_plus @ R0.T
        trace_val = np.clip((np.trace(dR) - 1) / 2, -1.0, 1.0)
        angle = np.arccos(trace_val)
        if abs(angle) < 1e-12:
            J[3:, i] = 0.0
        else:
            skew = (dR - dR.T) / (2 * np.sin(angle))
            axis = np.array([skew[2, 1], skew[0, 2], skew[1, 0]])
            J[3:, i] = axis * angle / delta

    return J


def _get_joint_limits(robot):
    """Extract joint limits from robot. Returns (lower, upper) arrays or (None, None)."""
    if hasattr(robot, 'joint_limits') and robot.joint_limits:
        limits = robot.joint_limits
        lower = np.array([l[0] if l[0] is not None else -2 * np.pi for l in limits])
        upper = np.array([l[1] if l[1] is not None else 2 * np.pi for l in limits])
        return lower, upper
    return None, None


def _get_num_joints(robot):
    if hasattr(robot, 'num_joints'):
        return robot.num_joints
    return len(robot.dh_params)


def inverse_kinematics(robot, target_pose, q0=None, max_iter=200,
                       pos_tol=1e-4, ori_tol=1e-3, damping=0.05,
                       orientation_weight=1.0, ee_index=-1,
                       respect_limits=True, step_size=1.0):
    """Solve inverse kinematics using damped least-squares (Levenberg-Marquardt).

    Args:
        robot: Robot or URDFRobot object
        target_pose: 4x4 target end-effector pose (homogeneous transform)
        q0: Initial joint angles (default: zeros or mid-range of limits)
        max_iter: Maximum iterations
        pos_tol: Position convergence tolerance (same units as robot model)
        ori_tol: Orientation convergence tolerance (radians)
        damping: Damping factor for pseudo-inverse (higher = more stable, slower)
        orientation_weight: Weight for orientation error relative to position
        ee_index: Index into the transforms list for the end-effector (-1 = last)
        respect_limits: Whether to clamp joints to their limits
        step_size: Scaling factor for joint updates (0-1, lower = more stable)

    Returns:
        IKResult with joint_angles, success, errors, and iteration count
    """
    n = _get_num_joints(robot)
    target_pose = np.asarray(target_pose, dtype=float)
    p_target = target_pose[:3, 3]
    R_target = target_pose[:3, :3]

    lower, upper = _get_joint_limits(robot) if respect_limits else (None, None)

    if q0 is not None:
        q = np.array(q0, dtype=float)
    elif lower is not None:
        q = (lower + upper) / 2.0
    else:
        q = np.zeros(n)

    for it in range(max_iter):
        transforms = robot.forward_kinematics(q)
        T_ee = transforms[ee_index]
        p_current = T_ee[:3, 3]
        R_current = T_ee[:3, :3]

        pos_err_vec = p_target - p_current
        ori_err_vec = _rotation_error(R_current, R_target)

        pos_err = np.linalg.norm(pos_err_vec)
        ori_err = np.linalg.norm(ori_err_vec)

        if pos_err < pos_tol and ori_err < ori_tol:
            return IKResult(q, True, pos_err, ori_err, it + 1, True)

        error = np.concatenate([pos_err_vec, orientation_weight * ori_err_vec])
        J = _numerical_jacobian(robot, q, ee_index=ee_index)
        J[3:, :] *= orientation_weight

        # Damped least-squares: dq = J^T (J J^T + lambda^2 I)^{-1} e
        JJt = J @ J.T
        lam = damping * (1 + pos_err)
        dq = J.T @ np.linalg.solve(JJt + lam**2 * np.eye(6), error)

        max_dq = np.max(np.abs(dq))
        if max_dq > 0.5:
            dq *= 0.5 / max_dq

        q = q + step_size * dq

        if lower is not None:
            q = np.clip(q, lower, upper)

    transforms = robot.forward_kinematics(q)
    T_ee = transforms[ee_index]
    pos_err = np.linalg.norm(p_target - T_ee[:3, 3])
    ori_err = np.linalg.norm(_rotation_error(T_ee[:3, :3], R_target))

    return IKResult(q, False, pos_err, ori_err, max_iter, True)


def inverse_kinematics_collision_free(robot, target_pose, obstacles=None,
                                      q0=None, max_attempts=10,
                                      collision_margin=0.0,
                                      self_collision=True,
                                      skip_adjacent=1, **ik_kwargs):
    """Solve IK while avoiding collisions.

    Tries multiple random initial configurations to find a collision-free solution.

    Args:
        robot: Robot or URDFRobot object
        target_pose: 4x4 target end-effector pose
        obstacles: List of other Robot objects to check collision against
        q0: Preferred initial joint angles (tried first)
        max_attempts: Number of random restarts
        collision_margin: Minimum required clearance distance
        self_collision: Whether to check self-collision
        skip_adjacent: Adjacency parameter for self-collision check
        **ik_kwargs: Additional arguments passed to inverse_kinematics()

    Returns:
        IKResult (collision_free=True if no collision, False otherwise)
    """
    from ..distance.collision import min_distance_between_robots, min_distance_within_robot

    n = _get_num_joints(robot)
    lower, upper = _get_joint_limits(robot)
    if obstacles is None:
        obstacles = []

    def _check_collision(q):
        transforms = robot.forward_kinematics(q)
        robot.update_geometric_model(transforms)

        if self_collision:
            d, _, _ = min_distance_within_robot(robot, skip_adjacent=skip_adjacent,
                                                threshold=collision_margin)
            if d < collision_margin:
                return False

        for obs in obstacles:
            d, _, _ = min_distance_between_robots(robot, obs, threshold=collision_margin)
            if d < collision_margin:
                return False

        return True

    def _random_q():
        if lower is not None:
            return lower + np.random.rand(n) * (upper - lower)
        return (np.random.rand(n) - 0.5) * 2 * np.pi

    # First try with preferred initial config
    starts = []
    if q0 is not None:
        starts.append(np.array(q0, dtype=float))
    if lower is not None:
        starts.append((lower + upper) / 2.0)
    for _ in range(max_attempts - len(starts)):
        starts.append(_random_q())

    best_result = None
    for q_init in starts:
        result = inverse_kinematics(robot, target_pose, q0=q_init, **ik_kwargs)
        if not result.success:
            if best_result is None or result.position_error < best_result.position_error:
                result.collision_free = False
                best_result = result
            continue

        if _check_collision(result.joint_angles):
            result.collision_free = True
            return result

        result.collision_free = False
        if best_result is None or not best_result.success:
            best_result = result

    return best_result
