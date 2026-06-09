import numpy as np
from numpy.testing import assert_allclose
import pytest

from robot_collision_detection.core.robot import Robot
from robot_collision_detection.core.ik_solver import (
    inverse_kinematics,
    inverse_kinematics_collision_free,
    _numerical_jacobian,
    _rotation_error,
)


def _make_robot():
    dh_params = [
        [0, 0, 1100],
        [600, np.pi / 2, 0],
        [1400, 0, 0],
    ]
    return Robot(dh_params, name="TestBot")


class TestRotationError:
    def test_identity(self):
        R = np.eye(3)
        err = _rotation_error(R, R)
        assert_allclose(err, [0, 0, 0], atol=1e-10)

    def test_90_deg_z(self):
        R_target = np.array([[0, -1, 0], [1, 0, 0], [0, 0, 1.0]])
        err = _rotation_error(np.eye(3), R_target)
        assert abs(np.linalg.norm(err) - np.pi / 2) < 1e-6


class TestNumericalJacobian:
    def test_shape(self):
        robot = _make_robot()
        q = [0.0, 0.0, 0.0]
        J = _numerical_jacobian(robot, q)
        assert J.shape == (6, 3)

    def test_nonzero(self):
        robot = _make_robot()
        q = [0.1, -0.2, 0.3]
        J = _numerical_jacobian(robot, q)
        assert np.any(np.abs(J) > 1e-6)


class TestInverseKinematics:
    def test_reach_fk_pose(self):
        robot = _make_robot()
        q_target = np.array([0.3, -0.5, 0.2])
        transforms = robot.forward_kinematics(q_target)
        T_target = transforms[-1]

        result = inverse_kinematics(robot, T_target, q0=np.zeros(3),
                                    pos_tol=1.0, ori_tol=0.01)
        assert result.success
        assert result.position_error < 1.0
        assert result.orientation_error < 0.01

    def test_recovers_known_angles(self):
        robot = _make_robot()
        q_target = np.array([0.1, -0.3, 0.15])
        transforms = robot.forward_kinematics(q_target)
        T_target = transforms[-1]

        result = inverse_kinematics(robot, T_target, q0=q_target + 0.05,
                                    pos_tol=0.1, ori_tol=0.001)
        assert result.success
        assert_allclose(result.joint_angles, q_target, atol=0.05)

    def test_unreachable_returns_fail(self):
        robot = _make_robot()
        T_far = np.eye(4)
        T_far[:3, 3] = [99999, 99999, 99999]
        result = inverse_kinematics(robot, T_far, max_iter=50)
        assert not result.success

    def test_result_repr(self):
        robot = _make_robot()
        T = robot.forward_kinematics([0, 0, 0])[-1]
        result = inverse_kinematics(robot, T, max_iter=5)
        s = repr(result)
        assert "IKResult" in s


class TestInverseKinematicsURDF:
    def test_urdf_ik(self):
        """Test IK with a URDFRobot using a minimal inline URDF."""
        import tempfile, os
        from robot_collision_detection.core.urdf_loader import URDFRobot

        urdf = """<?xml version="1.0"?>
<robot name="test2dof">
  <link name="base"/>
  <joint name="j1" type="revolute">
    <origin xyz="0 0 0.5" rpy="0 0 0"/>
    <parent link="base"/>
    <child link="l1"/>
    <axis xyz="0 0 1"/>
    <limit lower="-3.14" upper="3.14" effort="10" velocity="1"/>
  </joint>
  <link name="l1"/>
  <joint name="j2" type="revolute">
    <origin xyz="0.4 0 0" rpy="0 0 0"/>
    <parent link="l1"/>
    <child link="l2"/>
    <axis xyz="0 0 1"/>
    <limit lower="-3.14" upper="3.14" effort="10" velocity="1"/>
  </joint>
  <link name="l2"/>
</robot>
"""
        fd, path = tempfile.mkstemp(suffix=".urdf")
        with os.fdopen(fd, "w") as f:
            f.write(urdf)
        try:
            robot = URDFRobot(path, capsule_radius=0.02)
            q_known = np.array([0.5, -0.3])
            T_target = robot.forward_kinematics(q_known)[-1]

            result = inverse_kinematics(robot, T_target, q0=np.zeros(2),
                                        pos_tol=1e-3, ori_tol=1e-2,
                                        respect_limits=True)
            assert result.success
            assert result.position_error < 1e-3
        finally:
            os.unlink(path)


class TestCollisionFreeIK:
    def test_basic_collision_free(self):
        robot = _make_robot()
        q_target = np.array([0.2, -0.3, 0.1])
        T_target = robot.forward_kinematics(q_target)[-1]

        result = inverse_kinematics_collision_free(
            robot, T_target, obstacles=[],
            self_collision=False, max_attempts=3,
            pos_tol=1.0, ori_tol=0.01,
        )
        assert result.success
        assert result.collision_free
