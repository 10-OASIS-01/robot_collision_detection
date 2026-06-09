import numpy as np
from numpy.testing import assert_allclose

from robot_collision_detection.core.robot import Robot


def make_robot():
    dh_params = [
        [0, 0, 1100],
        [600, np.pi / 2, 0],
        [1400, 0, 0],
    ]
    return Robot(dh_params, name="TestRobot")


class TestForwardKinematics:
    def test_num_transforms(self):
        robot = make_robot()
        transforms = robot.forward_kinematics([0, 0, 0])
        assert len(transforms) == 4  # base + 3 joints

    def test_base_is_identity(self):
        robot = make_robot()
        transforms = robot.forward_kinematics([0, 0, 0])
        assert_allclose(transforms[0], np.eye(4), atol=1e-12)

    def test_zero_angles_positions(self):
        robot = make_robot()
        transforms = robot.forward_kinematics([0, 0, 0])
        assert_allclose(transforms[1][:3, 3], [0, 0, 1100], atol=1e-6)
        assert_allclose(transforms[2][:3, 3], [600, 0, 1100], atol=1e-6)
        assert_allclose(transforms[3][:3, 3], [2000, 0, 1100], atol=1e-6)

    def test_custom_base_transform(self):
        dh_params = [[0, 0, 1100], [600, np.pi / 2, 0], [1400, 0, 0]]
        T_base = np.eye(4)
        T_base[1, 3] = 2000
        robot = Robot(dh_params, base_transform=T_base, name="Offset")
        transforms = robot.forward_kinematics([0, 0, 0])
        assert_allclose(transforms[0][:3, 3], [0, 2000, 0], atol=1e-6)


class TestUpdateGeometricModel:
    def test_sphere_positions_updated(self):
        robot = make_robot()
        transforms = robot.forward_kinematics([0, 0, 0])
        robot.update_geometric_model(transforms)
        assert_allclose(robot.spheres[0][0], transforms[0][:3, 3], atol=1e-6)
        assert_allclose(robot.spheres[1][0], transforms[1][:3, 3], atol=1e-6)

    def test_capsule_positions_updated(self):
        robot = make_robot()
        transforms = robot.forward_kinematics([0, 0, 0])
        robot.update_geometric_model(transforms)
        assert_allclose(robot.capsules[0][0], transforms[0][:3, 3], atol=1e-6)
        assert_allclose(robot.capsules[0][1], transforms[1][:3, 3], atol=1e-6)

    def test_default_model_size(self):
        robot = make_robot()
        assert len(robot.spheres) == 4
        assert len(robot.capsules) == 3
