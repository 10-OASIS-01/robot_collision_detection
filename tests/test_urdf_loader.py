import numpy as np
from numpy.testing import assert_allclose
import tempfile
import os

from robot_collision_detection.core.urdf_loader import URDFRobot


MINIMAL_URDF = """<?xml version="1.0"?>
<robot name="test_robot">
  <link name="base_link"/>
  <joint name="joint1" type="revolute">
    <origin xyz="0 0 0.5" rpy="0 0 0"/>
    <parent link="base_link"/>
    <child link="link1"/>
    <axis xyz="0 0 1"/>
    <limit lower="-3.14" upper="3.14" effort="10" velocity="1"/>
  </joint>
  <link name="link1"/>
  <joint name="joint2" type="revolute">
    <origin xyz="0 0 0.3" rpy="0 0 0"/>
    <parent link="link1"/>
    <child link="link2"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="10" velocity="1"/>
  </joint>
  <link name="link2"/>
</robot>
"""


def _write_urdf(content):
    fd, path = tempfile.mkstemp(suffix=".urdf")
    with os.fdopen(fd, "w") as f:
        f.write(content)
    return path


class TestURDFRobot:
    def test_load_minimal(self):
        path = _write_urdf(MINIMAL_URDF)
        try:
            robot = URDFRobot(path)
            assert robot.name == "test_robot"
            assert robot.num_joints == 2
            assert robot.joint_names == ["joint1", "joint2"]
        finally:
            os.unlink(path)

    def test_joint_limits(self):
        path = _write_urdf(MINIMAL_URDF)
        try:
            robot = URDFRobot(path)
            assert_allclose(robot.joint_limits[0], (-3.14, 3.14))
            assert_allclose(robot.joint_limits[1], (-1.57, 1.57))
        finally:
            os.unlink(path)

    def test_forward_kinematics_zero(self):
        path = _write_urdf(MINIMAL_URDF)
        try:
            robot = URDFRobot(path)
            transforms = robot.forward_kinematics([0, 0])
            assert len(transforms) == 3
            assert_allclose(transforms[0][:3, 3], [0, 0, 0], atol=1e-10)
            assert_allclose(transforms[1][:3, 3], [0, 0, 0.5], atol=1e-10)
            assert_allclose(transforms[2][:3, 3], [0, 0, 0.8], atol=1e-10)
        finally:
            os.unlink(path)

    def test_forward_kinematics_rotation(self):
        path = _write_urdf(MINIMAL_URDF)
        try:
            robot = URDFRobot(path)
            transforms = robot.forward_kinematics([np.pi / 2, 0])
            # joint1 rotates around Z: link1 origin stays at (0,0,0.5)
            assert_allclose(transforms[1][:3, 3], [0, 0, 0.5], atol=1e-10)
            # link2 origin: after 90deg Z rotation, the local (0,0,0.3) should be at (0,0,0.8)
            # Z rotation doesn't change Z offset
            assert_allclose(transforms[2][2, 3], 0.8, atol=1e-10)
        finally:
            os.unlink(path)

    def test_geometric_model_size(self):
        path = _write_urdf(MINIMAL_URDF)
        try:
            robot = URDFRobot(path)
            # 2 joints in chain → 3 frames → 3 spheres, 2 capsules
            assert len(robot.spheres) == 3
            assert len(robot.capsules) == 2
        finally:
            os.unlink(path)

    def test_update_geometric_model(self):
        path = _write_urdf(MINIMAL_URDF)
        try:
            robot = URDFRobot(path)
            transforms = robot.forward_kinematics([0, 0])
            robot.update_geometric_model(transforms)
            assert_allclose(robot.spheres[0][0], [0, 0, 0], atol=1e-10)
            assert_allclose(robot.spheres[1][0], [0, 0, 0.5], atol=1e-10)
            assert_allclose(robot.capsules[0][0], [0, 0, 0], atol=1e-10)
            assert_allclose(robot.capsules[0][1], [0, 0, 0.5], atol=1e-10)
        finally:
            os.unlink(path)

    def test_custom_capsule_radius(self):
        path = _write_urdf(MINIMAL_URDF)
        try:
            robot = URDFRobot(path, capsule_radius=0.1)
            assert robot.capsules[0][2] == 0.1
            assert_allclose(robot.spheres[0][1], 0.12, atol=1e-10)
        finally:
            os.unlink(path)

    def test_with_fixed_joint(self):
        urdf = """<?xml version="1.0"?>
<robot name="fixed_test">
  <link name="base"/>
  <joint name="j1" type="revolute">
    <origin xyz="0 0 1" rpy="0 0 0"/>
    <parent link="base"/>
    <child link="l1"/>
    <axis xyz="0 0 1"/>
    <limit lower="-3.14" upper="3.14" effort="10" velocity="1"/>
  </joint>
  <link name="l1"/>
  <joint name="j_fixed" type="fixed">
    <origin xyz="0 0 0.5" rpy="0 0 0"/>
    <parent link="l1"/>
    <child link="l2"/>
  </joint>
  <link name="l2"/>
</robot>
"""
        path = _write_urdf(urdf)
        try:
            robot = URDFRobot(path)
            assert robot.num_joints == 1
            transforms = robot.forward_kinematics([0])
            assert len(transforms) == 3
            assert_allclose(transforms[2][:3, 3], [0, 0, 1.5], atol=1e-10)
        finally:
            os.unlink(path)
