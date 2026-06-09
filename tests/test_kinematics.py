import numpy as np
from numpy.testing import assert_allclose

from robot_collision_detection.core.kinematics import dh_transform


class TestDHTransform:
    def test_zero_params(self):
        T = dh_transform(0, 0, 0, 0)
        assert_allclose(T, np.eye(4), atol=1e-12)

    def test_rotation_90_deg(self):
        T = dh_transform(0, 0, 0, np.pi / 2)
        expected = np.array([
            [0, -1, 0, 0],
            [1, 0, 0, 0],
            [0, 0, 1, 0],
            [0, 0, 0, 1],
        ])
        assert_allclose(T, expected, atol=1e-12)

    def test_translation_a(self):
        T = dh_transform(5.0, 0, 0, 0)
        assert_allclose(T[:3, 3], [5.0, 0, 0], atol=1e-12)

    def test_translation_d(self):
        T = dh_transform(0, 0, 3.0, 0)
        assert_allclose(T[:3, 3], [0, 0, 3.0], atol=1e-12)

    def test_alpha_rotation(self):
        T = dh_transform(0, np.pi / 2, 0, 0)
        expected = np.array([
            [1, 0, 0, 0],
            [0, 0, -1, 0],
            [0, 1, 0, 0],
            [0, 0, 0, 1],
        ])
        assert_allclose(T, expected, atol=1e-12)

    def test_combined(self):
        T = dh_transform(1.0, np.pi / 2, 2.0, np.pi / 2)
        assert_allclose(T[2, 3], 2.0, atol=1e-12)
        assert_allclose(T[0, 3], 0.0, atol=1e-12)
        assert_allclose(T[1, 3], 1.0, atol=1e-12)
