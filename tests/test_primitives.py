import numpy as np
import pytest
from numpy.testing import assert_allclose

from robot_collision_detection.distance.primitives import (
    dist_point_segment,
    dist_sphere_sphere,
    dist_sphere_capsule,
    dist_segment_segment,
    dist_capsule_capsule,
)


class TestDistPointSegment:
    def test_point_at_start(self):
        p = np.array([0.0, 0.0, 0.0])
        seg_start = np.array([0.0, 0.0, 0.0])
        seg_end = np.array([1.0, 0.0, 0.0])
        assert_allclose(dist_point_segment(p, seg_start, seg_end), 0.0)

    def test_point_perpendicular(self):
        p = np.array([0.5, 1.0, 0.0])
        seg_start = np.array([0.0, 0.0, 0.0])
        seg_end = np.array([1.0, 0.0, 0.0])
        assert_allclose(dist_point_segment(p, seg_start, seg_end), 1.0)

    def test_point_beyond_end(self):
        p = np.array([3.0, 0.0, 0.0])
        seg_start = np.array([0.0, 0.0, 0.0])
        seg_end = np.array([1.0, 0.0, 0.0])
        assert_allclose(dist_point_segment(p, seg_start, seg_end), 2.0)

    def test_point_before_start(self):
        p = np.array([-2.0, 0.0, 0.0])
        seg_start = np.array([0.0, 0.0, 0.0])
        seg_end = np.array([1.0, 0.0, 0.0])
        assert_allclose(dist_point_segment(p, seg_start, seg_end), 2.0)


class TestDistSphereSphere:
    def test_separated(self):
        s1 = [np.array([0, 0, 0]), 1.0]
        s2 = [np.array([5, 0, 0]), 1.0]
        assert_allclose(dist_sphere_sphere(s1, s2), 3.0)

    def test_overlapping(self):
        s1 = [np.array([0, 0, 0]), 1.0]
        s2 = [np.array([1.5, 0, 0]), 1.0]
        assert_allclose(dist_sphere_sphere(s1, s2), -0.5)

    def test_touching(self):
        s1 = [np.array([0, 0, 0]), 1.0]
        s2 = [np.array([2, 0, 0]), 1.0]
        assert_allclose(dist_sphere_sphere(s1, s2), 0.0)

    def test_concentric(self):
        s1 = [np.array([0, 0, 0]), 1.0]
        s2 = [np.array([0, 0, 0]), 0.5]
        assert_allclose(dist_sphere_sphere(s1, s2), -1.5)


class TestDistSphereCapsule:
    def test_basic(self):
        sphere = [np.array([0, 3, 0]), 1.0]
        capsule = [np.array([-1, 0, 0]), np.array([1, 0, 0]), 0.5]
        assert_allclose(dist_sphere_capsule(sphere, capsule), 1.5)

    def test_overlapping(self):
        sphere = [np.array([0, 0, 0]), 1.0]
        capsule = [np.array([-1, 0, 0]), np.array([1, 0, 0]), 0.5]
        assert_allclose(dist_sphere_capsule(sphere, capsule), -1.5)


class TestDistSegmentSegment:
    def test_basic(self):
        s1s = np.array([0.0, 0.0, 0.0])
        s1e = np.array([1.0, 0.0, 0.0])
        s2s = np.array([0.5, 1.0, 0.0])
        s2e = np.array([0.5, 2.0, 0.0])
        assert_allclose(dist_segment_segment(s1s, s1e, s2s, s2e), 1.0)

    def test_parallel(self):
        s1s = np.array([0.0, 0.0, 0.0])
        s1e = np.array([1.0, 0.0, 0.0])
        s2s = np.array([0.0, 2.0, 0.0])
        s2e = np.array([1.0, 2.0, 0.0])
        assert_allclose(dist_segment_segment(s1s, s1e, s2s, s2e), 2.0)

    def test_intersecting(self):
        s1s = np.array([0.0, -1.0, 0.0])
        s1e = np.array([0.0, 1.0, 0.0])
        s2s = np.array([-1.0, 0.0, 0.0])
        s2e = np.array([1.0, 0.0, 0.0])
        assert_allclose(dist_segment_segment(s1s, s1e, s2s, s2e), 0.0, atol=1e-10)

    def test_degenerate_both_points(self):
        p1 = np.array([0.0, 0.0, 0.0])
        p2 = np.array([3.0, 4.0, 0.0])
        assert_allclose(dist_segment_segment(p1, p1, p2, p2), 5.0)

    def test_degenerate_one_point(self):
        p = np.array([0.0, 1.0, 0.0])
        s_start = np.array([0.0, 0.0, 0.0])
        s_end = np.array([1.0, 0.0, 0.0])
        assert_allclose(dist_segment_segment(p, p, s_start, s_end), 1.0)

    def test_degenerate_other_point(self):
        s_start = np.array([0.0, 0.0, 0.0])
        s_end = np.array([1.0, 0.0, 0.0])
        p = np.array([0.5, 2.0, 0.0])
        assert_allclose(dist_segment_segment(s_start, s_end, p, p), 2.0)

    def test_collinear_overlapping(self):
        s1s = np.array([0.0, 0.0, 0.0])
        s1e = np.array([2.0, 0.0, 0.0])
        s2s = np.array([1.0, 0.0, 0.0])
        s2e = np.array([3.0, 0.0, 0.0])
        assert_allclose(dist_segment_segment(s1s, s1e, s2s, s2e), 0.0, atol=1e-10)

    def test_collinear_separated(self):
        s1s = np.array([0.0, 0.0, 0.0])
        s1e = np.array([1.0, 0.0, 0.0])
        s2s = np.array([3.0, 0.0, 0.0])
        s2e = np.array([4.0, 0.0, 0.0])
        assert_allclose(dist_segment_segment(s1s, s1e, s2s, s2e), 2.0)

    def test_skew_3d(self):
        s1s = np.array([0.0, 0.0, 0.0])
        s1e = np.array([1.0, 0.0, 0.0])
        s2s = np.array([0.5, 0.0, 3.0])
        s2e = np.array([0.5, 0.0, 5.0])
        assert_allclose(dist_segment_segment(s1s, s1e, s2s, s2e), 3.0)


class TestDistCapsuleCapsule:
    def test_basic(self):
        c1 = [np.array([0, 0, 0]), np.array([1, 0, 0]), 0.5]
        c2 = [np.array([0, 3, 0]), np.array([1, 3, 0]), 0.5]
        assert_allclose(dist_capsule_capsule(c1, c2), 2.0)

    def test_overlapping(self):
        c1 = [np.array([0, 0, 0]), np.array([1, 0, 0]), 0.5]
        c2 = [np.array([0, 0.5, 0]), np.array([1, 0.5, 0]), 0.5]
        assert_allclose(dist_capsule_capsule(c1, c2), -0.5)
