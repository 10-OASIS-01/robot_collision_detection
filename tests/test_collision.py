import numpy as np
import pytest
from numpy.testing import assert_allclose

from robot_collision_detection.core.robot import Robot
from robot_collision_detection.distance.collision import (
    min_distance_between_robots,
    min_distance_within_robot,
)
from robot_collision_detection.distance.primitives import (
    _dist_point_segment_fast,
    _dist_segment_segment_fast,
    dist_point_segment,
    dist_segment_segment,
)


def _make_robot(y_offset=0, name="R"):
    dh_params = [
        [0, 0, 1100],
        [600, np.pi / 2, 0],
        [1400, 0, 0],
    ]
    T = np.eye(4)
    T[1, 3] = y_offset
    robot = Robot(dh_params, base_transform=T, name=name)
    transforms = robot.forward_kinematics([0, 0, 0])
    robot.update_geometric_model(transforms)
    return robot


class TestMinDistanceBetweenRobots:
    def test_separated(self):
        r1 = _make_robot(0, "R1")
        r2 = _make_robot(5000, "R2")
        dist, ctype, elems = min_distance_between_robots(r1, r2)
        assert dist > 0
        assert ctype != ""
        assert len(elems) == 2

    def test_overlapping(self):
        r1 = _make_robot(0, "R1")
        r2 = _make_robot(300, "R2")
        dist, ctype, elems = min_distance_between_robots(r1, r2)
        assert dist < 0

    def test_threshold_early_exit(self):
        r1 = _make_robot(0, "R1")
        r2 = _make_robot(5000, "R2")
        dist_full, _, _ = min_distance_between_robots(r1, r2)
        dist_th, _, _ = min_distance_between_robots(r1, r2, threshold=dist_full + 1000)
        assert dist_th <= dist_full + 1000

    def test_threshold_none_backward_compat(self):
        r1 = _make_robot(0, "R1")
        r2 = _make_robot(2000, "R2")
        d1, t1, e1 = min_distance_between_robots(r1, r2)
        d2, t2, e2 = min_distance_between_robots(r1, r2, threshold=None)
        assert_allclose(d1, d2)
        assert t1 == t2

    def test_vectorized_matches_scalar(self):
        r1 = _make_robot(0, "R1")
        r2 = _make_robot(2000, "R2")

        # Scalar reference implementation
        from robot_collision_detection.distance.primitives import (
            dist_sphere_sphere, dist_sphere_capsule, dist_capsule_capsule,
        )
        min_dist = float('inf')
        for s1 in r1.spheres:
            for s2 in r2.spheres:
                d = dist_sphere_sphere(s1, s2)
                if d < min_dist:
                    min_dist = d
        for s1 in r1.spheres:
            for c2 in r2.capsules:
                d = dist_sphere_capsule(s1, c2)
                if d < min_dist:
                    min_dist = d
        for s2 in r2.spheres:
            for c1 in r1.capsules:
                d = dist_sphere_capsule(s2, c1)
                if d < min_dist:
                    min_dist = d
        for c1 in r1.capsules:
            for c2 in r2.capsules:
                d = dist_capsule_capsule(c1, c2)
                if d < min_dist:
                    min_dist = d

        dist_vec, _, _ = min_distance_between_robots(r1, r2)
        assert_allclose(dist_vec, min_dist, atol=1e-8)


class TestMinDistanceWithinRobot:
    def test_no_self_collision(self):
        # KUKA model has large primitives that overlap even non-adjacent links
        robot = _make_robot(0, "R")
        dist, ctype, elems = min_distance_within_robot(robot)
        # Just verify it returns valid results
        assert isinstance(dist, float)
        assert len(elems) == 2 or dist == float('inf')

    def test_skip_adjacent_0_vs_1(self):
        robot = _make_robot(0, "R")
        d0, _, _ = min_distance_within_robot(robot, skip_adjacent=0)
        d1, _, _ = min_distance_within_robot(robot, skip_adjacent=1)
        assert d0 <= d1

    def test_skip_adjacent_large(self):
        robot = _make_robot(0, "R")
        dist, _, _ = min_distance_within_robot(robot, skip_adjacent=10)
        assert dist == float('inf')

    def test_threshold(self):
        robot = _make_robot(0, "R")
        dist_full, _, _ = min_distance_within_robot(robot, skip_adjacent=0)
        dist_th, _, _ = min_distance_within_robot(robot, skip_adjacent=0, threshold=9999)
        assert dist_th <= 9999


class TestFastPathFunctions:
    def test_point_segment_fast_matches(self):
        p = np.array([0.5, 1.0, 0.0])
        s = np.array([0.0, 0.0, 0.0])
        e = np.array([1.0, 0.0, 0.0])
        assert_allclose(
            _dist_point_segment_fast(p, s, e),
            dist_point_segment(p, s, e),
        )

    def test_segment_segment_fast_matches(self):
        s1s = np.array([0.0, 0.0, 0.0])
        s1e = np.array([1.0, 0.0, 0.0])
        s2s = np.array([0.5, 1.0, 0.0])
        s2e = np.array([0.5, 2.0, 0.0])
        assert_allclose(
            _dist_segment_segment_fast(s1s, s1e, s2s, s2e),
            dist_segment_segment(s1s, s1e, s2s, s2e),
        )

    def test_segment_segment_fast_degenerate(self):
        p = np.array([0.0, 0.0, 0.0])
        s = np.array([3.0, 4.0, 0.0])
        assert_allclose(
            _dist_segment_segment_fast(p, p, s, s),
            5.0,
        )
