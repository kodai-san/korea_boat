"""Continuous lookahead targets are independent of waypoint spacing."""

import math
import unittest

from path_guidance import interpolate_lookahead


class ContinuousLookaheadTests(unittest.TestCase):
    def assertPointAlmostEqual(self, actual, expected):
        self.assertAlmostEqual(actual[0], expected[0])
        self.assertAlmostEqual(actual[1], expected[1])

    def test_target_is_interpolated_from_off_path_projection(self):
        index, point = interpolate_lookahead([(0, 0), (0, 10), (0, 20)], 0, 4, 2, 3)
        self.assertEqual(index, 1)
        self.assertPointAlmostEqual(point, (0, 5))

    def test_distance_walks_around_corner_along_path(self):
        index, point = interpolate_lookahead([(0, 0), (0, 5), (10, 5)], 0, -2, 3, 5)
        self.assertEqual(index, 2)
        self.assertPointAlmostEqual(point, (3, 5))

    def test_diagonal_segment_projects_and_interpolates(self):
        # Projection of (1,2) onto the x=y line is (1.5,1.5).
        index, point = interpolate_lookahead([(0, 0), (10, 10)], 0, 1, 2, math.sqrt(2))
        self.assertEqual(index, 1)
        self.assertPointAlmostEqual(point, (2.5, 2.5))

    def test_sparse_and_dense_waypoints_produce_same_target(self):
        sparse = [(0, 0), (0, 10), (10, 10)]
        dense = [(0, y) for y in range(11)] + [(x, 10) for x in range(1, 11)]
        _, first = interpolate_lookahead(sparse, 0, 1, 8.4, 5)
        _, second = interpolate_lookahead(dense, 8, 1, 8.4, 5)
        self.assertPointAlmostEqual(first, second)
        self.assertPointAlmostEqual(first, (3.4, 10))

    def test_target_is_continuous_across_waypoint_transition(self):
        points = [(0, 0), (0, 5), (5, 5)]
        _, before = interpolate_lookahead(points, 0, 0, 4.999, 2)
        _, after = interpolate_lookahead(points, 1, .001, 5, 2)
        self.assertLess(math.hypot(after[0] - before[0], after[1] - before[1]), .003)
        self.assertPointAlmostEqual(before, (1.999, 5))
        self.assertPointAlmostEqual(after, (2.001, 5))

    def test_projection_clamps_before_and_after_segment(self):
        points = [(0, 0), (0, 5), (0, 10)]
        self.assertPointAlmostEqual(interpolate_lookahead(points, 0, 2, -5, 2)[1], (0, 2))
        self.assertPointAlmostEqual(interpolate_lookahead(points, 0, 2, 8, 2)[1], (0, 7))

    def test_zero_distance_and_exact_waypoint_distance(self):
        points = [(0, 0), (0, 5), (0, 10)]
        self.assertEqual(interpolate_lookahead(points, 0, 2, 3, 0), (1, (0., 3.)))
        self.assertEqual(interpolate_lookahead(points, 0, 2, 3, 2), (1, (0., 5.)))

    def test_duplicate_waypoints_do_not_consume_distance(self):
        points = [(0, 0), (0, 0), (0, 5), (0, 5), (5, 5)]
        self.assertEqual(interpolate_lookahead(points, 0, 0, 0, 7), (4, (2., 5.)))
        self.assertEqual(interpolate_lookahead([(1, 2)] * 4, 1, 0, 0, 5), (3, (1., 2.)))

    def test_end_of_path_singleton_and_clamped_segment_indices(self):
        points = [(0, 0), (0, 5), (0, 10)]
        self.assertEqual(interpolate_lookahead(points, 1, 0, 8, 100), (2, (0., 10.)))
        self.assertEqual(interpolate_lookahead([(1, 2)], 50, 0, 0, 5), (0, (1., 2.)))
        self.assertEqual(interpolate_lookahead(points, -50, 0, 1, 1), (1, (0., 2.)))
        self.assertEqual(interpolate_lookahead(points, 50, 0, 8, 1), (2, (0., 9.)))

    def test_invalid_inputs_fail_explicitly(self):
        calls = [([], 0, 0, 0, 1), ([(0, 0)], .5, 0, 0, 1),
                 ([(0, 0)], True, 0, 0, 1), ([(0, 0)], 0, 0, 0, -1),
                 ([(0, 0)], 0, 0, 0, float("inf")),
                 ([(0, float("nan"))], 0, 0, 0, 1),
                 ([(0, 0)], 0, float("nan"), 0, 1),
                 ([(0, 0, 0)], 0, 0, 0, 1)]
        for args in calls:
            with self.subTest(args=args), self.assertRaises(ValueError):
                interpolate_lookahead(*args)


if __name__ == "__main__":
    unittest.main()
