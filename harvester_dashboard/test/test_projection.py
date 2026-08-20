"""Tests for the LiDAR orthographic projection used by the HUD."""

import unittest

from harvester_dashboard.projection import (
    AXIS_LABELS,
    VIEW_LABELS,
    VIEWS,
    project_points,
)


class ProjectionTest(unittest.TestCase):

    def setUp(self):
        self.cx, self.cy, self.scale = 100.0, 100.0, 10.0
        # A point forward+left+up of the vehicle.
        self.fwd_left_up = [2.0, 1.0, 0.5]
        # A point behind+right+down.
        self.aft_right_down = [-1.0, -2.0, -0.5]

    def test_view_cycle_order(self):
        self.assertEqual(VIEWS, ('top', 'front', 'left', 'right', 'iso'))
        self.assertEqual(len(VIEW_LABELS), len(VIEWS))
        self.assertEqual(len(AXIS_LABELS), len(VIEWS))

    def test_top_view_forward_maps_right(self):
        out = project_points([self.fwd_left_up], 'top', self.cx, self.cy, self.scale)
        sx, sy, _ = out[0]
        self.assertEqual(sx, self.cx + 2.0 * self.scale)   # +x -> right
        self.assertEqual(sy, self.cy - 1.0 * self.scale)   # +y -> up

    def test_top_view_behind_maps_left_down(self):
        out = project_points([self.aft_right_down], 'top', self.cx, self.cy, self.scale)
        sx, sy, _ = out[0]
        self.assertEqual(sx, self.cx - 1.0 * self.scale)
        self.assertEqual(sy, self.cy + 2.0 * self.scale)

    def test_front_view_left_maps_right(self):
        # In a front view, the vehicle's +y (left) should appear on the
        # right of the screen (because the viewer faces -x).
        out = project_points([self.fwd_left_up], 'front', self.cx, self.cy, self.scale)
        sx, sy, _ = out[0]
        self.assertEqual(sx, self.cx + 1.0 * self.scale)   # +y -> right
        self.assertEqual(sy, self.cy - 0.5 * self.scale)   # +z -> up

    def test_front_view_is_yz_not_xz(self):
        # The bug the user reported: front view was showing x as horizontal,
        # which made it look like a side profile.  Confirm the fix uses y.
        out = project_points([self.fwd_left_up], 'front', self.cx, self.cy, self.scale)
        sx, _sy, _r = out[0]
        # +x (2) should NOT affect screen-x in a front view.
        self.assertNotEqual(sx, self.cx + 2.0 * self.scale)

    def test_left_view_forward_maps_right(self):
        # In a left-side view, the vehicle's +x (forward) should appear on
        # the right of the screen (because the viewer faces -y).
        out = project_points([self.fwd_left_up], 'left', self.cx, self.cy, self.scale)
        sx, sy, _ = out[0]
        self.assertEqual(sx, self.cx + 2.0 * self.scale)   # +x -> right
        self.assertEqual(sy, self.cy - 0.5 * self.scale)   # +z -> up

    def test_right_view_forward_maps_left(self):
        # In a right-side view, the vehicle's +x (forward) should appear on
        # the left of the screen (because the viewer faces +y).
        out = project_points([self.fwd_left_up], 'right', self.cx, self.cy, self.scale)
        sx, sy, _ = out[0]
        self.assertEqual(sx, self.cx - 2.0 * self.scale)   # +x -> left
        self.assertEqual(sy, self.cy - 0.5 * self.scale)   # +z -> up

    def test_left_and_right_are_mirrored(self):
        # The two side views must be exact mirrors: a point at (x, z) in the
        # left view sits at (-x, z) in the right view.
        l = project_points([self.fwd_left_up], 'left', self.cx, self.cy, self.scale)[0]
        r = project_points([self.fwd_left_up], 'right', self.cx, self.cy, self.scale)[0]
        self.assertEqual(l[1], r[1])            # same screen-y
        self.assertAlmostEqual(l[0] + r[0], 2 * self.cx)  # mirrored screen-x

    def test_front_and_back_views_differ(self):
        # The previous bug had front and back rendering identically because
        # both used x as horizontal.  Now front uses y, so the projection
        # screen-x must change when y changes (independent of x).
        a_screen = project_points(
            [[0.0, 1.0, 0.0]], 'front', self.cx, self.cy, self.scale)[0][:2]
        b_screen = project_points(
            [[1.0, 1.0, 0.0]], 'front', self.cx, self.cy, self.scale)[0][:2]
        self.assertEqual(a_screen, b_screen)   # x has no effect on screen-x
        c_screen = project_points(
            [[0.0, 2.0, 0.0]], 'front', self.cx, self.cy, self.scale)[0][:2]
        self.assertNotEqual(a_screen, c_screen)   # y DOES affect screen-x

    def test_range_is_returned(self):
        out = project_points([self.fwd_left_up], 'top', self.cx, self.cy, self.scale)
        _sx, _sy, rng = out[0]
        expected = (4.0 + 1.0 + 0.25) ** 0.5
        self.assertAlmostEqual(rng, expected)

    def test_empty_input(self):
        self.assertEqual(project_points([], 'top', 0, 0, 1), [])
        self.assertEqual(project_points(None, 'top', 0, 0, 1), [])

    def test_unknown_view_falls_through_to_iso(self):
        # A typo in the HUD should not crash; the worst case is iso.
        out = project_points([self.fwd_left_up], 'isometric', 0, 0, 1)
        self.assertEqual(len(out), 1)


if __name__ == '__main__':
    unittest.main()