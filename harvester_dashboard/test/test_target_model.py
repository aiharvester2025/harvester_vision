import unittest

import numpy as np

from helpers import base_header

from harvester_dashboard.model.target_model import AnnotationState, back_project
from harvester_dashboard.decoders.pointcloud import unproject_depth


class FakeClock:
    def __init__(self, start=50.0):
        self.now = start

    def __call__(self):
        return self.now


CAMERA_INFO = {'width': 64, 'height': 48, 'k': [
    100.0, 0.0, 32.0,
    0.0, 100.0, 24.0,
    0.0, 0.0, 1.0,
]}


class BackProjectTest(unittest.TestCase):
    def test_principal_point_maps_forward(self):
        x, y, z = back_project(32, 24, 2.0, CAMERA_INFO)
        self.assertAlmostEqual(x, 0.0)
        self.assertAlmostEqual(y, 0.0)
        self.assertAlmostEqual(z, 2.0)

    def test_offset_pixel_scales_by_focal_length(self):
        x, y, z = back_project(42, 14, 1.0, CAMERA_INFO)
        self.assertAlmostEqual(x, (42 - 32) / 100.0)
        self.assertAlmostEqual(y, (14 - 24) / 100.0)
        self.assertAlmostEqual(z, 1.0)

    def test_missing_intrinsics_return_none(self):
        self.assertIsNone(back_project(0, 0, 1.0, None))
        self.assertIsNone(back_project(0, 0, 1.0, {'k': [1, 2]}))
        self.assertIsNone(back_project(0, 0, 1.0, {'k': [0.0] * 9}))

    def test_unproject_matches_back_project(self):
        # The point-cloud unprojection must agree with the annotation
        # back-projection for the same pixel + depth + intrinsics.
        depth = np.full((48, 64), 2.0, dtype=np.float32)
        rgb = np.zeros((48, 64, 3), dtype=np.uint8)
        cloud = unproject_depth(depth, rgb, CAMERA_INFO, max_points=100000)
        # Find the point for pixel (42, 14).
        xs, ys, zs = cloud['points'][:, 0], cloud['points'][:, 1], cloud['points'][:, 2]
        # Reconstruct which pixel each point came from via the inverse mapping.
        # Instead of matching by index, verify the (u,v)->point math directly:
        # pick the pixel (42,14), its expected (x,y,z).
        expected = back_project(42, 14, 2.0, CAMERA_INFO)
        # The cloud contains exactly one point per pixel; locate it by its x,y.
        match = np.argmin(np.abs(xs - expected[0]) + np.abs(ys - expected[1]))
        self.assertAlmostEqual(float(xs[match]), expected[0], places=5)
        self.assertAlmostEqual(float(ys[match]), expected[1], places=5)
        self.assertAlmostEqual(float(zs[match]), expected[2], places=5)


class AnnotationStateTest(unittest.TestCase):
    def setUp(self):
        self.clock = FakeClock()
        self.annotation = AnnotationState(clock=self.clock)

    def test_build_accepts_valid_depth(self):
        accepted, message = self.annotation.build(
            'cutter', 40, 20, 2.0, CAMERA_INFO, frame_id='optical_frame')
        self.assertTrue(accepted)
        self.assertTrue(self.annotation.active)
        x, y, z = self.annotation.point_camera
        self.assertAlmostEqual(x, (40 - 32) / 100.0 * 2.0)
        self.assertAlmostEqual(y, (20 - 24) / 100.0 * 2.0)
        self.assertAlmostEqual(z, 2.0)
        self.assertIn('2.00 m', self.annotation.label())

    def test_no_depth_rejected_without_state(self):
        accepted, message = self.annotation.build(
            'cutter', 10, 10, None, CAMERA_INFO)
        self.assertFalse(accepted)
        self.assertIn('NO DEPTH', message)
        self.assertFalse(self.annotation.active)
        self.assertIsNone(self.annotation.point_camera)

    def test_zero_and_nan_depth_rejected(self):
        for depth in (0.0, float('nan')):
            accepted, _message = self.annotation.build(
                'cutter', 10, 10, depth, CAMERA_INFO)
            self.assertFalse(accepted)

    def test_clear_resets_state(self):
        self.annotation.build('docking', 5, 5, 1.0, CAMERA_INFO)
        self.annotation.clear(reason='esc')
        self.assertFalse(self.annotation.active)
        self.assertEqual(self.annotation.pixel, (0, 0))
        self.assertIsNone(self.annotation.depth_m)

    def test_never_claims_world_fixed(self):
        self.annotation.build('cutter', 5, 5, 1.0, CAMERA_INFO)
        self.assertFalse(self.annotation.is_world_fixed_claimed())

    def test_events_logged_in_app(self):
        self.annotation.build('cutter', 5, 5, 1.0, CAMERA_INFO)
        self.annotation.clear()
        self.assertTrue(any('cleared' in event for event in self.annotation.events))

    def test_backproject_uses_depth_pixel_not_rgb_pixel(self):
        # When depth is delivered at half the RGB resolution, the back-projected
        # 3-D point must use the depth-map pixel (du, dv), while the crosshair
        # pixel stays at the RGB pixel the operator clicked (u, v).
        accepted, _message = self.annotation.build(
            'cutter', u=80, v=60, depth_m=2.0, camera_info=CAMERA_INFO,
            frame_id='f', backproject_u=40, backproject_v=30)
        self.assertTrue(accepted)
        # Crosshair stays at the RGB pixel.
        self.assertEqual(self.annotation.pixel, (80, 60))
        # 3-D point is back-projected from the depth pixel (40, 30).
        x, y, z = self.annotation.point_camera
        self.assertAlmostEqual(x, (40 - 32) / 100.0 * 2.0)
        self.assertAlmostEqual(y, (30 - 24) / 100.0 * 2.0)
        self.assertAlmostEqual(z, 2.0)

    def test_backproject_defaults_to_rgb_pixel(self):
        # With no explicit backproject pixel, back-project uses (u, v) itself
        # (the pre-remap behaviour, for pixel-aligned depth).
        self.annotation.build('cutter', 42, 14, 1.0, CAMERA_INFO)
        x, y, z = self.annotation.point_camera
        self.assertAlmostEqual(x, (42 - 32) / 100.0)
        self.assertAlmostEqual(y, (14 - 24) / 100.0)
        self.assertAlmostEqual(z, 1.0)


if __name__ == '__main__':
    unittest.main()
