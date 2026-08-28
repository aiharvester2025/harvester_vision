"""Unit tests for the depth -> camera-frame point cloud derivation."""

import unittest

import numpy as np

from harvester_dashboard.decoders.pointcloud import unproject_depth
from harvester_dashboard.decoders.imustab import stabilize_points


def _camera_info(width=8, height=6, fx=500.0, fy=500.0, cx=4.0, cy=3.0):
    return {
        'width': width, 'height': height,
        'distortion_model': 'plumb_bob',
        'd': [0.0] * 5,
        'k': [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0],
        'r': [1.0] * 9,
        'p': [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0],
        'binning_x': 1, 'binning_y': 1,
        'roi': {'x_offset': 0, 'y_offset': 0, 'height': height,
                'width': width, 'do_rectify': False},
    }


class UnprojectDepthTest(unittest.TestCase):
    def test_constant_depth_matches_optical_distance(self):
        depth = np.full((6, 8), 1.0, dtype=np.float32)
        rgb = np.zeros((6, 8, 3), dtype=np.uint8)
        cloud = unproject_depth(depth, rgb, _camera_info(), max_points=1000)
        self.assertEqual(cloud['points'].shape, (48, 3))
        self.assertEqual(cloud['colors'].shape, (48, 3))
        # z is the optical-axis depth; x/y are small offsets from the optical
        # center.  Every point's Euclidean distance from the origin should be
        # ~1.0 m (the depth), not larger.
        dist = np.linalg.norm(cloud['points'], axis=1)
        np.testing.assert_allclose(dist, np.ones(48), atol=1e-3)

    def test_invalid_depth_pixels_are_absent(self):
        depth = np.full((6, 8), 2.0, dtype=np.float32)
        depth[0, 0] = 0.0     # zero -> invalid
        depth[1, 1] = np.nan  # NaN -> invalid
        rgb = np.zeros((6, 8, 3), dtype=np.uint8)
        cloud = unproject_depth(depth, rgb, _camera_info(), max_points=1000)
        # 48 valid pixels minus the two invalid ones.
        self.assertEqual(cloud['points'].shape, (46, 3))

    def test_missing_depth_returns_empty(self):
        cloud = unproject_depth(None, None, _camera_info(), max_points=100)
        self.assertEqual(cloud['points'].shape, (0, 3))
        self.assertEqual(cloud['colors'].shape, (0, 3))

    def test_missing_intrinsics_returns_empty(self):
        depth = np.full((6, 8), 1.0, dtype=np.float32)
        cloud = unproject_depth(depth, None, None, max_points=100)
        self.assertEqual(cloud['points'].shape, (0, 3))

    def test_downsampling_caps_points(self):
        depth = np.full((16, 16), 2.5, dtype=np.float32)
        rgb = np.zeros((16, 16, 3), dtype=np.uint8)
        cloud = unproject_depth(depth, rgb, _camera_info(16, 16, 500, 500, 8, 8),
                                max_points=50)
        self.assertLessEqual(len(cloud['points']), 50)

    def test_missing_rgb_returns_points_with_empty_colors(self):
        depth = np.full((6, 8), 1.0, dtype=np.float32)
        cloud = unproject_depth(depth, None, _camera_info(), max_points=1000)
        self.assertEqual(cloud['points'].shape, (48, 3))
        self.assertEqual(cloud['colors'].shape, (48, 3))

    def test_rgb_colors_sampled(self):
        depth = np.full((2, 2), 1.0, dtype=np.float32)
        rgb = np.array([[[255, 0, 0], [0, 255, 0]],
                        [[0, 0, 255], [255, 255, 0]]], dtype=np.uint8)
        cloud = unproject_depth(depth, rgb, _camera_info(2, 2, 500, 500, 1, 1),
                                max_points=100)
        self.assertEqual(cloud['colors'].shape, (4, 3))
        # The center pixel's colour must be preserved.
        self.assertEqual(list(cloud['colors'][0]), [255, 0, 0])

    def test_colors_mapped_when_rgb_is_larger(self):
        # Depth at 2x2, RGB at 4x4 (2x scale): a depth pixel (v,u) must sample
        # the RGB pixel (2v, 2u).
        depth = np.full((2, 2), 1.0, dtype=np.float32)
        rgb = np.zeros((4, 4, 3), dtype=np.uint8)
        rgb[2, 2] = [10, 20, 30]   # depth pixel (1,1) -> rgb (2,2)
        rgb[0, 0] = [200, 0, 0]    # depth pixel (0,0) -> rgb (0,0)
        info = _camera_info(2, 2, 500, 500, 1, 1)
        cloud = unproject_depth(depth, rgb, info, max_points=100,
                                rgb_width=4, rgb_height=4)
        # depth pixel (1,1) is the 4th point (row-major order of nonzero).
        # Its sampled colour must be [10,20,30].
        points = cloud['points']
        colors = cloud['colors']
        # Find the point whose depth pixel was (1,1): x,y from (1,1) with
        # cx=cy=1 -> x=y=0 (center of a 2x2 image).
        center_idx = None
        for i, (x, y, z) in enumerate(points):
            if abs(x) < 1e-6 and abs(y) < 1e-6:
                center_idx = i
                break
        self.assertIsNotNone(center_idx)
        self.assertEqual(list(colors[center_idx]), [10, 20, 30])

    def test_unproject_then_stabilize_preserves_distance(self):
        # A constant-depth cloud stabilized under a synthetic tilt must still
        # have every point at its original Euclidean distance (rotation only).
        depth = np.full((6, 8), 1.0, dtype=np.float32)
        rgb = np.zeros((6, 8, 3), dtype=np.uint8)
        cloud = unproject_depth(depth, rgb, _camera_info(), max_points=1000)
        before = np.linalg.norm(cloud['points'], axis=1)
        cloud['points'] = stabilize_points(
            cloud['points'], [0.25, -0.15], [0.0, 0.0])
        after = np.linalg.norm(cloud['points'], axis=1)
        np.testing.assert_allclose(after, before, atol=1e-4)


if __name__ == '__main__':
    unittest.main()
