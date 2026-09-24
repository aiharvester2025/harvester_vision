"""Unit tests for the MID-360 IMU frame conversion.

The conversion is the one place a sign error would tilt the LiDAR cloud instead
of levelling it, so the axis mapping is asserted against synthetic tilts rather
than only checked for shape.
"""

import math
import unittest

import numpy as np

from harvester_dashboard.decoders.livox_imu import (
    LIVOX_SENSOR_FRAME,
    OPTICAL_FRAME,
    sensor_rpy_to_optical,
)
from harvester_dashboard.decoders.imustab import stabilize_points


class SensorRpyConversionTest(unittest.TestCase):
    def test_level_attitude_is_level(self):
        self.assertEqual(sensor_rpy_to_optical([0.0, 0.0, 0.0]), (0.0, 0.0))

    def test_pitch_passes_through(self):
        # A sensor pitch about left/right maps to the optical pitch unchanged.
        roll, pitch = sensor_rpy_to_optical([0.0, 0.2, 0.0])
        self.assertAlmostEqual(roll, 0.0)
        self.assertAlmostEqual(pitch, 0.2)

    def test_roll_is_negated(self):
        # A sensor roll about forward maps to the negative optical roll.
        roll, pitch = sensor_rpy_to_optical([0.3, 0.0, 0.0])
        self.assertAlmostEqual(roll, -0.3)
        self.assertAlmostEqual(pitch, 0.0)

    def test_missing_and_malformed_return_none(self):
        for bad in (None, [], [0.0], 'nope', [float('nan'), 0.0],
                    [0.0, float('inf')]):
            with self.subTest(bad=bad):
                self.assertIsNone(sensor_rpy_to_optical(bad))

    def test_frame_names(self):
        self.assertEqual(LIVOX_SENSOR_FRAME, 'mid360_link')
        self.assertEqual(OPTICAL_FRAME, 'oak_optical_frame')


class ConversionThroughStabilizationTest(unittest.TestCase):
    """A converted attitude must level the cloud when fed to ``imustab``."""

    def test_tilted_cloud_is_levelled(self):
        # Build a cloud in the reference (level) pose, then express it in a
        # tilted pose using imustab's own rotation so the inverse is exact.
        from harvester_dashboard.decoders.imustab import _rpy_to_rotation
        reference = (0.0, 0.0)
        # A physical sensor tilt of roll=+0.2 about the sensor's forward axis.
        sensor_roll = 0.2
        optical_current = sensor_rpy_to_optical([sensor_roll, 0.0, 0.0])
        points_ref = np.array([[0.5, 0.0, 2.0], [1.0, -0.3, 3.0]],
                              dtype=np.float32)
        # The point as seen in the current (rotated) optical frame.
        points_cur = (points_ref @ _rpy_to_rotation(optical_current).T
                      ).astype(np.float32)
        out = stabilize_points(points_cur, optical_current, reference)
        np.testing.assert_allclose(out, points_ref, atol=1e-4)

    def test_conversion_preserves_rotation_only(self):
        attitude = sensor_rpy_to_optical([0.15, -0.1, 0.0])
        rng = np.random.default_rng(1)
        points = rng.normal(size=(200, 3)).astype(np.float32)
        out = stabilize_points(points, attitude, (0.0, 0.0))
        before = np.linalg.norm(points, axis=1)
        after = np.linalg.norm(out, axis=1)
        np.testing.assert_allclose(after, before, atol=1e-4)


if __name__ == '__main__':
    unittest.main()
