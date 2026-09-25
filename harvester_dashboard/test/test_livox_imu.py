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
    quaternion_to_rpy,
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


class QuaternionConversionTest(unittest.TestCase):
    """The ROS-style ``orientation`` shape in the tree_scan_002 recordings."""

    def test_identity_is_zero_rpy(self):
        rpy = quaternion_to_rpy({'x': 0.0, 'y': 0.0, 'z': 0.0, 'w': 1.0})
        for value in rpy:
            self.assertAlmostEqual(value, 0.0)

    def test_sequence_form_matches_mapping(self):
        mapping = {'x': 0.1, 'y': 0.2, 'z': 0.3, 'w': 0.9}
        seq = [0.1, 0.2, 0.3, 0.9]
        self.assertEqual(quaternion_to_rpy(mapping), quaternion_to_rpy(seq))

    def test_small_roll_about_x(self):
        # A rotation of angle a about X gives roll ~= a, pitch ~= 0.
        a = 0.2
        rpy = quaternion_to_rpy({'x': math.sin(a / 2), 'y': 0.0, 'z': 0.0,
                                 'w': math.cos(a / 2)})
        self.assertAlmostEqual(rpy[0], a, places=5)
        self.assertAlmostEqual(rpy[1], 0.0, places=5)

    def test_missing_or_bad_returns_none(self):
        for bad in (None, {}, {'x': 1.0}, 'nope', [0.0, 0.0],
                    {'x': float('nan'), 'y': 0, 'z': 0, 'w': 1},
                    {'x': 0.0, 'y': 0.0, 'z': 0.0, 'w': 0.0}):
            with self.subTest(bad=bad):
                self.assertIsNone(quaternion_to_rpy(bad))

    def test_recorded_style_orientation_feeds_the_conversion_chain(self):
        # The chain the bridge runs: recorded quaternion -> rpy -> optical.
        rpy = quaternion_to_rpy(
            {'x': 3.95e-13, 'y': -0.000769, 'z': -9.6e-09, 'w': 0.9999997})
        self.assertIsNotNone(rpy)
        optical = sensor_rpy_to_optical(rpy)
        self.assertIsNotNone(optical)
        self.assertAlmostEqual(optical[0], -rpy[0])   # roll negated
        self.assertAlmostEqual(optical[1], rpy[1])    # pitch unchanged


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
