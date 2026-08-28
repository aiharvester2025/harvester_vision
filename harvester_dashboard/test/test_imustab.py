"""Unit tests for IMU-based vibration stabilization of the camera cloud."""

import unittest

import numpy as np

from harvester_dashboard.decoders.imustab import (
    gravity_to_rpy,
    tilt_delta_rotation,
    stabilize_points,
)


class GravityToRpyTest(unittest.TestCase):
    def test_at_rest_identity(self):
        # Gravity along -Z in the optical frame -> zero tilt.
        roll, pitch, norm = gravity_to_rpy([0.0, 0.0, -9.80665])
        self.assertAlmostEqual(roll, 0.0, places=6)
        self.assertAlmostEqual(pitch, 0.0, places=6)
        self.assertAlmostEqual(norm, 9.80665, places=5)

    def test_roll_tilt(self):
        # Gravity along +Y (accel +Y) -> body gravity along -Y -> roll of
        # -90 deg about +Z (atan2(-1, 0)).
        roll, pitch, _norm = gravity_to_rpy([0.0, 9.80665, 0.0])
        self.assertAlmostEqual(roll, -np.pi / 2, places=6)
        self.assertAlmostEqual(pitch, 0.0, places=6)

    def test_pitch_tilt(self):
        # A small positive pitch: gravity tips toward +X.  Use a non-degenerate
        # vector (finite +Z) so atan2(0, gz) is well-defined and gimbal-lock at
        # exactly 90 deg is not exercised.
        roll, pitch, _norm = gravity_to_rpy([4.9, 0.0, -8.5])
        self.assertAlmostEqual(roll, 0.0, places=6)
        self.assertGreater(pitch, 0.0)
        self.assertLess(pitch, np.pi / 2)

    def test_zero_norm_returns_zero(self):
        self.assertEqual(gravity_to_rpy([0.0, 0.0, 0.0]), (0.0, 0.0, 0.0))

    def test_wrong_shape_returns_zero(self):
        self.assertEqual(gravity_to_rpy([0.0, 0.0]), (0.0, 0.0, 0.0))


class StabilizePointsTest(unittest.TestCase):
    def test_identity_reference_matches_current(self):
        points = np.array([[1.0, 0.0, 2.0], [0.0, -1.0, 3.0]], dtype=np.float32)
        out = stabilize_points(points, [0.1, -0.2], [0.1, -0.2])
        np.testing.assert_allclose(out, points, atol=1e-5)

    def test_preserves_distance(self):
        rng = np.random.default_rng(0)
        points = rng.normal(size=(500, 3)).astype(np.float32)
        out = stabilize_points(points, [0.3, 0.2], [0.0, 0.0])
        before = np.linalg.norm(points, axis=1)
        after = np.linalg.norm(out, axis=1)
        np.testing.assert_allclose(after, before, atol=1e-4)

    def test_removes_roll(self):
        # A point in the reference (level) frame, then expressed in the
        # current (rolled +0.3 rad about Z) frame via p_cur = R(roll) @ p_ref.
        # Stabilizing back to the reference frame must recover p_ref.
        reference = [0.0, 0.0]
        current = [0.3, 0.0]  # camera rolled +0.3 rad about Z
        p_ref = np.array([[0.0, 1.0, 1.0]], dtype=np.float32)
        from harvester_dashboard.decoders.imustab import _rpy_to_rotation
        p_cur = (_rpy_to_rotation([0.3, 0.0, 0.0]) @ p_ref[0]).astype(
            np.float32).reshape(1, 3)
        out = stabilize_points(p_cur, current, reference)
        np.testing.assert_allclose(out, p_ref, atol=1e-4)

    def test_empty_input_returns_empty(self):
        out = stabilize_points(np.empty((0, 3), dtype=np.float32), [0, 0], [0, 0])
        self.assertEqual(out.shape, (0, 3))

    def test_wrong_shape_returned_unchanged(self):
        out = stabilize_points(np.zeros((5, 4), dtype=np.float32), [0, 0], [0, 0])
        self.assertEqual(out.shape, (5, 4))


class TiltDeltaRotationTest(unittest.TestCase):
    def test_identity_delta_is_identity(self):
        r = tilt_delta_rotation([0.0, 0.0], [0.0, 0.0])
        np.testing.assert_allclose(r, np.eye(3), atol=1e-6)


if __name__ == '__main__':
    unittest.main()
