import math
import unittest

from geometry.transforms import (
    TransformConfigurationError,
    level_points_rotation_only,
    rotation_from_quaternion,
    rotation_from_rpy,
)
from lidar.leveling import (
    OrientationSource,
    level_from_tilt_rpy,
    level_orientation_quaternion,
    quaternion_from_euler,
)


class LevelingTests(unittest.TestCase):
    def test_identity_quaternion_is_identity_rotation(self):
        r = rotation_from_quaternion(0.0, 0.0, 0.0, 1.0)
        self.assertEqual(r, ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0)))

    def test_quaternion_is_normalised(self):
        # (2, 0, 0, 0) has norm 2; should normalise to a 180-degree x-rotation.
        r = rotation_from_quaternion(2.0, 0.0, 0.0, 0.0)
        # A 180-degree rotation about x maps (0,0,1) -> (0,0,-1).
        x, y, z = level_points_rotation_only([(0.0, 0.0, 1.0)], r)[0]
        self.assertAlmostEqual(x, 0.0, places=6)
        self.assertAlmostEqual(y, 0.0, places=6)
        self.assertAlmostEqual(z, -1.0, places=6)

    def test_leveling_undoes_sensor_pitch(self):
        # A tree is world-vertical at (0,0,5). After the sensor pitches up by
        # +theta about y, the tree appears rotated in the sensor frame. Applying
        # the sensor->gravity rotation recovers world-vertical.
        theta = 0.5
        sensor_to_world = rotation_from_rpy([0.0, theta, 0.0])
        # Sensor-frame coordinates of the world-vertical point (inverse rotation):
        world = (0.0, 0.0, 5.0)
        rt = tuple(tuple(sensor_to_world[j][i] for j in range(3)) for i in range(3))
        sensor = tuple(sum(rt[i][j] * world[j] for j in range(3)) for i in range(3))
        leveled = level_points_rotation_only([sensor], sensor_to_world)[0]
        for a, b in zip(leveled, world):
            self.assertAlmostEqual(a, b, places=6)

    def test_leveling_keeps_sensor_at_origin(self):
        # Rotation-only must not translate: the origin stays put.
        r = rotation_from_rpy([0.1, 0.2, 0.3])
        out = level_points_rotation_only([(0.0, 0.0, 0.0)], r)[0]
        self.assertEqual(out, (0.0, 0.0, 0.0))

    def test_invalid_orientation_source_raises(self):
        source = OrientationSource("imu", (0.0, 0.0, 0.0, 1.0), valid=False)
        with self.assertRaises(TransformConfigurationError):
            level_orientation_quaternion(source)

    def test_quaternion_from_euler_matches_rotation_from_rpy(self):
        roll, pitch, yaw = 0.1, 0.2, 0.3
        q = quaternion_from_euler(roll, pitch, yaw)
        r_q = rotation_from_quaternion(*q)
        r_rpy = rotation_from_rpy([roll, pitch, yaw])
        for row_q, row_r in zip(r_q, r_rpy):
            for a, b in zip(row_q, row_r):
                self.assertAlmostEqual(a, b, places=6)

    def test_level_from_tilt_rpy_uses_no_yaw(self):
        # A pure roll of +90 degrees about x maps (0,0,1) -> (0,-1,0).
        out = level_from_tilt_rpy([(0.0, 0.0, 1.0)], math.pi / 2, 0.0)
        x, y, z = out[0]
        self.assertAlmostEqual(x, 0.0, places=6)
        self.assertAlmostEqual(y, -1.0, places=6)
        self.assertAlmostEqual(z, 0.0, places=6)


if __name__ == "__main__":
    unittest.main()
