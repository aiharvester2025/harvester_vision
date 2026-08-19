import math
import unittest

from lidar.boom_kinematics import (
    BoomGeometry,
    BoomState,
    height_of_point_above_ground,
    lidar_height_above_ground,
)


class BoomKinematicsTests(unittest.TestCase):
    def setUp(self):
        self.geometry = BoomGeometry(
            pivot_height_m=1.0,
            boom_stage0_length_m=2.0,
            platform_level_offset_m=0.4,
            cutting_arm_lift_offset_m=0.3,
        )

    def test_horizontal_boom_height_is_pivot_plus_offsets(self):
        # Boom horizontal (pivot=0): no sin() contribution, only fixed offsets.
        state = BoomState(pivot_angle_rad=0.0, extension_m=0.0,
                          platform_pitch_rad=0.0, platform_roll_rad=0.0)
        expected = 1.0 + 0.0 + 0.4 + 0.3
        self.assertAlmostEqual(lidar_height_above_ground(state, self.geometry),
                               expected, places=7)

    def test_vertical_boom_adds_full_length(self):
        # Boom vertical (pivot=90deg): sin=1, full boom length contributes.
        state = BoomState(pivot_angle_rad=math.pi / 2, extension_m=1.0,
                          platform_pitch_rad=0.0, platform_roll_rad=0.0)
        expected = 1.0 + (2.0 + 1.0) * 1.0 + 0.4 + 0.3
        self.assertAlmostEqual(lidar_height_above_ground(state, self.geometry),
                               expected, places=7)

    def test_partial_pivot_uses_sine(self):
        state = BoomState(pivot_angle_rad=math.pi / 6, extension_m=0.0,
                          platform_pitch_rad=0.0, platform_roll_rad=0.0)
        # sin(30deg) = 0.5
        expected = 1.0 + 2.0 * 0.5 + 0.4 + 0.3
        self.assertAlmostEqual(lidar_height_above_ground(state, self.geometry),
                               expected, places=7)

    def test_extension_increases_height_through_sine(self):
        short = BoomState(pivot_angle_rad=math.pi / 6, extension_m=0.0,
                          platform_pitch_rad=0.0, platform_roll_rad=0.0)
        long = BoomState(pivot_angle_rad=math.pi / 6, extension_m=1.0,
                         platform_pitch_rad=0.0, platform_roll_rad=0.0)
        short_h = lidar_height_above_ground(short, self.geometry)
        long_h = lidar_height_above_ground(long, self.geometry)
        # Extension of 1 m at 30 deg adds 0.5 m vertically.
        self.assertAlmostEqual(long_h - short_h, 0.5, places=7)

    def test_point_height_conversion(self):
        lidar_h = lidar_height_above_ground(
            BoomState(pivot_angle_rad=math.pi / 2, extension_m=0.0,
                      platform_pitch_rad=0.0, platform_roll_rad=0.0),
            self.geometry,
        )
        point_z = 1.5  # trunk-end is 1.5 m above the LiDAR in leveled frame
        self.assertAlmostEqual(
            height_of_point_above_ground(point_z, lidar_h), lidar_h + 1.5, places=7)


if __name__ == "__main__":
    unittest.main()
