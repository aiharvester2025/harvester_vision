"""Tests for the encoder-free tree/boom estimate used by the scan HUD.

The acceptance numbers are taken from the validated ``ros2_ws`` plans so this
dashboard copy cannot silently drift from the simulation-proven math.
"""

import math
import unittest

import numpy as np

from harvester_dashboard.model.scan_estimate import (
    BOOM_FIXED_LENGTH_M,
    BOOM_PIVOT_WORLD_Z_M,
    EXTENSION_STROKE_M,
    PLATFORM_TAIL_M,
    BoomTarget,
    ScanEstimateConfig,
    crown_base,
    estimate_tree_height,
    estimate_trunk_axis,
    plan_scan,
    solve_boom_target,
    split_extension,
    trunk_top,
)


def _synthetic_tree(trunk_top_z=12.0, crown_base_z=9.2, axis=(0.0, 0.0),
                    trunk_r=0.25, n_trunk=400, n_canopy=600, ground=0.0,
                    top_offset=(0.05, -0.04), missing_band=None):
    """Build a synthetic vertical-trunk + outward-frond cloud (world frame)."""
    rng = np.random.default_rng(0)
    z = rng.uniform(1.0, trunk_top_z, size=n_trunk)
    theta = rng.uniform(0.0, 2.0 * math.pi, size=n_trunk)
    r = rng.uniform(0.0, trunk_r, size=n_trunk)
    trunk = np.stack([axis[0] + r * np.cos(theta),
                      axis[1] + r * np.sin(theta),
                      z + ground], axis=1)
    # The tight-cylinder top: a couple of returns right at the trunk top, with a
    # small lateral offset so the 99.9th percentile differs slightly from max.
    trunk = np.vstack([trunk, [axis[0] + top_offset[0], axis[1] + top_offset[1],
                               trunk_top_z + ground]])
    # Canopy fronds grow OUTWARD from the crown base, never above the trunk top.
    theta_c = rng.uniform(0.0, 2.0 * math.pi, size=n_canopy)
    r_c = rng.uniform(0.4, 3.0, size=n_canopy)
    z_c = rng.uniform(crown_base_z, trunk_top_z - 0.3, size=n_canopy)
    canopy = np.stack([axis[0] + r_c * np.cos(theta_c),
                       axis[1] + r_c * np.sin(theta_c),
                       z_c + ground], axis=1)
    cloud = np.vstack([trunk, canopy])
    if missing_band is not None:
        low, high = missing_band
        cloud = cloud[(cloud[:, 2] < low) | (cloud[:, 2] > high)]
    return cloud


class TrunkAxisTest(unittest.TestCase):
    def test_axis_recovers_offset(self):
        cloud = _synthetic_tree(axis=(0.35, -0.20), trunk_r=0.10)
        x, y, count = estimate_trunk_axis(cloud, ScanEstimateConfig())
        self.assertAlmostEqual(x, 0.35, delta=0.05)
        self.assertAlmostEqual(y, -0.20, delta=0.05)
        self.assertGreater(count, 0)

    def test_no_band_points(self):
        cloud = np.array([[0.0, 0.0, 20.0]] * 10)
        x, y, count = estimate_trunk_axis(cloud, ScanEstimateConfig())
        self.assertIsNone(x)
        self.assertEqual(count, 0)


class TrunkTopTest(unittest.TestCase):
    def test_tight_cylinder_beats_canopy(self):
        # The canopy extends outward but not above the top, so the tight
        # cylinder should read ~12.0 while a wide annulus would read the fronds.
        cloud = _synthetic_tree(trunk_top_z=12.0)
        x, y, _ = estimate_trunk_axis(cloud, ScanEstimateConfig())
        z_top, z_p999, count = trunk_top(cloud, (x, y), ScanEstimateConfig())
        self.assertAlmostEqual(z_top, 12.0, delta=0.1)
        self.assertGreater(count, 0)
        # The 99.9th percentile is a robust variant of the same measurement.
        self.assertLessEqual(z_p999, z_top + 1e-6)


class EstimateTreeHeightTest(unittest.TestCase):
    def test_world_frame_height(self):
        cloud = _synthetic_tree(trunk_top_z=12.0)
        est = estimate_tree_height(cloud, frame='world')
        self.assertTrue(est.valid)
        self.assertAlmostEqual(est.tree_height_m, 12.0, delta=0.15)
        self.assertIsNotNone(est.crown_base_m)
        self.assertAlmostEqual(est.crown_base_m, 9.2, delta=0.6)

    def test_ground_z_shifts_sensor_frame(self):
        # Sensor frame with the ground at z=1.5 (LiDAR 1.5 m above ground).
        cloud = _synthetic_tree(trunk_top_z=12.0, ground=1.5)
        est = estimate_tree_height(cloud, ground_z=1.5, frame='sensor')
        self.assertTrue(est.valid)
        self.assertAlmostEqual(est.tree_height_m, 12.0, delta=0.15)

    def test_small_cloud_is_invalid(self):
        est = estimate_tree_height(np.zeros((5, 3)), frame='world')
        self.assertFalse(est.valid)
        self.assertIn('small', est.reason)

    def test_no_trunk_returns_invalid(self):
        # Only far canopy points: no trunk band, so no confident estimate.
        cloud = _synthetic_tree(n_trunk=400, trunk_r=0.01)
        cloud[:, :2] += 0.0
        # Push every point outside the axis band by height.
        est = estimate_tree_height(cloud, frame='world',
                                   cfg=ScanEstimateConfig(axis_band_m=(50.0, 60.0)))
        self.assertFalse(est.valid)

    def test_rows_shape(self):
        est = estimate_tree_height(_synthetic_tree(), frame='world')
        rows = est.to_rows()
        keys = [r['key'] for r in rows]
        self.assertEqual(keys, ['tree_height', 'trunk_top', 'crown_base',
                                'uncertainty'])
        for row in rows:
            for field in ('key', 'label', 'value', 'valid'):
                self.assertIn(field, row)


class BoomIkTest(unittest.TestCase):
    """Acceptance numbers from the ros2_ws boom-plan validation."""

    def test_feasible_case_matches_simulation(self):
        # Reproduce the ros2_ws boom-plan acceptance case explicitly: H=11.90,
        # d_horiz=4.87, offset 1.0 -> H_dock=10.90, un-corrected pivot 1.76 ->
        # theta_b ~= 1.172 rad.  Asserting with the plan's own constants proves
        # the IK transcription matches the simulation-proven formula.
        plan_cfg = ScanEstimateConfig(
            docking_offset_below_top_m=1.0, boom_pivot_z_m=1.76)
        target = solve_boom_target(11.90, 4.87, plan_cfg)
        self.assertTrue(target.feasible)
        self.assertEqual(target.status, 'READY')
        self.assertAlmostEqual(target.boom_angle_rad, 1.172, delta=0.005)

    def test_as_built_offset_and_pivot(self):
        # With this module's shipped as-built constants (offset 2.0, pivot
        # 1.81) the same tree solves to ~1.127 rad / ~6.56 m.
        target = solve_boom_target(11.90, 4.87)
        self.assertTrue(target.feasible)
        self.assertAlmostEqual(target.boom_angle_rad, 1.127, delta=0.005)
        self.assertAlmostEqual(target.boom_extension_total_m, 6.56, delta=0.05)

    def test_as_built_docking_geometry(self):
        # Deployment: H=12.0, offset 2.0 -> H_dock=10.0; the sim's feasible
        # geometry used d_horiz=4.87.  Verify the leveling sign is +theta_b.
        target = solve_boom_target(12.0, 4.87)
        self.assertTrue(target.feasible)
        self.assertEqual(target.platform_level_rad, target.boom_angle_rad)
        self.assertGreater(target.boom_angle_deg, 0.0)
        self.assertAlmostEqual(target.docking_height_m, 10.0)

    def test_infeasible_reports_deficit(self):
        # Far away: required length exceeds L_fixed + stroke.
        target = solve_boom_target(12.0, 40.0)
        self.assertFalse(target.feasible)
        self.assertEqual(target.status, 'INFEASIBLE_DOCK_HEIGHT')
        self.assertGreater(target.deficit_m, 0.0)

    def test_trunk_inside_tail_is_infeasible(self):
        target = solve_boom_target(12.0, PLATFORM_TAIL_M - 0.1)
        self.assertFalse(target.feasible)
        self.assertEqual(target.status, 'INFEASIBLE_DOCK_HEIGHT')

    def test_no_height_is_no_data(self):
        target = solve_boom_target(None, 4.0)
        self.assertFalse(target.feasible)
        self.assertEqual(target.status, 'NO DATA')

    def test_no_horizontal_distance_is_no_data(self):
        target = solve_boom_target(12.0, None)
        self.assertFalse(target.feasible)
        self.assertEqual(target.status, 'NO DATA')

    def test_extension_split_sums_to_total(self):
        parts = split_extension(8.0)
        self.assertEqual(len(parts), 4)
        self.assertAlmostEqual(sum(parts), 8.0)

    def test_rows_shape(self):
        rows = solve_boom_target(12.0, 4.87).to_rows()
        keys = [r['key'] for r in rows]
        self.assertEqual(keys, ['docking_height', 'boom_angle', 'boom_extension',
                                'platform_level', 'boom_distance',
                                'docking_lower', 'status'])


class PlanScanTest(unittest.TestCase):
    def test_end_to_end(self):
        cloud = _synthetic_tree(trunk_top_z=12.0)
        tree, target = plan_scan(cloud, horizontal_distance_m=4.87,
                                 frame='world')
        self.assertTrue(tree.valid)
        self.assertTrue(target.feasible)

    def test_invalid_tree_yields_no_data_target(self):
        tree, target = plan_scan(np.zeros((5, 3)), horizontal_distance_m=4.0)
        self.assertFalse(tree.valid)
        self.assertEqual(target.status, 'NO DATA')

    def test_constants_match_as_built_corrections(self):
        # The two corrections the live simulation found must ship.
        self.assertAlmostEqual(BOOM_PIVOT_WORLD_Z_M, 1.81)
        self.assertAlmostEqual(BOOM_FIXED_LENGTH_M, 2.4)
        self.assertAlmostEqual(PLATFORM_TAIL_M, 1.02)
        self.assertAlmostEqual(EXTENSION_STROKE_M, 9.6)


if __name__ == '__main__':
    unittest.main()
