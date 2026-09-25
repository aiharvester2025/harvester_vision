"""Tests for the encoder-free tree/boom estimate used by the scan HUD.

The acceptance numbers are taken from the validated ``ros2_ws`` plans so this
dashboard copy cannot silently drift from the simulation-proven math.
"""

import math
import os
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
    docking_lower_angle,
    estimate_tree_height,
    estimate_trunk_axis,
    plan_scan,
    solve_boom_target,
    split_extension,
    trunk_horizontal_distance,
    trunk_top,
)


def _synthetic_tree(trunk_top_z=12.0, crown_base_z=9.2, axis=(0.0, 0.0),
                    trunk_r=0.25, n_trunk=20000, n_canopy=120000, ground=0.0,
                    top_offset=(0.05, -0.04), missing_band=None,
                    full_trunk=False):
    """Build a synthetic vertical-trunk + outward-frond cloud (world frame).

    Dense enough that the canopy annulus crosses the production crown-base
    density threshold (5000 points per 0.25 m bin, the ros2_ws live default), so
    the tests exercise the real detector rather than a lowered threshold.

    By default the trunk is a HALF cylinder (the near side only), matching what a
    single-sided LiDAR sweep actually sees.  This matters: ``estimate_trunk_axis``
    applies the half-cylinder bias correction ``+ (2/pi)*trunk_radius``, which is
    only correct for a single-sided scan.  Set ``full_trunk=True`` for a full
    cylinder (e.g. a stationary 360-degree sweep).
    """
    rng = np.random.default_rng(0)
    z = rng.uniform(1.0, trunk_top_z, size=n_trunk)
    if full_trunk:
        theta = rng.uniform(0.0, 2.0 * math.pi, size=n_trunk)
        r = rng.uniform(0.0, trunk_r, size=n_trunk)
        xr = r * np.cos(theta)
    else:
        # Near-surface half cylinder as a sensor at -X looking toward +X sees it:
        # the visible arc is centred on -X (theta near pi), so the median X sits
        # at centre - r*(2/pi) -- the bias the estimator corrects.  This matches
        # the tree_scan_002 recording (raw median 8.283 for a true centre 8.5).
        theta = rng.uniform(0.5 * math.pi, 1.5 * math.pi, size=n_trunk)
        xr = trunk_r * np.cos(theta)
    yr = trunk_r * np.sin(theta)
    trunk = np.stack([axis[0] + xr, axis[1] + yr, z + ground], axis=1)
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
    def test_axis_recovers_offset_half_cylinder(self):
        # Single-sided scan: the median X sits at centre - r*(2/pi), so the
        # correction must recover the true centre.
        cloud = _synthetic_tree(axis=(0.35, -0.20), trunk_r=0.25)
        x, y, count = estimate_trunk_axis(cloud, ScanEstimateConfig())
        self.assertAlmostEqual(x, 0.35, delta=0.08)
        self.assertAlmostEqual(y, -0.20, delta=0.05)
        self.assertGreater(count, 0)

    def test_radius_correction_shifts_toward_centre(self):
        # The raw median is biased toward the sensor; the correction adds
        # (2/pi)*trunk_radius.  Both a small and a large trunk should land near
        # the true centre 0.0 despite different radii (tolerance scales with the
        # larger correction, which is also more sensitive to sampling noise).
        small = _synthetic_tree(axis=(0.0, 0.0), trunk_r=0.10)
        big = _synthetic_tree(axis=(0.0, 0.0), trunk_r=0.40)
        cfg = ScanEstimateConfig()
        xs, _, _ = estimate_trunk_axis(small, cfg)
        xb, _, _ = estimate_trunk_axis(big, cfg)
        self.assertAlmostEqual(xs, 0.0, delta=0.18)
        self.assertAlmostEqual(xb, 0.0, delta=0.18)

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
        self.assertAlmostEqual(est.crown_base_m, 9.2, delta=0.5)

    def test_crown_base_detected_from_density(self):
        # The synthetic canopy starts at 9.2 m; the density detector should flag
        # a trunk-end near there and mark it valid.
        est = estimate_tree_height(_synthetic_tree(), frame='world')
        self.assertTrue(est.trunk_end_valid, est.trunk_end_reason)
        self.assertEqual(est.crown_base_method, 'density_drop')
        self.assertAlmostEqual(est.crown_base_m, 9.2, delta=0.5)

    def test_crown_base_missing_marks_trunk_end_invalid(self):
        # A bare trunk with no canopy annulus: the detector fails, so the
        # estimate falls back to top-2.0 but marks the trunk-end INVALID so it
        # is never docked on.
        rng = np.random.default_rng(3)
        z = rng.uniform(0.0, 12.0, 3000)
        r = rng.uniform(0.0, 0.1, 3000)
        th = rng.uniform(0.0, 2.0 * math.pi, 3000)
        cloud = np.column_stack([r * np.cos(th), r * np.sin(th), z])
        est = estimate_tree_height(cloud, frame='world')
        self.assertTrue(est.valid)
        self.assertFalse(est.trunk_end_valid)
        # Safe fallback value, but flagged: the crown_base row renders invalid.
        rows = {row['key']: row for row in est.to_rows()}
        self.assertFalse(rows['crown_base']['valid'])

    def test_ground_z_shifts_sensor_frame(self):
        # Sensor frame with the ground at z=1.5 (LiDAR 1.5 m above ground).
        cloud = _synthetic_tree(trunk_top_z=12.0, ground=1.5)
        est = estimate_tree_height(cloud, ground_z=1.5, frame='sensor')
        self.assertTrue(est.valid)
        self.assertAlmostEqual(est.tree_height_m, 12.0, delta=0.15)

    def test_sensor_frame_reanchors_the_bands_to_local_ground(self):
        # A sensor-frame cloud with NO ground_z: the world-anchored 1-8 m axis
        # band would miss the trunk, so the bands are re-anchored to the local
        # ground.  The height is ground-relative and therefore flagged with
        # ground_known=False (the absolute value carries the ground estimate's
        # error), but the trunk axis must still be recovered correctly.
        cloud = _synthetic_tree(trunk_top_z=12.0, ground=-1.8)
        est = estimate_tree_height(cloud, frame='sensor')
        self.assertTrue(est.valid, est.reason)
        self.assertFalse(est.ground_known)
        self.assertAlmostEqual(est.trunk_axis_xy[0], 0.0, delta=0.2)
        # The tree-height row is reduced-confidence without a ground datum.
        rows = {r['key']: r for r in est.to_rows()}
        self.assertFalse(rows['tree_height']['valid'])

    def test_sensor_frame_with_ground_is_trusted(self):
        # A live MID-360 sees the ground around the machine, so a sensor-frame
        # cloud that CONTAINS ground returns yields a trusted absolute height.
        rng = np.random.default_rng(1)
        ground = np.column_stack([
            rng.uniform(-6.0, 6.0, 4000), rng.uniform(-6.0, 6.0, 4000),
            rng.normal(-1.8, 0.02, 4000)])
        cloud = np.vstack([_synthetic_tree(trunk_top_z=12.0, ground=-1.8),
                           ground])
        est = estimate_tree_height(cloud, frame='sensor')
        self.assertTrue(est.valid)
        self.assertTrue(est.ground_known)
        self.assertAlmostEqual(est.tree_height_m, 12.0, delta=0.3)
        rows = {r['key']: r for r in est.to_rows()}
        self.assertTrue(rows['tree_height']['valid'])

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
            for field in ('key', 'label', 'value', 'valid', 'group'):
                self.assertIn(field, row)
            # The HUD splits the estimate into a tree card and a boom card, so
            # every tree row must carry the tree group.
            self.assertEqual(row['group'], 'tree')


class BoomIkTest(unittest.TestCase):
    """Acceptance numbers from the ros2_ws boom-plan validation.

    The docking height is now derived from the CROWN BASE (trunk-end), so these
    pass an explicit ``crown_base_m`` rather than relying on a tree-top offset.
    """

    def test_feasible_case_matches_simulation(self):
        # Reproduce the ros2_ws boom-plan acceptance case: d_horiz=4.87, offset
        # 1.0, un-corrected pivot 1.76 and a docking height of 10.90 -> theta_b
        # ~= 1.172 rad.  Passing crown_base = H_dock + offset reproduces their
        # exact geometry, proving the IK formula transcription is intact.
        plan_cfg = ScanEstimateConfig(
            docking_offset_below_top_m=1.0, boom_pivot_z_m=1.76)
        target = solve_boom_target(11.90, 4.87, plan_cfg, crown_base_m=11.90)
        self.assertTrue(target.feasible)
        self.assertEqual(target.status, 'READY')
        self.assertAlmostEqual(target.boom_angle_rad, 1.172, delta=0.005)

    def test_as_built_crown_base_geometry(self):
        # Reference tree: crown base 9.2, offset 2.0 -> H_dock = 7.2 m, clean
        # trunk (the tree top's 12.0 m is well inside the canopy).
        target = solve_boom_target(12.0, 9.37, crown_base_m=9.2)
        self.assertTrue(target.feasible)
        self.assertAlmostEqual(target.docking_height_m, 7.2)
        # The leveling command is +theta_b (opposite joint axis).
        self.assertEqual(target.platform_level_rad, target.boom_angle_rad)

    def test_docking_height_uses_crown_base_not_tree_top(self):
        # Same tree, different crown base -> different docking height; the tree
        # top must not influence it.  This is the regression the crash exposed.
        low = solve_boom_target(12.0, 9.37, crown_base_m=9.2)
        high = solve_boom_target(12.0, 9.37, crown_base_m=11.0)
        self.assertAlmostEqual(low.docking_height_m, 7.2)
        self.assertAlmostEqual(high.docking_height_m, 9.0)
        # tree_height_m=12.0 in both, so the top clearly is not used.
        self.assertNotAlmostEqual(low.docking_height_m, high.docking_height_m)

    def test_infeasible_reports_deficit(self):
        # Far away: required length exceeds L_fixed + stroke.
        target = solve_boom_target(12.0, 40.0, crown_base_m=9.2)
        self.assertFalse(target.feasible)
        self.assertEqual(target.status, 'INFEASIBLE_DOCK_HEIGHT')
        self.assertGreater(target.deficit_m, 0.0)

    def test_trunk_inside_tail_is_infeasible(self):
        target = solve_boom_target(12.0, PLATFORM_TAIL_M - 0.1, crown_base_m=9.2)
        self.assertFalse(target.feasible)
        self.assertEqual(target.status, 'INFEASIBLE_DOCK_HEIGHT')

    def test_no_height_is_no_data(self):
        target = solve_boom_target(None, 4.0, crown_base_m=9.2)
        self.assertFalse(target.feasible)
        self.assertEqual(target.status, 'NO DATA')

    def test_no_crown_base_is_no_data(self):
        # Without a confident trunk-end the target must refuse, never fall back
        # to tree_top - offset (which is inside the frond/FFB zone).
        target = solve_boom_target(12.0, 4.87, crown_base_m=None)
        self.assertFalse(target.feasible)
        self.assertEqual(target.status, 'NO DATA')
        self.assertIn('crown base', target.reason)

    def test_invalid_trunk_end_is_no_data(self):
        target = solve_boom_target(12.0, 4.87, crown_base_m=10.0,
                                   trunk_end_valid=False)
        self.assertFalse(target.feasible)
        self.assertEqual(target.status, 'NO DATA')

    def test_no_horizontal_distance_is_no_data(self):
        target = solve_boom_target(12.0, None, crown_base_m=9.2)
        self.assertFalse(target.feasible)
        self.assertEqual(target.status, 'NO DATA')

    def test_extension_split_sums_to_total(self):
        parts = split_extension(8.0)
        self.assertEqual(len(parts), 4)
        self.assertAlmostEqual(sum(parts), 8.0)

    def test_rows_shape(self):
        rows = solve_boom_target(12.0, 4.87, crown_base_m=9.2).to_rows()
        keys = [r['key'] for r in rows]
        # No docking_lower row: the live orchestrator PLAN does not publish it,
        # and in a scan-only HUD it is structurally always 0 (see the dataclass
        # note), so it would be a misleading constant.
        self.assertEqual(keys, ['docking_height', 'boom_angle', 'boom_extension',
                                'platform_level', 'boom_distance', 'status'])
        self.assertNotIn('docking_lower', keys)
        for row in rows:
            self.assertEqual(row['group'], 'boom')

    def test_rows_are_grouped_for_the_three_card_hud(self):
        # The overlay renders rowsInGroup('tree') and rowsInGroup('boom') into
        # separate cards; every row must belong to exactly one of them.
        tree = estimate_tree_height(_synthetic_tree(), frame='world')
        target = solve_boom_target(12.0, 9.37, crown_base_m=9.2)
        rows = tree.to_rows() + target.to_rows()
        groups = sorted({r['group'] for r in rows})
        self.assertEqual(groups, ['boom', 'tree'])
        for row in rows:
            self.assertIn(row['group'], ('tree', 'boom'))


class TrunkDistanceTest(unittest.TestCase):
    """d_horiz is boom-pivot-relative, not origin-relative (ros2_ws parity)."""

    def test_reference_geometry_matches_ros2_ws(self):
        # Trunk at world x=8.5, base at x=0 -> pivot at -0.87 -> d_horiz = 9.37,
        # exactly ros2_ws test_distance_estimate_reachable.
        d = trunk_horizontal_distance((8.5, 0.0))
        self.assertAlmostEqual(d, 9.37, delta=0.01)

    def test_uses_base_x(self):
        d = trunk_horizontal_distance((8.5, 0.0), base_x_m=1.0)
        self.assertAlmostEqual(d, 8.37, delta=0.01)

    def test_bad_axis_is_none(self):
        self.assertIsNone(trunk_horizontal_distance(None))
        self.assertIsNone(trunk_horizontal_distance((None, None)))


class DockingLowerAngleTest(unittest.TestCase):
    """theta_d mirrors ros2_ws kinematics.docking_lower_angle.

    The VALUE is retained on the dataclass (and this function) for consumers
    that have a live boom pose to descend FROM, but it is deliberately NOT a HUD
    row: a scan-only estimate always has theta_d == 0 by construction, and the
    live dock_orchestrator PLAN does not publish it either.
    """

    def test_zero_at_solved_configuration(self):
        # ros2_ws test_docking_lower_angle_zero_at_solved_config: at the solved
        # theta_b/extension the boom is already at H_dock, so theta_d == 0.
        target = solve_boom_target(12.0, 9.37, crown_base_m=9.2)
        self.assertAlmostEqual(target.docking_lower_angle_rad, 0.0, places=6)

    def test_non_negative_when_boom_is_above_the_dock(self):
        # ros2_ws test_docking_lower_angle_non_negative: a boom angled above a
        # low dock point needs a positive lowering angle.
        self.assertGreater(docking_lower_angle(0.5, 10.0, 0.0), 0.0)

    def test_not_a_hud_row(self):
        # It must NOT appear as a row: it is always 0 in a scan-only HUD and the
        # live PLAN omits it, so a constant-zero row would mislead the operator.
        target = solve_boom_target(12.0, 9.37, crown_base_m=9.2)
        rows = {r['key']: r for r in target.to_rows()}
        self.assertNotIn('docking_lower', rows)


class DeriveDistanceFromAxisTest(unittest.TestCase):
    """A LiDAR-only scan must not need a separate camera distance channel."""

    def test_plan_scan_derives_distance_from_the_axis(self):
        # No d_horiz supplied: it is derived from the scanned trunk axis, so the
        # boom rows are populated (this is what was blank on the recording).
        cloud = _synthetic_tree(axis=(8.5, 0.0))
        tree, target = plan_scan(cloud, frame='world')
        self.assertTrue(tree.valid)
        self.assertIsNotNone(target.boom_horizontal_distance_m)
        self.assertAlmostEqual(target.boom_horizontal_distance_m, 9.37, delta=0.3)
        self.assertTrue(target.feasible)
        # Every boom row carries a value, not a dash.
        for row in target.to_rows():
            self.assertNotEqual(row['value'], '—', row)

    def test_explicit_distance_overrides_derivation(self):
        cloud = _synthetic_tree(axis=(8.5, 0.0))
        _tree, target = plan_scan(cloud, horizontal_distance_m=7.77,
                                  frame='world')
        self.assertAlmostEqual(target.boom_horizontal_distance_m, 7.77)

    def test_derivation_can_be_disabled(self):
        cloud = _synthetic_tree(axis=(8.5, 0.0))
        _tree, target = plan_scan(cloud, frame='world',
                                  derive_distance_from_axis=False)
        self.assertIsNone(target.boom_horizontal_distance_m)
        self.assertEqual(target.status, 'NO DATA')


class PlanScanTest(unittest.TestCase):
    def test_end_to_end(self):
        cloud = _synthetic_tree(trunk_top_z=12.0)
        tree, target = plan_scan(cloud, horizontal_distance_m=9.37,
                                 frame='world')
        self.assertTrue(tree.valid)
        # The docking height comes from the detected crown base (~9.2) minus the
        # 2.0 m offset, NOT from the 12.0 m tree top.
        self.assertTrue(tree.trunk_end_valid)
        self.assertAlmostEqual(target.docking_height_m, 7.2, delta=0.6)
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

    def test_defaults_match_the_ros2_ws_live_estimator(self):
        # These are the live ros2_ws defaults, and getting any of them wrong
        # shifts the estimate (verified by running their estimator on the same
        # cloud).  In particular the axis radius must be the MID-TRUNK radius
        # (0.25), not the wider trunk-top radius (0.35), or the axis is
        # over-corrected by (2/pi)*(0.35-0.25) ~ 0.06 m.
        cfg = ScanEstimateConfig()
        self.assertAlmostEqual(cfg.axis_trunk_radius_m, 0.25)
        self.assertAlmostEqual(cfg.trunk_cylinder_radius_m, 0.35)
        self.assertEqual(cfg.crown_density_threshold, 5000)
        self.assertAlmostEqual(cfg.crown_bin_height_m, 0.25)
        self.assertAlmostEqual(cfg.crown_z_min_m, 5.0)
        # The adaptive ratio must be OFF by default so the threshold is exactly
        # the ros2_ws absolute value.
        self.assertEqual(cfg.crown_density_ratio, 0.0)


class RecordedScanParityTest(unittest.TestCase):
    """Parity against the real recorded tree_scan_002 sweep from the Xavier.

    The recording is pulled on demand into ``testdata/`` (gitignored) and is
    skipped when absent, so the suite still runs on a machine without it.  It is
    the strongest available check: the reference tree is at world (8.5, 0) with
    trunk top 12.0 m and crown base 9.2 m (tree_targets.yaml), and the recorded
    cloud is already world-registered.
    """

    CLOUD_PATH = os.path.join(
        os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))),
        'testdata', 'tree_scan_002', 'merged_cloud.npy')

    def setUp(self):
        if not os.path.exists(self.CLOUD_PATH):
            self.skipTest('recorded fixture not present ({})'.format(
                self.CLOUD_PATH))
        self.cloud = np.load(self.CLOUD_PATH)

    def _roi(self, x=8.5, y=0.0, radius=4.0):
        r = np.hypot(self.cloud[:, 0] - x, self.cloud[:, 1] - y)
        return self.cloud[r < radius]

    def test_height_and_crown_base_match_ground_truth(self):
        tree, _target = plan_scan(self._roi(), horizontal_distance_m=9.37,
                                  frame='world')
        self.assertTrue(tree.valid, tree.reason)
        # Height: trunk-top max, true 12.0 m.
        self.assertAlmostEqual(tree.tree_height_m, 12.0, delta=0.3)
        # Trunk axis: the half-cylinder correction must recover (8.5, 0).
        self.assertAlmostEqual(tree.trunk_axis_xy[0], 8.5, delta=0.15)
        self.assertAlmostEqual(tree.trunk_axis_xy[1], 0.0, delta=0.15)
        # Crown base: a 0.25 m bin detector, so within one bin of 9.2 m.
        self.assertTrue(tree.trunk_end_valid, tree.trunk_end_reason)
        self.assertAlmostEqual(tree.crown_base_m, 9.2, delta=0.5)

    def test_docking_height_is_crown_base_minus_offset(self):
        _tree, target = plan_scan(self._roi(), horizontal_distance_m=9.37,
                                  frame='world')
        # 9.2 - 2.0 = 7.2, NOT the legacy tree_top - 2.0 = 10.0 that crashed.
        self.assertAlmostEqual(target.docking_height_m, 7.2, delta=0.5)
        self.assertNotAlmostEqual(target.docking_height_m, 10.0, delta=0.5)

    def test_matches_the_ros2_ws_estimator_on_the_same_cloud(self):
        # The strongest parity check: the live ros2_ws estimate_height() run on
        # this exact cloud returns height 11.9274, crown 9.25, axis 8.437.  Any
        # drift in the ported algorithm (defaults, bands, threshold) fails here.
        tree, _target = plan_scan(self._roi(), frame='world')
        self.assertTrue(tree.valid, tree.reason)
        self.assertAlmostEqual(tree.tree_height_m, 11.9274, delta=0.01)
        self.assertAlmostEqual(tree.crown_base_m, 9.25, delta=0.01)
        self.assertAlmostEqual(tree.trunk_axis_xy[0], 8.437, delta=0.01)
        self.assertTrue(tree.trunk_end_valid)

    def test_full_estimate_from_the_recording(self):
        # End to end from the recording with NO external distance: the boom rows
        # must all be populated from the scan alone.
        tree, target = plan_scan(self._roi(), frame='world')
        self.assertTrue(tree.valid)
        self.assertEqual(target.status, 'READY')
        # d_horiz derived from the scanned axis (8.5) + pivot offset -> ~9.37.
        self.assertAlmostEqual(target.boom_horizontal_distance_m, 9.37,
                               delta=0.2)
        rows = {r['key']: r['value'] for r in target.to_rows()}
        for key in ('docking_height', 'boom_angle', 'boom_extension',
                    'platform_level', 'boom_distance'):
            with self.subTest(key=key):
                self.assertNotEqual(rows[key], '—')
        # Docking height comes from the crown base (~9.2 - 2.0), not the top.
        self.assertAlmostEqual(target.docking_height_m, 7.2, delta=0.6)


if __name__ == '__main__':
    unittest.main()
