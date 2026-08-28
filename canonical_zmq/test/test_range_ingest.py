"""Tests for the Pi range/boom ingest adapter mapping functions."""

import unittest

from canonical_zmq_publisher.range_ingest import (
    map_boom_state,
    map_docking_records,
    map_trunk_estimate,
    SENSOR_BINDINGS,
)


class DockingRecordMappingTest(unittest.TestCase):
    def test_maps_all_five_sensors_in_order(self):
        sensors = {
            'center_line': {'distance_m': 0.120, 'valid': True},
            'diagonal_left_45deg': {'distance_m': 0.5, 'valid': True},
            'diagonal_right_45deg': {'distance_m': 0.6, 'valid': True},
            'c_channel_left': {'distance_m': 0.7, 'valid': True},
            'c_channel_right': {'distance_m': 0.8, 'valid': True},
        }
        records = map_docking_records(sensors)
        self.assertEqual(len(records), 5)
        keys = [r['telemetry_key'] for r in records]
        self.assertEqual(keys, [k for k, _ in SENSOR_BINDINGS])
        self.assertEqual(records[0]['distance_m'], 0.120)
        self.assertTrue(records[0]['valid'])
        self.assertEqual(records[0]['frame_id'], 'sensor_center_line_frame')

    def test_invalid_sensor_is_none_distance(self):
        records = map_docking_records({'center_line': {'distance_m': None, 'valid': False}})
        self.assertIsNone(records[0]['distance_m'])
        self.assertFalse(records[0]['valid'])

    def test_missing_sensor_is_invalid(self):
        records = map_docking_records({})
        self.assertEqual(len(records), 5)
        self.assertTrue(all(not r['valid'] for r in records))
        self.assertTrue(all(r['distance_m'] is None for r in records))


class BoomStateMappingTest(unittest.TestCase):
    def test_maps_phase_and_angles(self):
        sim = {
            'phase': 'BOOM_EXTEND',
            'boom_angle_deg': 45.0,
            'boom_extension_m': 5.0,
            'platform_roll_deg': 0.1,
            'platform_pitch_deg': -0.2,
            'docked': False,
            'target_tree_height_m': 12.0,
        }
        boom = map_boom_state(sim)
        self.assertEqual(boom['phase'], 'BOOM_EXTEND')
        self.assertEqual(boom['boom_angle_deg'], 45.0)
        self.assertEqual(boom['boom_extension_m'], 5.0)
        self.assertFalse(boom['docked'])

    def test_missing_simulation_defaults(self):
        boom = map_boom_state(None)
        self.assertEqual(boom['phase'], 'UNKNOWN')
        self.assertFalse(boom['docked'])


class TrunkEstimateMappingTest(unittest.TestCase):
    def test_trunk_position_is_ahead_of_clearance(self):
        trunk = map_trunk_estimate({'phase': 'DOCKED', 'center_bark_distance_m': 0.120})
        position = trunk['pose']['position']
        self.assertAlmostEqual(position['x'], 0.420, places=3)  # 0.120 + 0.300
        self.assertEqual(position['y'], 0.0)

    def test_trunk_position_tracks_approach_distance(self):
        trunk = map_trunk_estimate(
            {'phase': 'ENTRY_GATE_ALIGNMENT', 'center_bark_distance_m': 2.50})
        self.assertAlmostEqual(trunk['pose']['position']['x'], 2.80, places=3)

    def test_trunk_position_falls_back_when_bark_missing(self):
        trunk = map_trunk_estimate({'phase': 'UNKNOWN'})
        self.assertAlmostEqual(trunk['pose']['position']['x'], 0.420, places=3)


if __name__ == '__main__':
    unittest.main()
