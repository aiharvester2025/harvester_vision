"""Tests for the MQTT ingest adapter mapping functions."""

import unittest

from canonical_zmq_publisher.mqtt_ingest import (
    map_boom_state,
    map_docking_records,
    SENSOR_BINDINGS,
)


class BoomStateMappingTest(unittest.TestCase):
    def test_maps_live_payload_fields(self):
        payload = {
            'RUN': True,
            'Platform Tilt X1': 0.296,
            'Platform Tilt Y1': -0.127,
            'PrimeMover Tilt X2': -36.41,
            'PrimeMover Tilt Y2': -43.10,
            'Boom_Length': 0.0025,
            'Boom_Angle': 0.101,
            'Slew Angle': 12.5,
            'Ultrasonic Left': 4.92,
        }
        boom = map_boom_state(payload)
        self.assertEqual(boom['phase'], 'RUN')
        self.assertAlmostEqual(boom['boom_angle_deg'], 0.101)
        self.assertAlmostEqual(boom['boom_extension_m'], 0.0025)
        self.assertAlmostEqual(boom['slew_angle_deg'], 12.5)
        self.assertAlmostEqual(boom['platform_tilt_x1_deg'], 0.296)
        self.assertAlmostEqual(boom['primemover_tilt_y2_deg'], -43.10)
        # platform roll/pitch mirror the platform tilt for dashboard compatibility
        self.assertAlmostEqual(boom['platform_roll_deg'], 0.296)
        self.assertAlmostEqual(boom['platform_pitch_deg'], -0.127)

    def test_idle_when_run_false(self):
        boom = map_boom_state({'RUN': False})
        self.assertEqual(boom['phase'], 'IDLE')

    def test_missing_fields_default_to_none(self):
        boom = map_boom_state({})
        self.assertIsNone(boom['boom_angle_deg'])
        self.assertIsNone(boom['slew_angle_deg'])
        self.assertFalse(boom['docked'])

    def test_non_numeric_fields_coerce_to_none(self):
        boom = map_boom_state({'Boom_Angle': 'not-a-number'})
        self.assertIsNone(boom['boom_angle_deg'])


class DockingRecordMappingTest(unittest.TestCase):
    def test_maps_laser_distances(self):
        payload = {
            'Laser Distance Left': 4.897,
            'Laser Distance Center': 5.152,
            'Laser Distance Right': 5.104,
        }
        records = map_docking_records(payload)
        by_key = {r['telemetry_key']: r for r in records}
        self.assertEqual(set(by_key), {
            'diagonal_left_45deg', 'center_line', 'diagonal_right_45deg'})
        self.assertAlmostEqual(by_key['diagonal_left_45deg']['distance_m'], 4.897)
        self.assertAlmostEqual(by_key['center_line']['distance_m'], 5.152)
        self.assertAlmostEqual(by_key['diagonal_right_45deg']['distance_m'], 5.104)
        self.assertTrue(all(r['valid'] for r in records))

    def test_right_key_typo_is_tolerated(self):
        # The PLC stream sometimes misspells the right sensor key.
        records = map_docking_records({'Laser Distanec Right': 5.104})
        by_key = {r['telemetry_key']: r for r in records}
        self.assertAlmostEqual(
            by_key['diagonal_right_45deg']['distance_m'], 5.104)
        self.assertTrue(by_key['diagonal_right_45deg']['valid'])

    def test_missing_distances_are_invalid(self):
        records = map_docking_records({})
        self.assertEqual(len(records), 3)
        self.assertTrue(all(not r['valid'] for r in records))
        self.assertTrue(all(r['distance_m'] is None for r in records))

    def test_null_distance_treated_as_invalid(self):
        records = map_docking_records({'Laser Distance Center': None})
        by_key = {r['telemetry_key']: r for r in records}
        self.assertIsNone(by_key['center_line']['distance_m'])
        self.assertFalse(by_key['center_line']['valid'])

    def test_bindings_shape(self):
        for telemetry_key, frame_id, wire_key in SENSOR_BINDINGS:
            self.assertTrue(telemetry_key)
            self.assertTrue(frame_id)
            self.assertTrue(wire_key)


if __name__ == '__main__':
    unittest.main()
