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
    def test_maps_ultrasonic_reading(self):
        payload = {'Ultrasonic Left': 4.92}
        records = map_docking_records(payload)
        self.assertEqual(len(records), 1)
        self.assertEqual(records[0]['telemetry_key'], 'ultrasonic_left')
        self.assertEqual(records[0]['frame_id'], 'sensor_ultrasonic_left_frame')
        self.assertAlmostEqual(records[0]['distance_m'], 4.92)
        self.assertTrue(records[0]['valid'])

    def test_missing_ultrasonic_is_invalid(self):
        records = map_docking_records({})
        self.assertEqual(len(records), 1)
        self.assertIsNone(records[0]['distance_m'])
        self.assertFalse(records[0]['valid'])

    def test_null_slew_angle_treated_as_none(self):
        records = map_docking_records({'Ultrasonic Left': None})
        self.assertIsNone(records[0]['distance_m'])
        self.assertFalse(records[0]['valid'])


if __name__ == '__main__':
    unittest.main()
