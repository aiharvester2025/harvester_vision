import unittest

from helpers import (
    base_header,
    depth_packet,
    json_packet,
    lidar_packet,
)

from harvester_dashboard.model.telemetry_model import TelemetryModel
from harvester_dashboard.protocol_shim import unpack_message


class FakeClock:
    def __init__(self, start=100.0):
        self.now = start

    def __call__(self):
        return self.now


class TelemetryModelTest(unittest.TestCase):
    def setUp(self):
        self.clock = FakeClock()
        self.model = TelemetryModel(clock=self.clock)

    def test_packet_updates_state_and_json(self):
        frames = json_packet('v1/range/cutter', {
            'telemetry_key': 'cutter_forward',
            'distance_m': 1.25,
            'valid': True,
        }, sequence=4)
        result = self.model.ingest_frames(frames)
        self.assertIsNotNone(result)
        state = self.model.state('v1/range/cutter')
        self.assertEqual(state.last_header['sequence'], 4)
        self.assertEqual(state.last_json['distance_m'], 1.25)
        self.assertTrue(state.ever_seen)

    def test_docking_ranges_overwrite_not_accumulate(self):
        for sequence, distance in ((1, 1.0), (2, 1.1), (3, 1.2)):
            payload = [{'telemetry_key': 'center_line', 'distance_m': distance,
                        'valid': True}]
            self.model.ingest_frames(
                json_packet('v1/range/docking', payload, sequence=sequence))
        docking, cutter = self.model.snapshot_ranges()
        self.assertEqual(len(docking), 1)
        self.assertEqual(docking[0]['distance_m'], 1.2)
        self.assertIsNone(cutter)

    def test_sequence_gap_counting(self):
        self.model.ingest_frames(json_packet('v1/range/cutter', {}, sequence=1))
        self.model.ingest_frames(json_packet('v1/range/cutter', {}, sequence=2))
        # Jump 2 -> 5 means packets 3 and 4 were missed.
        self.model.ingest_frames(json_packet('v1/range/cutter', {}, sequence=5))
        state = self.model.state('v1/range/cutter')
        self.assertEqual(state.sequence_gaps, 2)
        self.model.ingest_frames(json_packet('v1/range/cutter', {}, sequence=6))
        self.assertEqual(self.model.state('v1/range/cutter').sequence_gaps, 2)

    def test_staleness_uses_local_receipt_time(self):
        self.model.ingest_frames(json_packet('v1/range/cutter', {}, sequence=1))
        state = self.model.state('v1/range/cutter')
        self.clock.now += 1.0
        self.assertFalse(state.is_stale(self.clock.now, stale_after_s=2.0))
        self.clock.now += 1.5
        self.assertTrue(state.is_stale(self.clock.now, stale_after_s=2.0))

    def test_never_seen_stream_is_stale(self):
        state = self.model.state('v1/lidar/raw')
        self.assertTrue(state.is_stale(self.clock.now, stale_after_s=2.0))

    def test_protocol_violation_counted_not_raised(self):
        bad = [b'v1/unknown/channel', b'not-msgpack', b'']
        self.assertIsNone(self.model.ingest_frames(bad))
        state = self.model.state('v1/unknown/channel')
        self.assertEqual(state.decode_errors, 1)
        self.assertIn('protocol', state.last_error)

    def test_source_mode_aggregation(self):
        self.assertEqual(self.model.source_mode(), 'NO DATA')
        sim = {'source_mode': 'simulation'}
        self.model.ingest_packet('v1/range/cutter', dict(base_header()), b'{}')
        self.assertEqual(self.model.source_mode(), 'SIMULATION')
        self.model.ingest_packet(
            'v1/camera/cutter/rgb',
            dict(base_header(source_mode='hardware')), b'')
        self.assertEqual(self.model.source_mode(), 'MIXED')
        self.assertTrue(self.model.is_mixed())

    def test_capabilities_from_system_status(self):
        payload = {'streams': {'v1/range/cutter': {'enabled': True}},
                   'dropped_packets': {'v1/lidar/raw': 2},
                   'recording': {'enabled': False},
                   'capabilities': {'target.world_fixed': False}}
        self.model.ingest_frames(
            json_packet('v1/system/status', payload, clock_domain='utc_host'))
        self.assertEqual(self.model.last_system_status, payload)
        caps = self.model.latest_capabilities()
        self.assertIn('target.world_fixed', caps)

    def test_summary_rows_reflect_counters(self):
        self.model.ingest_frames(json_packet('v1/range/cutter', {}, sequence=1))
        self.model.ingest_frames(json_packet('v1/range/cutter', {}, sequence=3))
        self.model.ensure_channel('v1/range/cutter').record_drop(2)
        rows = self.model.summary_rows(stale_after_s=2.0)
        row = next(r for r in rows if r['channel'] == 'v1/range/cutter')
        self.assertEqual(row['sequence_gaps'], 1)
        self.assertEqual(row['drops'], 2)
        self.assertFalse(row['stale'])

    def test_camera_info_snapshot(self):
        info = {'width': 64, 'height': 48, 'k': [100.0] * 9}
        self.model.ingest_frames(
            json_packet('v1/camera/cutter/camera_info', info, sequence=1,
                        width=64, height=48))
        self.assertEqual(self.model.snapshot_camera_info('cutter'), info)

    def test_trunk_and_calibration_snapshots(self):
        self.model.ingest_frames(json_packet(
            'v1/docking/trunk_estimate',
            {'pose': {'position': {'x': 1, 'y': 0, 'z': 0}},
             'covariance': [0.0] * 36}))
        self.model.ingest_frames(json_packet(
            'v1/calibration/status', {'status': 'VALID'},
            transform_valid=True))
        trunk = self.model.snapshot_trunk()
        self.assertIn('pose', trunk)
        _payload, valid = self.model.snapshot_calibration()
        self.assertTrue(valid)


if __name__ == '__main__':
    unittest.main()
