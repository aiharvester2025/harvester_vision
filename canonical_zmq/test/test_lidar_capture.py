"""Tests for the canonical LiDAR capture adapter (no hardware)."""

import math
import unittest

from harvester_telemetry_contract import pack_message, unpack_message
from canonical_zmq_publisher.lidar_capture import (
    LIDAR_IMU_CHANNEL,
    POINT_FIELDS,
    POINT_STRIDE_BYTES,
    LidarCapture,
    build_lidar_header,
    build_lidar_imu_header,
    downsample,
    pack_points,
    range_clip,
    run_synthetic,
    sector_filter,
)


class HeaderTest(unittest.TestCase):
    def test_header_is_canonical(self):
        header = build_lidar_header('mid360_link', 200, 123456789000)
        self.assertEqual(header['codec'], 'lidar_xyz_f32')
        self.assertEqual(header['point_count'], 200)
        self.assertEqual(header['point_stride_bytes'], 12)
        self.assertEqual(header['frame_id'], 'mid360_link')
        self.assertEqual(header['source_mode'], 'hardware')
        self.assertEqual(header['clock_domain'], 'plc_rtc_utc')

    def test_synthetic_header_reports_simulation(self):
        header = build_lidar_header('mid360_link', 1, 1, source_mode='simulation')
        self.assertEqual(header['source_mode'], 'simulation')

    def test_invalid_source_mode_is_rejected(self):
        with self.assertRaises(ValueError):
            build_lidar_header('mid360_link', 1, 1, source_mode='unknown')

    def test_header_round_trips_through_contract(self):
        header = build_lidar_header('mid360_link', 3, 1)
        header['sequence'] = 1
        header['source_id'] = 'orin'
        header['gateway_monotonic_ns'] = 0
        payload = pack_points([(1.0, 2.0, 3.0), (4.0, 5.0, 6.0), (7.0, 8.0, 9.0)])
        channel, validated, decoded = unpack_message(
            pack_message('v1/lidar/raw', header, payload))
        self.assertEqual(channel, 'v1/lidar/raw')
        self.assertEqual(validated['point_count'], 3)
        self.assertEqual(len(decoded), POINT_STRIDE_BYTES * 3)

    def test_pack_points_is_little_endian_float32(self):
        blob = pack_points([(1.0, -2.0, 3.0)])
        self.assertEqual(len(blob), POINT_STRIDE_BYTES)
        self.assertEqual([f['type'] for f in POINT_FIELDS], ['float32'] * 3)

    def test_pack_points_accepts_a_numpy_array(self):
        # A (N, 3) array with more than one row used to raise "truth value of an
        # array is ambiguous" from the emptiness check.
        import numpy as np
        rows = [(1.0, 2.0, 3.0), (4.0, 5.0, 6.0)]
        self.assertEqual(
            pack_points(np.asarray(rows, dtype='<f4')),
            pack_points(rows))


class SectorFilterTest(unittest.TestCase):
    def test_keeps_forward_and_drops_behind(self):
        points = [(1.0, 0.0, 0.0), (-1.0, 0.0, 0.0), (0.0, 1.0, 0.0)]
        kept = sector_filter(points, 120.0)
        self.assertIn((1.0, 0.0, 0.0), kept)   # dead ahead
        self.assertNotIn((-1.0, 0.0, 0.0), kept)  # directly behind
        self.assertNotIn((0.0, 1.0, 0.0), kept)   # 90 deg off, outside +/-60

    def test_keeps_points_inside_half_angle(self):
        # 45 deg azimuth is inside a 120 deg sector (+/-60).
        r = math.sqrt(2.0)
        point = (r, r, 0.0)
        self.assertIn(point, sector_filter([point], 120.0))

    def test_full_sector_is_a_noop(self):
        points = [(1.0, 0.0, 0.0), (-1.0, 0.0, 0.0), (0.0, -5.0, 0.0)]
        self.assertEqual(len(sector_filter(points, 360.0)), 3)

    def test_drops_roughly_two_thirds_of_a_ring(self):
        ring = []
        for i in range(360):
            angle = math.radians(i)
            ring.append((math.cos(angle), math.sin(angle), 0.0))
        kept = sector_filter(ring, 120.0)
        self.assertAlmostEqual(len(kept), 120, delta=3)


class RangeClipTest(unittest.TestCase):
    def test_drops_points_inside_blind_zone(self):
        points = [(0.05, 0.0, 0.0), (1.0, 0.0, 0.0)]
        kept = range_clip(points, 0.15, 40.0)
        self.assertEqual(kept, [(1.0, 0.0, 0.0)])

    def test_drops_points_beyond_max_range(self):
        points = [(45.0, 0.0, 0.0), (10.0, 0.0, 0.0)]
        kept = range_clip(points, 0.15, 40.0)
        self.assertEqual(kept, [(10.0, 0.0, 0.0)])

    def test_uses_euclidean_distance(self):
        # Each axis is under 40 m but the norm is ~48 m, so it is clipped.
        point = (28.0, 28.0, 28.0)
        self.assertEqual(range_clip([point], 0.15, 40.0), [])


class DownsampleTest(unittest.TestCase):
    def test_caps_to_maximum(self):
        points = [(float(i), 0.0, 0.0) for i in range(100)]
        self.assertEqual(len(downsample(points, 10)), 10)

    def test_noop_when_under_cap(self):
        points = [(1.0, 2.0, 3.0)]
        self.assertEqual(downsample(points, 10), points)

    def test_spans_the_full_input(self):
        points = [(float(i), 0.0, 0.0) for i in range(100)]
        sampled = downsample(points, 3)
        self.assertEqual(sampled[0], points[0])
        self.assertEqual(sampled[-1], points[-1])


class SyntheticTest(unittest.TestCase):
    def test_synthetic_respects_cap_and_sector(self):
        points = run_synthetic(200, 120.0, 0.15, 40.0)
        self.assertLessEqual(len(points), 200)
        for x, y, _z in points:
            self.assertLessEqual(abs(math.degrees(math.atan2(y, x))), 60.001)


class ControlTest(unittest.TestCase):
    def test_starts_disabled_by_default(self):
        capture = LidarCapture(sdk_mode='synthetic',
                               control_endpoint='tcp://127.0.0.1:5599')
        try:
            self.assertFalse(capture.enabled)
        finally:
            capture.push_socket.close(0)
            capture.control_socket.close(0)

    def test_enabled_flag_starts_streaming(self):
        capture = LidarCapture(sdk_mode='synthetic', disabled=False,
                               control_endpoint='tcp://127.0.0.1:5598')
        try:
            self.assertTrue(capture.enabled)
        finally:
            capture.push_socket.close(0)
            capture.control_socket.close(0)

    def test_acquire_returns_bounded_cloud(self):
        capture = LidarCapture(sdk_mode='synthetic', max_points=50,
                               control_endpoint='tcp://127.0.0.1:5597')
        try:
            points = capture.acquire()
            self.assertLessEqual(len(points), 50)
        finally:
            capture.push_socket.close(0)
            capture.control_socket.close(0)

    def test_apply_enabled_noop_without_sdk(self):
        # Synthetic mode has no SDK; the flag alone gates output.
        capture = LidarCapture(sdk_mode='synthetic',
                               control_endpoint='tcp://127.0.0.1:5596')
        try:
            self.assertEqual(capture._apply_enabled(True), (False, False))
            self.assertEqual(capture._apply_enabled(False), (False, False))
        finally:
            capture.push_socket.close(0)
            capture.control_socket.close(0)

    def test_control_message_toggles_enabled(self):
        import time as _time
        import zmq as _zmq
        capture = LidarCapture(sdk_mode='synthetic', disabled=True,
                               control_endpoint='tcp://127.0.0.1:5595')
        sender = _zmq.Context.instance().socket(_zmq.PUSH)
        sender.setsockopt(_zmq.IMMEDIATE, 1)
        sender.connect('tcp://127.0.0.1:5595')
        try:
            self.assertFalse(capture.enabled)
            # ZMQ slow-joiner: give the PUSH/PULL pair a moment to connect.
            _time.sleep(0.3)
            sender.send_json({'enabled': True})
            for _ in range(200):
                if capture.poll_control():
                    break
                _time.sleep(0.01)
            self.assertTrue(capture.enabled)
        finally:
            sender.close(0)
            capture.push_socket.close(0)
            capture.control_socket.close(0)

    def test_control_ignores_non_boolean_enabled(self):
        import time as _time
        import zmq as _zmq
        capture = LidarCapture(sdk_mode='synthetic', disabled=True,
                               control_endpoint='tcp://127.0.0.1:5594')
        sender = _zmq.Context.instance().socket(_zmq.PUSH)
        sender.setsockopt(_zmq.IMMEDIATE, 1)
        sender.connect('tcp://127.0.0.1:5594')
        try:
            _time.sleep(0.3)
            sender.send_json({'enabled': 'yes'})
            for _ in range(50):
                capture.poll_control()
                _time.sleep(0.01)
            self.assertFalse(capture.enabled)
        finally:
            sender.close(0)
            capture.push_socket.close(0)
            capture.control_socket.close(0)


class TimingSnapshotTest(unittest.TestCase):
    """The emitted clock_domain/timestamp_source must match the timestamp used.

    The adapter must never label a host-clock value 'lidar_ptp_utc', and must
    never label a boot-relative device counter as synchronized.
    """

    def _capture_with_source(self, snapshot):
        # Build a capture without touching ZMQ/hardware, then swap in a fake
        # source whose timing_snapshot returns the case under test.
        capture = LidarCapture.__new__(LidarCapture)
        capture.sdk_mode = 'livox-sdk'
        capture._source = type('FakeSource', (), {
            'timing_snapshot': lambda self, max_age_s=1.0: dict(snapshot),
        })()

        class _NoSocket:
            def close(self, *_args):
                pass

        capture.push_socket = _NoSocket()
        capture.control_socket = _NoSocket()
        return capture

    def test_fresh_ptp_sample_maps_to_lidar_ptp_utc(self):
        capture = self._capture_with_source({
            'timestamp_ns': 1_700_000_000_000_000_000,
            'source': 'livox_ptp', 'clock_domain': 'lidar_ptp_utc',
            'synchronized': True, 'time_type': 1, 'lidar_source': 'ptp',
            'arrival_age_s': 0.0,
        })
        snap = capture.timing_snapshot()
        self.assertEqual(snap['clock_domain'], 'lidar_ptp_utc')
        self.assertEqual(snap['source'], 'livox_ptp')
        self.assertTrue(snap['synchronized'])

    def test_stale_ptp_sample_maps_to_orin_realtime_not_ptp(self):
        # This is the contradiction the single-snapshot design removes: a stale
        # absolute sample falls back to host time, so the clock domain must be
        # host, and the label must not say ptp.
        capture = self._capture_with_source({
            'timestamp_ns': 1_800_000_000_000_000_000,
            'source': 'host_arrival', 'clock_domain': 'orin_realtime',
            'synchronized': False, 'time_type': 1, 'lidar_source': 'ptp',
            'arrival_age_s': 5.0,
        })
        snap = capture.timing_snapshot()
        self.assertEqual(snap['clock_domain'], 'orin_realtime')
        self.assertEqual(snap['source'], 'host_arrival')
        self.assertFalse(snap['synchronized'])
        # The device time_type may still read ptp; the point is that the VALUE is
        # host time, so it is not published as synchronized.
        self.assertEqual(snap['time_type'], 1)

    def test_unsynchronized_source_maps_to_orin_realtime(self):
        capture = self._capture_with_source({
            'timestamp_ns': 1_800_000_000_000_000_000,
            'source': 'host_arrival', 'clock_domain': 'orin_realtime',
            'synchronized': False, 'time_type': 0, 'lidar_source': 'device_uptime',
            'arrival_age_s': 0.01,
        })
        snap = capture.timing_snapshot()
        self.assertEqual(snap['clock_domain'], 'orin_realtime')
        self.assertFalse(snap['synchronized'])
        self.assertEqual(snap['time_type'], 0)

    def test_synthetic_mode_is_host_time(self):
        capture = LidarCapture.__new__(LidarCapture)
        capture.sdk_mode = 'synthetic'
        capture._source = None
        snap = capture.timing_snapshot()
        self.assertEqual(snap['clock_domain'], 'orin_realtime')
        self.assertEqual(snap['source'], 'host_arrival')
        self.assertFalse(snap['synchronized'])
        self.assertEqual(snap['lidar_source'], 'simulation')


class LidarImuTest(unittest.TestCase):
    def test_channel_name(self):
        self.assertEqual(LIDAR_IMU_CHANNEL, 'v1/imu/lidar')

    def test_imu_header_is_canonical_json(self):
        header = build_lidar_imu_header('mid360_link', 123, source_mode='hardware',
                                        clock_domain='lidar_ptp_utc',
                                        timestamp_source='livox_ptp')
        self.assertEqual(header['codec'], 'json')
        self.assertEqual(header['frame_id'], 'mid360_link')
        self.assertTrue(header['capabilities']['lidar.imu'])
        # No image geometry on an IMU header.
        self.assertNotIn('width', header)
        self.assertNotIn('height', header)
        header['sequence'] = 1
        header['gateway_monotonic_ns'] = 0
        payload = b'{"attitude_rpy_rad": [0.1, 0.2, 0.0]}'
        channel, validated, decoded = unpack_message(
            pack_message(LIDAR_IMU_CHANNEL, header, payload))
        self.assertEqual(channel, LIDAR_IMU_CHANNEL)
        self.assertEqual(decoded, payload)

    def test_invalid_source_mode_is_rejected(self):
        with self.assertRaises(ValueError):
            build_lidar_imu_header('mid360_link', 1, source_mode='bogus')

    def test_synthetic_mode_emits_no_imu(self):
        capture = LidarCapture(sdk_mode='synthetic', disabled=False,
                               ingest_endpoint='inproc://imu-synth',
                               control_endpoint='inproc://imu-synth-ctl')
        self.assertFalse(capture.emit_imu({'attitude_rpy_rad': [0, 0, 0]}))

    def test_missing_sample_emits_no_imu(self):
        capture = LidarCapture(sdk_mode='synthetic', disabled=False,
                               ingest_endpoint='inproc://imu-none',
                               control_endpoint='inproc://imu-none-ctl')
        capture.sdk_mode = 'livox-sdk'
        self.assertFalse(capture.emit_imu(None))
        self.assertFalse(capture.emit_imu('not-a-dict'))


if __name__ == '__main__':
    unittest.main()
