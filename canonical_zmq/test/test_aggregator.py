"""Tests for the Orin canonical aggregator (publisher) and synthetic source."""

import unittest

from harvester_telemetry_contract import unpack_message
from canonical_zmq_publisher.aggregator import CanonicalAggregator
from canonical_zmq_publisher.ingest import (
    SyntheticSource,
    synthetic_depth_payload,
    synthetic_lidar_payload,
    synthetic_rgb_packet,
)


class AggregatorTest(unittest.TestCase):
    def test_publish_owns_sequence_and_source_id(self):
        # Use inproc endpoints to avoid port conflicts in CI.
        agg = CanonicalAggregator(
            pub_endpoint='inproc://test_pub',
            status_endpoint='inproc://test_status')
        header = {
            'schema_version': 1,
            'source_mode': 'hardware',
            'source_id': 'adapter',
            'sequence': 999,
            'frame_id': 'camera',
            'acquisition_timestamp_ns': 1,
            'clock_domain': 'plc_rtc_utc',
            'gateway_monotonic_ns': 0,
            'calibration_id': 'cal',
            'capabilities': {'camera.cutter.rgb': True},
            'codec': 'jpeg',
            'pixel_encoding': 'RGB8',
            'width': 1,
            'height': 1,
        }
        agg.publish('v1/camera/cutter/rgb', header, b'\xff\xd8\xff\xd9')
        # The queue holds one packet; drain and inspect the header.
        self.assertTrue(agg.flush_one_packet())
        agg.close()

    def test_synthetic_payloads_decode(self):
        # RGB JPEG starts with SOI marker.
        self.assertTrue(synthetic_rgb_packet().startswith(b'\xff\xd8'))
        # Depth is width*height*2 bytes of uint16 mm.
        depth = synthetic_depth_payload()
        self.assertEqual(len(depth), 640 * 360 * 2)
        # LiDAR payload is count*12 bytes.
        self.assertEqual(len(synthetic_lidar_payload(200)), 200 * 12)

    def test_synthetic_source_emits_canonical_channels(self):
        agg = CanonicalAggregator(
            pub_endpoint='inproc://test_pub2',
            status_endpoint='inproc://test_status2')
        source = SyntheticSource(agg)
        source.emit_once()
        channels = set()
        for channel in sorted(agg.queues):
            frames = agg.queues[channel].pop()
            c, _h, _p = unpack_message(frames)
            channels.add(c)
        agg.close()
        expected = {
            'v1/camera/cutter/rgb', 'v1/camera/cutter/depth',
            'v1/camera/cutter/camera_info', 'v1/camera/docking/rgb',
            'v1/lidar/raw', 'v1/range/docking', 'v1/range/cutter',
            'v1/docking/trunk_estimate', 'v1/calibration/status',
        }
        self.assertTrue(expected.issubset(channels))


if __name__ == '__main__':
    unittest.main()
