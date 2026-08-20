"""End-to-end replay validation without GUI: SUB -> model population.

Run with the replay publisher active on tcp://127.0.0.1:5591::

    PYTHONPATH=src/harvester_dashboard \\
    /usr/bin/python3 -m unittest test_replay_ingest -v

This is a manual/live test (skipped by default) because it needs the
replay fixture process.  It proves the dashboard receives, validates,
and decodes every channel from the real recorded audit.
"""

import os
import unittest

REPLAY_PUB = os.environ.get('DASHBOARD_TEST_PUB', 'tcp://127.0.0.1:5591')
REPLAY_REQUIRED = os.environ.get('DASHBOARD_TEST_REPLAY') == '1'


@unittest.skipUnless(REPLAY_REQUIRED, 'set DASHBOARD_TEST_REPLAY=1 with replay running')
class ReplayIngestTest(unittest.TestCase):
    def test_all_channels_ingested_from_replay(self):
        import time
        from harvester_dashboard.zmq_source import SocketDrainer
        from harvester_dashboard.model.telemetry_model import TelemetryModel

        model = TelemetryModel()
        drainer = SocketDrainer(REPLAY_PUB, hwm=256)
        drainer.on_packet = (
            lambda channel, header, payload, parsed:
            model.ingest_packet(channel, header, payload))
        required = {
            'v1/camera/cutter/rgb', 'v1/camera/cutter/depth',
            'v1/camera/cutter/camera_info',
            'v1/range/docking', 'v1/range/cutter',
            'v1/docking/trunk_estimate', 'v1/calibration/status',
            'v1/system/status', 'v1/lidar/raw',
        }
        deadline = time.monotonic() + 25.0
        while time.monotonic() < deadline:
            drainer.drain_once(max_packets=256)
            if required <= set(model.channels):
                break
            time.sleep(0.05)
        drainer.close()

        missing = required - set(model.channels)
        self.assertFalse(missing, 'replay did not deliver: {}'.format(missing))

        docking, cutter = model.snapshot_ranges()
        self.assertTrue(docking, 'docking ranges missing')
        self.assertIsInstance(cutter, dict)
        trunk = model.snapshot_trunk()
        self.assertIsInstance(trunk, dict)
        payload, valid = model.snapshot_calibration()
        self.assertIsInstance(payload, dict)
        self.assertEqual(model.source_mode(), 'SIMULATION')
        self.assertFalse(model.is_mixed())

        # Decode one RGB and depth payload end to end.
        from harvester_dashboard.zmq_source import SocketDrainer as SD
        rgb_state = model.state('v1/camera/cutter/rgb')
        depth_state = model.state('v1/camera/cutter/depth')
        rgb = SD.decode_frame('v1/camera/cutter/rgb',
                              rgb_state.last_header, rgb_state.last_payload)
        depth = SD.decode_frame('v1/camera/cutter/depth',
                                depth_state.last_header, depth_state.last_payload)
        self.assertEqual(rgb.ndim, 3)
        self.assertEqual(depth.ndim, 2)
        self.assertEqual(model.last_system_status.get('source_mode'), 'simulation')


if __name__ == '__main__':
    unittest.main()
