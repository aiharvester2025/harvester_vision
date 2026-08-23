import unittest

import zmq

from helpers import (
    depth_packet,
    json_packet,
    jpeg_packet,
    lidar_packet,
)

from harvester_dashboard.protocol_shim import unpack_message
from harvester_dashboard.zmq_source import SocketDrainer


class InprocPubSubTest(unittest.TestCase):
    """Drain/drop semantics against a real (inproc) PUB socket."""

    def setUp(self):
        self.context = zmq.Context()
        self.pub = self.context.socket(zmq.PUB)
        self.pub.setsockopt(zmq.LINGER, 0)
        self.endpoint = 'inproc://dashboard-test-{}'.format(id(self))
        self.pub.bind(self.endpoint)
        self.drainer = SocketDrainer(self.endpoint, hwm=64, context=self.context)
        self.received = []

        def collect(channel, header, payload, parsed):
            self.received.append((channel, header, payload, parsed))

        self.drainer.on_packet = collect
        # Allow the SUB subscription to propagate through the inproc pipe.
        self._drain_idle()

    def _drain_idle(self):
        for _ in range(4):
            self.drainer.drain_once()
            self.pub.send(b'')          # ping to flush subscription state
            self.drainer.drain_once()

    def tearDown(self):
        self.drainer.close()
        self.pub.close(0)
        self.context.term()

    def _send(self, frames):
        self.pub.send_multipart(frames)
        for _ in range(10):
            if self.drainer.drain_once():
                break

    def test_receives_valid_json_packet(self):
        self._send(json_packet('v1/range/cutter', {'distance_m': 0.7}))
        self.assertEqual(len(self.received), 1)
        channel, header, _payload, parsed = self.received[0]
        self.assertEqual(channel, 'v1/range/cutter')
        self.assertEqual(parsed['distance_m'], 0.7)
        self.assertEqual(self.drainer.total_received, 1)

    def test_receives_image_and_lidar_packets(self):
        self._send(jpeg_packet('v1/camera/cutter/rgb', width=16, height=12))
        self._send(depth_packet('v1/camera/cutter/depth', width=4, height=3))
        self._send(lidar_packet())
        channels = {entry[0] for entry in self.received}
        self.assertEqual(channels, {
            'v1/camera/cutter/rgb',
            'v1/camera/cutter/depth',
            'v1/lidar/raw',
        })

    def test_contract_violation_counted_not_crashing(self):
        self.pub.send_multipart([b'v1/range/cutter', b'garbage-header', b'x'])
        while not self.drainer.drain_once():
            pass
        self.assertEqual(self.drainer.total_decode_errors, 1)
        self.assertEqual(self.drainer.total_received, 0)
        # The socket still works afterwards.
        self._send(json_packet('v1/range/cutter', {'distance_m': 1.0}))
        self.assertEqual(self.drainer.total_received, 1)

    def test_drain_once_returns_zero_when_idle(self):
        self.assertEqual(self.drainer.drain_once(), 0)

    def test_decode_frame_decodes_jpeg(self):
        from harvester_dashboard.zmq_source import SocketDrainer as SD
        # Clear any cached JPEG session from earlier tests in this run so the
        # first push-buffer in this test sees a fresh pipeline.
        SD._stateful_decoders.pop(('jpeg', 'test_frame'), None)
        frames = jpeg_packet('v1/camera/cutter/rgb', width=16, height=12)
        _c, header, payload = unpack_message(frames)
        # Hardware JPEG decode is asynchronous and needs the GLib main context
        # to iterate so the appsink new-sample callback fires.  In the live
        # dashboard Qt drives this loop; in unit tests we iterate manually.
        import time
        try:
            from gi.repository import GLib
            ctx = GLib.MainContext.default()
        except Exception:
            ctx = None
        array = SD.decode_frame('v1/camera/cutter/rgb', header, payload)
        for _ in range(2000):
            if ctx is not None:
                ctx.iteration(False)
            if array is not None:
                break
            time.sleep(0.001)
            array = SD.decode_frame('v1/camera/cutter/rgb', header, b'')
        self.assertIsNotNone(array)
        self.assertEqual(array.shape, (12, 16, 3))

    def test_decode_frame_returns_none_for_json(self):
        frames = json_packet('v1/range/cutter', {})
        _c, header, payload = unpack_message(frames)
        self.assertIsNone(
            SocketDrainer.decode_frame('v1/range/cutter', header, payload))


if __name__ == '__main__':
    unittest.main()
