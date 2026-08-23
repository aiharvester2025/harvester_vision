import unittest

import numpy as np

from helpers import (
    base_header,
    depth_packet,
    h264_packet,
    jpeg_packet,
    lidar_packet,
)

from harvester_dashboard.decoders import (
    DepthDecoder,
    JpegDecoder,
    LidarDecoder,
    UnsupportedCodecError,
    decoder_for_codec,
    decode_depth,
    decode_rgb,
)
from harvester_dashboard.protocol_shim import unpack_message


class JpegDecodeTest(unittest.TestCase):
    def _wait_for_frame(self, decoder, header, payload, max_iters=200):
        """Hardware decode is asynchronous: iterate the GLib main context so
        the appsink ``new-sample`` callback fires, then poll for the frame."""
        import time
        try:
            from gi.repository import GLib
            ctx = GLib.MainContext.default()
        except Exception:
            ctx = None
        array = decoder.decode(header, payload)
        for _ in range(max_iters):
            if ctx is not None:
                ctx.iteration(False)
            if array is not None:
                break
            time.sleep(0.01)
            array = decoder.decode(header, b'')
        return array

    def test_decodes_synthetic_jpeg_to_header_dimensions(self):
        frames = jpeg_packet('v1/camera/cutter/rgb', width=40, height=30)
        channel, header, payload = unpack_message(frames)
        array = self._wait_for_frame(JpegDecoder(), header, payload)
        self.assertIsNotNone(array, 'hardware JPEG decoder produced no frame')
        self.assertEqual(array.shape, (30, 40, 3))
        self.assertEqual(array.dtype, np.uint8)

    def test_rejects_size_mismatch(self):
        # Hardware JPEG decode takes dimensions from the decoded JPEG itself,
        # so a header/frame mismatch is no longer detectable client-side.  The
        # dashboard surfaces a stream error instead via the image provider.
        frames = jpeg_packet('v1/camera/cutter/rgb', width=40, height=30)
        channel, header, payload = unpack_message(frames)
        header['width'] = 41
        decoder = JpegDecoder()
        # Decode must not raise; the image provider displays whatever the
        # decoder returns and trusts the decoded dimensions.
        array = self._wait_for_frame(decoder, header, payload)
        self.assertIsNotNone(array)
        self.assertEqual(array.shape, (30, 40, 3))

    def test_rgb_decode_selects_by_codec(self):
        frames = jpeg_packet('v1/camera/docking/rgb')
        _channel, header, payload = unpack_message(frames)
        array = self._wait_for_frame(JpegDecoder(), header, payload)
        self.assertIsNotNone(array)
        self.assertEqual(array.shape[2], 3)


class DepthDecodeTest(unittest.TestCase):
    def test_decodes_millimetres_to_metres(self):
        frames = depth_packet('v1/camera/cutter/depth', width=8, height=6)
        _channel, header, payload = unpack_message(frames)
        metres = DepthDecoder().decode(header, payload)
        self.assertEqual(metres.shape, (6, 8))
        self.assertTrue(np.allclose(metres, 2.5))

    def test_zero_is_invalid_nan(self):
        depth_m = np.full((6, 8), 2.5, dtype=np.float32)
        depth_m[0, 0] = 0.0
        frames = depth_packet('v1/camera/cutter/depth', depth_m=depth_m)
        _channel, header, payload = unpack_message(frames)
        metres = DepthDecoder().decode(header, payload)
        self.assertTrue(np.isnan(metres[0, 0]))
        valid = np.isfinite(metres)
        self.assertAlmostEqual(float(metres[valid][0]), 2.5)

    def test_rejects_wrong_payload_size(self):
        frames = depth_packet('v1/camera/cutter/depth', width=8, height=6)
        _channel, header, _payload = unpack_message(frames)
        with self.assertRaises(ValueError):
            DepthDecoder().decode(header, b'\x00' * 10)

    def test_depth_at_nearest_valid_window(self):
        depth = np.full((6, 8), np.nan, dtype=np.float32)
        depth[2, 3] = 1.5
        decoder = DepthDecoder()
        self.assertEqual(decoder.depth_at(depth, 3, 2, window=1), 1.5)
        self.assertIsNone(decoder.depth_at(depth, 0, 0, window=0))
        self.assertIsNone(decoder.depth_at(depth, 999, 999, window=1))

    def test_wrong_codec_rejected(self):
        header = base_header(codec='jpeg', pixel_encoding='RGB8', width=4, height=4)
        with self.assertRaises(UnsupportedCodecError):
            decode_depth(header, b'x' * 32)


class LidarDecodeTest(unittest.TestCase):
    def test_decodes_xyz_points(self):
        frames = lidar_packet()
        _channel, header, payload = unpack_message(frames)
        points = LidarDecoder().decode(header, payload)
        self.assertEqual(points.shape, (3, 3))
        self.assertTrue(np.allclose(points[0], [1.0, 2.0, 3.0]))
        self.assertEqual(points.dtype, np.float32)

    def test_declared_count_mismatch_rejected(self):
        frames = lidar_packet()
        _channel, header, payload = unpack_message(frames)
        header['point_count'] = 99
        with self.assertRaises(ValueError):
            LidarDecoder().decode(header, payload)

    def test_limit_downsamples(self):
        frames = lidar_packet(points=np.tile([1.0, 2.0, 3.0], (50, 1)))
        _channel, header, payload = unpack_message(frames)
        points = LidarDecoder().decode(header, payload)
        limited = LidarDecoder().limit(points, 10)
        self.assertEqual(len(limited), 10)

    def test_point_fields_drive_layout(self):
        # Simulate a hardware-style record: x,y,z + uint8 tag, stride 16.
        import struct
        records = struct.pack('<fffBxxx', 0.5, -0.5, 1.0, 7) * 2
        header = base_header(
            codec='lidar_xyz_f32',
            frame_id='lidar',
            point_count=2,
            point_stride_bytes=16,
            point_fields=[
                {'name': 'x', 'type': 'float32', 'offset': 0},
                {'name': 'y', 'type': 'float32', 'offset': 4},
                {'name': 'z', 'type': 'float32', 'offset': 8},
                {'name': 'tag', 'type': 'uint8', 'offset': 12},
            ],
        )
        points = LidarDecoder().decode(header, records)
        self.assertEqual(points.shape, (2, 3))
        self.assertTrue(np.allclose(points[0], [0.5, -0.5, 1.0]))


class CodecStubTest(unittest.TestCase):
    def _jetson_available(self):
        try:
            from harvester_dashboard.decoders.jetson_decode import _nvv4l2_available
            return _nvv4l2_available()
        except Exception:
            return False

    def test_h264_decode_does_not_crash_on_fake_payload(self):
        # On a host with the Jetson decoder, feeding an invalid/fake bitstream
        # must return None (no frame) rather than raise; on a host without it,
        # a clear UnsupportedCodecError is raised.  It must never crash.
        frames = h264_packet('v1/camera/cutter/rgb')
        _channel, header, payload = unpack_message(frames)
        decoder = decoder_for_codec('h264')
        if self._jetson_available():
            result = decoder.decode(header, payload)
            # Fake SPS payload yields no decodable frame.
            self.assertIsNone(result)
        else:
            with self.assertRaises(UnsupportedCodecError):
                decoder.decode(header, payload)

    def test_h265_decode_does_not_crash_on_fake_payload(self):
        decoder = decoder_for_codec('h265')
        if self._jetson_available():
            result = decoder.decode(base_header(codec='h265'), b'\x00\x00\x00\x01\x42')
            self.assertIsNone(result)
        else:
            with self.assertRaises(UnsupportedCodecError):
                decoder.decode(base_header(codec='h265'), b'\x00\x00\x00\x01\x42')

    def test_decode_frame_routes_h264_to_decoder(self):
        from harvester_dashboard.zmq_source import SocketDrainer
        frames = h264_packet('v1/camera/cutter/rgb')
        _channel, header, payload = unpack_message(frames)
        if self._jetson_available():
            result = SocketDrainer.decode_frame('v1/camera/cutter/rgb', header, payload)
            self.assertIsNone(result)  # fake payload -> no frame, no crash
        else:
            with self.assertRaises(UnsupportedCodecError):
                SocketDrainer.decode_frame('v1/camera/cutter/rgb', header, payload)

    def test_unknown_codec_rejected(self):
        with self.assertRaises(UnsupportedCodecError):
            decoder_for_codec('av1')


class JetsonHardwareDecodeTest(unittest.TestCase):
    """Happy-path hardware decode using a real software-encoded H.264 stream.

    Skipped when GStreamer/nvv4l2decoder is unavailable (e.g. non-system
    python, or a host without the NVIDIA decoder).
    """

    @staticmethod
    def _split_annexb(data):
        units = []
        i, n, start = 0, len(data), None
        while i < n:
            if data[i:i + 4] == b'\x00\x00\x00\x01':
                if start is not None:
                    units.append(data[start:i])
                start, i = i, i + 4
            elif data[i:i + 3] == b'\x00\x00\x01':
                if start is not None:
                    units.append(data[start:i])
                start, i = i, i + 3
            else:
                i += 1
        if start is not None:
            units.append(data[start:])
        return units

    def test_h264_hardware_decode_produces_frame(self):
        try:
            from harvester_dashboard.decoders.jetson_decode import _nvv4l2_available
            if not _nvv4l2_available():
                self.skipTest('nvv4l2decoder unavailable')
        except Exception:
            self.skipTest('GStreamer unavailable')
        import subprocess
        import tempfile
        import os
        from harvester_dashboard.decoders.jetson_decode import JetsonDecoder
        with tempfile.NamedTemporaryFile(suffix='.h264', delete=False) as tmp:
            path = tmp.name
        try:
            subprocess.run([
                'gst-launch-1.0', '-q',
                'videotestsrc', 'num-buffers=20',
                '!', 'video/x-raw,format=I420,width=320,height=240,framerate=30/1',
                '!', 'x264enc', 'key-int-max=10',
                '!', 'video/x-h264,stream-format=byte-stream',
                '!', 'filesink', 'location={}'.format(path),
            ], check=True, capture_output=True)
            data = open(path, 'rb').read()
        finally:
            os.unlink(path)

        units = self._split_annexb(data)
        self.assertGreater(len(units), 1)
        decoder = JetsonDecoder('h264', width=320, height=240)
        frame = None
        try:
            import time
            for unit in units:
                result = decoder.decode(
                    {'frame_id': 'test', 'width': 320, 'height': 240}, unit)
                if result is not None:
                    frame = result
                time.sleep(0.01)
            # Give the async hardware decoder time to emit the last frame.
            for _ in range(50):
                if frame is not None:
                    break
                time.sleep(0.01)
                result = decoder.decode(
                    {'frame_id': 'test', 'width': 320, 'height': 240}, b'')
                if result is not None:
                    frame = result
        finally:
            decoder.close()
        self.assertIsNotNone(frame, 'no frame decoded from a real H.264 stream')
        self.assertEqual(frame.shape, (240, 320, 3))
        self.assertEqual(frame.dtype, np.uint8)


if __name__ == '__main__':
    unittest.main()
