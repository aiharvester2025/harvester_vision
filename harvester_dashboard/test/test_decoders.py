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
    def test_decodes_synthetic_jpeg_to_header_dimensions(self):
        frames = jpeg_packet('v1/camera/cutter/rgb', width=40, height=30)
        channel, header, payload = unpack_message(frames)
        array = JpegDecoder().decode(header, payload)
        self.assertEqual(array.shape, (30, 40, 3))
        self.assertEqual(array.dtype, np.uint8)

    def test_rejects_size_mismatch(self):
        frames = jpeg_packet('v1/camera/cutter/rgb', width=40, height=30)
        channel, header, payload = unpack_message(frames)
        header['width'] = 41
        with self.assertRaises(ValueError):
            JpegDecoder().decode(header, payload)

    def test_rgb_decode_selects_by_codec(self):
        frames = jpeg_packet('v1/camera/docking/rgb')
        _channel, header, payload = unpack_message(frames)
        array = decode_rgb(header, payload)
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
    def test_h264_stub_raises_clear_error_without_crash(self):
        frames = h264_packet('v1/camera/cutter/rgb')
        _channel, header, payload = unpack_message(frames)
        decoder = decoder_for_codec('h264')
        with self.assertRaises(UnsupportedCodecError) as caught:
            decoder.decode(header, payload)
        self.assertIn('H.264', str(caught.exception))

    def test_h265_stub_raises_clear_error(self):
        decoder = decoder_for_codec('h265')
        with self.assertRaises(UnsupportedCodecError) as caught:
            decoder.decode(base_header(codec='h265'), b'\x00\x00\x00\x01\x42')
        self.assertIn('H.265', str(caught.exception))

    def test_decode_frame_routes_h264_to_stub(self):
        from harvester_dashboard.zmq_source import SocketDrainer
        frames = h264_packet('v1/camera/cutter/rgb')
        _channel, header, payload = unpack_message(frames)
        with self.assertRaises(UnsupportedCodecError):
            SocketDrainer.decode_frame('v1/camera/cutter/rgb', header, payload)

    def test_unknown_codec_rejected(self):
        with self.assertRaises(UnsupportedCodecError):
            decoder_for_codec('av1')


if __name__ == '__main__':
    unittest.main()
