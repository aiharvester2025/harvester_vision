"""Tests for the OAK capture adapter's pure-Python helpers (no hardware)."""

import unittest

from harvester_telemetry_contract import unpack_message, pack_message
from canonical_zmq_publisher.oak_capture import (
    build_rgb_header,
    detect_keyframe,
)


def _h264_keyframe():
    # Annex-B: SPS + PPS + IDR slice (nal_type 5).
    return (b'\x00\x00\x00\x01\x67' + b'\x00' * 8 +
            b'\x00\x00\x00\x01\x68' + b'\x00' * 4 +
            b'\x00\x00\x00\x01\x65' + b'\x00' * 16)


def _h264_pframe():
    # Annex-B: non-IDR slice (nal_type 1).
    return b'\x00\x00\x00\x01\x41' + b'\x00' * 16


def _h265_keyframe():
    # Annex-B: IDR_W_RADL (nal_type 19 -> 19<<1 = 0x26).
    return b'\x00\x00\x00\x01\x26' + b'\x00' * 16


class HeaderTest(unittest.TestCase):
    def test_build_rgb_header_is_canonical(self):
        header = build_rgb_header(
            'docking_camera', 'h264', 1280, 720, True,
            123456789000, 'docking_camera_optical_frame')
        self.assertEqual(header['codec'], 'h264')
        self.assertEqual(header['pixel_encoding'], 'H264')
        self.assertEqual(header['source_mode'], 'hardware')
        self.assertEqual(header['clock_domain'], 'plc_rtc_utc')
        self.assertTrue(header['keyframe'])
        self.assertEqual(header['width'], 1280)
        self.assertEqual(header['height'], 720)
        # The header must round-trip through the contract (sequence is owned by
        # the aggregator, so fill a value before packing).
        header['sequence'] = 7
        header['source_id'] = 'orin'
        header['gateway_monotonic_ns'] = 1
        channel, validated, _payload = unpack_message(
            pack_message('v1/camera/docking/rgb', header, b'\x00\x00\x00\x01\x65'))
        self.assertEqual(channel, 'v1/camera/docking/rgb')
        self.assertEqual(validated['pixel_encoding'], 'H264')

    def test_h265_header_pixel_encoding(self):
        header = build_rgb_header(
            'cutting_camera', 'h265', 640, 360, False, 1, 'cutter_camera_optical_frame')
        self.assertEqual(header['pixel_encoding'], 'H265')
        self.assertFalse(header['keyframe'])

    def test_jpeg_header(self):
        header = build_rgb_header('docking_camera', 'jpeg', 640, 360, True, 1, 'f')
        self.assertEqual(header['pixel_encoding'], 'RGB8')


class KeyframeTest(unittest.TestCase):
    def test_h264_idr_detected(self):
        self.assertTrue(detect_keyframe('h264', _h264_keyframe()))

    def test_h264_pframe_not_keyframe(self):
        self.assertFalse(detect_keyframe('h264', _h264_pframe()))

    def test_h265_irap_detected(self):
        self.assertTrue(detect_keyframe('h265', _h265_keyframe()))

    def test_jpeg_always_keyframe(self):
        self.assertTrue(detect_keyframe('jpeg', b'\xff\xd8\xff\xd9'))


if __name__ == '__main__':
    unittest.main()
