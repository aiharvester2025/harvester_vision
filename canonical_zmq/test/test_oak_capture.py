"""Tests for the OAK capture adapter's pure-Python helpers (no hardware)."""

import unittest

from harvester_telemetry_contract import unpack_message, pack_message
from canonical_zmq_publisher.oak_capture import (
    build_rgb_header,
    build_depth_header,
    build_camera_info_header,
    build_imu_header,
    build_capabilities,
    detect_keyframe,
    gravity_to_rpy,
    _rotate_vec3,
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

    def test_rgb_capabilities_depth_disabled_by_default(self):
        # The RGB header alone must NOT claim depth/camera_info (only the
        # adapter that actually publishes those channels flips them on).
        header = build_rgb_header('docking_camera', 'h265', 640, 360, True, 1, 'f')
        caps = header['capabilities']
        self.assertTrue(caps['camera.docking.rgb'])
        self.assertFalse(caps['camera.docking.depth'])
        self.assertFalse(caps['camera.docking.camera_info'])


class DepthHeaderTest(unittest.TestCase):
    def test_build_depth_header_is_canonical(self):
        header = build_depth_header(
            'docking_camera', 1280, 720, 123456789000,
            'docking_camera_optical_frame', publish_camera_info=True)
        self.assertEqual(header['codec'], 'depth_uint16_le')
        self.assertEqual(header['width'], 1280)
        self.assertEqual(header['height'], 720)
        self.assertTrue(header['keyframe'])
        # No pixel_encoding for depth channels (only /rgb declares it).
        self.assertNotIn('pixel_encoding', header)
        # Depth publishing flips the depth + camera_info capability flags.
        caps = header['capabilities']
        self.assertTrue(caps['camera.docking.depth'])
        self.assertTrue(caps['camera.docking.camera_info'])
        self.assertTrue(caps['camera.docking.rgb'])
        # Round-trips through the contract (sequence/source_id/gateway owned by
        # the aggregator, so fill them before packing).
        header['sequence'] = 7
        header['source_id'] = 'orin'
        header['gateway_monotonic_ns'] = 1
        channel, validated, _payload = unpack_message(
            pack_message('v1/camera/docking/depth', header, b'\x00' * (1280 * 720 * 2)))
        self.assertEqual(channel, 'v1/camera/docking/depth')
        self.assertEqual(validated['codec'], 'depth_uint16_le')

    def test_depth_capabilities_flag_off_when_not_publishing_info(self):
        header = build_depth_header(
            'cutting_camera', 640, 360, 1, 'cutter_camera_optical_frame',
            publish_camera_info=False)
        self.assertTrue(header['capabilities']['camera.cutter.depth'])
        self.assertFalse(header['capabilities']['camera.cutter.camera_info'])


class CameraInfoHeaderTest(unittest.TestCase):
    def test_build_camera_info_header_is_canonical(self):
        header = build_camera_info_header(
            'docking_camera', 1280, 720, 123456789000,
            'docking_camera_optical_frame', publish_depth=True)
        self.assertEqual(header['codec'], 'json')
        self.assertEqual(header['width'], 1280)
        self.assertEqual(header['height'], 720)
        caps = header['capabilities']
        self.assertTrue(caps['camera.docking.camera_info'])
        self.assertTrue(caps['camera.docking.depth'])
        header['sequence'] = 1
        header['source_id'] = 'orin'
        header['gateway_monotonic_ns'] = 1
        channel, validated, _payload = unpack_message(
            pack_message('v1/camera/docking/camera_info', header, b'{}'))
        self.assertEqual(channel, 'v1/camera/docking/camera_info')
        self.assertEqual(validated['codec'], 'json')


class ImuHeaderTest(unittest.TestCase):
    def test_build_imu_header_is_canonical(self):
        header = build_imu_header(
            'docking_camera', 123456789000, 'docking_camera_optical_frame')
        self.assertEqual(header['codec'], 'json')
        # IMU has no image geometry -> no width/height, no pixel_encoding.
        self.assertNotIn('width', header)
        self.assertNotIn('height', header)
        self.assertNotIn('pixel_encoding', header)
        self.assertTrue(header['capabilities']['camera.docking.imu'])
        header['sequence'] = 7
        header['source_id'] = 'orin'
        header['gateway_monotonic_ns'] = 1
        channel, validated, _payload = unpack_message(
            pack_message('v1/camera/docking/imu', header, b'{}'))
        self.assertEqual(channel, 'v1/camera/docking/imu')
        self.assertEqual(validated['codec'], 'json')

    def test_imu_capability_default_off(self):
        caps = build_capabilities('cutting_camera')
        self.assertFalse(caps['camera.cutter.imu'])
        caps = build_capabilities('cutting_camera', imu=True)
        self.assertTrue(caps['camera.cutter.imu'])

    def test_gravity_to_rpy_optical_frame(self):
        roll, pitch, norm = gravity_to_rpy([0.0, 0.0, -9.80665])
        self.assertAlmostEqual(roll, 0.0, places=6)
        self.assertAlmostEqual(pitch, 0.0, places=6)
        self.assertAlmostEqual(norm, 9.80665, places=5)
        roll, _pitch, _n = gravity_to_rpy([0.0, 9.80665, 0.0])
        self.assertAlmostEqual(roll, -1.57079632679, places=4)

    def test_rotate_vec3_identity(self):
        identity = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        result = _rotate_vec3(identity, (1.0, 2.0, 3.0))
        self.assertEqual(result, (1.0, 2.0, 3.0))

    def test_rotate_vec3_rotates(self):
        # 90-degree rotation about Z maps +X -> +Y.
        rot_z_90 = [0.0, -1.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0]
        result = _rotate_vec3(rot_z_90, (1.0, 0.0, 0.0))
        self.assertAlmostEqual(result[0], 0.0, places=6)
        self.assertAlmostEqual(result[1], 1.0, places=6)
        self.assertAlmostEqual(result[2], 0.0, places=6)


class CapabilitiesTest(unittest.TestCase):
    def test_build_capabilities_defaults(self):
        caps = build_capabilities('cutting_camera')
        self.assertTrue(caps['camera.cutter.rgb'])
        self.assertFalse(caps['camera.cutter.depth'])
        self.assertFalse(caps['camera.cutter.camera_info'])
        self.assertFalse(caps['target.world_fixed'])

    def test_build_capabilities_depth_and_info(self):
        caps = build_capabilities('cutting_camera', depth=True, camera_info=True)
        self.assertTrue(caps['camera.cutter.depth'])
        self.assertTrue(caps['camera.cutter.camera_info'])


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
