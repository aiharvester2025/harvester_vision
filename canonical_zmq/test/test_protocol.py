import unittest

from harvester_telemetry_contract.protocol import (
    ProtocolError,
    pack_message,
    unpack_message,
)


def image_header():
    return {
        'schema_version': 1,
        'source_mode': 'simulation',
        'source_id': 'xavier',
        'sequence': 12,
        'frame_id': 'platform_depth_camera_optical_frame',
        'acquisition_timestamp_ns': 1000000000,
        'clock_domain': 'ros_sim_time',
        'gateway_monotonic_ns': 2000000000,
        'calibration_id': 'gazebo_nominal_camera_lidar_v1',
        'capabilities': {'packet.recording': True},
        'codec': 'jpeg',
        'pixel_encoding': 'RGB8',
        'width': 640,
        'height': 400,
    }


class ProtocolTest(unittest.TestCase):
    def test_round_trip_image_packet(self):
        frames = pack_message('v1/camera/cutter/rgb', image_header(), b'jpeg')
        channel, header, payload = unpack_message(frames)
        self.assertEqual(channel, 'v1/camera/cutter/rgb')
        self.assertEqual(header['sequence'], 12)
        self.assertEqual(payload, b'jpeg')

    def test_depth_requires_normalized_codec(self):
        header = image_header()
        header.update({'codec': 'jpeg', 'pixel_encoding': ''})
        with self.assertRaises(ProtocolError):
            pack_message('v1/camera/cutter/depth', header, b'')

    def test_lidar_requires_layout(self):
        header = image_header()
        header.update({
            'frame_id': 'vehicle_lidar_link',
            'codec': 'lidar_xyz_f32',
            'point_count': 1,
            'point_stride_bytes': 12,
            'point_fields': [
                {'name': 'x', 'type': 'float32'},
                {'name': 'y', 'type': 'float32'},
                {'name': 'z', 'type': 'float32'},
            ],
        })
        frames = pack_message('v1/lidar/raw', header, b'\0' * 12)
        self.assertEqual(unpack_message(frames)[1]['point_count'], 1)

    def test_rejects_noncanonical_packet_shape(self):
        with self.assertRaises(ProtocolError):
            unpack_message([b'v1/system/status'])

    def test_boom_state_round_trip(self):
        header = {
            'schema_version': 1,
            'source_mode': 'hardware',
            'source_id': 'pi_plc',
            'sequence': 1,
            'frame_id': 'boom_link',
            'acquisition_timestamp_ns': 1000000000,
            'clock_domain': 'plc_rtc_utc',
            'gateway_monotonic_ns': 2000000000,
            'calibration_id': 'boom_provisional_v0',
            'capabilities': {'boom.state': True},
            'codec': 'json',
        }
        payload = b'{"phase":"BOOM_RAISE","boom_angle_deg":12.0}'
        frames = pack_message('v1/boom/state', header, payload)
        channel, _h, out = unpack_message(frames)
        self.assertEqual(channel, 'v1/boom/state')
        self.assertEqual(out, payload)


if __name__ == '__main__':
    unittest.main()
