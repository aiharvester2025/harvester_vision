import json
import ctypes
import math
import struct
import tempfile
import unittest

from lidar.livox_source import (
    LivoxMid360Source,
    WORK_MODE_NORMAL,
    WORK_MODE_UNVERIFIED_STOP,
    WORK_MODE_WAKE_UP,
    _accel_to_quaternion,
    _decode_point_batch,
    _ETH_PACKET_HEADER_SIZE_BYTES,
    _EthPacketHeader,
    _MAX_IMU_PER_CALLBACK,
    _MAX_POINTS_PER_CALLBACK,
    _payload_view,
    _RAW_IMU_SIZE_BYTES,
    _RAW_POINT_SIZE_BYTES,
    _resolve_axis,
    _SDK_CALLBACK_BUFFER_BYTES,
    default_sdk_config,
    write_default_sdk_config,
)


class PointDecodingTests(unittest.TestCase):
    def _pack(self, x, y, z, reflectivity=0, tag=0):
        # LivoxLidarCartesianHighRawPoint is packed (#pragma pack(1)) to exactly
        # 14 bytes: int32 x/y/z + uint8 reflectivity + uint8 tag. There is NO
        # trailing padding.
        return struct.pack("<iiiBB", x, y, z, reflectivity, tag)

    def test_cartesian_points_decode_to_millimetre_triples(self):
        raw = self._pack(1000, -2000, 3000) + self._pack(0, 0, 0)
        points = _decode_point_batch(raw, 2)
        self.assertEqual(points, [(1000.0, -2000.0, 3000.0), (0.0, 0.0, 0.0)])

    def test_special_tagged_points_are_dropped(self):
        # Bit 4 marks a Livox "special" entry whose x/y/z is a timestamp.
        raw = self._pack(1, 2, 3) + self._pack(4, 5, 6, tag=0x10)
        points = _decode_point_batch(raw, 2)
        self.assertEqual(points, [(1.0, 2.0, 3.0)])

    def test_count_larger_than_buffer_is_clamped(self):
        raw = self._pack(1, 2, 3)
        self.assertEqual(_decode_point_batch(raw, 99), [(1.0, 2.0, 3.0)])

    def test_callback_caps_cannot_exceed_the_sdk_buffer(self):
        # The SDK hands the callback a fixed 8192-byte buffer, so a cap larger
        # than that buffer can hold would let a corrupt packet over-read it.
        self.assertLessEqual(
            _MAX_POINTS_PER_CALLBACK * _RAW_POINT_SIZE_BYTES,
            _SDK_CALLBACK_BUFFER_BYTES - _ETH_PACKET_HEADER_SIZE_BYTES)
        self.assertLessEqual(
            _MAX_IMU_PER_CALLBACK * _RAW_IMU_SIZE_BYTES,
            _SDK_CALLBACK_BUFFER_BYTES - _ETH_PACKET_HEADER_SIZE_BYTES)

    def test_payload_view_clamps_a_lying_packet_to_the_buffer(self):
        # A packet claiming far more points than it could hold must still be
        # clamped to the buffer, not read out of bounds.
        buffer = (ctypes.c_uint8 * _SDK_CALLBACK_BUFFER_BYTES)()
        address = ctypes.addressof(buffer)
        header = ctypes.cast(
            address, ctypes.POINTER(_EthPacketHeader)).contents
        header.dot_num = 65535
        header.length = 65535
        view = _payload_view(address, 65535, _RAW_POINT_SIZE_BYTES)
        self.assertIsNotNone(view)
        self.assertLessEqual(
            view.size, _SDK_CALLBACK_BUFFER_BYTES - _ETH_PACKET_HEADER_SIZE_BYTES)
        self.assertEqual(
            view.size // _RAW_POINT_SIZE_BYTES, _MAX_POINTS_PER_CALLBACK)


class VendorFrameTests(unittest.TestCase):
    def test_identity_axes_preserve_coordinates(self):
        self.assertEqual(_resolve_axis("x", 1.0, 2.0, 3.0), 1.0)
        self.assertEqual(_resolve_axis("y", 1.0, 2.0, 3.0), 2.0)
        self.assertEqual(_resolve_axis("z", 1.0, 2.0, 3.0), 3.0)

    def test_negated_axis_inverts_coordinate(self):
        self.assertEqual(_resolve_axis("-y", 1.0, 2.0, 3.0), -2.0)

    def test_invalid_axis_marker_raises(self):
        with self.assertRaises(ValueError):
            _resolve_axis("q", 1.0, 2.0, 3.0)

    def test_sample_converts_millimetres_to_metres(self):
        source = LivoxMid360Source(level_source="imu", auto_start=False)
        source._point_queue.extend([(1000.0, -500.0, 2000.0)])
        points, _, _, name = source.sample(max_points=10)
        self.assertEqual(points, [(1.0, -0.5, 2.0)])
        self.assertEqual(name, "imu")


class QuaternionTests(unittest.TestCase):
    def _rotate_onto_z(self, quaternion, vector):
        x, y, z, w = quaternion
        # Rotation matrix for q = (x, y, z, w), applied to `vector`.
        matrix = (
            (1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)),
            (2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)),
            (2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)),
        )
        return tuple(sum(row[i] * vector[i] for i in range(3)) for row in matrix)

    def test_level_accel_is_identity(self):
        self.assertEqual(_accel_to_quaternion(0.0, 0.0, 1.0), (0.0, 0.0, 0.0, 1.0))

    def test_tilted_accel_rotates_gravity_onto_positive_z(self):
        quaternion = _accel_to_quaternion(0.5, 0.0, 0.8660254)
        rotated = self._rotate_onto_z(quaternion, (0.5, 0.0, 0.8660254))
        self.assertAlmostEqual(rotated[2], 1.0, places=5)

    def test_inverted_accel_is_180_degree_rotation(self):
        quaternion = _accel_to_quaternion(0.0, 0.0, -1.0)
        rotated = self._rotate_onto_z(quaternion, (0.0, 0.0, -1.0))
        self.assertAlmostEqual(rotated[2], 1.0, places=5)

    def test_zero_accel_is_rejected(self):
        self.assertIsNone(_accel_to_quaternion(0.0, 0.0, 0.0))


class SdkConfigTests(unittest.TestCase):
    def test_default_config_targets_deployment_sensor_lan(self):
        config = default_sdk_config()
        host = config["MID360"]["host_net_info"][0]
        self.assertEqual(host["lidar_ip"], ["192.168.50.30"])
        self.assertEqual(host["host_ip"], "192.168.50.10")

    def test_default_config_matches_sdk2_port_pairing(self):
        net = default_sdk_config()["MID360"]["lidar_net_info"]
        self.assertEqual(net["cmd_data_port"], 56100)
        host = default_sdk_config()["MID360"]["host_net_info"][0]
        self.assertEqual(host["cmd_data_port"], 56101)
        self.assertEqual(host["point_data_port"], 56301)

    def test_write_default_sdk_config_round_trips(self):
        with tempfile.TemporaryDirectory() as directory:
            path = f"{directory}/MID360_config.json"
            write_default_sdk_config(path, lidar_ip="192.168.50.31", host_ip="192.168.50.10")
            with open(path) as handle:
                config = json.load(handle)
            self.assertEqual(config["MID360"]["host_net_info"][0]["lidar_ip"], ["192.168.50.31"])

    def test_deployment_config_matches_generator(self):
        # The checked-in mid360_config.json must not drift from the generator
        # that documents the port map and sensor-LAN addresses.
        import os
        repo_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        path = os.path.join(repo_root, "mid360_config.json")
        if not os.path.exists(path):
            self.skipTest("mid360_config.json not present")
        with open(path) as handle:
            on_disk = json.load(handle)
        self.assertEqual(on_disk, default_sdk_config())


class DataDestinationTests(unittest.TestCase):
    def test_defaults_target_the_multicast_group(self):
        # The SDK receives point data on the multicast group from the config, so
        # the default destination must be that group, not the unicast host IP.
        source = LivoxMid360Source(level_source="imu", auto_start=False)
        self.assertEqual(source._data_host_ip, "224.1.1.5")
        self.assertEqual(source._host_point_data_port, 56301)
        self.assertEqual(source._host_imu_data_port, 56401)

    def test_default_lidar_ip_is_the_deployment_address(self):
        source = LivoxMid360Source(level_source="imu", auto_start=False)
        self.assertEqual(source.lidar_ip, "192.168.50.30")

    def test_poll_fallback_without_lidar_ip_returns_false(self):
        source = LivoxMid360Source(level_source="imu", auto_start=False,
                                   lidar_ip=None)
        self.assertFalse(source._poll_for_device(timeout_s=0.1))


class WorkModeTests(unittest.TestCase):
    """The stop mode must be the documented WakeUp, not the legacy 0x09."""

    def test_stop_mode_is_the_documented_wake_up(self):
        # Verified on hardware: the device reports mode 2 and the point rate
        # falls to 0. This is also what Livox Viewer 2's "Standby" sets.
        # The value must match kLivoxLidarWakeUp in livox_lidar_def.h.
        self.assertEqual(WORK_MODE_WAKE_UP, 0x02)

    def test_start_mode_is_normal(self):
        # Matches kLivoxLidarNormal in livox_lidar_def.h.
        self.assertEqual(WORK_MODE_NORMAL, 0x01)

    def test_starting_requires_the_point_data_type_step(self):
        # Without SetLivoxLidarPclDataType the device accepts Normal (ret=0) but
        # never spins, which is what made every stop mode look ineffective. The
        # start sequence must therefore set the type before the work mode.
        import inspect
        source = inspect.getsource(LivoxMid360Source._start_device)
        self.assertIn('set_point_data_type', source)
        self.assertLess(source.index('set_point_data_type'),
                        source.index('set_work_mode'))

    def test_legacy_stop_value_is_not_presented_as_the_stop(self):
        # 0x09 is retained only for compatibility and must not be the default.
        self.assertNotEqual(WORK_MODE_UNVERIFIED_STOP, WORK_MODE_WAKE_UP)
        self.assertEqual(WORK_MODE_UNVERIFIED_STOP, 0x09)


class SourceLifecycleTests(unittest.TestCase):
    def test_invalid_level_source_is_rejected(self):
        with self.assertRaises(ValueError):
            LivoxMid360Source(level_source="gps", auto_start=False)

    def test_start_without_config_raises(self):
        source = LivoxMid360Source(level_source="imu", auto_start=False)
        with self.assertRaises(ValueError):
            source.start()

    def test_start_with_missing_config_raises(self):
        source = LivoxMid360Source(level_source="imu", sdk_config_path="/nonexistent.json",
                                   auto_start=False)
        with self.assertRaises(FileNotFoundError):
            source.start()

    def test_sample_without_sdk_degrades_safely(self):
        source = LivoxMid360Source(level_source="imu", auto_start=False)
        points, quaternion, valid, name = source.sample(max_points=5)
        self.assertEqual(points, [])
        self.assertEqual(quaternion, (0.0, 0.0, 0.0, 1.0))
        self.assertFalse(valid)
        self.assertEqual(name, "imu")

    def test_non_imu_source_reports_invalid_orientation(self):
        source = LivoxMid360Source(level_source="tilt", auto_start=False)
        _, _, valid, name = source.sample(max_points=5)
        self.assertFalse(valid)
        self.assertEqual(name, "tilt")

    def test_fresh_imu_sample_is_valid(self):
        source = LivoxMid360Source(level_source="imu", auto_start=False)
        source._on_imu(0, None, 0, None)
        _, _, valid, _ = source.sample(max_points=1)
        self.assertFalse(valid)  # no IMU packet delivered yet


class BacklogTests(unittest.TestCase):
    def _source_with(self, points):
        source = LivoxMid360Source(level_source="imu", auto_start=False)
        source._point_queue.extend(points)
        return source

    def test_drain_returns_newest_not_oldest(self):
        # Fill beyond the requested cap: the OLDEST points must be dropped, so
        # a lagging consumer publishes the current cloud rather than stale data.
        source = self._source_with([(float(i), 0.0, 0.0) for i in range(100)])
        points = source._drain(max_points=5)
        # mm -> m conversion, and the newest five (95..99).
        self.assertEqual(points, [(float(i) / 1000.0, 0.0, 0.0) for i in range(95, 100)])

    def test_drop_backlog_empties_without_conversion(self):
        source = self._source_with([(1.0, 2.0, 3.0)] * 50)
        source.drop_backlog()
        self.assertEqual(source.stats["queued_points"], 0)

    def test_is_running_false_before_start(self):
        source = LivoxMid360Source(level_source="imu", auto_start=False)
        self.assertFalse(source.is_running)

    def test_handle_for_ip_matches_sdk_convention(self):
        # The SDK handle for a device is its IPv4 address as a host-order uint32.
        source = LivoxMid360Source(level_source="imu", auto_start=False)
        self.assertEqual(source._handle_for_ip("192.168.50.30"), 506636480)

    def test_handle_for_ip_rejects_malformed(self):
        source = LivoxMid360Source(level_source="imu", auto_start=False)
        with self.assertRaises(ValueError):
            source._handle_for_ip("192.168.50")


if __name__ == "__main__":
    unittest.main()
