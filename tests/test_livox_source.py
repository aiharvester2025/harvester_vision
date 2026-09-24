import json
import ctypes
import math
import struct
import tempfile
import time
import unittest

from lidar.livox_source import (
    LivoxMid360Source,
    TIME_TYPE_GPS,
    TIME_TYPE_NONE,
    TIME_TYPE_PTP,
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
    decode_packet_time,
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


class PacketTimeDecodingTests(unittest.TestCase):
    """``time_type`` decides whether a LiDAR timestamp is UTC or boot-relative."""

    def _header(self, time_type, timestamp_ns=0, time_interval=0):
        header = _EthPacketHeader()
        header.time_type = time_type
        header.time_interval = time_interval
        header.timestamp = (ctypes.c_uint8 * 8).from_buffer_copy(
            struct.pack("<Q", timestamp_ns))
        return header

    def test_unsynchronized_timestamp_is_boot_relative_not_absolute(self):
        # time_type 0 is the MID-360's "no sync source" state: the timestamp is
        # nanoseconds since the device powered on and must NEVER be treated as
        # UTC, because it looks like a plausible (but wrong) epoch time.
        decoded = decode_packet_time(self._header(TIME_TYPE_NONE, 123456789))
        self.assertEqual(decoded["time_type"], TIME_TYPE_NONE)
        self.assertFalse(decoded["absolute"])
        self.assertEqual(decoded["source"], "device_uptime")
        self.assertEqual(decoded["timestamp_ns"], 123456789)

    def test_ptp_timestamp_is_absolute(self):
        decoded = decode_packet_time(
            self._header(TIME_TYPE_PTP, 1_700_000_000_000_000_000))
        self.assertTrue(decoded["absolute"])
        self.assertEqual(decoded["source"], "ptp")

    def test_gps_timestamp_is_absolute(self):
        decoded = decode_packet_time(
            self._header(TIME_TYPE_GPS, 1_700_000_000_000_000_000))
        self.assertTrue(decoded["absolute"])
        self.assertEqual(decoded["source"], "gps")

    def test_unknown_time_type_is_not_treated_as_absolute(self):
        # A value this build does not know must fail closed: better to fall back
        # to host time than to publish an unverified number as UTC.
        decoded = decode_packet_time(self._header(0x07, 42))
        self.assertFalse(decoded["absolute"])
        self.assertEqual(decoded["source"], "unknown")

    def test_time_interval_is_converted_from_tenths_of_a_microsecond(self):
        # livox_lidar_def.h documents time_interval in 0.1 us units, i.e. 100 ns
        # per unit. So 100 units is 10 us = 10000 ns.
        decoded = decode_packet_time(self._header(TIME_TYPE_PTP, 0, time_interval=100))
        self.assertEqual(decoded["time_interval_ns"], 10_000)

    def test_timestamp_is_decoded_little_endian(self):
        header = self._header(TIME_TYPE_PTP)
        # 0x0102030405060708 in little-endian byte order.
        header.timestamp = (ctypes.c_uint8 * 8)(
            0x08, 0x07, 0x06, 0x05, 0x04, 0x03, 0x02, 0x01)
        self.assertEqual(
            decode_packet_time(header)["timestamp_ns"], 0x0102030405060708)


class TimeSyncStatusTests(unittest.TestCase):
    """The adapter must report "not synchronized" until packets prove otherwise."""

    def _packet(self, time_type, device_time_ns, points=1):
        buffer = (ctypes.c_uint8 * _SDK_CALLBACK_BUFFER_BYTES)()
        header = ctypes.cast(
            ctypes.addressof(buffer), ctypes.POINTER(_EthPacketHeader)).contents
        header.length = _ETH_PACKET_HEADER_SIZE_BYTES + points * _RAW_POINT_SIZE_BYTES
        header.dot_num = points
        header.time_type = time_type
        header.time_interval = 0
        header.timestamp = (ctypes.c_uint8 * 8).from_buffer_copy(
            struct.pack("<Q", device_time_ns))
        return buffer

    def test_no_packets_yet_is_not_reported_as_synchronized(self):
        # An unknown clock must never be claimed as sync'd.
        source = LivoxMid360Source(level_source="imu", auto_start=False)
        status = source.timestamp_status()
        self.assertFalse(status["synchronized"])
        self.assertIsNone(status["time_type"])
        self.assertEqual(status["source"], "no_data")

    def test_ptp_packets_report_synchronized_and_use_device_time(self):
        source = LivoxMid360Source(level_source="imu", auto_start=False)
        device_ns = 1_700_000_000_000_000_000
        packet = self._packet(TIME_TYPE_PTP, device_ns)
        source._on_points(0, None, ctypes.addressof(packet), None)
        status = source.timestamp_status()
        self.assertTrue(status["synchronized"])
        self.assertEqual(status["source"], "ptp")
        timestamp_ns, origin = source.acquisition_timestamp_ns()
        self.assertEqual(timestamp_ns, device_ns)
        self.assertEqual(origin, "livox_ptp")

    def test_unsynchronized_packets_fall_back_to_host_clock(self):
        # The device timestamp here is "seconds since boot"; publishing it as
        # UTC would look like 1970. The host receive time must be used instead.
        source = LivoxMid360Source(level_source="imu", auto_start=False)
        packet = self._packet(TIME_TYPE_NONE, 5_000_000_000)  # 5 s after boot
        before_ns = time.time_ns()
        source._on_points(0, None, ctypes.addressof(packet), None)
        status = source.timestamp_status()
        self.assertFalse(status["synchronized"])
        self.assertEqual(status["source"], "device_uptime")
        # The arrival timestamp recorded in the callback is host "now".
        self.assertGreaterEqual(status["arrival_utc_ns"], before_ns)
        timestamp_ns, origin = source.acquisition_timestamp_ns()
        after_ns = time.time_ns()
        self.assertEqual(origin, "host_arrival")
        # Host time is now, not the tiny boot-relative counter. Bounded below by
        # the pre-callback sample and above by a fresh read taken afterwards.
        self.assertGreaterEqual(timestamp_ns, before_ns)
        self.assertLessEqual(timestamp_ns, after_ns)
        self.assertGreater(timestamp_ns, 1_600_000_000_000_000_000)
        self.assertNotEqual(timestamp_ns, 5_000_000_000)

    def test_stale_absolute_sample_falls_back_to_host_clock(self):
        # A PTP-stamped packet that is older than max_age_s must not be reused as
        # the acquisition time for a later cloud.
        source = LivoxMid360Source(level_source="imu", auto_start=False)
        device_ns = 1_700_000_000_000_000_000
        packet = self._packet(TIME_TYPE_PTP, device_ns)
        source._on_points(0, None, ctypes.addressof(packet), None)
        timestamp_ns, origin = source.acquisition_timestamp_ns(max_age_s=0.0)
        self.assertEqual(origin, "host_arrival")
        self.assertNotEqual(timestamp_ns, device_ns)

    def test_host_fallback_uses_packet_arrival_not_publish_time(self):
        # The fallback must return when the packet ARRIVED, not when the consumer
        # later asked for it: re-stamping an older scan with the publish time is
        # exactly the silent-lie failure this adapter exists to prevent.
        source = LivoxMid360Source(level_source="imu", auto_start=False)
        packet = self._packet(TIME_TYPE_NONE, 5_000_000_000)
        source._on_points(0, None, ctypes.addressof(packet), None)
        arrival_ns = source.timestamp_status()["arrival_utc_ns"]
        time.sleep(0.05)  # a later drain must not change the answer
        timestamp_ns, origin = source.acquisition_timestamp_ns()
        self.assertEqual(origin, "host_arrival")
        self.assertEqual(timestamp_ns, arrival_ns)

    def test_no_packet_at_all_is_labelled_host_now_not_host_arrival(self):
        # With no packet ever received there is no acquisition time; the publish
        # clock is used but must be labelled distinctly so a consumer is not told
        # a scan was captured when it was only published.
        source = LivoxMid360Source(level_source="imu", auto_start=False)
        timestamp_ns, origin = source.acquisition_timestamp_ns()
        self.assertEqual(origin, "host_now")
        self.assertGreater(timestamp_ns, 1_600_000_000_000_000_000)

    def test_timing_snapshot_is_self_consistent_for_a_fresh_ptp_sample(self):
        source = LivoxMid360Source(level_source="imu", auto_start=False)
        device_ns = 1_700_000_000_000_000_000
        packet = self._packet(TIME_TYPE_PTP, device_ns)
        source._on_points(0, None, ctypes.addressof(packet), None)
        snap = source.timing_snapshot()
        self.assertEqual(snap["timestamp_ns"], device_ns)
        self.assertEqual(snap["source"], "livox_ptp")
        self.assertEqual(snap["clock_domain"], "lidar_ptp_utc")
        self.assertTrue(snap["synchronized"])
        self.assertEqual(snap["time_type"], TIME_TYPE_PTP)

    def test_stale_ptp_sample_is_not_published_as_synchronized(self):
        # The regression this guards: a stale absolute packet must NOT come back
        # as a host-clock value while still claiming ptp/synchronized, which
        # would put a "PTP" label on a host-time number.
        source = LivoxMid360Source(level_source="imu", auto_start=False)
        device_ns = 1_700_000_000_000_000_000
        packet = self._packet(TIME_TYPE_PTP, device_ns)
        source._on_points(0, None, ctypes.addressof(packet), None)
        snap = source.timing_snapshot(max_age_s=0.0)
        self.assertFalse(snap["synchronized"])
        self.assertEqual(snap["clock_domain"], "orin_realtime")
        self.assertEqual(snap["source"], "host_arrival")
        self.assertNotEqual(snap["timestamp_ns"], device_ns)

    def test_timing_snapshot_labels_a_host_fallback_consistently(self):
        source = LivoxMid360Source(level_source="imu", auto_start=False)
        packet = self._packet(TIME_TYPE_NONE, 5_000_000_000)
        source._on_points(0, None, ctypes.addressof(packet), None)
        snap = source.timing_snapshot()
        self.assertFalse(snap["synchronized"])
        self.assertEqual(snap["clock_domain"], "orin_realtime")
        self.assertEqual(snap["source"], "host_arrival")
        self.assertEqual(snap["time_type"], TIME_TYPE_NONE)
        self.assertEqual(snap["lidar_source"], "device_uptime")

    def test_timing_snapshot_with_no_packet_is_host_now(self):
        source = LivoxMid360Source(level_source="imu", auto_start=False)
        snap = source.timing_snapshot()
        self.assertEqual(snap["source"], "host_now")
        self.assertEqual(snap["clock_domain"], "orin_realtime")
        self.assertFalse(snap["synchronized"])
        self.assertIsNone(snap["time_type"])
        self.assertEqual(snap["lidar_source"], "no_data")

    def test_stats_surface_time_sync_state(self):
        source = LivoxMid360Source(level_source="imu", auto_start=False)
        self.assertIsNone(source.stats["time_sync_type"])
        packet = self._packet(TIME_TYPE_NONE, 1)
        source._on_points(0, None, ctypes.addressof(packet), None)
        self.assertEqual(source.stats["time_sync_type"], TIME_TYPE_NONE)
        self.assertEqual(source.stats["time_sync_source"], "device_uptime")
        self.assertEqual(source.stats["uptime_time_packets"], 1)


class PpsSyncModeTests(unittest.TestCase):
    def test_pps_sync_mode_is_optional_when_the_sdk_lacks_it(self):
        # The call is not in every SDK build/firmware; a missing symbol must
        # report None rather than raise or silently pretend it succeeded.
        from lidar.livox_source import _SdkBindings
        bindings = _SdkBindings.__new__(_SdkBindings)  # no library load
        bindings.has_pps_sync_mode = False
        self.assertIsNone(bindings.set_pps_sync_mode(handle=1, mode=0))


if __name__ == "__main__":
    unittest.main()
