"""Thin adapter between the Livox-SDK2 point/IMU callbacks and this publisher.

This file is the ONLY place that needs to know the Livox SDK API. Everything
else (``mid360_publisher.py``, ``lidar/leveling.py``) is SDK-agnostic.

The adapter talks to ``liblivox_lidar_sdk_shared.so`` through ``ctypes`` rather
than a compiled binding, so it works unchanged on the Orin (aarch64) and on an
x86 host. Callbacks run on SDK threads and only append to bounded queues; the
publisher thread drains them in ``sample()``.

Vendor-frame note: the MID-360 reports points with ``+X forward / +Y left /
+Z up`` (Livox "sensor" frame), which already matches this project's mechanical
convention, so ``sample()`` applies no axis permutation by default. The
installed mounting can flip or rotate that frame, so the permutation is kept
explicit and configurable in ``VENDOR_TO_PROJECT_AXES`` and can be overridden
per deployment. Confirm the real mounting during calibration and record the
verified vendor frame in the calibration session.

Time note: the MID-360 does NOT run a UTC clock by itself. Its packet timestamp
is meaningful in UTC only while it is slaved to an external time master
(PTP/gPTP from this host, or GPS). Otherwise the timestamp is time since the
device powered on. The per-packet ``time_type`` field (see ``TIME_TYPE_*`` and
``decode_packet_time`` below) says which, and this adapter records it so callers
can label the data honestly instead of assuming the LiDAR is synchronized. See
``LivoxMid360Source.timestamp_status`` and the README's "MID-360 LiDAR time
synchronization" section.

SDK2 configuration is data-driven: ``sdk_config_path`` points at a JSON file
(``LivoxLidarSdkInit`` reads it) whose ``MID360`` block holds the LiDAR and host
addresses. This adapter does not invent a config; the deployment config for the
``192.168.50.0/24`` sensor LAN must already exist. The default ``host_net_info``
below documents the project addresses (LiDAR ``192.168.50.30``, host
``192.168.50.10``) and can be written to a file with
``write_default_sdk_config()`` as a starting point.
"""

from __future__ import annotations

import ctypes
import itertools
import json
import os
import struct
import threading
import time
from collections import deque
from typing import List, Optional, Tuple

import numpy as np

# Sensor LAN for this deployment (see calibration/frames.deployment.template.json).
DEFAULT_LIDAR_IP = "192.168.50.30"
DEFAULT_HOST_IP = "192.168.50.10"

DEFAULT_SDK_LIBRARY = "liblivox_lidar_sdk_shared.so"

# ---------------------------------------------------------------------------
# Timestamps (Mid-360 Communication Protocol, section "Timestamp")
# ---------------------------------------------------------------------------
# Every point packet carries a 1-byte ``time_type`` that states how its 8-byte
# ``timestamp`` field should be read. The MID-360 does NOT keep a UTC clock on
# its own: it only becomes UTC-meaningful while it is slaved to a time master
# (PTP/gPTP from this host, or GPS). This is the field that tells us whether
# "synchronized" is true, so it is decoded rather than assumed.
#
#   0 -> no sync source; timestamp is ns since DEVICE POWER-ON (not UTC, and
#        not comparable across reboots)
#   1 -> PTP or gPTP; timestamp is the master clock in ns (PTP and gPTP are
#        indistinguishable here: the protocol has no separate value)
#   2 -> GPS; timestamp is GPS time in ns (valid 2000-01-01..2037-12-31)
TIME_TYPE_NONE = 0x00
TIME_TYPE_PTP = 0x01
TIME_TYPE_GPS = 0x02

# time_type values whose packet timestamp is an absolute (UTC/GPS) time rather
# than a boot-relative counter. Only these may be published as UTC.
ABSOLUTE_TIME_TYPES = (TIME_TYPE_PTP, TIME_TYPE_GPS)

# ``time_interval`` is the span between the FIRST and LAST point in a packet and
# is expressed in 0.1 us units (livox_lidar_def.h: "unit: 0.1 us"), so one unit
# is 0.1 us = 100 ns. Multiply the raw field by this to get nanoseconds.
_TIME_INTERVAL_NS_PER_UNIT = 100

# LivoxLidarPpsSyncMode (livox_lidar_def.h). Consumed by ``SetLivoxLidarPpsSyncMode``
# (protocol key 0x0026, "time_filter"). This is a GPS time-*robustness* flag, not
# a general "turn sync on" switch: PTP needs no SDK call at all (the device
# auto-syncs whenever a master is present on its link). Only the normal mode is
# used here; see ``set_pps_sync_mode`` for the optional call.
PPS_SYNC_NORMAL = 0x00

# LivoxLidarWorkMode / LivoxLidarPointDataType (livox_lidar_def.h).
WORK_MODE_NORMAL = 0x01
# ``kLivoxLidarWakeUp``. This is the mode to stop the motor with, and it is the
# one Livox Viewer 2's "Standby" action sets (verified by watching the device's
# own broadcast work-mode field change to 2 while the Viewer ran).
#
# Verified on the deployment unit, with a fresh SDK and the device running:
#   set Normal (1)  -> device reports 1, ~2083 point packets/s
#   set WakeUp (2)  -> device reports 2, 0 packets/s, and stays stopped
# The motor halts and the mode is a documented enum value, so no undocumented
# command is needed.
#
# IMPORTANT: a stop only works if the device was actually started, and starting
# requires ``SetLivoxLidarPclDataType`` (see ``POINT_TYPE_CARTESIAN_HIGH``) before
# ``Normal``. Without that call the device ACCEPTS ``Normal`` and reports
# ``ret=0`` but never spins, which makes every stop mode appear to do nothing.
# That is what previously led this adapter to use the undocumented value 0x09
# below; the real fault was an incomplete start sequence, not a rejecting device.
WORK_MODE_WAKE_UP = 0x02
# ``kLivoxLidarSleep``. Rejected by this firmware while the device is running
# (ret_code=3, error_key=26). Kept for reference/tests only; use
# ``WORK_MODE_WAKE_UP`` to stop.
WORK_MODE_SLEEP = 0x03
# Retained for backwards compatibility with earlier deployments and callers.
#
# This value is NOT in the installed SDK2 ``LivoxLidarWorkMode`` enum (which ends
# at 0x08 ``kLivoxLidarUpgrade``). It happens to halt the motor, but it was only
# ever needed because of the incomplete start sequence described above, and it
# leaves the device reporting a mode value the SDK does not define. Prefer
# ``WORK_MODE_WAKE_UP``; do not use this for new code.
WORK_MODE_UNVERIFIED_STOP = 0x09
POINT_TYPE_CARTESIAN_HIGH = 0x01  # 14-byte single-return point

# The MID-360 powers up in standby: after detection the SDK must send
# ``SetLivoxLidarWorkMode(handle, kLivoxLidarNormal)`` or the motor never spins
# and no point/IMU callback ever fires. The point type is set explicitly to the
# 14-byte Cartesian-high layout so the decoder matches the wire format (the
# MID-360 default is double-echo, which is 28 bytes/point).
STARTUP_SETTLE_S = 0.2

# Vendor axes reported in the MID-360 point payload, expressed as the project
# axis each one maps to. MID-360: +X forward, +Y left, +Z up -> already the
# project frame. Resolved once into (column_index, sign) pairs by
# ``_compile_axis_map`` so the per-point conversion never parses strings.
VENDOR_TO_PROJECT_AXES = ("x", "y", "z")

# Tag bit set on Livox "special" raw points, which carry a timestamp instead of
# a range. Their x/y/z are not coordinates and must not enter the cloud.
_SPECIAL_POINT_TAG_BIT = 0x10

# Live packet layout. ``livox_lidar_def.h`` is compiled with ``#pragma pack(1)``,
# so ``LivoxLidarCartesianHighRawPoint`` is 14 bytes (int32 x/y/z + uint8
# reflectivity + uint8 tag) and ``LivoxLidarImuRawPoint`` is 24 bytes
# (6 x float32 + uint8 flag). Do NOT assume C default alignment: the SDK structs
# are packed.
_RAW_POINT_SIZE_BYTES = 14
_RAW_IMU_SIZE_BYTES = 24

# The SDK copies each received datagram into a fixed 8192-byte buffer and hands
# the callback a pointer to it (``kMaxBufferSize`` in the SDK's
# device_manager.h). That buffer is the real upper bound on what we may read, so
# the per-callback caps below are derived from it rather than chosen arbitrarily:
# reading past it is a heap over-read. A corrupt ``dot_num`` or ``length`` field
# alone must never be enough to exceed this.
_SDK_CALLBACK_BUFFER_BYTES = 8192
# ``LivoxLidarEthernetPacket`` header (see ``livox_lidar_def.h``, packed):
# version(1) + length(2) + time_interval(2) + dot_num(2) + udp_cnt(2) +
# frame_cnt(1) + data_type(1) + time_type(1) + rsvd(12) + crc32(4) +
# timestamp(8) = 36 bytes, then ``data[]``.
# ``length`` counts the WHOLE packet including this 36-byte header. Verified
# against the live MID-360: length=1380 with dot_num=96 and 96*14=1344 payload
# bytes (36 + 1344 = 1380).
_ETH_PACKET_HEADER_SIZE_BYTES = 36
# ``dot_num`` offset within the packed header. Field order is version(1) @0,
# length(2) @1, time_interval(2) @3, dot_num(2) @5, so the offset is 5, not 7.
# Prefer ``_EthPacketHeader.dot_num``, which ctypes resolves from the struct.
_ETH_PACKET_DOT_NUM_OFFSET = 5

# Safety cap on payloads decoded from a single callback, so a corrupt or
# hostile ``dot_num``/``length`` can never drive a read past the buffer the SDK
# handed us. Derived from the SDK's 8192-byte callback buffer so the cap cannot
# exceed what the buffer can physically hold: the MID-360 sends 96-point packets
# (1344 payload bytes), so 582 leaves ample headroom while making an over-read
# impossible.
_MAX_POINTS_PER_CALLBACK = (
    (_SDK_CALLBACK_BUFFER_BYTES - _ETH_PACKET_HEADER_SIZE_BYTES)
    // _RAW_POINT_SIZE_BYTES
)
_MAX_IMU_PER_CALLBACK = (
    (_SDK_CALLBACK_BUFFER_BYTES - _ETH_PACKET_HEADER_SIZE_BYTES)
    // _RAW_IMU_SIZE_BYTES
)

# Retry pacing for the poll fallback that starts the device without the
# info-change callback. Backs off so repeated blocking SDK control round-trips
# do not hammer the device or the CPU during startup.
_POLL_INITIAL_INTERVAL_S = 0.5
_POLL_MAX_INTERVAL_S = 2.0


def _validated_ipv4(address: str) -> str:
    """Return ``address`` after checking it is a dotted-quad IPv4 literal.

    Single validating helper for every address this module parses, so no caller
    can pack an out-of-range octet into a device handle or write it to hardware.
    """
    pieces = address.split(".")
    if len(pieces) != 4:
        raise ValueError(f"invalid IPv4 address {address!r}")
    for piece in pieces:
        if not piece.isdigit():
            raise ValueError(f"invalid IPv4 address {address!r}")
        if not 0 <= int(piece) <= 255:
            raise ValueError(f"invalid IPv4 address {address!r}")
    return address


def _resolve_axis(axis: str, x: float, y: float, z: float) -> float:
    """Return the coordinate of ``axis`` for an inverted ``axis`` marker.

    ``VENDOR_TO_PROJECT_AXES`` entries are ``"x"``, ``"y"``, ``"z"`` for a
    pass-through or ``"-x"``, ``"-y"``, ``"-z"`` to negate that vendor axis.
    """
    sign = 1.0
    name = axis
    if name.startswith("-"):
        sign = -1.0
        name = name[1:]
    if name == "x":
        return sign * x
    if name == "y":
        return sign * y
    if name == "z":
        return sign * z
    raise ValueError(f"invalid axis marker {axis!r}")


def _compile_axis_map(axes=VENDOR_TO_PROJECT_AXES):
    """Return ``((src_index, sign), ...)`` for the project x/y/z order.

    Resolved once so the hot per-point path does no string parsing.
    """
    columns = {"x": 0, "y": 1, "z": 2}
    compiled = []
    for axis in axes:
        sign = -1.0 if axis.startswith("-") else 1.0
        name = axis[1:] if axis.startswith("-") else axis
        if name not in columns:
            raise ValueError(f"invalid axis marker {axis!r}")
        compiled.append((columns[name], sign))
    return tuple(compiled)


VENDOR_TO_PROJECT_COLUMNS = _compile_axis_map()


class LivoxSdkUnavailable(RuntimeError):
    """Raised when the Livox SDK shared library cannot be loaded."""


class _LidarInfo(ctypes.Structure):
    """``LivoxLidarInfo`` (packed): dev_type + 16-byte serial + 16-byte IP."""

    _pack_ = 1
    _fields_ = [
        ("dev_type", ctypes.c_uint8),
        ("sn", ctypes.c_char * 16),
        ("lidar_ip", ctypes.c_char * 16),
    ]


class _HostPointIpInfo(ctypes.Structure):
    """``HostPointIPInfo``: where the LiDAR should send point data."""

    _fields_ = [
        ("host_ip_addr", ctypes.c_char * 16),
        ("host_point_data_port", ctypes.c_uint16),
        ("lidar_point_data_port", ctypes.c_uint16),
    ]


class _HostImuIpInfo(ctypes.Structure):
    """``HostImuDataIPInfo``: where the LiDAR should send IMU data."""

    _fields_ = [
        ("host_ip_addr", ctypes.c_char * 16),
        ("host_imu_data_port", ctypes.c_uint16),
        ("lidar_imu_data_port", ctypes.c_uint16),
    ]


class _LivoxLidarIpInfo(ctypes.Structure):
    """``LivoxLidarIpInfo``: a LiDAR's IPv4 address, mask and gateway.

    Three fixed 16-byte char arrays (``livox_lidar_def.h``). ``SetLivoxLidarIp``
    takes a pointer to THIS struct, not three separate octet buffers.
    """

    _fields_ = [
        ("ip_addr", ctypes.c_char * 16),
        ("net_mask", ctypes.c_char * 16),
        ("gw_addr", ctypes.c_char * 16),
    ]


# ``LivoxLidarInfoChangeCallback(handle, const LivoxLidarInfo*, void*)``.
_INFO_CHANGE_CB = ctypes.CFUNCTYPE(
    None,
    ctypes.c_uint32,
    ctypes.POINTER(_LidarInfo),
    ctypes.c_void_p,
)


class _EthPacketHeader(ctypes.Structure):
    """``LivoxLidarEthernetPacket`` header (packed, 36 bytes).

    The SDK callbacks receive a pointer to the whole Ethernet packet, not to the
    point/IMU array, so the payload always starts after this header. ``dot_num``
    is the number of points in this packet; ``length`` is the payload byte
    length. Field order and sizes mirror ``livox_lidar_def.h``.
    """

    _pack_ = 1
    _fields_ = [
        ("version", ctypes.c_uint8),
        ("length", ctypes.c_uint16),
        ("time_interval", ctypes.c_uint16),
        ("dot_num", ctypes.c_uint16),
        ("udp_cnt", ctypes.c_uint16),
        ("frame_cnt", ctypes.c_uint8),
        ("data_type", ctypes.c_uint8),
        ("time_type", ctypes.c_uint8),
        ("rsvd", ctypes.c_uint8 * 12),
        ("crc32", ctypes.c_uint32),
        ("timestamp", ctypes.c_uint8 * 8),
    ]


# Callback signatures from livox_lidar_api.h. Both point and IMU callbacks are
# ``(uint32 handle, uint8 dev_type, LivoxLidarEthernetPacket* data, void*
# client_data)``: the second argument is the device type (a small integer), NOT
# a data pointer, and there is no separate ``data_num`` argument. Getting this
# wrong makes ``data`` a small integer reinterpreted as a pointer.
_PACKET_CB = ctypes.CFUNCTYPE(
    None,
    ctypes.c_uint32,  # handle
    ctypes.c_uint8,  # dev_type
    ctypes.c_void_p,  # LivoxLidarEthernetPacket*
    ctypes.c_void_p,  # client_data
)
_POINT_CB = _PACKET_CB
_IMU_CB = _PACKET_CB


def _payload_view(packet_ptr, max_items, item_size_bytes):
    """Return the packet payload as a numpy byte view, or ``None`` if unusable.

    Bounds the read four ways, so no single corrupt field can cause an over-read:

      * the SDK's fixed 8192-byte callback buffer, which is the hard ceiling on
        what may be read at all;
      * the packet's own ``length`` field (whole packet size, including header);
      * ``dot_num``, the count the packet claims;
      * ``max_items``, the caller's cap.

    ``length`` and ``dot_num`` are attacker/corruption-influenced, so the buffer
    size is what actually guarantees safety. Returns ``None`` for a packet that
    cannot be read sanely.
    """
    if not packet_ptr:
        return None
    header = ctypes.cast(
        packet_ptr, ctypes.POINTER(_EthPacketHeader)).contents
    # Bytes actually present after the header, per the packet's own length field.
    declared = int(header.length)
    if declared < _ETH_PACKET_HEADER_SIZE_BYTES:
        return None
    available = declared - _ETH_PACKET_HEADER_SIZE_BYTES
    # Never read beyond the SDK's callback buffer, whatever the packet claims.
    available = min(available, _SDK_CALLBACK_BUFFER_BYTES
                    - _ETH_PACKET_HEADER_SIZE_BYTES)
    count = int(header.dot_num)
    if count <= 0:
        return None
    count = min(count, max_items, available // item_size_bytes)
    if count <= 0:
        return None
    size = count * item_size_bytes
    raw = ctypes.string_at(
        packet_ptr + _ETH_PACKET_HEADER_SIZE_BYTES, size)
    if len(raw) < size:
        return None
    return np.frombuffer(raw, dtype=np.uint8)


def decode_packet_time(header) -> dict:
    """Return the timing provenance of one Ethernet packet.

    The MID-360 states how to read its timestamp in the packet header itself, so
    this is the single place that decides whether the LiDAR is actually
    synchronized. Returns a dict with:

      * ``time_type``      -- raw header byte (0 none, 1 PTP/gPTP, 2 GPS)
      * ``timestamp_ns``   -- the header's 8-byte little-endian timestamp
      * ``time_interval_ns`` -- span first..last point, converted from 0.1 us
      * ``absolute``       -- True when ``timestamp_ns`` is a real UTC/GPS time
      * ``source``         -- ``"ptp"``, ``"gps"``, ``"device_uptime"`` or
                              ``"unknown"`` for a value this build does not know

    ``timestamp_ns`` is only a wall-clock time when ``absolute`` is true. When it
    is false the value is nanoseconds since the device powered on and must NOT be
    published as UTC: a boot-relative counter silently masquerading as epoch time
    is worse than no timestamp, because it looks plausible and is wrong.
    """
    time_type = int(header.time_type)
    # ``timestamp`` is a packed ``uint8[8]``, little-endian, nanoseconds.
    timestamp_ns = int.from_bytes(bytes(header.timestamp), "little")
    # ``time_interval`` counts 0.1 us units between the first and last point,
    # i.e. 100 ns per unit.
    time_interval_ns = int(header.time_interval) * _TIME_INTERVAL_NS_PER_UNIT
    if time_type == TIME_TYPE_PTP:
        source = "ptp"
    elif time_type == TIME_TYPE_GPS:
        source = "gps"
    elif time_type == TIME_TYPE_NONE:
        source = "device_uptime"
    else:
        source = "unknown"
    return {
        "time_type": time_type,
        "timestamp_ns": timestamp_ns,
        "time_interval_ns": time_interval_ns,
        "absolute": time_type in ABSOLUTE_TIME_TYPES,
        "source": source,
    }


def _decode_points(payload: "np.ndarray") -> List[Tuple[float, float, float]]:
    """Decode packed 14-byte Cartesian points into millimetre XYZ tuples.

    Vectorised: one numpy load of the x/y/z int32 columns and one of the tag
    bytes, then a mask for the Livox "special" entries whose x/y/z is a
    timestamp rather than a coordinate.

    The values stay ``float32`` rather than being upcast to ``float64``: that
    halves the conversion and allocation work on this hot path (this runs on the
    SDK callback thread at up to ~200k points/s), and the consumer does range
    comparison and downsampling, where float32 precision is ample for
    millimetre-range coordinates.
    """
    if payload is None or payload.size < _RAW_POINT_SIZE_BYTES:
        return []
    count = payload.size // _RAW_POINT_SIZE_BYTES
    records = payload[: count * _RAW_POINT_SIZE_BYTES].reshape(count, _RAW_POINT_SIZE_BYTES)
    # x/y/z are the first 12 bytes; tag is byte 13 (bit 4 marks a special point).
    xyz = records[:, :12].copy().view("<i4").reshape(count, 3)
    tags = records[:, 13]
    keep = (tags & _SPECIAL_POINT_TAG_BIT) == 0
    if not keep.any():
        return []
    xyz = xyz[keep].astype(np.float32, copy=False)
    # ``tolist()`` already yields Python lists, so wrap once into tuples rather
    # than re-wrapping each element needlessly.
    return [(point[0], point[1], point[2]) for point in xyz.tolist()]


def _decode_point_batch(raw: bytes, data_num: int) -> List[Tuple[float, float, float]]:
    """Decode a packed ``LivoxLidarCartesianHighRawPoint`` byte batch.

    Kept as the pure-bytes entry point used by the tests; ``raw`` is the payload
    *after* the Ethernet header.
    """
    usable = min(data_num, len(raw) // _RAW_POINT_SIZE_BYTES)
    if usable <= 0:
        return []
    return _decode_points(np.frombuffer(
        raw[: usable * _RAW_POINT_SIZE_BYTES], dtype=np.uint8))


def _accel_to_quaternion(acc_x: float, acc_y: float, acc_z: float) -> Optional[Tuple[float, float, float, float]]:
    """Return the gravity-leveling quaternion ``(x, y, z, w)`` from an accel vector.

    The MID-360 IMU reports specific force; at rest it points along the gravity
    axis. The quaternion rotates the sensor frame so the measured gravity vector
    lands on ``-Z`` (project ``+Z up``), matching the rotation-only leveling
    contract in ``geometry.transforms``. Yaw is deliberately left at identity
    because it does not affect whether geometry appears to lean and IMU yaw
    drifts. Returns ``None`` when the vector is too weak to trust.
    """
    norm = (acc_x * acc_x + acc_y * acc_y + acc_z * acc_z) ** 0.5
    if norm <= 1e-6:
        return None
    ux, uy, uz = acc_x / norm, acc_y / norm, acc_z / norm
    # Rotation taking the measured up direction (ux, uy, uz) onto +Z.
    # Special-cased to avoid a degenerate cross product at the antipode.
    if uz <= -0.999999:
        return (1.0, 0.0, 0.0, 0.0)
    if uz >= 0.999999:
        return (0.0, 0.0, 0.0, 1.0)
    # q = (cross(measured, target), 1 + dot(measured, target)), normalised.
    # target = +Z, so cross(measured, +Z) = (uy, -ux, 0).
    qx, qy, qz = uy, -ux, 0.0
    qw = 1.0 + uz
    length = (qx * qx + qy * qy + qz * qz + qw * qw) ** 0.5
    return (qx / length, qy / length, qz / length, qw / length)


def default_sdk_config(lidar_ip: str = DEFAULT_LIDAR_IP, host_ip: str = DEFAULT_HOST_IP) -> dict:
    """Return the SDK2 MID360 JSON block for the deployment sensor LAN.

    This function is the single source of truth for the deployment SDK2 config;
    ``mid360_config.json`` is generated from it (see
    ``scripts/write_mid360_config.py``) and a test asserts the two agree, so the
    port map and addresses cannot silently drift.
    """
    return {
        "MID360": {
            "lidar_net_info": {
                "cmd_data_port": 56100,
                "push_msg_port": 56200,
                "point_data_port": 56300,
                "imu_data_port": 56400,
                "log_data_port": 56500,
            },
            "host_net_info": [
                {
                    "lidar_ip": [lidar_ip],
                    "host_ip": host_ip,
                    "multicast_ip": "224.1.1.5",
                    "cmd_data_port": 56101,
                    "push_msg_port": 56201,
                    "point_data_port": 56301,
                    "imu_data_port": 56401,
                    "log_data_port": 56501,
                }
            ],
            "lidar_log_enable": False,
        }
    }


def write_default_sdk_config(path: str, lidar_ip: str = DEFAULT_LIDAR_IP,
                             host_ip: str = DEFAULT_HOST_IP) -> None:
    """Write the deployment SDK2 config for the sensor LAN to ``path``."""
    with open(path, "w", encoding="utf-8") as handle:
        json.dump(default_sdk_config(lidar_ip=lidar_ip, host_ip=host_ip), handle, indent=2)
        handle.write("\n")


class _SdkBindings:
    """Loaded ``liblivox_lidar_sdk_shared`` with the symbols this adapter uses."""

    def __init__(self, library_path: Optional[str] = None):
        candidates = [library_path] if library_path else [DEFAULT_SDK_LIBRARY]
        last_error: Optional[Exception] = None
        self.lib = None
        for candidate in candidates:
            try:
                self.lib = ctypes.CDLL(candidate)
                break
            except OSError as error:  # pragma: no cover - depends on install
                last_error = error
        if self.lib is None:
            raise LivoxSdkUnavailable(
                "could not load the Livox SDK2 shared library "
                f"({last_error}); build and install Livox-SDK2, or set "
                "sdk_library_path to liblivox_lidar_sdk_shared.so"
            )
        self._bind()

    def _bind(self) -> None:
        self.lib.LivoxLidarSdkInit.argtypes = [
            ctypes.c_char_p,
            ctypes.c_char_p,
            ctypes.c_void_p,
        ]
        self.lib.LivoxLidarSdkInit.restype = ctypes.c_bool

        self.lib.LivoxLidarSdkStart.argtypes = []
        self.lib.LivoxLidarSdkStart.restype = ctypes.c_bool

        self.lib.LivoxLidarSdkUninit.argtypes = []
        self.lib.LivoxLidarSdkUninit.restype = None

        self.lib.SetLivoxLidarPointCloudCallBack.argtypes = [_POINT_CB, ctypes.c_void_p]
        self.lib.SetLivoxLidarPointCloudCallBack.restype = None

        self.lib.SetLivoxLidarImuDataCallback.argtypes = [_IMU_CB, ctypes.c_void_p]
        self.lib.SetLivoxLidarImuDataCallback.restype = None

        self.lib.SetLivoxLidarInfoChangeCallback.argtypes = [
            _INFO_CHANGE_CB, ctypes.c_void_p]
        self.lib.SetLivoxLidarInfoChangeCallback.restype = None

        self.lib.SetLivoxLidarWorkMode.argtypes = [
            ctypes.c_uint32, ctypes.c_int, ctypes.c_void_p, ctypes.c_void_p]
        self.lib.SetLivoxLidarWorkMode.restype = ctypes.c_int

        self.lib.SetLivoxLidarPointDataHostIPCfg.argtypes = [
            ctypes.c_uint32, ctypes.POINTER(_HostPointIpInfo),
            ctypes.c_void_p, ctypes.c_void_p]
        self.lib.SetLivoxLidarPointDataHostIPCfg.restype = ctypes.c_int

        self.lib.SetLivoxLidarImuDataHostIPCfg.argtypes = [
            ctypes.c_uint32, ctypes.POINTER(_HostImuIpInfo),
            ctypes.c_void_p, ctypes.c_void_p]
        self.lib.SetLivoxLidarImuDataHostIPCfg.restype = ctypes.c_int

        self.lib.LivoxLidarRequestReboot.argtypes = [
            ctypes.c_uint32, ctypes.c_void_p, ctypes.c_void_p]
        self.lib.LivoxLidarRequestReboot.restype = ctypes.c_int

        self.lib.SetLivoxLidarPclDataType.argtypes = [
            ctypes.c_uint32, ctypes.c_int, ctypes.c_void_p, ctypes.c_void_p
        ]
        self.lib.SetLivoxLidarPclDataType.restype = ctypes.c_int

        self.lib.SetLivoxLidarIp.argtypes = [
            ctypes.c_uint32,
            ctypes.POINTER(_LivoxLidarIpInfo),
            ctypes.c_void_p,
            ctypes.c_void_p,
        ]
        self.lib.SetLivoxLidarIp.restype = ctypes.c_int

        # ``SetLivoxLidarPpsSyncMode`` exists in the installed SDK for the
        # MID-360S ("[mid360s] support this function, other not support"). Treat
        # it as optional: bind it when present so the adapter stays loadable on
        # SDK builds that lack it, and leave ``self.has_pps_sync_mode`` marking
        # whether the call is actually available.
        self.has_pps_sync_mode = hasattr(self.lib, "SetLivoxLidarPpsSyncMode")
        if self.has_pps_sync_mode:
            self.lib.SetLivoxLidarPpsSyncMode.argtypes = [
                ctypes.c_uint32, ctypes.c_int, ctypes.c_void_p, ctypes.c_void_p]
            self.lib.SetLivoxLidarPpsSyncMode.restype = ctypes.c_int

    def set_point_cloud_callback(self, callback, client_data) -> None:
        self.lib.SetLivoxLidarPointCloudCallBack(callback, client_data)

    def set_imu_callback(self, callback, client_data) -> None:
        self.lib.SetLivoxLidarImuDataCallback(callback, client_data)

    def set_info_change_callback(self, callback, client_data) -> None:
        self.lib.SetLivoxLidarInfoChangeCallback(callback, client_data)

    def set_work_mode(self, handle: int, work_mode: int) -> int:
        return int(self.lib.SetLivoxLidarWorkMode(handle, work_mode, None, None))

    def stop_motor(self, handle: int) -> int:
        """Halt the LiDAR motor by putting it in ``WakeUp`` (standby).

        This is the documented ``kLivoxLidarWakeUp`` mode, and it is what Livox
        Viewer 2's "Standby" action sets. Verified on the deployment unit: the
        device reports mode 2 and the point rate falls to 0 and stays there.
        """
        return int(self.lib.SetLivoxLidarWorkMode(
            handle, WORK_MODE_WAKE_UP, None, None))

    def stop_motor_unverified(self, handle: int) -> int:
        """Legacy stop using the undocumented ``0x09``.

        Kept only so existing callers keep working. Prefer :meth:`stop_motor`;
        ``0x09`` is not in the SDK enum and leaves the device reporting a mode
        the SDK cannot name, so it should not be used for new code.
        """
        return int(self.lib.SetLivoxLidarWorkMode(
            handle, WORK_MODE_UNVERIFIED_STOP, None, None))

    def reboot(self, handle: int) -> int:
        """Reboot the device.

        Note: a reboot does NOT stop the motor. The MID-360 rescans on boot, so
        the motor comes back within seconds. Kept for recovery/standby use only.
        """
        return int(self.lib.LivoxLidarRequestReboot(handle, None, None))

    def set_point_data_host(self, handle: int, host_ip: str,
                            host_port: int, lidar_port: int) -> int:
        """Point the LiDAR's point stream at ``host_ip:host_port``.

        The SDK receives point data on the multicast group named in the config
        (``multicast_ip``), NOT on the unicast host IP, so the LiDAR must be told
        to send to that group or the data never reaches the SDK socket.
        """
        config = _HostPointIpInfo()
        config.host_ip_addr = host_ip.encode("ascii")
        config.host_point_data_port = int(host_port)
        config.lidar_point_data_port = int(lidar_port)
        return int(self.lib.SetLivoxLidarPointDataHostIPCfg(
            handle, ctypes.byref(config), None, None))

    def set_imu_data_host(self, handle: int, host_ip: str,
                          host_port: int, lidar_port: int) -> int:
        """Point the LiDAR's IMU stream at ``host_ip:host_port``."""
        config = _HostImuIpInfo()
        config.host_ip_addr = host_ip.encode("ascii")
        config.host_imu_data_port = int(host_port)
        config.lidar_imu_data_port = int(lidar_port)
        return int(self.lib.SetLivoxLidarImuDataHostIPCfg(
            handle, ctypes.byref(config), None, None))

    def init(self, config_path: str, host_ip: str) -> bool:
        return bool(self.lib.LivoxLidarSdkInit(
            config_path.encode("utf-8"), host_ip.encode("utf-8"), None
        ))

    def start(self) -> bool:
        return bool(self.lib.LivoxLidarSdkStart())

    def uninit(self) -> None:
        self.lib.LivoxLidarSdkUninit()

    def set_point_data_type(self, handle: int, data_type: int) -> int:
        # The trailing arguments are the (unused by us) response callback and
        # its client data; NULL means "fire and forget".
        return int(self.lib.SetLivoxLidarPclDataType(handle, data_type, None, None))

    def set_ip(self, handle: int, ip: str, netmask: str, gateway: str) -> int:
        """Move one LiDAR to a new IPv4 address (``SetLivoxLidarIp``).

        The SDK takes a single ``LivoxLidarIpInfo`` struct holding three 16-byte
        address strings. Passing separate octet buffers here would hand the SDK a
        data pointer where it expects the async-control callback.
        """
        config = _LivoxLidarIpInfo()
        config.ip_addr = _validated_ipv4(ip).encode("ascii")
        config.net_mask = _validated_ipv4(netmask).encode("ascii")
        config.gw_addr = _validated_ipv4(gateway).encode("ascii")
        return int(self.lib.SetLivoxLidarIp(
            handle, ctypes.byref(config), None, None))

    def set_pps_sync_mode(self, handle: int, mode: int) -> int:
        """Set the optional PPS/GPS time-filter robustness mode.

        This is NOT how PTP is enabled: the MID-360 slaves to a PTP/gPTP master
        automatically as soon as one is present on its link, with no SDK call.
        ``SetLivoxLidarPpsSyncMode`` maps to protocol key ``0x0026``
        ("time_filter"), which only tunes how the device reacts to GPS time
        rollback. Returns ``None`` when the installed SDK does not expose the
        call, so the caller can report that instead of guessing.
        """
        if not getattr(self, "has_pps_sync_mode", False):
            return None
        return int(self.lib.SetLivoxLidarPpsSyncMode(
            handle, int(mode), None, None))


class LivoxMid360Source:
    """Livox-SDK2 point + IMU source for the MID-360.

    ``level_source`` selects which orientation to use for leveling:

      * ``imu``  -> MID-360 built-in IMU (pitch/roll gravity-referenced).
      * ``tilt`` -> platform 2-axis tilt sensor (via PLC/ZMQ bridge).
      * ``boom`` -> boom angle sensor (via PLC/ZMQ bridge).

    Only ``imu`` is self-contained inside the LiDAR; the others are consumed
    from the PLC sensor bridge and combined by the caller, so this class reports
    ``valid=False`` for them (the caller supplies the orientation instead).
    """

    def __init__(
        self,
        level_source: str = "imu",
        sdk_config_path: Optional[str] = None,
        host_ip: str = DEFAULT_HOST_IP,
        sdk_library_path: Optional[str] = None,
        max_queued_points: int = 200_000,
        auto_start: bool = True,
        imu_valid_max_age_s: float = 1.0,
        startup_timeout_s: float = 8.0,
        reboot_settle_s: float = 4.0,
        lidar_ip: Optional[str] = DEFAULT_LIDAR_IP,
        data_host_ip: str = "224.1.1.5",
        host_point_data_port: int = 56301,
        lidar_point_data_port: int = 56300,
        host_imu_data_port: int = 56401,
        lidar_imu_data_port: int = 56400,
    ):
        if level_source not in ("imu", "tilt", "boom", "none"):
            raise ValueError(f"unsupported level_source {level_source!r}")
        self.level_source = level_source
        self.sdk_config_path = sdk_config_path
        self.host_ip = host_ip
        self.sdk_library_path = sdk_library_path
        self.imu_valid_max_age_s = imu_valid_max_age_s
        self.startup_timeout_s = float(startup_timeout_s)
        # Time to wait after a reboot for the device to come back before the SDK
        # is unloaded (the motor needs a moment to spin down and restart).
        self.reboot_settle_s = float(reboot_settle_s)
        # Address of the LiDAR on the sensor LAN. The SDK handle is derived from
        # it, which lets the fallback start path work without the info callback.
        self.lidar_ip = lidar_ip
        # Where the LiDAR must send its data. The SDK receives on the multicast
        # group named in the config, so these must match the config's
        # ``multicast_ip`` and the host-side ports.
        self._data_host_ip = data_host_ip
        self._host_point_data_port = int(host_point_data_port)
        self._lidar_point_data_port = int(lidar_point_data_port)
        self._host_imu_data_port = int(host_imu_data_port)
        self._lidar_imu_data_port = int(lidar_imu_data_port)

        self._lock = threading.Lock()
        self._point_queue: deque = deque(maxlen=max_queued_points)
        self._latest_quaternion: Tuple[float, float, float, float] = (0.0, 0.0, 0.0, 1.0)
        self._latest_quaternion_at: float = 0.0
        self._point_callbacks = 0
        # Newest packet timing provenance, filled by ``_on_points``. ``None``
        # until a packet arrives, so "no data yet" stays distinguishable from
        # "data arrived and the LiDAR is unsynchronized".
        self._latest_packet_time: Optional[dict] = None
        # UTC ns at the moment the newest packet was received, and its monotonic
        # twin so an age can be computed without a second clock read in a
        # callback (which runs on an SDK thread and must stay cheap).
        self._latest_arrival_utc_ns: Optional[int] = None
        self._latest_arrival_monotonic_ns: Optional[int] = None
        # How many packets arrived with an absolute (UTC/GPS) timestamp versus a
        # boot-relative one. Lets a caller report "0 of N packets synchronized"
        # instead of inferring sync from a single lucky sample.
        self._absolute_time_packets = 0
        self._uptime_time_packets = 0

        self._bindings: Optional[_SdkBindings] = None
        self._running = False
        self._handles: List[int] = []
        self._streaming = threading.Event()
        # Keep references alive: the SDK stores raw function pointers.
        self._point_cb = _POINT_CB(self._on_points)
        self._imu_cb = _IMU_CB(self._on_imu)
        self._info_cb = _INFO_CHANGE_CB(self._on_info_change)

        if auto_start:
            self.start()

    # ------------------------------------------------------------------ SDK

    def start(self) -> None:
        """Load the SDK, register callbacks, and begin streaming.

        Registration includes the info-change callback, which is what actually
        starts the motor: the MID-360 powers up in standby and only streams once
        ``SetLivoxLidarWorkMode(kLivoxLidarNormal)`` has been sent for its
        handle. ``start()`` blocks briefly (up to ``startup_timeout_s``) for that
        detection so callers can sample immediately afterwards.
        """
        if self._running:
            return
        if not self.sdk_config_path:
            raise ValueError(
                "sdk_config_path is required for livox-sdk mode; write one with "
                "write_default_sdk_config() or create the deployment JSON for "
                f"the {DEFAULT_LIDAR_IP} / {DEFAULT_HOST_IP} sensor LAN"
            )
        if not os.path.exists(self.sdk_config_path):
            raise FileNotFoundError(f"SDK config not found: {self.sdk_config_path}")

        bindings = _SdkBindings(self.sdk_library_path)
        self._streaming.clear()
        self._handles = []
        bindings.set_point_cloud_callback(self._point_cb, None)
        bindings.set_imu_callback(self._imu_cb, None)
        bindings.set_info_change_callback(self._info_cb, None)
        if not bindings.init(self.sdk_config_path, self.host_ip):
            bindings.uninit()
            raise RuntimeError(
                "LivoxLidarSdkInit failed; check that the host IP is the address "
                f"bound to the lidar NIC ({self.host_ip}) and the config matches "
                "the deployment sensor LAN"
            )
        if not bindings.start():
            bindings.uninit()
            raise RuntimeError("LivoxLidarSdkStart failed")
        self._bindings = bindings
        self._running = True
        # Wait for the info-change callback, which normally starts the motor.
        self._streaming.wait(timeout=min(2.0, self.startup_timeout_s))
        if not self._streaming.is_set():
            # The SDK does not emit the info-change callback on the non-view
            # path, so fall back to starting the configured device directly.
            if self._poll_for_device(
                    max(1.0, self.startup_timeout_s - 2.0)):
                print(f"[livox] started via poll fallback (lidar {self.lidar_ip})")
            else:
                print(f"[livox] no streaming device after "
                      f"{self.startup_timeout_s:.0f}s; check the LiDAR is "
                      f"reachable at {self.lidar_ip or '<unset>'}")
        self.report_time_sync()

    def report_time_sync(self, wait_s: float = 1.0) -> dict:
        """Print the LiDAR's actual time-sync state, once a packet has arrived.

        Reads ``time_type`` from the first point packets rather than trusting a
        configured assumption, and says plainly whether device timestamps are
        usable UTC. This is the check that would have caught the unsynchronized
        LiDAR: ``time_type 0`` means the device clock is boot-relative.
        """
        deadline = time.monotonic() + max(0.0, wait_s)
        status = self.timestamp_status()
        while (status["time_type"] is None and time.monotonic() < deadline):
            time.sleep(0.05)
            status = self.timestamp_status()
        if status["time_type"] is None:
            print("[livox] time sync: no point packets yet; cannot confirm the "
                  "LiDAR clock (device time stays unverified until data arrives)")
            return status
        if status["synchronized"]:
            print(f"[livox] time sync: SYNCHRONIZED via {status['source']} "
                  f"(time_type={status['time_type']}); device timestamps are "
                  "absolute UTC and used as acquisition time")
        else:
            print(f"[livox] time sync: NOT SYNCHRONIZED (time_type="
                  f"{status['time_type']}, source={status['source']}); the "
                  "device timestamp is time since LiDAR power-on, so the host "
                  "receive clock is used as acquisition time. Start a PTP "
                  "master on the sensor NIC (see deploy/ptp/) to fix this.")
        return status

    def close(self, stop_motor: bool = True) -> None:
        """Stop streaming and release the SDK.

        ``stop_motor`` puts the device in ``WakeUp`` (standby) before the SDK is
        unloaded, which halts the motor. ``WakeUp`` is a documented mode and is
        what Livox Viewer 2's "Standby" sets; see ``WORK_MODE_WAKE_UP``.

        A reboot is deliberately not used: the MID-360 rescans on boot, so the
        motor comes back within seconds.
        """
        if not self._running:
            return
        self._running = False
        self._streaming.clear()
        bindings, self._bindings = self._bindings, None
        if bindings is not None:
            if stop_motor:
                # WakeUp is durable across SDK unload, so a short settle period
                # is enough before releasing the SDK.
                for handle in self._handles:
                    try:
                        bindings.stop_motor(handle)
                    except Exception:
                        pass
                time.sleep(self.reboot_settle_s)
            bindings.uninit()

    def __enter__(self) -> "LivoxMid360Source":
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        self.close()

    # ------------------------------------------------------------ callbacks

    def _start_device(self, handle: int, sn: str = "") -> bool:
        """Configure the data destination and start the motor for one device.

        The SDK does not reliably emit the info-change callback on the non-view
        path (its ``is_load_mode`` guard is inverted for normal firmware), so
        this is also invoked from a polling fallback in :meth:`start`.
        """
        bindings = self._bindings
        if bindings is None:
            return False
        # Send point/IMU data to the multicast group the SDK listens on. This
        # must happen BEFORE work mode: the LiDAR otherwise sends unicast to its
        # stored default, which the multicast-bound SDK socket never receives.
        point_status = bindings.set_point_data_host(
            handle, self._data_host_ip,
            self._host_point_data_port, self._lidar_point_data_port)
        if self._host_imu_data_port:
            bindings.set_imu_data_host(
                handle, self._data_host_ip,
                self._host_imu_data_port, self._lidar_imu_data_port)
        # Match the decoder to the wire format: 14-byte single-return Cartesian.
        bindings.set_point_data_type(handle, POINT_TYPE_CARTESIAN_HIGH)
        # Optional GPS time-filter robustness flag. This does NOT enable PTP: the
        # MID-360 slaves to a PTP/gPTP master automatically whenever one is on
        # its link. Only attempt it when the installed SDK exposes the call, and
        # never let it block the start sequence.
        if getattr(bindings, "has_pps_sync_mode", False):
            pps_status = bindings.set_pps_sync_mode(handle, PPS_SYNC_NORMAL)
            if pps_status not in (0, None):
                print(f"[livox] PPS sync mode returned {pps_status} for handle "
                      f"{handle} (optional; continuing)")
        status = bindings.set_work_mode(handle, WORK_MODE_NORMAL)
        if status != 0:
            print(f"[livox] work-mode start failed for handle {handle} "
                  f"(sn {sn or 'unknown'}): status {status}")
            return False
        if point_status != 0:
            print(f"[livox] point-destination config returned {point_status} "
                  f"for handle {handle}; expecting data on "
                  f"{self._data_host_ip}:{self._host_point_data_port}")
        self._streaming.set()
        return True

    def _on_info_change(self, handle, info_ptr, client_data) -> None:
        """Configure the data destination, then start the LiDAR motor."""
        sn = ""
        if info_ptr:
            try:
                raw = bytes(info_ptr.contents.sn)
                sn = raw.split(b"\x00", 1)[0].decode("ascii", "replace")
            except (ValueError, AttributeError):
                sn = ""
        if handle not in self._handles:
            self._handles.append(handle)
        self._start_device(handle, sn)

    def _handle_for_ip(self, ip: str) -> int:
        """Return the SDK device handle for an IPv4 address.

        The SDK uses the device's address as a host-order uint32 handle.
        """
        address = _validated_ipv4(ip)
        return struct.unpack(
            "<I", bytes(int(piece) for piece in address.split(".")))[0]

    def _poll_for_device(self, timeout_s: float) -> bool:
        """Fallback: start the device without waiting for the info callback.

        The SDK's info-change callback does not fire for normal firmware on the
        non-view path, so poll until the control channel answers, then issue the
        start sequence for the configured LiDAR address.
        """
        if not self.lidar_ip:
            return False
        handle = self._handle_for_ip(self.lidar_ip)
        deadline = time.monotonic() + timeout_s
        # Back off between attempts: each retry issues three blocking SDK control
        # round-trips, and this host runs close to CPU saturation during startup.
        interval_s = _POLL_INITIAL_INTERVAL_S
        while time.monotonic() < deadline:
            if self._streaming.is_set():
                return True
            if self._start_device(handle, "poll"):
                if handle not in self._handles:
                    self._handles.append(handle)
                return True
            # Wake early if the info-change callback starts the device for us.
            if self._streaming.wait(timeout=interval_s):
                return True
            interval_s = min(interval_s * 2, _POLL_MAX_INTERVAL_S)
        return self._streaming.is_set()

    def _on_points(self, handle, dev_type, packet_ptr, client_data) -> None:
        payload = _payload_view(
            packet_ptr, _MAX_POINTS_PER_CALLBACK, _RAW_POINT_SIZE_BYTES)
        if payload is None:
            return
        # Read the packet's timing provenance BEFORE decoding points: even a
        # packet whose points are all filtered out still tells us whether the
        # LiDAR is currently synchronized, which is what the operator needs.
        # The arrival timestamp is taken here, on the SDK thread, so it reflects
        # when the datagram actually arrived rather than when a consumer later
        # drained the queue.
        arrival_utc_ns = time.time_ns()
        arrival_monotonic_ns = time.monotonic_ns()
        try:
            header = ctypes.cast(
                packet_ptr, ctypes.POINTER(_EthPacketHeader)).contents
            packet_time = decode_packet_time(header)
        except (ValueError, TypeError):  # pragma: no cover - defensive
            packet_time = None
        points = _decode_points(payload)
        with self._lock:
            if packet_time is not None:
                self._latest_packet_time = packet_time
                self._latest_arrival_utc_ns = arrival_utc_ns
                self._latest_arrival_monotonic_ns = arrival_monotonic_ns
                if packet_time["absolute"]:
                    self._absolute_time_packets += 1
                else:
                    self._uptime_time_packets += 1
            if not points:
                return
            self._point_callbacks += 1
            self._point_queue.extend(points)

    def _on_imu(self, handle, dev_type, packet_ptr, client_data) -> None:
        payload = _payload_view(
            packet_ptr, _MAX_IMU_PER_CALLBACK, _RAW_IMU_SIZE_BYTES)
        if payload is None or payload.size < _RAW_IMU_SIZE_BYTES:
            return
        count = payload.size // _RAW_IMU_SIZE_BYTES
        # The newest sample is the last record; accel is the 4th..6th float.
        offset = (count - 1) * _RAW_IMU_SIZE_BYTES
        acc_x, acc_y, acc_z = struct.unpack_from("<fff", payload.tobytes(), offset + 12)
        quaternion = _accel_to_quaternion(acc_x, acc_y, acc_z)
        if quaternion is None:
            return
        with self._lock:
            self._latest_quaternion = quaternion
            self._latest_quaternion_at = time.monotonic()

    # -------------------------------------------------------------- contract

    def drop_backlog(self) -> None:
        """Discard queued points without converting them.

        Used while the LiDAR is disabled so the queue cannot grow and no CPU is
        spent on points that will never be published.
        """
        with self._lock:
            self._point_queue.clear()

    def _drain(self, max_points: int) -> List[Tuple[float, float, float]]:
        """Return up to ``max_points`` of the NEWEST queued points, in project frame.

        Draining from the newest end (not the oldest) keeps each published scan
        current: when the sensor produces faster than we publish, the backlog at
        the old end is stale and is discarded rather than replayed seconds late.

        The stale old end is dropped in bulk rather than one ``popleft`` per point,
        because the queue can hold ~200k points and a per-item loop under the lock
        would stall the SDK callback thread at every publish tick.
        """
        with self._lock:
            available = len(self._point_queue)
            take = available if max_points <= 0 else min(available, max_points)
            if take <= 0:
                return []
            # Keep only the newest ``take``; rebuild once instead of popping the
            # stale end item by item.
            points = list(itertools.islice(
                self._point_queue, max(0, available - take), None))
            self._point_queue.clear()
            self._point_queue.extend(points)
        return [self._to_project_frame(*point) for point in points]

    @staticmethod
    def _to_project_frame(x_mm: float, y_mm: float, z_mm: float) -> Tuple[float, float, float]:
        """Convert one vendor-frame point (mm) to the project frame (m)."""
        metres = (x_mm / 1000.0, y_mm / 1000.0, z_mm / 1000.0)
        (ix, sx), (iy, sy), (iz, sz) = VENDOR_TO_PROJECT_COLUMNS
        return (sx * metres[ix], sy * metres[iy], sz * metres[iz])

    def _orientation(self) -> Tuple[Tuple[float, float, float, float], bool]:
        if self.level_source != "imu":
            # tilt/boom orientation is supplied by the PLC bridge, not the LiDAR.
            return (self._latest_quaternion, False)
        with self._lock:
            quaternion = self._latest_quaternion
            age = time.monotonic() - self._latest_quaternion_at
        valid = self._latest_quaternion_at > 0.0 and age <= self.imu_valid_max_age_s
        return (quaternion, valid)

    def timestamp_status(self) -> dict:
        """Return how this scan's timestamps should be labelled.

        The single source of truth for "is the LiDAR synchronized". Callers must
        decide what to publish from this, rather than assuming the device runs
        UTC. Keys:

          * ``synchronized``       -- the newest packet carried an absolute
                                     (PTP/GPS) timestamp, so device time is real
                                     UTC and may be published as such
          * ``time_type``          -- raw newest ``time_type`` (or ``None``)
          * ``source``             -- ``"ptp"``/``"gps"``/``"device_uptime"``/
                                     ``"unknown"``/``"no_data"``
          * ``device_timestamp_ns``-- newest packet's own timestamp
          * ``arrival_utc_ns``     -- UTC ns this host received that packet
          * ``arrival_age_s``      -- how long ago that was (``None`` if never)
          * ``absolute_packets``/``uptime_packets`` -- running counts

        ``synchronized`` is False when nothing has arrived yet: an unknown clock
        must never be reported as sync'd.

        This reports the DEVICE's current state. To stamp a scan, use
        :meth:`timing_snapshot`, which additionally applies the freshness check
        and returns the chosen value alongside its provenance.
        """
        now_monotonic_ns = time.monotonic_ns()
        with self._lock:
            packet_time = self._latest_packet_time
            arrival_utc_ns = self._latest_arrival_utc_ns
            arrival_monotonic_ns = self._latest_arrival_monotonic_ns
            absolute_packets = self._absolute_time_packets
            uptime_packets = self._uptime_time_packets
        if packet_time is None:
            return {
                "synchronized": False,
                "time_type": None,
                "source": "no_data",
                "device_timestamp_ns": None,
                "arrival_utc_ns": None,
                "arrival_age_s": None,
                "absolute_packets": absolute_packets,
                "uptime_packets": uptime_packets,
            }
        age_s = None
        if arrival_monotonic_ns is not None:
            age_s = max(0.0, (now_monotonic_ns - arrival_monotonic_ns) / 1e9)
        return {
            "synchronized": bool(packet_time["absolute"]),
            "time_type": packet_time["time_type"],
            "source": packet_time["source"],
            "device_timestamp_ns": packet_time["timestamp_ns"],
            "arrival_utc_ns": arrival_utc_ns,
            "arrival_age_s": age_s,
            "absolute_packets": absolute_packets,
            "uptime_packets": uptime_packets,
        }

    def timing_snapshot(self, max_age_s: float = 1.0) -> dict:
        """Return the chosen acquisition time and its provenance in ONE read.

        This is the only method a publisher should use to stamp a scan. It exists
        so the timestamp and the status that describes it cannot disagree: when a
        caller instead called ``acquisition_timestamp_ns()`` and
        ``timestamp_status()`` separately, a stale PTP sample could be returned as
        host time while the status still reported ``ptp`` and ``synchronized``,
        producing a header that says "PTP" over a host-clock number.

        Keys:

          * ``timestamp_ns`` -- the acquisition time to publish
          * ``source``       -- ``livox_ptp`` / ``livox_gps`` / ``host_arrival`` /
                                ``host_now``
          * ``clock_domain`` -- ``lidar_ptp_utc`` for a LiDAR clock, else
                                ``orin_realtime``
          * ``synchronized`` -- True only when the value is the LiDAR's own
                                absolute time (so it may be called UTC)
          * ``time_type``    -- raw newest ``time_type`` (``None`` if no packet)
          * ``lidar_source`` -- the raw device state: ``ptp``/``gps``/
                                ``device_uptime``/``unknown``/``no_data``
          * ``arrival_age_s``-- how long ago the newest packet arrived

        Preference order, and why:

          1. ``livox_ptp``/``livox_gps`` -- the LiDAR's own timestamp, only when
             the packet ``time_type`` is absolute AND fresh. This is the true
             acquisition time the sensor measured.
          2. ``host_arrival`` -- this host's ``CLOCK_REALTIME`` at the moment the
             newest packet arrived, used when the LiDAR is not synchronized
             (``time_type == 0``). Real UTC only insofar as the Orin is
             disciplined; the header's ``time_quality`` reports that separately.
          3. ``host_now`` -- the host clock sampled now, used when no packet has
             ever arrived. It is publish time, not acquisition time, and is
             labelled as such.
        """
        now_utc_ns = time.time_ns()
        now_monotonic_ns = time.monotonic_ns()
        with self._lock:
            packet_time = self._latest_packet_time
            arrival_utc_ns = self._latest_arrival_utc_ns
            arrival_monotonic_ns = self._latest_arrival_monotonic_ns
        if packet_time is None:
            return {
                "timestamp_ns": int(now_utc_ns),
                "source": "host_now",
                "clock_domain": "orin_realtime",
                "synchronized": False,
                "time_type": None,
                "lidar_source": "no_data",
                "arrival_age_s": None,
            }
        age_s = None
        if arrival_monotonic_ns is not None:
            age_s = max(0.0, (now_monotonic_ns - arrival_monotonic_ns) / 1e9)
        fresh = age_s is not None and age_s <= max_age_s
        if packet_time["absolute"] and fresh:
            source = "livox_ptp" if packet_time["time_type"] == TIME_TYPE_PTP else "livox_gps"
            return {
                "timestamp_ns": int(packet_time["timestamp_ns"]),
                "source": source,
                "clock_domain": "lidar_ptp_utc",
                "synchronized": True,
                "time_type": packet_time["time_type"],
                "lidar_source": packet_time["source"],
                "arrival_age_s": age_s,
            }
        # Not an absolute-and-fresh LiDAR sample. Prefer the newest packet's real
        # receive time; fall back to publish time only if nothing ever arrived.
        if arrival_utc_ns is not None:
            timestamp_ns, source = int(arrival_utc_ns), "host_arrival"
        else:
            timestamp_ns, source = int(now_utc_ns), "host_now"
        return {
            "timestamp_ns": timestamp_ns,
            "source": source,
            "clock_domain": "orin_realtime",
            # The value is host time, so it is NOT the LiDAR's synchronized time
            # even if the device clock itself is absolute: the sample was stale.
            "synchronized": False,
            "time_type": packet_time["time_type"],
            "lidar_source": packet_time["source"],
            "arrival_age_s": age_s,
        }

    def acquisition_timestamp_ns(self, max_age_s: float = 1.0) -> Tuple[int, str]:
        """Return ``(timestamp_ns, source)`` for a freshly drained scan.

        Thin wrapper over :meth:`timing_snapshot`; callers that also need the
        status should use ``timing_snapshot`` so the two cannot disagree.
        """
        snapshot = self.timing_snapshot(max_age_s=max_age_s)
        return snapshot["timestamp_ns"], snapshot["source"]

    def sample(self, max_points: int) -> Tuple[List[Tuple[float, float, float]],
                                               Tuple[float, float, float, float],
                                               bool,
                                               str]:
        """Return (points, quaternion, valid, source_name) for one scan.

        Points are drained from the SDK queue, converted to the project frame in
        metres, and capped at ``max_points``. ``valid`` is true only when the
        requested orientation source has a fresh sample.

        Timing is NOT part of this tuple: use :meth:`timestamp_status` or
        :meth:`acquisition_timestamp_ns` so the 4-tuple stays back-compatible for
        the existing ``level_source`` callers.
        """
        points = self._drain(max_points)
        quaternion, valid = self._orientation()
        return points, quaternion, valid, self.level_source

    @property
    def is_running(self) -> bool:
        """True while the SDK is loaded and streaming."""
        return self._running

    @property
    def stats(self) -> dict:
        """Diagnostics for the publisher log line."""
        with self._lock:
            return {
                "queued_points": len(self._point_queue),
                "point_callbacks": self._point_callbacks,
                "imu_seen": self._latest_quaternion_at > 0.0,
                "time_sync_type": (
                    None if self._latest_packet_time is None
                    else self._latest_packet_time["time_type"]),
                "time_sync_source": (
                    "no_data" if self._latest_packet_time is None
                    else self._latest_packet_time["source"]),
                "absolute_time_packets": self._absolute_time_packets,
                "uptime_time_packets": self._uptime_time_packets,
            }

