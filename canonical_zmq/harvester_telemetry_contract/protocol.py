"""Packing and validation for the canonical ZeroMQ telemetry v1 contract.

This module deliberately has no ROS or ZeroMQ dependency.  A producer passes
the returned three frames to ``send_multipart``; a consumer passes the received
three frames to ``unpack_message``.  Keeping this layer pure lets Xavier and
Orin test the same wire contract without sharing a ROS installation.
"""

from __future__ import annotations

from typing import Any, Dict, Mapping, Sequence, Tuple

import msgpack


SCHEMA_VERSION = 1

CANONICAL_CHANNELS = frozenset({
    'v1/camera/cutter/rgb',
    'v1/camera/cutter/depth',
    'v1/camera/cutter/camera_info',
    'v1/camera/cutter/imu',
    'v1/camera/docking/rgb',
    'v1/camera/docking/depth',
    'v1/camera/docking/camera_info',
    'v1/camera/docking/imu',
    'v1/lidar/raw',
    'v1/range/docking',
    'v1/range/cutter',
    'v1/docking/trunk_estimate',
    'v1/calibration/status',
    'v1/system/status',
    'v1/operator/target_selection',
})

_SOURCE_MODES = frozenset({'simulation', 'hardware'})
_CLOCK_DOMAINS = frozenset({'ros_sim_time', 'utc_host', 'plc_rtc_utc'})
_GLOBAL_FIELDS = {
    'schema_version': int,
    'source_mode': str,
    'source_id': str,
    'sequence': int,
    'frame_id': str,
    'acquisition_timestamp_ns': int,
    'clock_domain': str,
    'gateway_monotonic_ns': int,
    'calibration_id': str,
    'capabilities': dict,
}


class ProtocolError(ValueError):
    """Raised when a packet violates the canonical telemetry contract."""


def _require_integer(name: str, value: Any) -> None:
    if isinstance(value, bool) or not isinstance(value, int):
        raise ProtocolError("header field {!r} must be an integer".format(name))


def _require_nonnegative(name: str, value: int) -> None:
    if value < 0:
        raise ProtocolError("header field {!r} must be non-negative".format(name))


def _validate_image_header(channel: str, header: Mapping[str, Any]) -> None:
    codec = header.get('codec')
    if channel.endswith('/rgb'):
        if codec not in {'jpeg', 'h264', 'h265'}:
            raise ProtocolError("{} requires jpeg, h264, or h265 codec".format(channel))
        if not isinstance(header.get('pixel_encoding'), str):
            raise ProtocolError("{} requires pixel_encoding".format(channel))
    elif channel.endswith('/depth'):
        if codec != 'depth_uint16_le':
            raise ProtocolError("{} requires depth_uint16_le codec".format(channel))
    else:
        if codec != 'json':
            raise ProtocolError("{} requires json codec".format(channel))

    for name in ('width', 'height'):
        _require_integer(name, header.get(name))
        if header[name] <= 0:
            raise ProtocolError("header field {!r} must be positive".format(name))


def _validate_imu_header(channel: str, header: Mapping[str, Any]) -> None:
    """IMU has no image geometry, so it uses a JSON payload and must NOT be
    forced through the image validator (which requires positive width/height)."""
    if header.get('codec') != 'json':
        raise ProtocolError("{} requires json codec".format(channel))


def _validate_lidar_header(header: Mapping[str, Any]) -> None:
    if header.get('codec') != 'lidar_xyz_f32':
        raise ProtocolError("v1/lidar/raw requires lidar_xyz_f32 codec")
    for name in ('point_count', 'point_stride_bytes'):
        _require_integer(name, header.get(name))
        _require_nonnegative(name, header[name])
    fields = header.get('point_fields')
    if not isinstance(fields, list) or not fields:
        raise ProtocolError("v1/lidar/raw requires a non-empty point_fields list")


def _validate_json_header(channel: str, header: Mapping[str, Any]) -> None:
    if header.get('codec') != 'json':
        raise ProtocolError("{} requires json codec".format(channel))


def validate_header(channel: str, header: Mapping[str, Any]) -> Dict[str, Any]:
    """Validate and return a plain copy of a canonical v1 header."""
    if channel not in CANONICAL_CHANNELS:
        raise ProtocolError("unknown canonical channel {!r}".format(channel))
    if not isinstance(header, Mapping):
        raise ProtocolError("header must be a mapping")

    result = dict(header)
    for name, expected_type in _GLOBAL_FIELDS.items():
        if name not in result:
            raise ProtocolError("header is missing {!r}".format(name))
        value = result[name]
        if expected_type is int:
            _require_integer(name, value)
        elif not isinstance(value, expected_type):
            raise ProtocolError("header field {!r} has the wrong type".format(name))

    if result['schema_version'] != SCHEMA_VERSION:
        raise ProtocolError("unsupported schema_version {!r}".format(result['schema_version']))
    if result['source_mode'] not in _SOURCE_MODES:
        raise ProtocolError("unsupported source_mode {!r}".format(result['source_mode']))
    if result['clock_domain'] not in _CLOCK_DOMAINS:
        raise ProtocolError("unsupported clock_domain {!r}".format(result['clock_domain']))
    if not result['source_id']:
        raise ProtocolError("source_id must not be empty")
    if not result['calibration_id']:
        raise ProtocolError("calibration_id must not be empty")
    if not result['capabilities']:
        raise ProtocolError('capabilities must not be empty')
    for capability, supported in result['capabilities'].items():
        if not isinstance(capability, str) or not capability:
            raise ProtocolError('capability names must be non-empty strings')
        if not isinstance(supported, bool):
            raise ProtocolError('capability values must be boolean')
    if channel != 'v1/system/status' and not result['frame_id']:
        raise ProtocolError("frame_id must not be empty for {}".format(channel))
    _require_nonnegative('sequence', result['sequence'])
    _require_nonnegative('acquisition_timestamp_ns', result['acquisition_timestamp_ns'])
    _require_nonnegative('gateway_monotonic_ns', result['gateway_monotonic_ns'])

    if channel.startswith('v1/camera/') and channel.endswith('/imu'):
        _validate_imu_header(channel, result)
    elif channel.startswith('v1/camera/'):
        _validate_image_header(channel, result)
    elif channel == 'v1/lidar/raw':
        _validate_lidar_header(result)
    else:
        _validate_json_header(channel, result)

    if 'transform_valid' in result and not isinstance(result['transform_valid'], bool):
        raise ProtocolError("transform_valid must be boolean")
    return result


def pack_message(channel: str, header: Mapping[str, Any], payload: bytes) -> Tuple[bytes, bytes, bytes]:
    """Return the exact three frames required by ``zmq.Socket.send_multipart``."""
    validated_header = validate_header(channel, header)
    if not isinstance(payload, (bytes, bytearray, memoryview)):
        raise ProtocolError("payload must be bytes-like")
    return (
        channel.encode('utf-8'),
        msgpack.packb(validated_header, use_bin_type=True),
        bytes(payload),
    )


def unpack_message(frames: Sequence[bytes]) -> Tuple[str, Dict[str, Any], bytes]:
    """Validate three received ZeroMQ frames and return channel, header, payload."""
    if len(frames) != 3:
        raise ProtocolError("canonical packet must contain exactly three frames")
    try:
        channel = bytes(frames[0]).decode('utf-8')
    except UnicodeDecodeError as exc:
        raise ProtocolError("channel must be UTF-8") from exc
    try:
        decoded_header = msgpack.unpackb(bytes(frames[1]), raw=False, strict_map_key=False)
    except (msgpack.ExtraData, msgpack.FormatError, msgpack.StackError, msgpack.UnpackException) as exc:
        raise ProtocolError("header is not valid MessagePack") from exc
    header = validate_header(channel, decoded_header)
    return channel, header, bytes(frames[2])
