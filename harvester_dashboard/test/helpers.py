"""Shared synthetic canonical-packet builders (contract-based, no fixtures)."""

import io

import numpy as np
import msgpack
from PIL import Image as PilImage

from harvester_dashboard.protocol_shim import pack_message


def base_header(**overrides):
    header = {
        'schema_version': 1,
        'source_mode': 'simulation',
        'source_id': 'xavier',
        'sequence': 1,
        'frame_id': 'test_frame',
        'acquisition_timestamp_ns': 1_000_000_000,
        'clock_domain': 'ros_sim_time',
        'gateway_monotonic_ns': 1_000_000_000,
        'calibration_id': 'test_calibration',
        'capabilities': {'packet.recording': True, 'target.world_fixed': False},
        'codec': 'json',
    }
    header.update(overrides)
    return header


def json_packet(channel, payload, sequence=1, **header_overrides):
    import json as _json
    header = base_header(sequence=sequence, **header_overrides)
    return pack_message(channel, header, _json.dumps(payload).encode('utf-8'))


def jpeg_packet(channel, width=32, height=24, sequence=1, color=(255, 0, 0),
                **header_overrides):
    image = PilImage.new('RGB', (width, height), color)
    buffer = io.BytesIO()
    image.save(buffer, format='JPEG', quality=90)
    header = base_header(
        sequence=sequence,
        codec='jpeg',
        pixel_encoding='RGB8',
        width=width,
        height=height,
        **header_overrides)
    return pack_message(channel, header, buffer.getvalue())


def depth_packet(channel, depth_m=None, width=8, height=6, sequence=1,
                 **header_overrides):
    if depth_m is None:
        depth_m = np.full((height, width), 2.5, dtype=np.float32)
    millimetres = np.rint(depth_m * 1000.0).astype('<u2')
    header = base_header(
        sequence=sequence,
        codec='depth_uint16_le',
        width=width,
        height=height,
        **header_overrides)
    return pack_message(channel, header, millimetres.tobytes())


def camera_info_packet(channel, width=8, height=6, sequence=1, **header_overrides):
    import json as _json
    info = {
        'width': width, 'height': height,
        'distortion_model': 'plumb_bob',
        'd': [0.0, 0.0, 0.0, 0.0, 0.0],
        'k': [500.0, 0.0, width / 2.0, 0.0, 500.0, height / 2.0,
              0.0, 0.0, 1.0],
        'r': [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
        'p': [500.0, 0.0, width / 2.0, 0.0, 0.0, 500.0, height / 2.0,
              0.0, 0.0, 0.0, 1.0, 0.0],
        'binning_x': 1, 'binning_y': 1,
        'roi': {'x_offset': 0, 'y_offset': 0, 'height': height, 'width': width,
                'do_rectify': False},
    }
    header = base_header(
        sequence=sequence,
        codec='json',
        width=width,
        height=height,
        **header_overrides)
    return pack_message(channel, header, _json.dumps(info).encode('utf-8'))


def imu_packet(channel, accel_ms2=None, gyro_rad_s=None, attitude=None,
               sequence=1, **header_overrides):
    import json as _json
    payload = {
        'frame_id': 'test_optical_frame',
        'accel_ms2': accel_ms2 if accel_ms2 is not None else [0.0, 0.0, -9.80665],
        'gyro_rad_s': gyro_rad_s if gyro_rad_s is not None else [0.0, 0.0, 0.0],
        'attitude_rpy_rad': attitude if attitude is not None else [0.0, 0.0, 0.0],
        'accel_norm_ms2': 9.80665,
        'sample_rate_hz': 200,
        'n_samples': 1,
    }
    header = base_header(
        sequence=sequence,
        codec='json',
        **header_overrides)
    return pack_message(channel, header, _json.dumps(payload).encode('utf-8'))


def lidar_packet(points=None, sequence=1, **header_overrides):
    points = (np.array([[1.0, 2.0, 3.0], [4.0, 5.0, 6.0], [-1.0, 0.5, 2.0]],
                       dtype='<f4') if points is None else
              np.ascontiguousarray(points, dtype='<f4'))
    header = base_header(
        sequence=sequence,
        codec='lidar_xyz_f32',
        frame_id='vehicle_lidar_link',
        point_count=int(len(points)),
        point_stride_bytes=12,
        point_fields=[
            {'name': 'x', 'type': 'float32'},
            {'name': 'y', 'type': 'float32'},
            {'name': 'z', 'type': 'float32'},
        ],
        **header_overrides)
    return pack_message('v1/lidar/raw', header, points.tobytes())


def h264_packet(channel, sequence=1, **header_overrides):
    header = base_header(
        sequence=sequence,
        codec='h264',
        pixel_encoding='H264',
        width=640,
        height=400,
        keyframe=True,
        **header_overrides)
    return pack_message(channel, header, b'\x00\x00\x00\x01\x67fake-sps')
