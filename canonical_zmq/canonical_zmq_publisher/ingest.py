"""Synthetic canonical telemetry source for validating the Orin aggregator.

Emits sample canonical packets for every channel the dashboard renders, so the
Orin canonical publisher + dashboard can be validated end-to-end without any
hardware and without the Xavier.  Data is clearly marked ``source_mode:
hardware`` (``source_id: orin``) and uses placeholder values.
"""

from __future__ import annotations

import math
import struct
import time

import numpy as np

from harvester_telemetry_contract import pack_message


def _base_header(channel, clock_domain='plc_rtc_utc'):
    return {
        'schema_version': 1,
        'source_mode': 'hardware',
        'source_id': 'orin',
        'sequence': 0,
        'frame_id': 'cutter_camera_optical_frame',
        'acquisition_timestamp_ns': time.time_ns(),
        'clock_domain': clock_domain,
        'gateway_monotonic_ns': 0,
        'calibration_id': 'synthetic_orin_v0',
        'capabilities': {
            'camera.cutter.rgb': True,
            'camera.cutter.depth': True,
            'camera.cutter.camera_info': True,
            'lidar.raw_xyz': True,
            'range.docking': True,
            'range.cutter': True,
            'docking.trunk_estimate': True,
            'calibration.status': True,
            'target.world_fixed': False,
        },
    }


def synthetic_rgb_packet(width=640, height=360, step=0):
    """Build a small deterministic RGB JPEG packet."""
    # Minimal in-memory JPEG via numpy -> raw RGB (pillow-free fallback is a
    # tiny gradient written as a valid JPEG using cv2 if available, else a
    # placeholder that the decoder will flag).  For dashboard validation we use
    # cv2 (present in the depthai-env) to encode a real JPEG.
    import cv2
    image = np.zeros((height, width, 3), dtype=np.uint8)
    for row in range(height):
        image[row, :, 0] = int(255 * row / max(1, height))  # red gradient
        image[row, :, 1] = int(255 * row / max(1, height))  # green gradient
        image[row, :, 2] = 128
    ok, encoded = cv2.imencode('.jpg', image, [int(cv2.IMWRITE_JPEG_QUALITY), 85])
    if not ok:
        raise ValueError('synthetic JPEG encode failed')
    return encoded.tobytes()


def synthetic_depth_payload(width=640, height=360):
    """A uint16 millimetre depth plane (constant 2.0 m)."""
    values = np.full((height, width), 2000, dtype='<u2')
    return np.ascontiguousarray(values).tobytes()


def synthetic_camera_info():
    import json
    return json.dumps({
        'width': 640,
        'height': 360,
        'distortion_model': 'plumb_bob',
        'd': [0.0, 0.0, 0.0, 0.0, 0.0],
        'k': [500.0, 0.0, 320.0, 0.0, 500.0, 180.0, 0.0, 0.0, 1.0],
        'r': [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
        'p': [500.0, 0.0, 320.0, 0.0, 0.0, 500.0, 180.0, 0.0, 0.0, 0.0, 1.0, 0.0],
        'binning_x': 1,
        'binning_y': 1,
        'roi': {'x_offset': 0, 'y_offset': 0, 'height': 360, 'width': 640,
                'do_rectify': False},
    }, separators=(',', ':')).encode('utf-8')


def synthetic_lidar_payload(count=200):
    """A synthetic XYZ float32 cloud (a flat ring)."""
    output = bytearray()
    for i in range(count):
        angle = 2.0 * math.pi * i / count
        x = 3.0 * math.cos(angle)
        y = 3.0 * math.sin(angle)
        z = 1.0
        output.extend(struct.pack('<fff', x, y, z))
    return bytes(output)


class SyntheticSource:
    """Repeatedly feeds one sample of each channel to an aggregator."""

    def __init__(self, aggregator, period_s=1.0):
        self.aggregator = aggregator
        self.period_s = max(0.05, float(period_s))
        self._tick = 0

    def emit_once(self):
        import json
        agg = self.aggregator
        self._tick += 1

        # Cutter RGB
        rgb = synthetic_rgb_packet()
        header = _base_header('v1/camera/cutter/rgb')
        header.update({'codec': 'jpeg', 'pixel_encoding': 'RGB8',
                       'width': 640, 'height': 360})
        agg.publish('v1/camera/cutter/rgb', header, rgb)

        # Cutter depth
        header = _base_header('v1/camera/cutter/depth')
        header.update({'codec': 'depth_uint16_le', 'width': 640, 'height': 360})
        agg.publish('v1/camera/cutter/depth', header, synthetic_depth_payload())

        # Cutter camera info
        header = _base_header('v1/camera/cutter/camera_info')
        header.update({'codec': 'json', 'width': 640, 'height': 360})
        agg.publish('v1/camera/cutter/camera_info', header, synthetic_camera_info())

        # Docking camera (reuse cutter payload as a placeholder)
        header = _base_header('v1/camera/docking/rgb')
        header.update({'codec': 'jpeg', 'pixel_encoding': 'RGB8',
                       'width': 640, 'height': 360})
        agg.publish('v1/camera/docking/rgb', header, rgb)

        # LiDAR
        header = _base_header('v1/lidar/raw')
        header.update({'frame_id': 'vehicle_lidar_link',
                       'codec': 'lidar_xyz_f32',
                       'point_count': 200, 'point_stride_bytes': 12,
                       'point_fields': [
                           {'name': 'x', 'type': 'float32'},
                           {'name': 'y', 'type': 'float32'},
                           {'name': 'z', 'type': 'float32'},
                       ]})
        agg.publish('v1/lidar/raw', header, synthetic_lidar_payload())

        # Docking ranges (five named readings)
        docking_records = [
            {'telemetry_key': key, 'distance_m': round(0.5 + 0.01 * (i + self._tick), 3),
             'valid': True, 'frame_id': 'range_sensor_{}'.format(key),
             'acquisition_timestamp_ns': time.time_ns(),
             'calibration_id': 'synthetic_orin_v0',
             'min_range_m': 0.1, 'max_range_m': 2.0}
            for i, key in enumerate(('center_line', 'left_45_deg', 'right_45_deg',
                                     'left_side', 'right_side'))
        ]
        header = _base_header('v1/range/docking')
        header.update({'codec': 'json'})
        agg.publish_json('v1/range/docking', header, docking_records)

        # Cutter range
        header = _base_header('v1/range/cutter')
        header.update({'codec': 'json'})
        agg.publish_json('v1/range/cutter', header, {
            'telemetry_key': 'cutter_forward', 'distance_m': 0.42,
            'valid': True, 'frame_id': 'cutter_range',
            'acquisition_timestamp_ns': time.time_ns(),
            'calibration_id': 'synthetic_orin_v0',
            'min_range_m': 0.1, 'max_range_m': 2.0})

        # Trunk estimate
        header = _base_header('v1/docking/trunk_estimate')
        header.update({'codec': 'json'})
        agg.publish_json('v1/docking/trunk_estimate', header, {
            'pose': {'position': {'x': 0.0, 'y': 0.0, 'z': 3.0},
                     'orientation': {'x': 0.0, 'y': 0.0, 'z': 0.0, 'w': 1.0}},
            'covariance': [0.0] * 36})

        # Calibration status
        header = _base_header('v1/calibration/status', clock_domain='utc_host')
        header.update({'codec': 'json', 'transform_valid': True,
                       'transform_freshness_s': None})
        agg.publish_json('v1/calibration/status', header, {
            'status': 'VALID', 'calibration_id': 'synthetic_orin_v0'})

    def run_forever(self):
        while True:
            self.emit_once()
            time.sleep(self.period_s)
