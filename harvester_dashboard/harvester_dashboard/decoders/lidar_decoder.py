"""LiDAR decoder driven entirely by the header ``point_fields`` layout."""

from __future__ import annotations

import struct
from typing import Dict

import numpy as np

_FIELD_DTYPES = {
    'float32': np.dtype('<f4'),
    'float64': np.dtype('<f8'),
    'int8': np.dtype('<i1'),
    'uint8': np.dtype('<u1'),
    'int16': np.dtype('<i2'),
    'uint16': np.dtype('<u2'),
    'int32': np.dtype('<i4'),
    'uint32': np.dtype('<u4'),
}


class LidarDecoder:
    """Unpack ``lidar_xyz_f32`` style records into an ``Nx3`` array.

    The canonical layout is declared per packet in ``point_fields``.
    Only the ``x``/``y``/``z`` fields are extracted; optional intensity
    or tag fields are ignored.  All fields are little-endian per the
    contract, and the point stride is taken from ``point_stride_bytes``.
    """

    def decode(self, header, payload: bytes) -> np.ndarray:
        fields = header.get('point_fields') or []
        stride = int(header.get('point_stride_bytes') or 0)
        declared_count = header.get('point_count')
        if not fields:
            raise ValueError('lidar header is missing point_fields')
        if stride <= 0:
            stride = sum(_FIELD_DTYPES[f['type']].itemsize for f in fields)
        if len(payload) % stride:
            raise ValueError(
                'lidar payload size {} is not a multiple of stride {}'.format(
                    len(payload), stride))
        count = len(payload) // stride
        if isinstance(declared_count, int) and not isinstance(declared_count, bool):
            if declared_count != count:
                raise ValueError(
                    'lidar header declares {} points but payload holds {}'.format(
                        declared_count, count))

        columns: Dict[str, np.ndarray] = {}
        cursor = 0
        for field in fields:
            name = field['name']
            dtype = _FIELD_DTYPES.get(field['type'])
            if dtype is None:
                raise ValueError('unsupported lidar field type {!r}'.format(field['type']))
            # Fields without an explicit offset are packed in declared order.
            offset = int(field.get('offset', cursor))
            cursor = offset + dtype.itemsize
            raw = np.frombuffer(payload, dtype=dtype, count=count, offset=offset)
            stride_records = np.lib.stride_tricks.as_strided(
                raw, shape=(count,), strides=(stride,))
            columns[name] = np.array(stride_records, dtype=np.float32, copy=True)

        missing = [axis for axis in ('x', 'y', 'z') if axis not in columns]
        if missing:
            raise ValueError('lidar point_fields is missing axes {}'.format(missing))
        points = np.stack([columns['x'], columns['y'], columns['z']], axis=1)
        return points.astype(np.float32, copy=False)

    def limit(self, points: np.ndarray, maximum: int) -> np.ndarray:
        """Uniformly downsample to at most ``maximum`` points."""
        if points is None or len(points) <= maximum:
            return points
        indices = np.linspace(0, len(points) - 1, num=maximum, dtype=np.int64)
        return points[indices]
