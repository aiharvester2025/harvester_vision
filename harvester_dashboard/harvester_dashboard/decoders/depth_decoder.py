"""Canonical depth decoder: uint16 LE millimetres -> float32 metres.

The payload is exactly ``width * height * 2`` little-endian uint16 bytes
in millimetres; ``0`` means invalid/no return and decodes to ``NaN`` so
distance math and visualisation can share one convention.
"""

from __future__ import annotations

import numpy as np

from .errors import UnsupportedCodecError


class DepthDecoder:

    def decode(self, header, payload: bytes) -> np.ndarray:
        width = int(header['width'])
        height = int(header['height'])
        expected = width * height * 2
        if len(payload) != expected:
            raise ValueError(
                'depth payload is {} bytes; expected {} ({}x{}x2)'.format(
                    len(payload), expected, width, height))
        millimetres = np.frombuffer(payload, dtype='<u2').reshape((height, width))
        metres = millimetres.astype(np.float32) * 1e-3
        metres[millimetres == 0] = np.float32('nan')
        return metres

    def depth_at(self, depth_m: np.ndarray, u: int, v: int, window: int = 3):
        """Nearest valid depth in a ``(2*window+1)`` square around (u, v).

        Returns a float in metres or ``None`` when every pixel in the
        window is invalid, out of bounds, or the array is absent.
        """
        if depth_m is None or depth_m.ndim != 2:
            return None
        height, width = depth_m.shape
        u = int(u)
        v = int(v)
        if not (0 <= u < width and 0 <= v < height):
            return None
        window = max(0, int(window))
        u0 = max(0, u - window)
        u1 = min(width, u + window + 1)
        v0 = max(0, v - window)
        v1 = min(height, v + window + 1)
        patch = depth_m[v0:v1, u0:u1]
        valid = patch[np.isfinite(patch)]
        if valid.size == 0:
            return None
        return float(np.min(valid))
