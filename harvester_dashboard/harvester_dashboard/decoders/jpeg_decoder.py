"""JPEG RGB decoder.

Uses the Jetson ``nvjpegdec`` GStreamer element (hardware JPEG engine) when
GStreamer is available — matches the goal of avoiding CPU burden for LiDAR
processing.  Falls back to PIL when GStreamer is unavailable (e.g. on the
depthai-env python), so the dashboard stays portable.
"""

from __future__ import annotations

import io

import numpy as np

try:
    from PIL import Image as PilImage
    _PIL_AVAILABLE = True
except ImportError:  # pragma: no cover
    _PIL_AVAILABLE = False


class JpegDecoder:
    """Decode canonical JPEG payloads into RGB uint8 arrays."""

    def __init__(self):
        self._hw = None
        # Lazy import: ``jetson_jpeg`` imports gi/Gst which is only available
        # under the system python.  Importing it eagerly would break the
        # depthai-env python path (which has no gi).
        from .jetson_jpeg import _GST_AVAILABLE, _nvjpegdec_available
        self._hw_available = _GST_AVAILABLE and _nvjpegdec_available()

    def decode(self, header, payload: bytes) -> np.ndarray:
        if self._hw_available and self._hw is None:
            try:
                from .jetson_jpeg import JetsonJpegDecoder
                self._hw = JetsonJpegDecoder()
            except Exception:
                self._hw_available = False
        if self._hw is not None:
            return self._hw.decode(header, payload)
        # PIL fallback (no GStreamer / no nvjpegdec).
        if not _PIL_AVAILABLE:
            raise RuntimeError(
                'no JPEG decoder available (neither Jetson nvjpegdec nor PIL)')
        width = int(header['width'])
        height = int(header['height'])
        image = PilImage.open(io.BytesIO(payload))
        if image.mode != 'RGB':
            image = image.convert('RGB')
        array = np.asarray(image, dtype=np.uint8)
        if array.shape != (height, width, 3):
            raise ValueError(
                'decoded JPEG is {} but the header declares {}x{}'.format(
                    array.shape[:2], width, height))
        return array