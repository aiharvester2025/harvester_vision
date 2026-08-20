"""QQuickImageProvider bridging the latest decoded frames to QML Images.

Each camera exposes ``image://frames/<name>?n=<counter>``.  The counter
is what makes Qt re-request the image when the source updates; the
provider converts the newest RGB (or depth-coloured) numpy array into a
QImage on the render thread without copying through Python-side caches
that could grow unbounded.
"""

from __future__ import annotations

import threading
from typing import Dict, Optional, Tuple

import numpy as np

try:
    from PySide2.QtCore import QSize
    from PySide2.QtGui import QColor, QImage, QPainter
    from PySide2.QtQuick import QQuickImageProvider
    _QT_AVAILABLE = True
except ImportError:  # pragma: no cover
    _QT_AVAILABLE = False
    QQuickImageProvider = object


if _QT_AVAILABLE:

    class FrameImageProvider(QQuickImageProvider):
        """Serves the newest decoded frame per image name on demand."""

        def __init__(self):
            super().__init__(QQuickImageProvider.Image)
            self._lock = threading.Lock()
            self._frames: Dict[str, np.ndarray] = {}
            self._sizes: Dict[str, Tuple[int, int]] = {}
            self._misses: Dict[str, int] = {}

        # -- producer side (UI thread) ------------------------------------
        def publish_rgb(self, name: str, rgb: np.ndarray) -> None:
            if rgb is None or rgb.ndim != 3 or rgb.shape[2] < 3:
                return
            with self._lock:
                self._frames[name] = np.ascontiguousarray(rgb[:, :, :3])
                self._sizes[name] = (int(rgb.shape[1]), int(rgb.shape[0]))
                self._misses.pop(name, None)

        def publish_depth_colored(self, name: str, depth_m: np.ndarray,
                                  near_m: float = 0.5, far_m: float = 6.0) -> None:
            colored = colorize_depth(depth_m, near_m, far_m)
            if colored is not None:
                self.publish_rgb(name, colored)

        def clear(self, name: str) -> None:
            with self._lock:
                self._frames.pop(name, None)
                self._sizes.pop(name, None)

        # -- consumer side (QML render thread) ----------------------------
        def requestImage(self, image_id: str, size, requested_size):
            # QML appends the ``?n=<counter>`` query string to the source URL
            # to force a re-request; the query becomes part of ``image_id``.
            # Strip it so the lookup key matches the publish key.
            key = image_id.split('?', 1)[0]
            with self._lock:
                frame = self._frames.get(key)
                stored = self._sizes.get(key)
            if frame is None:
                self._misses[image_id] = self._misses.get(image_id, 0) + 1
                image = QImage(4, 4, QImage.Format_RGB32)
                image.fill(QColor(20, 20, 24))
                size.setWidth(4)
                size.setHeight(4)
                return image
            height, width, _channels = frame.shape
            image = QImage(
                frame.data, width, height, 3 * width, QImage.Format_RGB888)
            if stored is not None:
                size.setWidth(stored[0])
                size.setHeight(stored[1])
            else:
                size.setWidth(width)
                size.setHeight(height)
            return image.copy()   # detach from the numpy buffer


def colorize_depth(depth_m: Optional[np.ndarray],
                   near_m: float = 0.5, far_m: float = 6.0) -> Optional[np.ndarray]:
    """Map a metres depth array to aJet-like RGB uint8 cube for display."""
    if depth_m is None or depth_m.ndim != 2:
        return None
    span = max(1e-6, float(far_m - near_m))
    normalized = np.clip((depth_m - near_m) / span, 0.0, 1.0)
    # Simple cool-to-warm ramp; NaN stays black.
    invalid = ~np.isfinite(depth_m)
    red = (normalized * 255).astype(np.uint8)
    green = ((1.0 - np.abs(normalized - 0.5) * 2.0) * 200).astype(np.uint8)
    blue = ((1.0 - normalized) * 255).astype(np.uint8)
    rgb = np.stack([red, green, blue], axis=2)
    rgb[invalid] = (0, 0, 0)
    return rgb


__all__ = ['FrameImageProvider', 'colorize_depth', '_QT_AVAILABLE']
