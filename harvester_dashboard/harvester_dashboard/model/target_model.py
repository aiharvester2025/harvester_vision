"""Camera-relative operator annotation state (Phase 1 scope).

A click on the active camera view looks up depth with the latest decoded
depth frame and ``camera_info`` intrinsics, back-projects to a
camera-relative 3-D point, and stays overlaid on the image until cleared
with ``0`` / ``Esc``.  World-fixed anchoring is deliberately absent until
``v1/pose/*`` exists and ``target.world_fixed`` is true; the UI must not
claim a world-fixed target.
"""

from __future__ import annotations

import math
import time
from typing import Optional, Tuple


class AnnotationState:
    """One active annotation plus an in-app event log."""

    def __init__(self, clock=time.monotonic):
        self._clock = clock
        self.active = False
        self.camera = ''            # 'cutter' or 'docking'
        self.pixel = (0, 0)         # (u, v) in camera pixels
        self.point_camera = None    # (x, y, z) metres, camera frame
        self.frame_id = ''
        self.depth_m = None
        self.created_monotonic_s = None
        self.events = []            # in-app log of annotation actions

    # ---------------------------------------------------------------- create
    def clear(self, reason: str = 'cleared') -> None:
        if self.active:
            self._log('annotation cleared ({})'.format(reason))
        self.active = False
        self.camera = ''
        self.pixel = (0, 0)
        self.point_camera = None
        self.frame_id = ''
        self.depth_m = None
        self.created_monotonic_s = None

    def build(self, camera: str, u: int, v: int, depth_m: Optional[float],
              camera_info: Optional[dict], frame_id: str = '') -> Tuple[bool, str]:
        """Create or refuse an annotation from a pixel click.

        Returns ``(accepted, message)``.  Missing/zero depth is refused
        with a ``NO DEPTH`` style message and nothing is stored or
        published.
        """
        if depth_m is None or not math.isfinite(depth_m) or depth_m <= 0.0:
            message = 'NO DEPTH at ({}, {}) — annotation rejected'.format(u, v)
            self._log(message)
            return False, message
        point = back_project(u, v, depth_m, camera_info)
        self.active = True
        self.camera = camera
        self.pixel = (int(u), int(v))
        self.depth_m = float(depth_m)
        self.point_camera = point
        self.frame_id = frame_id
        self.created_monotonic_s = self._clock()
        message = 'annotation camera-relative {:.2f} m at ({}, {}) in {}'.format(
            depth_m, u, v, frame_id or camera)
        self._log(message)
        return True, message

    # ------------------------------------------------------------------ view
    def label(self) -> str:
        if not self.active or self.point_camera is None:
            return ''
        x, y, z = self.point_camera
        return '{:+.2f},{:+.2f},{:+.2f} m  |  {:.2f} m'.format(x, y, z, self.depth_m or 0.0)

    def is_world_fixed_claimed(self) -> bool:
        """Phase 1 never claims world-fixed anchoring."""
        return False

    def _log(self, message: str) -> None:
        stamp = time.strftime('%H:%M:%S')
        self.events.append('[{}] {}'.format(stamp, message))
        del self.events[:-100]


def back_project(u: int, v: int, depth_m: float, camera_info: Optional[dict]):
    """Back-project a pixel using camera_info ``k`` (fx, fy, cx, cy).

    Returns ``(x, y, z)`` in the camera optical frame, or ``None`` when
    intrinsics are unusable.  ``z`` is along the optical axis (forward).
    """
    if not isinstance(camera_info, dict):
        return None
    k = camera_info.get('k')
    if not isinstance(k, (list, tuple)) or len(k) < 9:
        return None
    try:
        fx, fy = float(k[0]), float(k[4])
        cx, cy = float(k[2]), float(k[5])
    except (TypeError, ValueError):
        return None
    if not fx or not fy or not math.isfinite(fx) or not math.isfinite(fy):
        return None
    x = (float(u) - cx) / fx * depth_m
    y = (float(v) - cy) / fy * depth_m
    return (x, y, float(depth_m))
