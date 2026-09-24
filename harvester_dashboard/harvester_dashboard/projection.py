"""Pure-Python orthographic projection of LiDAR points for the dashboard HUD.

Projection conventions match the Gazebo/RViz ``vehicle_lidar_link`` frame
on Xavier:

    +x = forward
    +y = left
    +z = up

The HUD shows five vehicle-relative views plus one **camera-overlay** view.  In
every vehicle view the observer stands at the named side of the vehicle and
looks toward the origin; the screen axes are written explicitly so the math is
auditable.

The ``camera`` view is different in kind: it draws the cloud in the **camera
optical frame** (``+X`` image-right, ``+Y`` image-down, ``+Z`` forward through the
lens) so it can be overlaid on the live camera image.  It is *overlay by
projection*, not sensor fusion: the horizontal alignment depends on the
camera<->LiDAR extrinsic supplied by the calibration file, which is unverified
until the commissioning survey.  The caller owns the extrinsic; this module only
applies it.
"""

from __future__ import annotations

import math
from typing import List, Optional, Sequence, Tuple

# Supported views in cycle order.  ``camera`` is last so the existing five-view
# cycle for the small inset is unchanged up to the new final step.
VIEWS: Tuple[str, ...] = ('top', 'front', 'left', 'right', 'iso', 'camera')

# Display labels for the HUD title.
VIEW_LABELS = {
    'top':    'top-down (x-y)',
    'front':  'front (y-z)',
    'left':   'left (x-z)',
    'right':  'right (x-z)',
    'iso':    'isometric',
    'camera': 'camera overlay',
}

# Per-view axis annotations: which vehicle axis maps to which screen axis.
# The right_label/up_label are drawn as the corner indicator in the HUD.
AXIS_LABELS = {
    'top':    ('+x fwd',    '+y left'),
    'front':  ('+y left',   '+z up'),
    'left':   ('+x fwd',    '+z up'),
    'right':  ('\u2212x aft', '+z up'),
    'iso':    ('iso',       ''),
    'camera': ('camera',    ''),
}


def project_points(
    points: Optional[List[List[float]]],
    view: str,
    cx: float,
    cy: float,
    scale: float,
) -> List[Tuple[float, float, float]]:
    """Project 3-D points into screen space for the given view.

    Returns a list of ``(screen_x, screen_y, range_m)`` tuples.  Points
    outside the visible window are still returned (negative coords or
    coords beyond the window size) — clipping is the renderer's job.

    The ``camera`` view needs a camera<->LiDAR extrinsic; it cannot be expressed
    with just ``(cx, cy, scale)``, so it is handled by :func:`project_camera`
    instead.  Passing ``'camera'`` here falls back to the vehicle isometric view
    rather than raising, so a stale view name can never blank the HUD.
    """
    if points is None:
        return []
    if view == 'camera':
        view = 'iso'
    out: List[Tuple[float, float, float]] = []
    if view == 'top':
        for p in points:
            x, y, _z = p[0], p[1], p[2]
            out.append((cx + x * scale, cy - y * scale,
                        (x * x + y * y + p[2] * p[2]) ** 0.5))
    elif view == 'front':
        # Observer in front of the vehicle looking toward -x.
        # When facing -x the vehicle's +y (left) is to the viewer's right.
        for p in points:
            x, y, z = p[0], p[1], p[2]
            out.append((cx + y * scale, cy - z * scale,
                        (x * x + y * y + z * z) ** 0.5))
    elif view == 'left':
        # Observer on the left looking toward -y.
        # When facing -y the vehicle's +x (forward) is to the viewer's right.
        for p in points:
            x, _y, z = p[0], p[1], p[2]
            out.append((cx + x * scale, cy - z * scale,
                        (x * x + p[1] * p[1] + z * z) ** 0.5))
    elif view == 'right':
        # Observer on the right looking toward +y.
        # When facing +y the vehicle's +x (forward) is to the viewer's left.
        for p in points:
            x, _y, z = p[0], p[1], p[2]
            out.append((cx - x * scale, cy - z * scale,
                        (x * x + p[1] * p[1] + z * z) ** 0.5))
    else:  # iso
        # 30-degree isometric: floor plane tilted, vertical shortened.
        iso = 0.5
        lift = 0.866
        for p in points:
            x, y, z = p[0], p[1], p[2]
            sx = cx + (x - y) * iso * scale
            sy = cy - ((x + y) * iso * 0.5 + z * lift) * scale
            out.append((sx, sy, (x * x + y * y + z * z) ** 0.5))
    return out


def _apply_rotation(row: Sequence[float], point: Sequence[float]):
    """Apply a row-major 3x3 rotation (length-9) to a 3-vector."""
    (m00, m01, m02, m10, m11, m12, m20, m21, m22) = row
    x, y, z = point[0], point[1], point[2]
    return (
        m00 * x + m01 * y + m02 * z,
        m10 * x + m11 * y + m12 * z,
        m20 * x + m21 * y + m22 * z,
    )


def project_camera(
    points: Optional[List[List[float]]],
    cx: float,
    cy: float,
    scale: float,
    rotation: Optional[Sequence[float]] = None,
    translation: Optional[Sequence[float]] = None,
) -> List[Tuple[float, float, float]]:
    """Project points into the camera optical screen frame for overlay.

    ``rotation`` is the row-major 3x3 LiDAR->camera rotation and
    ``translation`` the LiDAR origin in the camera frame (both from the
    deployment calibration; identity when omitted).  The screen mapping puts
    optical ``+X`` to the right, optical ``+Y`` downward and optical ``+Z``
    (forward, into the scene) upward on screen, which is the natural "look
    through the lens" framing used by ``PointCloudInset.qml``.

    Returns ``(screen_x, screen_y, range_m)`` tuples; clipping is the
    renderer's job.
    """
    if points is None:
        return []
    tx, ty, tz = (0.0, 0.0, 0.0)
    if translation is not None and len(translation) >= 3:
        tx, ty, tz = float(translation[0]), float(translation[1]), float(translation[2])
    out: List[Tuple[float, float, float]] = []
    for p in points:
        x, y, z = float(p[0]), float(p[1]), float(p[2])
        if rotation is not None and len(rotation) >= 9:
            x, y, z = _apply_rotation(rotation, (x, y, z))
        # LiDAR origin offset in the camera frame (translation is LiDAR-in-cam;
        # subtracting it expresses the point relative to the camera).
        px = x - tx
        py = y - ty
        pz = z - tz
        sx = cx + px * scale
        sy = cy + py * scale          # optical +Y is image-down => screen-down
        out.append((sx, sy, math.sqrt(px * px + py * py + pz * pz)))
    return out


__all__ = [
    'VIEWS', 'VIEW_LABELS', 'AXIS_LABELS',
    'project_points', 'project_camera',
]
