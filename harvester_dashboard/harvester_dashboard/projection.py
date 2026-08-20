"""Pure-Python orthographic projection of LiDAR points for the dashboard HUD.

Projection conventions match the Gazebo/RViz ``vehicle_lidar_link`` frame
on Xavier:

    +x = forward
    +y = left
    +z = up

The HUD shows five views.  In every case the observer stands at the named
side of the vehicle and looks toward the origin; the screen axes are
written explicitly so the math is auditable.
"""

from __future__ import annotations

from typing import List, Optional, Tuple

# Supported views in cycle order.
VIEWS: Tuple[str, ...] = ('top', 'front', 'left', 'right', 'iso')

# Display labels for the HUD title.
VIEW_LABELS = {
    'top':   'top-down (x-y)',
    'front': 'front (y-z)',
    'left':  'left (x-z)',
    'right': 'right (x-z)',
    'iso':   'isometric',
}

# Per-view axis annotations: which vehicle axis maps to which screen axis.
# The right_label/up_label are drawn as the corner indicator in the HUD.
AXIS_LABELS = {
    'top':   ('+x fwd',  '+y left'),
    'front': ('+y left', '+z up'),
    'left':  ('+x fwd',  '+z up'),
    'right': ('\u2212x aft', '+z up'),
    'iso':   ('iso',     ''),
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
    """
    if points is None:
        return []
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