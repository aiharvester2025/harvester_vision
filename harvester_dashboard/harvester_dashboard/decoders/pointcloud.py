"""Camera-relative point cloud built from depth + RGB + camera intrinsics.

This is a **render-time, UI-only** derivation: it consumes the live depth map,
the live RGB frame, and the ``camera_info`` intrinsics (all already decoded by
the dashboard) and back-projects valid depth pixels into the camera optical
frame (``+X`` image-right, ``+Y`` image-down, ``+Z`` forward through the lens)
using the same math as ``model/target_model.py:back_project``.

No new wire channel is introduced; the canonical bus stays RGB + depth +
camera_info and the point cloud is produced entirely in the dashboard.
"""

from __future__ import annotations

import math
from typing import Optional, Tuple

import numpy as np


def _intrinsics(camera_info) -> Optional[Tuple[float, float, float, float]]:
    """Extract ``(fx, fy, cx, cy)`` from a camera_info dict, or ``None``."""
    if not isinstance(camera_info, dict):
        return None
    k = camera_info.get('k')
    if not isinstance(k, (list, tuple)) or len(k) < 9:
        return None
    try:
        fx = float(k[0])
        fy = float(k[4])
        cx = float(k[2])
        cy = float(k[5])
    except (TypeError, ValueError):
        return None
    if not (fx and fy):
        return None
    if not all(np.isfinite(v) for v in (fx, fy, cx, cy)):
        return None
    return fx, fy, cx, cy


def unproject_depth(depth_m, rgb, camera_info, max_points=4096,
                    rgb_width=None, rgb_height=None):
    """Back-project valid depth pixels into the camera optical frame.

    Args:
        depth_m: ``HxW`` float32 depth in metres (NaN/0 = invalid), or ``None``.
        rgb: ``Rh x Rw x 3`` uint8 colour frame, or ``None`` (points still
            returned).  When the depth map is delivered at a different
            resolution than RGB, pass ``rgb_width``/``rgb_height`` so colour
            is sampled at the corresponding RGB pixel.
        camera_info: dict with ``k`` intrinsics for the **depth** resolution,
            or ``None`` (no cloud).
        max_points: cap on the number of returned points (uniform downsample).
        rgb_width/rgb_height: the delivered RGB frame size (for colour mapping).

    Returns a dict ``{'points': Nx3 float32 (m), 'colors': Nx3 uint8}``.
    ``points``/``colors`` are empty ``(0, 3)`` arrays when there is no valid
    input, so callers can iterate without special-casing.
    """
    empty = {'points': np.empty((0, 3), dtype=np.float32),
             'colors': np.empty((0, 3), dtype=np.uint8)}
    if depth_m is None or depth_m.ndim != 2:
        return empty
    params = _intrinsics(camera_info)
    if params is None:
        return empty
    fx, fy, cx, cy = params

    height, width = depth_m.shape

    # Build a uniformly strided pixel grid and sample depth at those pixels,
    # keeping only valid ones.  Striding the grid FIRST (rather than running
    # ``np.nonzero`` over the full valid mask) bounds the index arrays to
    # ~max_points even for a dense 1080p/960x540 depth map, where a full
    # ``np.nonzero`` would transiently allocate two ~518K-element int64 arrays.
    total = height * width
    if max_points and total > int(max_points):
        # Stride so the strided grid has at most ~max_points cells.
        step = int(math.ceil(math.sqrt(total / float(max_points))))
    else:
        step = 1
    grid_v = np.arange(0, height, step, dtype=np.int64)
    grid_u = np.arange(0, width, step, dtype=np.int64)
    vs, us = np.meshgrid(grid_v, grid_u, indexing='ij')
    vs = vs.ravel()
    us = us.ravel()

    # Sample depth at the strided grid; NaN/zero are invalid.
    z_flat = depth_m[vs, us]
    finite = np.isfinite(z_flat)
    valid = finite & np.greater(z_flat, 0.0, where=finite)
    vs = vs[valid]
    us = us[valid]
    if vs.size == 0:
        return empty

    # Final cap: if the strided grid still exceeds max_points (e.g. max_points
    # is small), take a uniform subsample of the already-small valid set.
    if max_points and vs.size > int(max_points):
        indices = np.linspace(0, vs.size - 1, num=int(max_points),
                              dtype=np.int64)
        vs = vs[indices]
        us = us[indices]

    z = depth_m[vs, us].astype(np.float32)
    x = (us.astype(np.float32) - cx) / fx * z
    y = (vs.astype(np.float32) - cy) / fy * z
    points = np.stack([x, y, z], axis=1).astype(np.float32)

    # Colour sampling: map depth pixels to RGB pixels when the two streams
    # differ in resolution (depth is delivered at half the RGB size).
    if rgb is not None and rgb.ndim == 3 and rgb.shape[2] >= 3:
        rgb_h, rgb_w = rgb.shape[0], rgb.shape[1]
        if rgb_width and rgb_height and (rgb_w != width or rgb_h != height):
            # depth pixel (us, vs) -> RGB pixel via the delivered-size ratio.
            scale_u = rgb_width / float(width)
            scale_v = rgb_height / float(height)
            rgb_us = np.clip((us * scale_u).astype(np.int64), 0, rgb_w - 1)
            rgb_vs = np.clip((vs * scale_v).astype(np.int64), 0, rgb_h - 1)
            colors = np.ascontiguousarray(rgb[:, :, :3])[rgb_vs, rgb_us]
        else:
            colors = np.ascontiguousarray(rgb[:, :, :3])[vs, us]
        colors = np.ascontiguousarray(colors, dtype=np.uint8)
    else:
        colors = np.empty((len(points), 3), dtype=np.uint8)

    return {'points': points, 'colors': colors}


__all__ = ['unproject_depth']
