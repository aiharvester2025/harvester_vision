"""IMU-based vibration stabilization of the camera point cloud.

The camera point cloud is expressed in the camera **optical frame**
(``+X`` image-right, ``+Y`` image-down, ``+Z`` forward through the lens), which
is the same frame Luxonis reports its ``*_UNCALIBRATED`` IMU outputs in.

A hydraulic harvester vibrates, which appears as small, fast roll/pitch tilt of
the camera that is *not* real scene motion.  This module derives the current
attitude (roll/pitch) from the gravity direction measured by the accelerometer
and removes the tilt **relative to a reference attitude** from the cloud, so the
operator sees a de-jittered scene while long-term boom motion (the reference,
tracked slowly) is preserved.

Everything here is pure numpy and unit-tested; it has no Qt or depthai
dependency, so the identical math can be reused in the OAK adapter if the
compensation is ever moved to the producer.
"""

from __future__ import annotations

from typing import Optional, Sequence, Tuple

import numpy as np


def gravity_to_rpy(accel_ms2: Sequence[float]) -> Tuple[float, float, float]:
    """Return ``(roll_rad, pitch_rad, accel_norm_ms2)`` from an accelerometer
    reading in the camera optical frame.

    At rest the accelerometer measures the specific force (the reaction to
    gravity), so the gravity direction in the body frame is ``-a / |a|``.
    Roll is rotation about the optical ``+Z`` axis; pitch is rotation about
    ``+X``.
    """
    a = np.asarray(accel_ms2, dtype=np.float64)
    if a.size != 3:
        return 0.0, 0.0, 0.0
    norm = float(np.linalg.norm(a))
    if not np.isfinite(norm) or norm <= 0.0:
        return 0.0, 0.0, 0.0
    gx, gy, gz = -a[0] / norm, -a[1] / norm, -a[2] / norm
    roll = float(np.arctan2(gy, gz))
    pitch = float(np.arctan2(-gx, np.hypot(gy, gz)))
    return roll, pitch, norm


def _rpy_to_rotation(rpy: Sequence[float]) -> np.ndarray:
    """Return the 3x3 rotation for the optical-frame tilt convention.

    ``rpy`` is ``(roll, pitch)`` in the **same axes** as :func:`gravity_to_rpy`
    produces: ``roll`` is rotation about the optical ``+Z`` axis and ``pitch``
    is rotation about ``+X`` (the third element, if present, is ignored — the
    accelerometer cannot observe yaw about gravity).  The rotation is built as
    ``Rz(roll) @ Rx(pitch)``; for the small vibration angles this compensates,
    composition order is a second-order effect.
    """
    roll = float(rpy[0])
    pitch = float(rpy[1])
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    rz = np.array([
        [cr, -sr, 0.0],
        [sr, cr, 0.0],
        [0.0, 0.0, 1.0],
    ], dtype=np.float64)
    rx = np.array([
        [1.0, 0.0, 0.0],
        [0.0, cp, -sp],
        [0.0, sp, cp],
    ], dtype=np.float64)
    return rz @ rx


def tilt_delta_rotation(current_rpy: Sequence[float],
                        reference_rpy: Sequence[float]) -> np.ndarray:
    """Return the 3x3 rotation that removes the tilt ``current`` relative to
    ``reference`` (i.e. the small rotation back toward the reference attitude).

    For an attitude-change ``d = current - reference`` this is the rotation
    that maps a point expressed in the *current* (vibrated) frame back into the
    *reference* frame.  It is a rotation only (no translation, no scaling), so
    distances are preserved.
    """
    c = np.asarray(current_rpy, dtype=np.float64)
    r = np.asarray(reference_rpy, dtype=np.float64)
    delta = c - r
    # Small-angle inverse: R(delta)^-1 == R(-delta).
    return _rpy_to_rotation(-delta)


def stabilize_points(points: np.ndarray, current_rpy: Sequence[float],
                     reference_rpy: Sequence[float]) -> np.ndarray:
    """Apply the tilt-delta rotation to an ``Nx3`` array of camera-frame points.

    Returns a new ``Nx3`` float32 array.  Inputs of the wrong shape (or empty)
    are returned unchanged (as a float32 copy when possible).
    """
    points = np.asarray(points, dtype=np.float64)
    if points.ndim != 2 or points.shape[1] != 3 or points.size == 0:
        return np.asarray(points, dtype=np.float32)
    rotation = tilt_delta_rotation(current_rpy, reference_rpy)
    rotated = points @ rotation.T
    return rotated.astype(np.float32)


__all__ = ['gravity_to_rpy', 'tilt_delta_rotation', 'stabilize_points']
