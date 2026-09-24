"""MID-360 IMU frame conversion for vibration stabilization.

The OAK-camera stabilization in :mod:`harvester_dashboard.decoders.imustab` was
written for the **camera optical frame** (``+X`` image-right, ``+Y`` image-down,
``+Z`` forward through the lens), and its ``_rpy_to_rotation`` builds
``Rz(roll) @ Rx(pitch)`` in that convention.

The MID-360 reports points (and its gravity vector) in the Livox sensor frame
``+X forward / +Y left / +Z up``, which is this project's mechanical frame.  That
is a **different** axis convention from the optical frame, so feeding a MID-360
roll/pitch straight into ``imustab`` would rotate the cloud about the wrong axes
and tilt it instead of levelling it.

This module is the single, documented conversion point.  It converts a MID-360
attitude expressed as ``(roll, pitch)`` **about the sensor's own ``+X`` / ``+Y``
axes** into the ``(roll, pitch)`` pair ``imustab`` expects for the *same physical
tilt*, so the camera and LiDAR paths share one stabilization implementation.

The relation is exact and axis-algebraic, not a small-angle fudge.  For small
vibration angles either ordering is second order, matching the note in
``imustab``; the mapping below keeps the signs deliberate so a levelled cloud
stays level.

Sensor frame:  X forward, Y left, Z up.
  * a positive sensor *pitch* tips the nose up/down about the sensor ``+Y``
    (left) axis, and
  * a positive sensor *roll* tips about the sensor ``+X`` (forward) axis.

Optical frame: X right, Y down, Z forward.
  * a positive optical *pitch* tips about the optical ``+X`` (right) axis, and
  * a positive optical *roll* tips about the optical ``+Z`` (forward) axis.

Mapping a sensor-frame tilt of the gravity vector onto the optical axes gives::

    optical_pitch = +sensor_pitch      # same physical rotation about right/forward
    optical_roll  = -sensor_roll       # sensor roll about forward == -optical roll

The sign of ``optical_roll`` is the one that matters: getting it wrong rolls the
cloud the wrong way.  It is asserted by the unit tests with a synthetic tilt.
"""

from __future__ import annotations

from typing import Optional, Sequence, Tuple

import numpy as np

#: The frame this module converts *from*.
LIVOX_SENSOR_FRAME = 'mid360_link'
#: The frame :mod:`harvester_dashboard.decoders.imustab` operates in.
OPTICAL_FRAME = 'oak_optical_frame'


def sensor_rpy_to_optical(attitude_rpy: Optional[Sequence[float]]
                          ) -> Optional[Tuple[float, float]]:
    """Convert a MID-360 ``(roll, pitch, yaw)`` to optical ``(roll, pitch)``.

    Returns ``None`` for a missing/malformed/non-finite input so a bad IMU
    sample is treated as "no attitude" (identical to the camera path) rather
    than silently rotating the cloud by garbage.
    """
    if attitude_rpy is None:
        return None
    if not isinstance(attitude_rpy, (list, tuple)) or len(attitude_rpy) < 2:
        return None
    try:
        sensor_roll = float(attitude_rpy[0])
        sensor_pitch = float(attitude_rpy[1])
    except (TypeError, ValueError, OverflowError):
        return None
    if not (np.isfinite(sensor_roll) and np.isfinite(sensor_pitch)):
        return None
    # Sensor +X is forward; the optical frame's forward is +Z.  A sensor pitch
    # (about left/right) maps to the optical pitch (about right) unchanged; a
    # sensor roll (about forward) maps to the negative optical roll (about
    # forward).  See the module docstring for the axis derivation.
    return (-sensor_roll, sensor_pitch)


__all__ = [
    'LIVOX_SENSOR_FRAME', 'OPTICAL_FRAME', 'sensor_rpy_to_optical',
]
