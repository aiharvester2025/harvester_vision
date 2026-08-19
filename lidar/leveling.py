"""Source-agnostic LiDAR leveling: turn an orientation into a gravity rotation.

The MID-360 is mounted on a moving arm. Its raw points are expressed in the
sensor frame, so a world-vertical tree appears to lean when the arm pitches or
rolls. Leveling re-aligns the cloud to gravity using only the *orientation*,
which is available from several sources:

  * the MID-360's built-in IMU (pitch/roll are gravity-referenced and stable),
  * the platform's 2-axis tilt sensor (via PLC/Modbus), and/or
  * the boom angle sensor (via PLC/Modbus).

Yaw is intentionally not required: it only rotates the scene around the
vertical axis and does not affect whether a tree appears to lean. IMU yaw also
drifts without an external reference, so we deliberately do not depend on it.

This module is dependency-free (no ROS, no numpy) and mirrors the convention
used by ``geometry/transforms.py``: ``T_parent_child`` maps a point from child
into parent, and a rotation matrix ``R`` maps ``p_parent = R @ p_child``.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional, Sequence, Tuple

from geometry.transforms import (
    Matrix3,
    TransformConfigurationError,
    level_points_rotation_only,
    rotation_from_quaternion,
    rotation_from_rpy,
)

Vector3 = Tuple[float, float, float]


@dataclass(frozen=True)
class OrientationSource:
    """A named orientation estimate with a quality/time stamp.

    ``quaternion`` is ``(x, y, z, w)`` in the sensor frame such that applying
    ``rotation_from_quaternion`` maps sensor-frame points into a gravity-aligned
    frame. ``valid`` is false when the source is stale or uncalibrated.
    """

    name: str
    quaternion: Tuple[float, float, float, float]
    valid: bool = True
    timestamp_us: Optional[int] = None


def quaternion_from_euler(roll: float, pitch: float, yaw: float = 0.0) -> Tuple[float, float, float, float]:
    """Build a quaternion ``(x, y, z, w)`` from intrinsic roll/pitch/yaw (rad).

    Matches ``geometry.transforms.rotation_from_rpy`` so a tilt source can be
    converted to the same quaternion convention used by an IMU source.
    """
    cr, sr = math.cos(roll / 2.0), math.sin(roll / 2.0)
    cp, sp = math.cos(pitch / 2.0), math.sin(pitch / 2.0)
    cy, sy = math.cos(yaw / 2.0), math.sin(yaw / 2.0)
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy
    w = cr * cp * cy + sr * sp * sy
    return (x, y, z, w)


def level_orientation_quaternion(source: OrientationSource) -> Matrix3:
    """Return the gravity-aligned rotation matrix for an orientation source.

    Raises ``TransformConfigurationError`` if the source is not valid, so a
    stale or uncalibrated orientation never silently produces a wrong cloud.
    """
    if not source.valid:
        raise TransformConfigurationError(
            f"orientation source {source.name!r} is not valid"
        )
    return rotation_from_quaternion(*source.quaternion)


def level_points(points: Sequence[Sequence[float]], source: OrientationSource) -> list[Vector3]:
    """Level a point cloud using an orientation source (rotation only)."""
    return level_points_rotation_only(points, level_orientation_quaternion(source))


def level_from_tilt_rpy(
    points: Sequence[Sequence[float]],
    roll_rad: float,
    pitch_rad: float,
    source_name: str = "platform_tilt",
    valid: bool = True,
) -> list[Vector3]:
    """Convenience wrapper: level using roll/pitch directly (no yaw)."""
    source = OrientationSource(
        name=source_name,
        quaternion=quaternion_from_euler(roll_rad, pitch_rad, 0.0),
        valid=valid,
    )
    return level_points(points, source)
