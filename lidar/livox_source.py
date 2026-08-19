"""Thin adapter between the Livox-SDK2 point/IMU callbacks and this publisher.

This file is the ONLY place that needs to know the Livox SDK API. Everything
else (``mid360_publisher.py``, ``lidar/leveling.py``) is SDK-agnostic.

To wire the real hardware, implement ``sample()`` against ``livox_sdk2`` (or
the Livox ROS 2 driver's data structs if you later adopt it on a companion
node). The contract is simple: return raw sensor-frame points plus a
gravity-aligned orientation quaternion ``(x, y, z, w)``.

Important vendor-frame note: the MID-360 native axes differ from this project's
``+X forward / +Y left / +Z up`` mechanical convention. Confirm the installed
vendor frame from the Livox SDK point struct and the mounting orientation, and
convert to the project convention *here* so the rest of the stack is correct.
"""

from __future__ import annotations

from typing import List, Optional, Tuple


class LivoxMid360Source:
    """Placeholder for the Livox SDK point + IMU source.

    ``level_source`` selects which orientation to use for leveling:

      * ``imu``  -> MID-360 built-in IMU (pitch/roll gravity-referenced).
      * ``tilt`` -> platform 2-axis tilt sensor (via PLC/ZMQ bridge).
      * ``boom`` -> boom angle sensor (via PLC/ZMQ bridge).

    Only ``imu`` is self-contained inside the LiDAR; the others are consumed
    from the PLC sensor bridge and combined here.
    """

    def __init__(self, level_source: str = "imu"):
        self.level_source = level_source
        # TODO: initialise the Livox SDK2 device and its data queues here.

    def sample(self, max_points: int) -> Tuple[List[Tuple[float, float, float]],
                                               Tuple[float, float, float, float],
                                               bool,
                                               str]:
        """Return (points, quaternion, valid, source_name) for one scan.

        This stub returns an empty cloud and an invalid orientation so callers
        degrade safely until the SDK is implemented.
        """
        # TODO: pull the latest point batch from the SDK queue, convert from the
        # vendor frame to the project frame, and downsample to ``max_points``.
        points: List[Tuple[float, float, float]] = []
        quaternion: Tuple[float, float, float, float] = (0.0, 0.0, 0.0, 1.0)
        valid = False
        return points, quaternion, valid, self.level_source
