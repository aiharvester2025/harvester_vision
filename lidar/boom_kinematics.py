"""Forward kinematics for the LiDAR's height above ground (dependency-free).

The MID-360 is mounted rigidly on ``cutting_arm_base_link`` (see the URDF
``vehicle_lidar_link``), which sits at the end of this chain:

    ground
      -> base_link (prime mover)
           -> boom_turret_joint        [yaw, UNMEASURED]
                -> boom_elevation_joint  [pivot angle, PLC length sensor]
                     -> boom_extension     [telescopic, PLC length sensor]
                          -> platform_level_joint [pitch+roll, tilt sensor]
                               -> rail_carriage_joint [UNMEASURED]
                                    -> cutting_arm_lift_joint [UNMEASURED]
                                         -> cutting_arm_base_link -> mid360_link

Only the LiDAR's *height above ground* (Z) is recovered here, because:

  * tree height / trunk-end are vertical (gravity-aligned) measurements, and
  * the missing turret yaw / rail / cutting-arm-lift affect horizontal pose,
    not the vertical offset (apart from the cutting-arm lift, which is treated
    as a calibrated constant -- see ``cutting_arm_lift_offset_m``).

Orientation is NOT computed here: leveling uses the IMU directly (gravity), so
this module only supplies the vertical datum to convert a leveled cloud's
relative height into an absolute height above ground.
"""

from __future__ import annotations

import math
from dataclasses import dataclass


@dataclass(frozen=True)
class BoomGeometry:
    """Calibrated static offsets that turn PLC readings into a LiDAR height.

    All values are in metres/radians and must come from a measured calibration
    session (or the URDF), never from CAD guesses:

      * ``pivot_height_m``: vertical offset from ground to the boom elevation
        pivot point (the boom's hinge on the turret).
      * ``boom_stage0_length_m``: fixed (non-telescopic) boom length from the
        pivot to the start of the telescopic section.
      * ``platform_level_offset_m``: fixed vertical offset from the boom tip to
        the platform (the ``platform_level_joint`` origin).
      * ``cutting_arm_lift_offset_m``: vertical offset added by the (unmeasured)
        cutting-arm lift, plus the LiDAR's 0.30 m mount on ``cutting_arm_base_link``.
        Hold this constant during a scan; survey it once.
    """

    pivot_height_m: float
    boom_stage0_length_m: float
    platform_level_offset_m: float
    cutting_arm_lift_offset_m: float


@dataclass(frozen=True)
class BoomState:
    """Live PLC readings for the measured boom degrees of freedom."""

    pivot_angle_rad: float = 0.0
    extension_m: float = 0.0
    platform_pitch_rad: float = 0.0
    platform_roll_rad: float = 0.0


def lidar_height_above_ground(state: BoomState, geometry: BoomGeometry) -> float:
    """Return the LiDAR's height above ground (metres).

    The boom pivots about the elevation joint, so both the fixed stage and the
    telescopic extension contribute a ``length * sin(pivot_angle)`` vertical
    component. The platform tilt is small and its vertical contribution is
    approximated by its pitch; roll is lateral and does not change height to
    first order. The cutting-arm lift is a calibrated constant.
    """
    total_boom_length = geometry.boom_stage0_length_m + state.extension_m
    sin_pivot = math.sin(state.pivot_angle_rad)
    boom_vertical = total_boom_length * sin_pivot
    platform_vertical = geometry.platform_level_offset_m * math.cos(
        state.platform_pitch_rad
    )
    return (
        geometry.pivot_height_m
        + boom_vertical
        + platform_vertical
        + geometry.cutting_arm_lift_offset_m
    )


def height_of_point_above_ground(
    point_z_in_lidar: float,
    lidar_height: float,
) -> float:
    """Convert a leveled-cloud Z (height relative to the LiDAR) to absolute Z.

    In the leveled (gravity-aligned, LiDAR-origin) frame, a point's Z is its
    height relative to the LiDAR. Adding the LiDAR's own height above ground
    yields the point's absolute height above ground.
    """
    return lidar_height + point_z_in_lidar
