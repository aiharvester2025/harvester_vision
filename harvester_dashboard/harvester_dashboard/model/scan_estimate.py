"""Encoder-free tree-height / boom-target estimation for the operator HUD.

This module is the Orin, human-operated counterpart of the ROS 2 / Gazebo
tree-scan work in ``aiharvester2025/ros2_ws``.  It is **Qt-free and ROS-free**
(pure numpy) so it can be unit-tested headlessly and reused by any consumer.

The math and the constants are deliberately lifted from the validated
``ros2_ws`` plans (``.kilo/plans/1787450000000-tree-height-estimation-v2.md``
and ``1787600000000-boom-distance-angle-extension-plan.md``) rather than
re-derived, including the two as-built corrections discovered during the live
simulation:

  * the platform-level command is ``+theta_b`` (the level joint axis is
    ``(0,+1,0)``, opposite the boom's ``(0,-1,0)``), and
  * the boom pivot sits at world ``z = 0.05 + 1.76 = 1.81 m`` (the base parks
    0.05 m above ground and the pivot is 1.76 m above ``base_link``).

Conventions
-----------
Points are in the project mechanical frame: ``+X forward / +Y left / +Z up``
(the MID-360 vendor frame already matches this).  Levels are metres, angles
radians unless a ``_deg`` suffix says otherwise.

Safety boundary
---------------
Everything here is **advisory**.  It is a geometry estimate from one scan, not
a measured contact; the authoritative docking guard remains the five measured
range sensors (see ``harvester_dashboard.safety_guidance``).  Nothing in this
module writes to a socket or commands motion.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import List, Optional, Sequence

import numpy as np

# ---------------------------------------------------------------------------
# Geometry constants (metres), from the URDF + as-built corrections.
# ---------------------------------------------------------------------------
# Boom pivot height above the GROUND plane, in the ``base_link`` frame plus the
# 0.05 m base park offset.  ``ros2_ws`` validation found the un-corrected value
# (1.76) produced a 0.05 m vertical docking error, so the corrected value ships.
BOOM_PIVOT_WORLD_Z_M = 0.05 + 1.76  # 1.81
# Rigid boom tip-to-mount offset (``platform_level_joint`` origin x).
BOOM_FIXED_LENGTH_M = 2.4
# Platform tail length (``platform_fixed_joint`` xyz.x -> c_channel_reference).
PLATFORM_TAIL_M = 1.02
# Total telescopic stroke: four prismatic stages of 2.4 m.
EXTENSION_STROKE_M = 9.6
EXTENSION_STAGES = 4
# Default docking point offset below the trunk top (deployment decision).
DEFAULT_DOCKING_OFFSET_BELOW_TOP_M = 2.0
# Trunk-top tight-cylinder radius that rejects the outward-growing frond canopy.
DEFAULT_TRUNK_CYLINDER_RADIUS_M = 0.35
# Canopy annulus for the crown-base cross-check.
DEFAULT_CANOPY_INNER_R_M = 0.35
DEFAULT_CANOPY_OUTER_R_M = 4.0
# Mid-trunk band (relative to the local ground) used to estimate the axis.
DEFAULT_AXIS_BAND_M = (1.0, 8.0)


@dataclass(frozen=True)
class ScanEstimateConfig:
    """Tunable thresholds for :func:`estimate_tree_height`."""

    trunk_cylinder_radius_m: float = DEFAULT_TRUNK_CYLINDER_RADIUS_M
    canopy_inner_r_m: float = DEFAULT_CANOPY_INNER_R_M
    canopy_outer_r_m: float = DEFAULT_CANOPY_OUTER_R_M
    axis_band_m: tuple = DEFAULT_AXIS_BAND_M
    docking_offset_below_top_m: float = DEFAULT_DOCKING_OFFSET_BELOW_TOP_M
    boom_pivot_z_m: float = BOOM_PIVOT_WORLD_Z_M
    boom_fixed_length_m: float = BOOM_FIXED_LENGTH_M
    platform_tail_m: float = PLATFORM_TAIL_M
    extension_stroke_m: float = EXTENSION_STROKE_M
    # Minimum points for the estimate to be trusted at all.  Below this the
    # result is ``valid=False`` so the HUD shows NO DATA rather than a
    # confident-looking number computed from a handful of stray returns.
    min_trunk_points: int = 30
    min_cloud_points: int = 200
    # Docking range sensor -> trunk-centreline datum used for ``d_horiz`` when
    # the caller supplies no explicit horizontal distance.
    default_horizontal_distance_m: Optional[float] = None


@dataclass
class TreeEstimate:
    """One tree-height estimate with its uncertainty and provenance."""

    valid: bool
    reason: str = ''
    tree_height_m: Optional[float] = None
    trunk_top_m: Optional[float] = None
    crown_base_m: Optional[float] = None
    trunk_axis_xy: Optional[tuple] = None
    uncertainty_m: Optional[float] = None
    trunk_point_count: int = 0
    cloud_point_count: int = 0

    def to_rows(self) -> List[dict]:
        """Render as ``{key, label, value, valid}`` rows for the shared HUD."""
        def _m(value, fmt='{:.2f} m'):
            if value is None:
                return '—'
            try:
                return fmt.format(float(value))
            except (TypeError, ValueError, OverflowError):
                return '—'

        return [
            {'key': 'tree_height', 'label': 'Tree Height',
             'value': _m(self.tree_height_m), 'valid': self.valid},
            {'key': 'trunk_top', 'label': 'Trunk Top',
             'value': _m(self.trunk_top_m), 'valid': self.valid},
            {'key': 'crown_base', 'label': 'Crown Base',
             'value': _m(self.crown_base_m), 'valid': self.valid},
            {'key': 'uncertainty', 'label': 'Uncertainty',
             'value': _m(self.uncertainty_m), 'valid': self.valid},
        ]


@dataclass
class BoomTarget:
    """Closed-form IK targets for the docking manoeuvre (advisory only)."""

    feasible: bool
    status: str = 'NO DATA'
    reason: str = ''
    docking_height_m: Optional[float] = None
    boom_angle_rad: Optional[float] = None
    boom_angle_deg: Optional[float] = None
    boom_extension_total_m: Optional[float] = None
    platform_level_rad: Optional[float] = None
    platform_level_deg: Optional[float] = None
    boom_horizontal_distance_m: Optional[float] = None
    docking_lower_angle_rad: Optional[float] = None
    docking_lower_angle_deg: Optional[float] = None
    required_length_m: Optional[float] = None
    deficit_m: Optional[float] = None

    def to_rows(self) -> List[dict]:
        """Render as ``{key, label, value, valid}`` rows for the shared HUD."""
        def _num(value, fmt):
            if value is None:
                return '—'
            try:
                return fmt.format(float(value))
            except (TypeError, ValueError, OverflowError):
                return '—'

        valid = self.feasible
        return [
            {'key': 'docking_height', 'label': 'Docking Height',
             'value': _num(self.docking_height_m, '{:.2f} m'), 'valid': valid},
            {'key': 'boom_angle', 'label': 'Boom Angle',
             'value': _num(self.boom_angle_deg, '{:+.1f}°'), 'valid': valid},
            {'key': 'boom_extension', 'label': 'Boom Extension',
             'value': _num(self.boom_extension_total_m, '{:.3f} m'),
             'valid': valid},
            {'key': 'platform_level', 'label': 'Platform Level',
             'value': _num(self.platform_level_deg, '{:+.1f}°'), 'valid': valid},
            {'key': 'boom_distance', 'label': 'Boom Distance',
             'value': _num(self.boom_horizontal_distance_m, '{:.2f} m'),
             'valid': valid},
            {'key': 'docking_lower', 'label': 'Docking Lower Angle',
             'value': _num(self.docking_lower_angle_deg, '{:+.1f}°'),
             'valid': valid},
            {'key': 'status', 'label': 'Status',
             'value': self.status, 'valid': valid},
        ]


def _as_point_array(points) -> Optional[np.ndarray]:
    """Return an ``(N, 3)`` float64 array, or ``None`` when unusable."""
    if points is None:
        return None
    array = np.asarray(points, dtype=np.float64)
    if array.size == 0:
        return None
    if array.ndim != 2 or array.shape[1] < 3:
        return None
    return array[:, :3]


def estimate_trunk_axis(cloud: np.ndarray, cfg: ScanEstimateConfig):
    """Estimate the trunk XY axis from a mid-trunk band (encoder-free).

    Mirrors ``analyze_tree_scan_v2.py``: the axis is the median XY of the
    points between ``axis_band_m`` in height.  Returns ``(x, y, band_count)``
    or ``(None, None, 0)`` when the band holds too few points.
    """
    z = cloud[:, 2]
    low, high = cfg.axis_band_m
    band = cloud[(z >= low) & (z <= high)]
    if band.shape[0] < cfg.min_trunk_points:
        return None, None, int(band.shape[0])
    return float(np.median(band[:, 0])), float(np.median(band[:, 1])), int(band.shape[0])


def trunk_top(cloud: np.ndarray, axis_xy, cfg: ScanEstimateConfig):
    """Highest point within a tight cylinder about the trunk axis.

    Fronds attach at the crown base and grow *outward*, so a tight radius
    rejects the canopy bias of a 99th-percentile estimate and returns the
    actual trunk top.  Returns ``(z_top, z_p999, count)``.
    """
    if axis_xy[0] is None:
        return None, None, 0
    dx = cloud[:, 0] - axis_xy[0]
    dy = cloud[:, 1] - axis_xy[1]
    radial = np.hypot(dx, dy)
    inside = cloud[radial < cfg.trunk_cylinder_radius_m]
    if inside.shape[0] == 0:
        return None, None, 0
    z = inside[:, 2]
    return float(np.max(z)), float(np.percentile(z, 99.9)), int(z.size)


def crown_base(cloud: np.ndarray, axis_xy, cfg: ScanEstimateConfig):
    """Crown-base height from the canopy annulus density transition.

    Reuses the legacy density method: the first 0.1 m bin (walking upward)
    whose annulus point count exceeds the density threshold.  Returns the
    height (m) or ``None``.  This is a cross-check, not the primary estimate.
    """
    if axis_xy[0] is None or cloud.shape[0] == 0:
        return None
    dx = cloud[:, 0] - axis_xy[0]
    dy = cloud[:, 1] - axis_xy[1]
    radial = np.hypot(dx, dy)
    annulus = cloud[(radial >= cfg.canopy_inner_r_m)
                    & (radial <= cfg.canopy_outer_r_m)]
    if annulus.shape[0] < cfg.min_trunk_points:
        return None
    z = annulus[:, 2]
    low = float(np.min(z))
    high = float(np.max(z))
    if not (math.isfinite(low) and math.isfinite(high)) or high <= low:
        return None
    bins = np.arange(low, high + 0.1, 0.1)
    counts, edges = np.histogram(z, bins=bins)
    # A crown is a sustained density jump: require the bin to hold at least
    # 10% of the densest bin, so a single stray return cannot trip it.
    threshold = max(20, int(0.1 * counts.max()) if counts.size else 20)
    for index, count in enumerate(counts):
        if count >= threshold:
            return float(edges[index])
    return None


def estimate_tree_height(points, cfg: Optional[ScanEstimateConfig] = None,
                         ground_z: Optional[float] = None,
                         frame: str = 'sensor') -> TreeEstimate:
    """Estimate tree height (and its parts) from one level cloud.

    ``frame`` is ``'sensor'`` when ``z`` is relative to the LiDAR origin
    (requires ``ground_z`` to express an absolute height) or ``'world'`` when
    the cloud is already ground-registered (``z=0`` is the ground).  Height is
    always reported relative to the ground datum so the HUD number agrees with
    the physical tree.

    Returns a :class:`TreeEstimate`; ``valid=False`` (with a ``reason``) when
    the cloud is too small or has no usable trunk band, so the caller can show
    NO DATA instead of a confident-looking wrong number.
    """
    cfg = cfg or ScanEstimateConfig()
    cloud = _as_point_array(points)
    if cloud is None or cloud.shape[0] < cfg.min_cloud_points:
        count = 0 if cloud is None else int(cloud.shape[0])
        return TreeEstimate(valid=False, cloud_point_count=count,
                            reason='cloud too small ({} pts)'.format(count))

    # Work in a ground-referenced copy for the height comparisons.
    if frame == 'world':
        datum = 0.0
    elif ground_z is not None:
        datum = float(ground_z)
    else:
        # Sensor frame with no ground datum: assume the lowest point is ground.
        datum = float(np.min(cloud[:, 2]))
    cloud = cloud.copy()
    cloud[:, 2] = cloud[:, 2] - datum

    axis_x, axis_y, band_count = estimate_trunk_axis(cloud, cfg)
    if axis_x is None:
        return TreeEstimate(valid=False, cloud_point_count=int(cloud.shape[0]),
                            trunk_point_count=band_count,
                            reason='no trunk band points ({} pts)'.format(band_count))

    z_top, z_p999, trunk_count = trunk_top(cloud, (axis_x, axis_y), cfg)
    if z_top is None or trunk_count < cfg.min_trunk_points:
        return TreeEstimate(
            valid=False, cloud_point_count=int(cloud.shape[0]),
            trunk_point_count=trunk_count, trunk_axis_xy=(axis_x, axis_y),
            reason='no trunk cylinder points ({} pts)'.format(trunk_count))

    base = crown_base(cloud, (axis_x, axis_y), cfg)

    # Fuse the tight-cylinder estimates; the 99.9th percentile is a robust
    # variant of the same measurement and bounds the spread.  The crown-base
    # transition is a cross-check only (it was the noisier legacy method).
    candidates = [value for value in (z_top, z_p999) if value is not None]
    height = float(np.median(candidates))
    uncertainty = float(abs(z_top - z_p999)) if z_p999 is not None else None

    return TreeEstimate(
        valid=True,
        tree_height_m=height,
        trunk_top_m=z_top,
        crown_base_m=base,
        trunk_axis_xy=(axis_x, axis_y),
        uncertainty_m=uncertainty,
        trunk_point_count=trunk_count,
        cloud_point_count=int(cloud.shape[0]),
    )


def solve_boom_target(tree_height_m: Optional[float],
                      horizontal_distance_m: Optional[float],
                      cfg: Optional[ScanEstimateConfig] = None) -> BoomTarget:
    """Return the closed-form boom targets for a docking point.

    ``horizontal_distance_m`` is the distance from the turret/boom pivot to the
    trunk centreline (``d_horiz``).  ``docking_height_m`` is
    ``tree_height_m - cfg.docking_offset_below_top_m``.

    The IK is the as-built ``ros2_ws`` solution::

        dx = d_horiz - L_plat
        dz = H_dock - pivot_z
        L_needed = hypot(dx, dz)
        theta_b = atan2(dz, dx)
        e = L_needed - L_fixed          (clamped to [0, stroke])
        level = +theta_b                (NOT -theta_b: opposite joint axis)

    ``INFEASIBLE_DOCK_HEIGHT`` is reported (with the exact deficit) when the
    required length exceeds ``L_fixed + stroke``, so the operator knows to move
    the machine closer rather than over-extending.
    """
    cfg = cfg or ScanEstimateConfig()
    if tree_height_m is None:
        return BoomTarget(feasible=False, status='NO DATA',
                          reason='no tree height')
    if horizontal_distance_m is None:
        horizontal_distance_m = cfg.default_horizontal_distance_m
    if horizontal_distance_m is None:
        return BoomTarget(
            feasible=False, status='NO DATA',
            docking_height_m=tree_height_m - cfg.docking_offset_below_top_m,
            reason='no trunk horizontal distance (d_horiz)')

    h_dock = float(tree_height_m) - cfg.docking_offset_below_top_m
    d_horiz = float(horizontal_distance_m)

    # Docking height below the ground plane makes no sense: clamp to the pivot.
    if h_dock <= 0.0:
        return BoomTarget(
            feasible=False, status='INFEASIBLE_DOCK_HEIGHT',
            docking_height_m=h_dock,
            boom_horizontal_distance_m=d_horiz,
            reason='docking height at/below ground')

    dx = d_horiz - cfg.platform_tail_m
    dz = h_dock - cfg.boom_pivot_z_m
    if dx <= 0.0:
        # The platform tail already reaches past the trunk centreline.
        return BoomTarget(
            feasible=False, status='INFEASIBLE_DOCK_HEIGHT',
            docking_height_m=h_dock,
            boom_horizontal_distance_m=d_horiz,
            reason='trunk inside platform tail (d_horiz <= L_plat)')

    required = math.hypot(dx, dz)
    theta_b = math.atan2(dz, dx)
    total_stroke = cfg.boom_fixed_length_m + cfg.extension_stroke_m
    deficit = required - total_stroke

    target = BoomTarget(
        feasible=deficit <= 0.0,
        docking_height_m=h_dock,
        boom_angle_rad=theta_b,
        boom_angle_deg=math.degrees(theta_b),
        boom_horizontal_distance_m=d_horiz,
        required_length_m=required,
        deficit_m=max(0.0, deficit),
    )
    if deficit > 0.0:
        target.status = 'INFEASIBLE_DOCK_HEIGHT'
        target.reason = 'need {:.2f} m more reach'.format(deficit)
        # Still report the achievable extension clamped to the stroke, so the
        # operator sees how far it *can* go.
        target.boom_extension_total_m = cfg.extension_stroke_m
        target.platform_level_rad = theta_b
        target.platform_level_deg = math.degrees(theta_b)
        return target

    extension = max(0.0, required - cfg.boom_fixed_length_m)
    target.boom_extension_total_m = min(extension, cfg.extension_stroke_m)
    # As-built correction: the level joint axis is opposite the boom's, so the
    # command is +theta_b (the ``ros2_ws`` validation found -theta_b left the
    # platform tilted ~89 deg and the c-channel ~1.0 m off-target).
    target.platform_level_rad = theta_b
    target.platform_level_deg = math.degrees(theta_b)
    target.status = 'READY'
    return target


def split_extension(total_m: Optional[float],
                    stages: int = EXTENSION_STAGES) -> List[float]:
    """Split a total telescopic extension evenly across ``stages``."""
    if total_m is None or stages <= 0:
        return [0.0] * max(0, stages)
    per = max(0.0, float(total_m)) / float(stages)
    return [per] * stages


def plan_scan(points, horizontal_distance_m: Optional[float] = None,
              ground_z: Optional[float] = None, frame: str = 'sensor',
              cfg: Optional[ScanEstimateConfig] = None):
    """Estimate the tree and solve the boom targets in one call.

    Returns ``(TreeEstimate, BoomTarget)``.  When the tree estimate is not
    valid the boom target is ``NO DATA`` and the HUD shows the reason rather
    than an estimate.
    """
    cfg = cfg or ScanEstimateConfig()
    tree = estimate_tree_height(points, cfg=cfg, ground_z=ground_z, frame=frame)
    if not tree.valid:
        return tree, BoomTarget(feasible=False, status='NO DATA',
                                reason=tree.reason)
    target = solve_boom_target(tree.tree_height_m, horizontal_distance_m, cfg)
    return tree, target


__all__ = [
    'BOOM_PIVOT_WORLD_Z_M', 'BOOM_FIXED_LENGTH_M', 'PLATFORM_TAIL_M',
    'EXTENSION_STROKE_M', 'EXTENSION_STAGES',
    'DEFAULT_DOCKING_OFFSET_BELOW_TOP_M',
    'ScanEstimateConfig', 'TreeEstimate', 'BoomTarget',
    'estimate_trunk_axis', 'trunk_top', 'crown_base', 'estimate_tree_height',
    'solve_boom_target', 'split_extension', 'plan_scan',
]
