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
from dataclasses import dataclass, field, replace
from typing import List, Optional, Sequence

import numpy as np

# ---------------------------------------------------------------------------
# Geometry constants (metres), from the URDF + as-built corrections.
# ---------------------------------------------------------------------------
# Boom pivot height above the GROUND plane, in the ``base_link`` frame plus the
# 0.05 m base park offset.  ``ros2_ws`` validation found the un-corrected value
# (1.76) produced a 0.05 m vertical docking error, so the corrected value ships.
BOOM_PIVOT_WORLD_Z_M = 0.05 + 1.76  # 1.81
# Boom pivot X offset in the base_link frame: turret origin x (-1.05) +
# elevation origin x (+0.18) = -0.87 (ros2_ws kinematics.boom_pivot_world_xy,
# turret yaw 0).
BOOM_PIVOT_BASE_X_OFFSET_M = -1.05 + 0.18  # -0.87
# Default harvester base world X.  In the Gazebo reference run the base parks at
# world x = 0 (ros2_ws test_estimators.py), so the boom pivot sits at x = -0.87
# and the reference trunk at x = 8.5 gives d_horiz = 9.37 m.
DEFAULT_BASE_X_M = 0.0
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
# Trunk radius used for the half-cylinder axis bias correction (ros2_ws
# ``fit_trunk_axis`` default; the reference trunk is ~0.25-0.35 m).
DEFAULT_AXIS_TRUNK_RADIUS_M = 0.35
# Drop fronds/FFB clutter from the axis band with |y| > this.
DEFAULT_AXIS_Y_MAX_M = 2.0
# Canopy annulus for the crown-base density scan.
DEFAULT_CANOPY_INNER_R_M = 0.35
DEFAULT_CANOPY_OUTER_R_M = 2.0
# Crown-base density histogram: 0.25 m bins (matches the tree geometry), a
# threshold matched to the live sweep density, and fixed lower/upper bounds so
# the histogram is deterministic.  z_min must sit above harvester self-clutter
# (~3 m) and below the true crown base (~9 m).
DEFAULT_CROWN_BIN_HEIGHT_M = 0.25
DEFAULT_CROWN_DENSITY_THRESHOLD = 50
# The crown bin must exceed the bare-trunk clutter by this multiple.  On the
# tree_scan_002 recording the trunk runs ~100-150 pts/bin and the crown base
# jumps to ~28k, so a 3x-clutter ratio plus the absolute floor cleanly
# separates them while still firing on a sparse sweep.
DEFAULT_CROWN_DENSITY_RATIO = 3.0
DEFAULT_CROWN_Z_MIN_M = 5.0
DEFAULT_CROWN_Z_MAX_M = 14.0
# Mid-trunk band (relative to the local ground) used to estimate the axis.
DEFAULT_AXIS_BAND_M = (1.0, 8.0)


@dataclass(frozen=True)
class ScanEstimateConfig:
    """Tunable thresholds for :func:`estimate_tree_height`."""

    trunk_cylinder_radius_m: float = DEFAULT_TRUNK_CYLINDER_RADIUS_M
    axis_trunk_radius_m: float = DEFAULT_AXIS_TRUNK_RADIUS_M
    axis_y_max_m: float = DEFAULT_AXIS_Y_MAX_M
    canopy_inner_r_m: float = DEFAULT_CANOPY_INNER_R_M
    canopy_outer_r_m: float = DEFAULT_CANOPY_OUTER_R_M
    crown_bin_height_m: float = DEFAULT_CROWN_BIN_HEIGHT_M
    crown_density_threshold: int = DEFAULT_CROWN_DENSITY_THRESHOLD
    # The effective threshold is max(crown_density_threshold, ratio * trunk
    # clutter median) so it adapts to the sweep density.  Set
    # crown_absolute_only to use the absolute threshold alone (the ros2_ws
    # behaviour, matched to a specific sweep density).
    crown_density_ratio: float = DEFAULT_CROWN_DENSITY_RATIO
    crown_absolute_only: bool = False
    crown_z_min_m: float = DEFAULT_CROWN_Z_MIN_M
    crown_z_max_m: float = DEFAULT_CROWN_Z_MAX_M
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
    # Low percentile used as the ground estimate when the caller gives no ground
    # datum (sensor frame).  Robust to stray points below the ground.
    ground_percentile: float = 1.0
    # Fraction of returns that must sit in a +-0.15 m band about the estimated
    # ground for the ground (and hence the absolute height) to be trusted.
    ground_band_fraction: float = 0.05
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
    # Crown-base (trunk-end) detection provenance.  ``trunk_end_valid`` is False
    # when the detector failed and ``crown_base_m`` is only the unsafe
    # top-minus-offset fallback, so callers must NOT dock on it.
    crown_base_method: str = ''
    trunk_end_valid: bool = False
    trunk_end_reason: str = ''
    # False when the height is relative to an ESTIMATED ground (sensor frame
    # with no ground_z), so the absolute tree height carries that error.
    ground_known: bool = True
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
            # The tree height is only as good as the ground reference: with no
            # datum (sensor frame, no ground_z) it is ground-relative, so mark
            # the row reduced-confidence rather than presenting it as absolute.
            {'key': 'tree_height', 'label': 'Tree Height', 'group': 'tree',
             'value': _m(self.tree_height_m),
             'valid': self.valid and self.ground_known},
            {'key': 'trunk_top', 'label': 'Trunk Top', 'group': 'tree',
             'value': _m(self.trunk_top_m),
             'valid': self.valid and self.ground_known},
            # A crown base from the unsafe fallback is flagged invalid so the
            # operator does not dock on it.
            {'key': 'crown_base', 'label': 'Crown Base', 'group': 'tree',
             'value': _m(self.crown_base_m),
             'valid': self.valid and self.trunk_end_valid},
            {'key': 'uncertainty', 'label': 'Uncertainty', 'group': 'tree',
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
            {'key': 'docking_height', 'label': 'Docking Height', 'group': 'boom',
             'value': _num(self.docking_height_m, '{:.2f} m'), 'valid': valid},
            {'key': 'boom_angle', 'label': 'Boom Angle', 'group': 'boom',
             'value': _num(self.boom_angle_deg, '{:+.1f}°'), 'valid': valid},
            {'key': 'boom_extension', 'label': 'Boom Extension', 'group': 'boom',
             'value': _num(self.boom_extension_total_m, '{:.3f} m'),
             'valid': valid},
            {'key': 'platform_level', 'label': 'Platform Level', 'group': 'boom',
             'value': _num(self.platform_level_deg, '{:+.1f}°'), 'valid': valid},
            {'key': 'boom_distance', 'label': 'Boom Distance', 'group': 'boom',
             'value': _num(self.boom_horizontal_distance_m, '{:.2f} m'),
             'valid': valid},
            {'key': 'docking_lower', 'label': 'Docking Lower Angle',
             'group': 'boom',
             'value': _num(self.docking_lower_angle_deg, '{:+.1f}°'),
             'valid': valid},
            {'key': 'status', 'label': 'Status', 'group': 'boom',
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
    low, high = cfg.axis_band_m
    dx = cloud[:, 0]
    dy = cloud[:, 1]
    z = cloud[:, 2]
    # Drop fronds/FFB clutter that extends sideways before taking the median,
    # exactly as the validated ros2_ws ``fit_trunk_axis`` does.
    band = (z >= low) & (z <= high) & (np.abs(dy) <= cfg.axis_y_max_m)
    count = int(band.sum())
    if count < cfg.min_trunk_points:
        return None, None, count
    med_x = float(np.median(dx[band]))
    med_y = float(np.median(dy[band]))
    # Half-cylinder bias: a single-sided (half-cylinder) scan's median X sits at
    # centre - r*(2/pi), so recover the true centreline by adding r*(2/pi).
    # Verified against tree_scan_002: median 8.283 + 0.637*0.35 = 8.506 vs the
    # true 8.5.  Without this the axis is biased ~0.22 m toward the sensor.
    corrected_x = med_x + (2.0 / math.pi) * cfg.axis_trunk_radius_m
    return corrected_x, med_y, count


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
    """Crown-base (trunk-end) height from the canopy-annulus density jump.

    The validated ``ros2_ws`` ``crown_base_from_density`` method (proven on
    tree_scan_001/002: 9.00-9.25 m vs the 9.2 m ground truth).  The canopy
    annulus ``r in [top_radius, canopy_outer_r]`` is nearly empty along the bare
    trunk and densely populated above the crown base, so the bottom of the first
    0.25 m histogram bin crossing the density threshold is the crown base.

    The scan starts at a **fixed** ``crown_z_min_m`` (default 5 m): a bound tied
    to the trunk top clips into the frond zone and biases the crown base low,
    while no lower bound lets harvester/ground self-clutter at low z be misread
    as the crown.  Returns ``(crown_base_m, method, reason)``.
    """
    if axis_xy[0] is None or cloud.shape[0] == 0:
        return None, 'none', 'no cloud'
    dx = cloud[:, 0] - axis_xy[0]
    dy = cloud[:, 1] - axis_xy[1]
    radial = np.hypot(dx, dy)
    annulus = cloud[(radial >= cfg.canopy_inner_r_m)
                    & (radial < cfg.canopy_outer_r_m)]
    if annulus.shape[0] < cfg.min_trunk_points:
        return None, 'none', 'no canopy-annulus points'
    z = annulus[:, 2]
    z = z[z >= cfg.crown_z_min_m]
    if z.size == 0:
        return None, 'none', 'no canopy-annulus points above crown_z_min'
    # Fixed upper bound well above the tree top, so the histogram is deterministic.
    bins = np.arange(cfg.crown_z_min_m,
                     cfg.crown_z_max_m + cfg.crown_bin_height_m,
                     cfg.crown_bin_height_m)
    counts, edges = np.histogram(z, bins=bins)
    if counts.size == 0:
        return None, 'none', 'empty histogram'
    # The density threshold must scale with the sweep density: the offline
    # 40-step recording puts ~28k points in the crown-base bin but the trunk
    # clutter is ~100-150/bin, while a sparse 5-step live sweep gives only
    # ~100-150 canopy points/bin (dock.yaml).  A fixed absolute threshold
    # therefore either fires on trunk clutter (dense) or never fires (sparse).
    # Instead, treat the bare-trunk clutter as the baseline and require the
    # crown bin to exceed it by a clear multiple: the median of the bins below
    # the crown is the clutter level (the trunk is most of the low histogram).
    clutter = float(np.median(counts[:max(1, counts.size // 2)]))
    threshold = max(float(cfg.crown_density_threshold), cfg.crown_density_ratio * clutter)
    if cfg.crown_absolute_only:
        threshold = float(cfg.crown_density_threshold)
    for index, count in enumerate(counts):
        if count >= threshold:
            return (float(edges[index]), 'density_drop',
                    '{} canopy-annulus points in z=[{:.2f},{:.2f}] >= threshold '
                    '{:.0f} (clutter median {:.0f})'.format(
                        count, edges[index], edges[index + 1],
                        threshold, clutter))
    return (None, 'none',
            'max canopy-annulus bin = {} < threshold {:.0f} (scan too sparse)'.format(
                int(counts.max()) if counts.size else 0, threshold))


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
        ground_relative = True
        ground_known = True
    elif ground_z is not None:
        datum = float(ground_z)
        ground_relative = True
        ground_known = True
    else:
        # Sensor frame with no explicit ground datum.  Estimate the ground from
        # a LOW PERCENTILE of z (robust to a stray return below ground) and mark
        # the height ground-relative.  Whether that estimate is trustworthy
        # depends on the cloud actually CONTAINING ground: a live MID-360 sees
        # the ground around the machine, while a tree-only cloud (or the Gazebo
        # recording's world frame) may not.  Detect a dense low band and only
        # then treat the ground as known.
        datum = float(np.percentile(cloud[:, 2], cfg.ground_percentile))
        ground_relative = False
        band = (cloud[:, 2] >= datum - 0.15) & (cloud[:, 2] <= datum + 0.15)
        # A real ground plane puts a good fraction of returns in the low band.
        ground_known = (band.sum() / max(1, cloud.shape[0])) >= cfg.ground_band_fraction
    cloud = cloud.copy()
    cloud[:, 2] = cloud[:, 2] - datum

    if not ground_relative:
        # Re-anchor the ground-relative bands to the local ground (z=0 here).
        z_span = float(cloud[:, 2].max())
        axis_low = 1.0
        axis_high = min(8.0, max(axis_low + 1.0, z_span - 4.0))
        crown_min = min(cfg.crown_z_min_m, max(axis_high + 0.5, z_span - 4.5))
        cfg = replace(cfg, axis_band_m=(axis_low, axis_high),
                      crown_z_min_m=crown_min)

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

    base, base_method, base_reason = crown_base(cloud, (axis_x, axis_y), cfg)

    # Height: the trunk-top ``max`` upper envelope (fronds/occlusion can only
    # hide the top, never exceed it).  The 99.9th percentile is a robust variant
    # used only to bound the spread.
    height = float(z_top)
    uncertainty = float(abs(z_top - z_p999)) if z_p999 is not None else None

    # The docking reference is the CROWN BASE (trunk-end), not tree_top minus an
    # offset: the offset lands inside the frond/FFB zone and is why the platform
    # crashed into them.  When the detector fails we fall back to top-2.0 but
    # mark the trunk-end INVALID so callers never dock on the fallback.
    if base is None:
        crown_base_m = max(0.0, height - cfg.docking_offset_below_top_m)
        trunk_end_valid = False
        trunk_end_reason = 'crown-base detector failed ({}); unsafe default ' \
            'top-2.0 used'.format(base_reason)
    else:
        crown_base_m = base
        trunk_end_valid = True
        trunk_end_reason = base_reason

    return TreeEstimate(
        valid=True,
        tree_height_m=height,
        trunk_top_m=z_top,
        crown_base_m=crown_base_m,
        crown_base_method=base_method,
        trunk_end_valid=trunk_end_valid,
        trunk_end_reason=trunk_end_reason,
        ground_known=ground_known,
        trunk_axis_xy=(axis_x, axis_y),
        uncertainty_m=uncertainty,
        trunk_point_count=trunk_count,
        cloud_point_count=int(cloud.shape[0]),
    )


def solve_boom_target(tree_height_m: Optional[float],
                      horizontal_distance_m: Optional[float],
                      cfg: Optional[ScanEstimateConfig] = None,
                      crown_base_m: Optional[float] = None,
                      trunk_end_valid: bool = True) -> BoomTarget:
    """Return the closed-form boom targets for a docking point.

    ``horizontal_distance_m`` is the distance from the turret/boom pivot to the
    trunk centreline (``d_horiz``).

    The docking height is derived from the **crown base** (the trunk-end where
    fronds/FFBs first appear), NOT from the tree top minus an offset: that lands
    inside the frond/FFB zone (fronds ~9.45 m, FFBs ~9.55 m for the reference
    tree) and is why the platform crashed into them.  So::

        H_dock = crown_base - cfg.docking_offset_below_top_m

    ``tree_height_m`` is still required (it bounds the geometry), but the docking
    height no longer uses it.  When ``crown_base_m`` is None or the trunk-end is
    invalid, the target is returned as ``NO DATA`` — never docked on the unsafe
    ``tree_top - offset`` fallback.

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
    if crown_base_m is None or not trunk_end_valid:
        # No confident trunk-end: refuse to produce a docking height, because
        # the fallback (tree_top - offset) is inside the frond/FFB zone.
        return BoomTarget(
            feasible=False, status='NO DATA',
            reason='no confident crown base (trunk-end) for docking height')
    if horizontal_distance_m is None:
        horizontal_distance_m = cfg.default_horizontal_distance_m
    if horizontal_distance_m is None:
        return BoomTarget(
            feasible=False, status='NO DATA',
            docking_height_m=float(crown_base_m) - cfg.docking_offset_below_top_m,
            reason='no trunk horizontal distance (d_horiz)')

    h_dock = float(crown_base_m) - cfg.docking_offset_below_top_m
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
        target.docking_lower_angle_rad = docking_lower_angle(
            theta_b, h_dock, target.boom_extension_total_m, cfg)
        target.docking_lower_angle_deg = math.degrees(
            target.docking_lower_angle_rad)
        return target

    extension = max(0.0, required - cfg.boom_fixed_length_m)
    target.boom_extension_total_m = min(extension, cfg.extension_stroke_m)
    # As-built correction: the level joint axis is opposite the boom's, so the
    # command is +theta_b (the ``ros2_ws`` validation found -theta_b left the
    # platform tilted ~89 deg and the c-channel ~1.0 m off-target).
    target.platform_level_rad = theta_b
    target.platform_level_deg = math.degrees(theta_b)
    # theta_d: extra boom-lower angle for the final descent onto the docking
    # point (ros2_ws kinematics.docking_lower_angle).  It closes any vertical
    # gap left after raising+leveling; it is 0 when already at H_dock, which is
    # the case for the solved configuration.
    target.docking_lower_angle_rad = docking_lower_angle(
        theta_b, h_dock, target.boom_extension_total_m, cfg)
    target.docking_lower_angle_deg = math.degrees(
        target.docking_lower_angle_rad)
    target.status = 'READY'
    return target


def docking_lower_angle(theta_b_rad: float, h_dock_m: float,
                        extension_total_m: float,
                        cfg: Optional[ScanEstimateConfig] = None) -> float:
    """Extra boom-lower angle (``theta_d``) to descend onto the docking point.

    Mirrors ``ros2_ws`` ``kinematics.docking_lower_angle``: after raising and
    leveling, the platform may still sit above ``H_dock``; ``theta_d`` is the
    angle the boom must lower to close that vertical gap while the C-opening
    stays facing the trunk.  Returns 0 when already at height (the solved
    configuration), so it is normally small.
    """
    cfg = cfg or ScanEstimateConfig()
    boom_length = cfg.boom_fixed_length_m + float(extension_total_m or 0.0)
    if boom_length <= 0.0:
        return 0.0
    current_z = cfg.boom_pivot_z_m + boom_length * math.sin(theta_b_rad)
    dz = current_z - float(h_dock_m)
    horizontal = boom_length * math.cos(theta_b_rad)
    if horizontal <= 0.0:
        return 0.0
    return max(0.0, math.atan2(-dz, horizontal))


def trunk_horizontal_distance(trunk_axis_xy,
                              base_x_m: float = DEFAULT_BASE_X_M) -> Optional[float]:
    """Return ``d_horiz`` from a scanned trunk axis (world frame).

    ``d_horiz`` is the horizontal distance from the **boom pivot** to the trunk
    centreline, NOT the trunk's distance from the world origin: the pivot is
    offset from the base by ``BOOM_PIVOT_BASE_X_OFFSET_M`` (-0.87 m), so for the
    reference trunk at x=8.5 with the base at x=0 the answer is 9.37 m
    (ros2_ws ``test_distance_estimate_reachable``).

    Returns ``None`` when the axis is unusable, so the caller reports NO DATA
    rather than a distance measured from the wrong datum.
    """
    if not trunk_axis_xy or trunk_axis_xy[0] is None:
        return None
    try:
        axis_x = float(trunk_axis_xy[0])
        axis_y = float(trunk_axis_xy[1])
        pivot_x = float(base_x_m) + BOOM_PIVOT_BASE_X_OFFSET_M
    except (TypeError, ValueError, OverflowError):
        return None
    distance = math.hypot(axis_x - pivot_x, axis_y)
    if not math.isfinite(distance) or distance <= 0.0:
        return None
    return distance


def split_extension(total_m: Optional[float],
                    stages: int = EXTENSION_STAGES) -> List[float]:
    """Split a total telescopic extension evenly across ``stages``."""
    if total_m is None or stages <= 0:
        return [0.0] * max(0, stages)
    per = max(0.0, float(total_m)) / float(stages)
    return [per] * stages


def plan_scan(points, horizontal_distance_m: Optional[float] = None,
              ground_z: Optional[float] = None, frame: str = 'sensor',
              cfg: Optional[ScanEstimateConfig] = None,
              base_x_m: float = DEFAULT_BASE_X_M,
              derive_distance_from_axis: bool = True):
    """Estimate the tree and solve the boom targets in one call.

    Returns ``(TreeEstimate, BoomTarget)``.  When the tree estimate is not
    valid the boom target is ``NO DATA`` and the HUD shows the reason rather
    than an estimate.

    When ``horizontal_distance_m`` is None and ``derive_distance_from_axis`` is
    true, ``d_horiz`` is derived from the **scanned trunk axis** — the whole
    point of a LiDAR scan is to locate the trunk, so a scan must not depend on a
    separate camera channel for the distance.  ``base_x_m`` is the harvester
    base world X (0 in the reference geometry).  Pass
    ``derive_distance_from_axis=False`` to require an explicit distance.
    """
    cfg = cfg or ScanEstimateConfig()
    tree = estimate_tree_height(points, cfg=cfg, ground_z=ground_z, frame=frame)
    if not tree.valid:
        return tree, BoomTarget(feasible=False, status='NO DATA',
                                reason=tree.reason)
    distance = horizontal_distance_m
    if distance is None and derive_distance_from_axis:
        distance = trunk_horizontal_distance(tree.trunk_axis_xy, base_x_m)
    # Pass the resolved distance explicitly; do not mutate the caller's config
    # (dataclasses.replace keeps the frozen dataclass contract rather than
    # poking __dict__).
    target = solve_boom_target(
        tree.tree_height_m, distance, cfg,
        crown_base_m=tree.crown_base_m, trunk_end_valid=tree.trunk_end_valid)
    return tree, target


__all__ = [
    'BOOM_PIVOT_WORLD_Z_M', 'BOOM_FIXED_LENGTH_M', 'PLATFORM_TAIL_M',
    'EXTENSION_STROKE_M', 'EXTENSION_STAGES',
    'DEFAULT_DOCKING_OFFSET_BELOW_TOP_M',
    'ScanEstimateConfig', 'TreeEstimate', 'BoomTarget',
    'estimate_trunk_axis', 'trunk_top', 'crown_base', 'estimate_tree_height',
    'solve_boom_target', 'split_extension', 'plan_scan',
]
