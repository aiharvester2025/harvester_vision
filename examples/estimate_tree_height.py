#!/usr/bin/env python3
"""End-to-end example: estimate tree height and trunk-end from a MID-360 cloud.

This script ties together the two dependency-free pieces already in the repo:

  1. ``lidar/leveling.py``        -> level the cloud to gravity (rotation-only)
  2. ``lidar/boom_kinematics.py`` -> LiDAR height above ground from PLC values

It shows the complete pipeline a real deployment would run:

    raw MID-360 cloud (sensor frame)
      -> level with IMU orientation (tree stands straight)
      -> split trunk vs. canopy in the leveled frame
      -> convert to absolute height above ground using boom kinematics

Run it with no arguments to use a synthetic cloud, or point it at a JSON/CSV
of ``[[x, y, z], ...]`` points with ``--points``.

Example::

    python3 examples/estimate_tree_height.py
    python3 examples/estimate_tree_height.py --pivot-deg 45 --extension-m 1.5
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
from typing import List, Sequence, Tuple

from geometry.transforms import level_points_rotation_only, rotation_from_quaternion
from lidar.boom_kinematics import (
    BoomGeometry,
    BoomState,
    height_of_point_above_ground,
    lidar_height_above_ground,
)
from lidar.leveling import quaternion_from_euler


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--points", help="JSON file of [[x,y,z], ...] in the sensor frame")
    # Boom geometry (surveyed; replace with your calibrated values).
    parser.add_argument("--pivot-height-m", type=float, default=1.0)
    parser.add_argument("--boom-stage0-m", type=float, default=2.0)
    parser.add_argument("--platform-offset-m", type=float, default=0.4)
    parser.add_argument("--cutting-arm-lift-m", type=float, default=0.3)
    # Live PLC readings (computed from length sensors).
    parser.add_argument("--pivot-deg", type=float, default=30.0, help="boom pivot angle (deg)")
    parser.add_argument("--extension-m", type=float, default=1.0)
    parser.add_argument("--platform-pitch-deg", type=float, default=2.0)
    parser.add_argument("--platform-roll-deg", type=float, default=1.0)
    # Leveling source (IMU orientation; here synthesized from the tilt for demo).
    parser.add_argument("--trunk-radius-m", type=float, default=0.3,
                        help="max horizontal distance from the trunk axis to classify a point as trunk")
    args = parser.parse_args()
    if args.pivot_height_m < 0 or args.boom_stage0_m < 0:
        parser.error("boom geometry values must be non-negative")
    return args


def synthetic_tree_cloud() -> List[Tuple[float, float, float]]:
    """A vertical trunk plus a wider canopy, expressed in the LiDAR frame.

    The tree is ~1.5 m ahead and runs from 0.2 m below the LiDAR up to ~5 m
    above it. Trunk points cluster near the vertical axis; canopy points spread
    outward. This is only a demo cloud.
    """
    points: List[Tuple[float, float, float]] = []
    # Trunk: narrow column.
    for i in range(60):
        z = -0.2 + i * 0.08  # -0.2 .. ~4.5 m relative to LiDAR
        x = 1.5 + math.sin(i * 0.9) * 0.1
        y = math.cos(i * 0.7) * 0.1
        points.append((x, y, z))
    # Canopy: wide, higher up.
    for i in range(120):
        z = 3.5 + (i % 20) * 0.15
        radius = 0.8 + (i % 5) * 0.15
        angle = i * 0.5
        x = 1.5 + radius * math.cos(angle)
        y = radius * math.sin(angle)
        points.append((x, y, z))
    return points


def classify_trunk_vs_canopy(
    leveled: Sequence[Sequence[float]], trunk_radius_m: float
) -> Tuple[List[Tuple[float, float, float]], List[Tuple[float, float, float]]]:
    """Split leveled points into trunk (near the vertical axis) vs canopy.

    The trunk axis is the vertical line through the cloud's horizontal centroid
    of the *lowest* points; points within ``trunk_radius_m`` of that axis are
    trunk, everything else is canopy.
    """
    pts = [tuple(p) for p in leveled]
    if not pts:
        return [], []
    # Estimate the trunk axis as the centroid of the lowest 20% of points.
    sorted_by_z = sorted(pts, key=lambda p: p[2])
    low = sorted_by_z[: max(1, len(pts) // 5)]
    axis_x = sum(p[0] for p in low) / len(low)
    axis_y = sum(p[1] for p in low) / len(low)

    trunk: List[Tuple[float, float, float]] = []
    canopy: List[Tuple[float, float, float]] = []
    for p in pts:
        horizontal = math.hypot(p[0] - axis_x, p[1] - axis_y)
        (trunk if horizontal <= trunk_radius_m else canopy).append(p)
    return trunk, canopy


def main() -> None:
    args = parse_args()

    # 1. Raw cloud in the sensor frame.
    if args.points:
        raw = [tuple(p) for p in json.loads(Path(args.points).read_text())]
    else:
        raw = synthetic_tree_cloud()

    # 2. Level the cloud using the (synthesized) IMU/tilt orientation.
    #    In production this quaternion comes from the MID-360 IMU.
    roll = math.radians(args.platform_roll_deg)
    pitch = math.radians(args.platform_pitch_deg)
    orientation = quaternion_from_euler(roll, pitch, 0.0)
    rotation = rotation_from_quaternion(*orientation)
    leveled = level_points_rotation_only(raw, rotation)

    # 3. Compute the LiDAR's height above ground from PLC boom values.
    geometry = BoomGeometry(
        pivot_height_m=args.pivot_height_m,
        boom_stage0_length_m=args.boom_stage0_m,
        platform_level_offset_m=args.platform_offset_m,
        cutting_arm_lift_offset_m=args.cutting_arm_lift_m,
    )
    state = BoomState(
        pivot_angle_rad=math.radians(args.pivot_deg),
        extension_m=args.extension_m,
        platform_pitch_rad=pitch,
        platform_roll_rad=roll,
    )
    lidar_height = lidar_height_above_ground(state, geometry)

    # 4. Split trunk vs canopy and read heights.
    trunk, canopy = classify_trunk_vs_canopy(leveled, args.trunk_radius_m)
    trunk_end_rel = max((p[2] for p in trunk), default=float("nan"))
    tree_top_rel = max((p[2] for p in leveled), default=float("nan"))

    trunk_end_abs = height_of_point_above_ground(trunk_end_rel, lidar_height)
    tree_top_abs = height_of_point_above_ground(tree_top_rel, lidar_height)
    tree_height = tree_top_abs - height_of_point_above_ground(
        min((p[2] for p in trunk), default=0.0), lidar_height
    )

    print("=== Tree height estimate ===")
    print(f"points            : {len(raw)} raw, {len(leveled)} leveled")
    print(f"trunk points      : {len(trunk)}, canopy points: {len(canopy)}")
    print(f"LiDAR height      : {lidar_height:.2f} m above ground")
    print(f"trunk end (rel)   : {trunk_end_rel:.2f} m  -> absolute {trunk_end_abs:.2f} m")
    print(f"tree top (rel)    : {tree_top_rel:.2f} m  -> absolute {tree_top_abs:.2f} m")
    print(f"tree height       : {tree_height:.2f} m")
    print()
    print("The optimum docking height is derived downstream from trunk_end_abs")
    print("so the cutting arm's vertical reach covers [0, trunk_end_abs].")


if __name__ == "__main__":
    main()
