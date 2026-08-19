#!/usr/bin/env python3
"""Publish MID-360 LiDAR points (leveled) over ZeroMQ + MessagePack.

This is the Orin-side publisher for the Livox MID-360. It is intentionally
dependency-free except for the Livox SDK: no ROS, no numpy, no tf2. Points are
leveled to gravity using only orientation (rotation-only), so the LiDAR stays
at the HUD origin while static geometry (a tree) stands straight.

Data flow::

    MID-360 (UDP) -> Livox-SDK2 callback -> downsample -> level -> ZMQ PUB

The published MessagePack envelope mirrors ``oak_rgb_publisher.py`` so the same
viewer/adapter can consume both. It carries the point cloud as a flat list of
``(x, y, z)`` floats plus the leveling orientation and its source.

The Livox SDK integration point is isolated behind ``--sdk-mode`` so this file
runs unmodified in three modes:

  * ``synthetic`` (default): emits a synthetic vertical "tree" cloud so the
    leveling and ZMQ plumbing can be tested without hardware.
  * ``livox-sdk``: reads points + IMU from ``livox_mid360_source`` (a thin
    adapter you fill in once the SDK is wired; see ``lidar/livox_source.py``).
  * ``file``: replays an ``.lvx2`` / CSV of points for offline validation.

Timestamps follow the existing contract: capture time is expressed in the
PLC-RTC UTC domain (``timestamp_us``) and the local monotonic domain
(``timestamp_monotonic_us``), with ``time_quality`` from chrony.
"""

from __future__ import annotations

import argparse
import struct
import time
from typing import Optional

from time_sync import TIME_AUTHORITY, ChronyStatus


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--topic", default="v1/lidar/raw", help="ZMQ PUB topic")
    parser.add_argument("--pub-port", dest="pub_port", type=int, default=5560)
    parser.add_argument("--sdk-mode", choices=("synthetic", "livox-sdk", "file"), default="synthetic")
    parser.add_argument("--hz", type=float, default=10.0, help="target publish rate (downsampled)")
    parser.add_argument("--max-points", type=int, default=2000, help="max points per message")
    parser.add_argument("--frame-id", dest="frame_id", default="mid360_link")
    parser.add_argument("--level-source", choices=("imu", "tilt", "boom", "none"), default="imu")
    parser.add_argument("--disabled", action="store_true", help="start paused until enabled via control")
    args = parser.parse_args()
    if args.hz <= 0:
        parser.error("--hz must be positive")
    if args.max_points <= 0:
        parser.error("--max-points must be positive")
    return args


def downsample(points, maximum: int):
    """Uniformly downsample a list of (x, y, z) triples to at most ``maximum``."""
    if len(points) <= maximum:
        return points
    step = (len(points) - 1) / float(maximum - 1) if maximum > 1 else 0.0
    return [points[int(round(i * step))] for i in range(maximum)]


def pack_points_message(points, header: dict) -> bytes:
    """Pack a leveled point cloud into the canonical MessagePack envelope.

    Points are encoded as a flat little-endian float32 blob so large clouds stay
    compact; the header carries the count and stride so a decoder needs no
    schema beyond what is already in the OAK publisher contract.
    """
    flat = []
    for x, y, z in points:
        flat.extend((float(x), float(y), float(z)))
    blob = struct.pack("<%df" % len(flat), *flat)
    payload = dict(header)
    payload["point_blob"] = blob
    payload["point_count"] = len(points)
    payload["point_stride"] = 3
    import msgpack
    return msgpack.packb(payload, use_bin_type=True)


def run_synthetic(hz: float, max_points: int):
    """Produce a synthetic vertical tree at ~1 m for offline plumbing tests."""
    import math
    points = []
    for i in range(max_points):
        # A vertical trunk: x ~ 1 m, small y scatter, z from 0 to 5 m.
        x = 1.0 + (i % 7) * 0.01
        y = math.sin(i * 0.1) * 0.05
        z = (i / max_points) * 5.0
        points.append((x, y, z))
    return points


def main() -> None:
    import msgpack
    import zmq

    args = parse_args()

    context = zmq.Context()
    pub = context.socket(zmq.PUB)
    pub.bind(f"tcp://*:{args.pub_port}")
    ctl = context.socket(zmq.PULL)
    ctl.bind(f"tcp://*:{args.pub_port + 1}")

    chrony = ChronyStatus()
    is_enabled = not args.disabled
    sequence_number = 0
    period_s = 1.0 / args.hz

    print(f"[{args.topic}] MID-360 publisher on port {args.pub_port} (mode={args.sdk_mode})")
    print(f"[{args.topic}] leveling source: {args.level_source}, frame: {args.frame_id}")

    # Orientation is injected by the chosen source. In `synthetic` mode we use
    # an identity (already-gravity-aligned) orientation so the tree is upright.
    # In `livox-sdk` mode this is replaced by the IMU/leveling adapter below.
    orientation_quaternion = (0.0, 0.0, 0.0, 1.0)
    orientation_valid = True
    orientation_name = args.level_source

    if args.sdk_mode == "livox-sdk":
        try:
            from lidar.livox_source import LivoxMid360Source
            source = LivoxMid360Source(level_source=args.level_source)
        except ImportError as error:
            print(f"[{args.topic}] Livox SDK source not available: {error}")
            print(f"[{args.topic}] Install the SDK adapter or run with --sdk-mode synthetic")
            return

    while True:
        # 1. Non-blocking control.
        try:
            msg = ctl.recv_json(flags=zmq.NOBLOCK)
            if "enabled" in msg:
                is_enabled = msg["enabled"]
                print(f"[{args.topic}] State: {'ENABLED' if is_enabled else 'DISABLED'}")
        except zmq.Again:
            pass

        # 2. Acquire raw points + orientation.
        if args.sdk_mode == "synthetic":
            raw_points = run_synthetic(args.hz, args.max_points)
        elif args.sdk_mode == "livox-sdk":
            raw_points, orientation_quaternion, orientation_valid, orientation_name = (
                source.sample(max_points=args.max_points)
            )
        else:  # file
            raw_points = []  # TODO: replay .lvx2/CSV here.

        # 3. Level (rotation-only) if the orientation source is valid.
        if is_enabled and orientation_valid:
            from geometry.transforms import level_points_rotation_only, rotation_from_quaternion
            rotation = rotation_from_quaternion(*orientation_quaternion)
            points = level_points_rotation_only(raw_points, rotation)
        else:
            points = raw_points

        # 4. Publish.
        if is_enabled:
            received_utc_ns = time.time_ns()
            quality, chrony_offset_us = chrony.get()
            sequence_number += 1
            header = {
                "topic": args.topic,
                "frame_id": args.frame_id,
                "timestamp_us": received_utc_ns // 1_000,
                "received_timestamp_us": received_utc_ns // 1_000,
                "timestamp_source": "plc_rtc_ntp" if quality == "synchronized" else "monotonic",
                "time_authority": TIME_AUTHORITY,
                "time_quality": quality,
                "chrony_offset_us": chrony_offset_us,
                "sequence_number": sequence_number,
                "level_source": orientation_name,
                "level_valid": bool(orientation_valid),
                "device": "MID-360",
            }
            pub.send(pack_points_message(points, header))

        time.sleep(period_s)


if __name__ == "__main__":
    main()
