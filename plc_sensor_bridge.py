#!/usr/bin/env python3
"""Bridge PLC/Modbus sensors (boom angle, tilt, ranges) to ZeroMQ topics.

The PLC exposes the harvester's slow sensors over Modbus. This process polls
them and republishes as MessagePack over ZMQ so the LiDAR leveling (via
``--level-source tilt`` / ``--level-source boom``) and the range-sensor viewer
can consume a uniform, timestamped stream without speaking Modbus themselves.

This is a SKELETON: the Modbus register map (addresses, scaling, units, and
validity flags) must be filled from the PLC program. Until then it publishes a
clearly-marked ``provisional`` payload so downstream code can be developed
against the contract without waiting for the physical PLC.

Topics (configurable):
  * ``harvester.sensors.orientation`` -> boom angle + platform tilt (roll/pitch)
  * ``harvester.sensors.ranges``      -> the five docking range readings
"""

from __future__ import annotations

import argparse
import time
from typing import Optional

from time_sync import TIME_AUTHORITY, ChronyStatus


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="192.168.50.40", help="PLC/Modbus host")
    parser.add_argument("--port", type=int, default=502, help="Modbus TCP port")
    parser.add_argument("--pub-port", dest="pub_port", type=int, default=5562)
    parser.add_argument("--hz", type=float, default=10.0)
    parser.add_argument("--orientation-topic", default="harvester.sensors.orientation")
    parser.add_argument("--ranges-topic", default="harvester.sensors.ranges")
    args = parser.parse_args()
    if args.hz <= 0:
        parser.error("--hz must be positive")
    return args


def read_orientation() -> dict:
    """Read boom pivot angle, boom extension, and 2-axis platform tilt.

    TODO: replace with real Modbus reads. The PLC computes the boom pivot angle
    and boom extension from length sensors and the 2-axis tilt sensor provides
    platform roll/pitch. Yaw is not needed for gravity leveling.
    """
    return {
        "boom_pivot_angle_rad": None,
        "boom_extension_m": None,
        "platform_roll_rad": None,
        "platform_pitch_rad": None,
        "valid": False,
        "status": "provisional",
    }


def read_ranges() -> list:
    """Read the five docking range sensors from Modbus.

    TODO: replace with real Modbus reads. See the ``sensor_telemetry_bindings``
    keys in ``calibration/frames.*.json`` for the exact telemetry keys.
    """
    return [
        {"telemetry_key": "diagonal_left_45deg", "distance_m": None, "valid": False},
        {"telemetry_key": "diagonal_right_45deg", "distance_m": None, "valid": False},
        {"telemetry_key": "center_line", "distance_m": None, "valid": False},
        {"telemetry_key": "c_channel_left", "distance_m": None, "valid": False},
        {"telemetry_key": "c_channel_right", "distance_m": None, "valid": False},
    ]


def main() -> None:
    import msgpack
    import zmq

    args = parse_args()
    context = zmq.Context()
    pub = context.socket(zmq.PUB)
    pub.bind(f"tcp://*:{args.pub_port}")

    chrony = ChronyStatus()
    period_s = 1.0 / args.hz
    sequence_number = 0
    print(f"[plc_sensor_bridge] publishing to port {args.pub_port} @ {args.hz:g} Hz")

    while True:
        received_utc_ns = time.time_ns()
        quality, chrony_offset_us = chrony.get()
        sequence_number += 1
        base = {
            "timestamp_us": received_utc_ns // 1_000,
            "time_authority": TIME_AUTHORITY,
            "time_quality": quality,
            "chrony_offset_us": chrony_offset_us,
            "sequence_number": sequence_number,
        }

        orientation = read_orientation()
        orientation.update(base)
        orientation["topic"] = args.orientation_topic
        pub.send(msgpack.packb(orientation, use_bin_type=True))

        ranges = {"topic": args.ranges_topic, "sensors": read_ranges()}
        ranges.update(base)
        pub.send(msgpack.packb(ranges, use_bin_type=True))

        time.sleep(period_s)


if __name__ == "__main__":
    main()
