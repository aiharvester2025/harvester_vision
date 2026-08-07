#!/usr/bin/env python3
"""Validate DepthAI capture timestamp health from the two publisher streams."""

import argparse
import statistics
import sys
import time
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from camera_config import CAMERAS


def percentile(values, fraction):
    values = sorted(values)
    return values[min(len(values) - 1, round((len(values) - 1) * fraction))]


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=60.0)
    parser.add_argument("--max-drift-ms", type=float, default=10.0)
    parser.add_argument("--require-synchronized", action="store_true")
    args = parser.parse_args()
    if args.duration <= 0 or args.max_drift_ms <= 0:
        parser.error("--duration and --max-drift-ms must be positive")

    import msgpack
    import zmq

    context = zmq.Context()
    socket = context.socket(zmq.SUB)
    socket.setsockopt(zmq.SUBSCRIBE, b"")
    socket.connect(f"tcp://localhost:{CAMERAS['docking_camera'].pub_port}")
    socket.connect(f"tcp://localhost:{CAMERAS['cutting_camera'].pub_port}")
    poller = zmq.Poller()
    poller.register(socket, zmq.POLLIN)
    samples = {role: [] for role in CAMERAS}
    qualities = {role: set() for role in CAMERAS}
    previous_capture = {}
    bad_monotonic = []
    deadline = time.monotonic() + args.duration
    try:
        while time.monotonic() < deadline:
            if not poller.poll(500):
                continue
            payload = msgpack.unpackb(socket.recv(), raw=False)
            role = payload.get("camera_role")
            capture = payload.get("timestamp_monotonic_us")
            received = payload.get("received_timestamp_us")
            if role not in samples or not isinstance(capture, int) or not isinstance(received, int):
                continue
            qualities[role].add(payload.get("time_quality", "unknown"))
            if capture <= previous_capture.get(role, -1):
                bad_monotonic.append(role)
            previous_capture[role] = capture
            samples[role].append(received - capture)
    finally:
        socket.close()
        context.term()

    failures = []
    all_deviations = []
    for role, latencies in samples.items():
        if not latencies:
            failures.append(f"{role}: no frames received")
            continue
        baseline = statistics.median(latencies)
        deviations = [latency - baseline for latency in latencies]
        all_deviations.extend(deviations)
        print(f"{role}: frames={len(latencies)}, latency median={baseline / 1000:.3f} ms, "
              f"p95={percentile(latencies, .95) / 1000:.3f} ms, drift range="
              f"{(max(deviations) - min(deviations)) / 1000:.3f} ms, "
              f"time_quality={','.join(sorted(qualities[role]))}")
        if args.require_synchronized and qualities[role] != {"synchronized"}:
            failures.append(f"{role}: chrony is not continuously synchronized")
    drift_ms = (max(all_deviations) - min(all_deviations)) / 1000 if all_deviations else float("inf")
    print(f"inferred combined clock-offset drift: {drift_ms:.3f} ms")
    if bad_monotonic:
        failures.append("non-monotonic capture timestamps: " + ", ".join(sorted(set(bad_monotonic))))
    if drift_ms > args.max_drift_ms:
        failures.append(f"drift exceeds {args.max_drift_ms:.3f} ms")
    if failures:
        raise SystemExit("FAIL: " + "; ".join(failures))
    print("PASS: both cameras produced monotonic, host-aligned capture timestamps")


if __name__ == "__main__":
    main()
