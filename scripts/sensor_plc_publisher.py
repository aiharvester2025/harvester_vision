#!/usr/bin/env python3
"""Raspberry Pi PLC docking-sequence simulation.

Represents the harvester as a PLC and publishes a repeating, full docking
maneuver over ZeroMQ (topic ``harvester.sensors.v1``) on the sensor network:

    BOOM_RAISE (0 -> 45 deg)
      -> BOOM_EXTEND (extension out to reach a 12 m tree)
      -> PLATFORM_LEVEL (roll/pitch -> 0 deg)
      -> BOOM_LOWER (lower the boom toward the trunk)
      -> ENTRY_GATE_ALIGNMENT
      -> SIDE_CLEARANCE_VERIFY
      -> FINAL_DEPTH_STOP
      -> DOCKED (brief hold)
      -> loop back to BOOM_RAISE

The five distance sensors keep the drawing-derived trunk geometry (800 mm
C-channel, 600 mm average trunk) but their readings are now driven by the
sequence: they are out-of-range (``valid: false``) during the boom/leveling
phases and converge on the trunk as the platform approaches, reaching the
final bark clearance at dock.

The payload envelope (schema, topic, ``sequence``, ``timestamp_unix_s``,
``sensors.*``, ``derived.*``) is unchanged from the previous version so the
existing ``sensor_viewer.py`` keeps working; the ``simulation`` block is
extended with boom/extension/leveling/dock state for the canonical gateway
adapter and the operator HUD.
"""

import argparse
import json
import math
import time
import zmq

TOPIC = b"harvester.sensors.v1"
SCHEMA = "harvester.sensor-telemetry.v2"
MAX_RANGE_M = 9.999

# Drawing-derived geometry: 800 mm C-channel, 600 mm average trunk.
CHANNEL_HALF_WIDTH_M = 0.400
TRUNK_RADIUS_M = 0.300

# Measured forward from S_C. Adjust these after real sensor mounting is known.
L1_GATE_X_M = 0.650       # S_L1/S_R1 entry-gate plane
SIDE_SENSOR_X_M = 0.250   # S_L2/S_R2 lateral measurement plane
FINAL_CLEARANCE_M = 0.120  # d_scb: final S_C bark stop distance

# Maneuver setpoints (units: degrees and metres, per operator).
BOOM_ANGLE_MAX_DEG = 45.0      # boom pivot angle at full raise
BOOM_ANGLE_LOWER_DEG = 12.0    # boom pivot angle after lowering toward trunk
TREE_HEIGHT_M = 12.0           # target tree height
BOOM_BASE_HEIGHT_M = 2.0       # pivot height above ground (nominal)
PLATFORM_ROLL_MAX_DEG = 2.4    # initial roll to level out
PLATFORM_PITCH_MAX_DEG = -1.8  # initial pitch to level out
START_DISTANCE_M = 2.50        # S_C bark distance at the start of approach
INITIAL_OFFSET_M = 0.060       # initial lateral bark offset


def boom_extension_full_m():
    """Telescopic reach that places the platform at the 12 m tree at full raise.

    Platform height ~ pivot height + extension * sin(angle).
    """
    angle = math.radians(BOOM_ANGLE_MAX_DEG)
    need = TREE_HEIGHT_M - BOOM_BASE_HEIGHT_M
    if math.sin(angle) <= 1e-6:
        return need
    return need / math.sin(angle)


def clamp01(value):
    return max(0.0, min(1.0, value))


def ramp(progress):
    """Smooth 0->1 easing for a phase progress in [0, 1]."""
    return clamp01(progress)


def ray_circle_range(origin, direction, circle_center, radius):
    """Nearest forward ray intersection with the trunk circular cross-section."""
    ox, oy = origin
    dx, dy = direction
    cx, cy = circle_center

    qx, qy = ox - cx, oy - cy
    b = qx * dx + qy * dy
    c = qx * qx + qy * qy - radius * radius
    discriminant = b * b - c

    if discriminant < 0:
        return None

    near = -b - math.sqrt(discriminant)
    far = -b + math.sqrt(discriminant)

    if near >= 0:
        return near
    if far >= 0:
        return far
    return None


def sensor_reading(value):
    return {
        "distance_m": round(value, 3) if value is not None else MAX_RANGE_M,
        "valid": value is not None,
    }


class DockingSequence:
    """Advances the maneuver state once per tick and returns a payload dict."""

    # (phase_name, duration_s) in order; looped forever.
    PHASES = [
        ("BOOM_RAISE", 4.0),
        ("BOOM_EXTEND", 4.0),
        ("PLATFORM_LEVEL", 3.0),
        ("BOOM_LOWER", 3.0),
        ("ENTRY_GATE_ALIGNMENT", 3.0),
        ("SIDE_CLEARANCE_VERIFY", 3.0),
        ("FINAL_DEPTH_STOP", 2.0),
        ("DOCKED", 3.0),
    ]

    DOCKING_PHASES = frozenset({
        "ENTRY_GATE_ALIGNMENT", "SIDE_CLEARANCE_VERIFY",
        "FINAL_DEPTH_STOP", "DOCKED",
    })

    def __init__(self, start_distance_m=START_DISTANCE_M,
                 initial_offset_m=INITIAL_OFFSET_M,
                 final_clearance_m=FINAL_CLEARANCE_M):
        self.start_distance_m = start_distance_m
        self.initial_offset_m = initial_offset_m
        self.final_clearance_m = final_clearance_m
        self._phase_index = 0
        self._phase_start = time.monotonic()

        # Sensor coordinates and beam directions in plan view.
        self.center_sensor = (0.0, 0.0)
        self.left_gate_sensor = (L1_GATE_X_M, -CHANNEL_HALF_WIDTH_M)
        self.right_gate_sensor = (L1_GATE_X_M, CHANNEL_HALF_WIDTH_M)
        self.left_side_sensor = (SIDE_SENSOR_X_M, -CHANNEL_HALF_WIDTH_M)
        self.right_side_sensor = (SIDE_SENSOR_X_M, CHANNEL_HALF_WIDTH_M)
        self.left_45deg = (math.cos(math.radians(45)), math.sin(math.radians(45)))
        self.right_45deg = (math.cos(math.radians(45)), -math.sin(math.radians(45)))

    def _phase_progress(self):
        """Progress in [0, 1] through the current phase."""
        name, duration = self.PHASES[self._phase_index]
        elapsed = time.monotonic() - self._phase_start
        return clamp01(elapsed / duration), name

    def _advance_if_done(self, progress, name):
        if progress >= 1.0:
            self._phase_index = (self._phase_index + 1) % len(self.PHASES)
            self._phase_start = time.monotonic()
            return True
        return False

    def _boom_state(self, progress, name):
        """Return (angle_deg, extension_m, roll_deg, pitch_deg) for this tick."""
        # Values hold across phases unless explicitly animated below.
        angle = 0.0
        extension = 0.0
        roll = 0.0
        pitch = 0.0
        full = boom_extension_full_m()

        if name == "BOOM_RAISE":
            angle = BOOM_ANGLE_MAX_DEG * ramp(progress)
            extension = 0.0
            roll = 0.0
            pitch = 0.0
        elif name == "BOOM_EXTEND":
            angle = BOOM_ANGLE_MAX_DEG
            extension = full * ramp(progress)
        elif name == "PLATFORM_LEVEL":
            angle = BOOM_ANGLE_MAX_DEG
            extension = full
            roll = PLATFORM_ROLL_MAX_DEG * (1.0 - ramp(progress))
            pitch = PLATFORM_PITCH_MAX_DEG * (1.0 - ramp(progress))
        elif name == "BOOM_LOWER":
            angle = BOOM_ANGLE_MAX_DEG - (
                BOOM_ANGLE_MAX_DEG - BOOM_ANGLE_LOWER_DEG) * ramp(progress)
            extension = full
        else:
            # Docking phases: boom lowered and holding; platform leveled.
            angle = BOOM_ANGLE_LOWER_DEG
            extension = full

        return angle, extension, roll, pitch

    def _approach_distance(self, progress, name):
        """S_C bark distance through the docking phases (None outside)."""
        if name == "ENTRY_GATE_ALIGNMENT":
            # Still far out; begin closing from the start distance.
            return self.start_distance_m
        if name == "SIDE_CLEARANCE_VERIFY":
            span = self.start_distance_m - self.final_clearance_m
            return self.start_distance_m - 0.55 * span * ramp(progress)
        if name == "FINAL_DEPTH_STOP":
            span = self.start_distance_m - self.final_clearance_m
            base = self.start_distance_m - 0.55 * span
            return max(self.final_clearance_m, base - (base - self.final_clearance_m) * ramp(progress))
        if name == "DOCKED":
            return self.final_clearance_m
        return None

    def _lateral_offset(self, center_bark_distance):
        """Lateral offset shrinks as the platform closes on the trunk."""
        span = self.start_distance_m - self.final_clearance_m
        progress = 1.0 - (center_bark_distance - self.final_clearance_m) / span
        progress = clamp01(progress)
        return self.initial_offset_m * (1.0 - progress) ** 2

    def tick(self):
        progress, name = self._phase_progress()

        angle, extension, roll, pitch = self._boom_state(progress, name)
        center_bark_distance = self._approach_distance(progress, name)

        # Trunk centre is one trunk radius beyond the bark seen by S_C.
        if center_bark_distance is not None:
            lateral_offset = self._lateral_offset(center_bark_distance)
            trunk_center = (
                center_bark_distance + TRUNK_RADIUS_M,
                lateral_offset,
            )
        else:
            trunk_center = None

        if trunk_center is not None:
            center = ray_circle_range(self.center_sensor, (1.0, 0.0),
                                      trunk_center, TRUNK_RADIUS_M)
            l1 = ray_circle_range(self.left_gate_sensor, self.left_45deg,
                                  trunk_center, TRUNK_RADIUS_M)
            r1 = ray_circle_range(self.right_gate_sensor, self.right_45deg,
                                  trunk_center, TRUNK_RADIUS_M)
            l2 = ray_circle_range(self.left_side_sensor, (0.0, 1.0),
                                  trunk_center, TRUNK_RADIUS_M)
            r2 = ray_circle_range(self.right_side_sensor, (0.0, -1.0),
                                  trunk_center, TRUNK_RADIUS_M)
        else:
            center = l1 = r1 = l2 = r2 = None

        alignment_error = (l1 - r1) if l1 is not None and r1 is not None else None
        lateral_estimate = (l2 - r2) / 2 if l2 is not None and r2 is not None else None
        diameter_estimate = (
            2 * CHANNEL_HALF_WIDTH_M - (l2 + r2)
            if l2 is not None and r2 is not None else None
        )

        # Recompute a display phase from the ranges only during docking, else
        # use the state-machine phase name.
        if name in self.DOCKING_PHASES:
            if center_bark_distance <= self.final_clearance_m:
                phase = "DOCKED" if name == "DOCKED" else "FINAL_DEPTH_STOP"
            elif l2 is not None and r2 is not None:
                phase = "SIDE_CLEARANCE_VERIFY"
            elif l1 is not None and r1 is not None:
                phase = "ENTRY_GATE_ALIGNMENT"
            else:
                phase = "APPROACHING"
        else:
            phase = name

        payload = {
            "schema": SCHEMA,
            "sequence": 0,  # filled by caller
            "timestamp_unix_s": round(time.time(), 3),
            "simulation": {
                "phase": phase,
                "boom_angle_deg": round(angle, 2),
                "boom_extension_m": round(extension, 3),
                "platform_roll_deg": round(roll, 3),
                "platform_pitch_deg": round(pitch, 3),
                "docked": name == "DOCKED" and center_bark_distance <= self.final_clearance_m,
                "target_tree_height_m": TREE_HEIGHT_M,
                "center_bark_distance_m": (
                    round(center_bark_distance, 3)
                    if center_bark_distance is not None else None
                ),
                "forward_speed_mps": 0.035 if name in self.DOCKING_PHASES else 0.0,
                "trunk_in_c_channel": l2 is not None and r2 is not None,
            },
            "sensors": {
                "center_line": sensor_reading(center),
                "diagonal_left_45deg": sensor_reading(l1),
                "diagonal_right_45deg": sensor_reading(r1),
                "c_channel_left": sensor_reading(l2),
                "c_channel_right": sensor_reading(r2),
            },
            "derived": {
                "entry_alignment_error_m": round(alignment_error, 3)
                    if alignment_error is not None else None,
                "lateral_offset_estimate_m": round(lateral_estimate, 3)
                    if lateral_estimate is not None else None,
                "equivalent_diameter_estimate_m": round(diameter_estimate, 3)
                    if diameter_estimate is not None else None,
            },
        }

        self._advance_if_done(progress, name)
        return payload


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--bind-address", default="192.168.50.40")
    parser.add_argument("--port", type=int, default=5555)
    parser.add_argument("--rate-hz", type=float, default=10.0)
    parser.add_argument("--start-distance-m", type=float, default=START_DISTANCE_M)
    parser.add_argument("--initial-offset-m", type=float, default=INITIAL_OFFSET_M)
    args = parser.parse_args()

    context = zmq.Context()
    pub = context.socket(zmq.PUB)
    pub.bind("tcp://{}:{}".format(args.bind_address, args.port))

    print("Publishing at tcp://{}:{}".format(args.bind_address, args.port))
    print("Docking-sequence simulation (BOOM_RAISE -> ... -> DOCKED, loop). Ctrl+C to stop.")

    sequence = DockingSequence(
        start_distance_m=args.start_distance_m,
        initial_offset_m=args.initial_offset_m,
    )
    seq_number = 0

    try:
        while True:
            payload = sequence.tick()
            payload["sequence"] = seq_number

            pub.send_multipart([TOPIC, json.dumps(payload).encode()])
            sim = payload["simulation"]
            sen = payload["sensors"]
            print(
                "{:24} ang={:5.2f}deg ext={:6.3f}m roll={:+5.2f} pitch={:+5.2f} "
                "S_C={} L1={} R1={} L2={} R2={}".format(
                    sim["phase"], sim["boom_angle_deg"], sim["boom_extension_m"],
                    sim["platform_roll_deg"], sim["platform_pitch_deg"],
                    sen["center_line"]["distance_m"],
                    sen["diagonal_left_45deg"]["distance_m"],
                    sen["diagonal_right_45deg"]["distance_m"],
                    sen["c_channel_left"]["distance_m"],
                    sen["c_channel_right"]["distance_m"],
                )
            )

            seq_number += 1
            time.sleep(1.0 / args.rate_hz)

    except KeyboardInterrupt:
        pass
    finally:
        pub.close()
        context.term()


if __name__ == "__main__":
    main()
