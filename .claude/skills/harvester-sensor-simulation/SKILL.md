---
name: harvester-sensor-simulation
description: "Build or modify the harvester's ZeroMQ sensor/PLC simulation publisher: state-machine maneuvers and geometric ray-cast sensor readings."
disable-model-invocation: true
argument-hint: "the sensor/maneuver to simulate, or the publisher script to modify"
metadata:
  author: harvester
  version: "2.0.0"
  status: stable
---

# Harvester Sensor Simulation

Write or modify the harvester's ZeroMQ **sensor/PLC simulation publisher** — a looping state
machine that models a boom/docking maneuver and synthesizes distance-sensor readings from
geometry (ray–circle intersection), not hard-coded numbers.

Reference implementation: `scripts/sensor_plc_publisher.py` (the Pi PLC docking-sequence
simulation). It publishes single-part JSON `[topic, payload]` on topic `harvester.sensors.v1`
over `tcp://*:5555` at 10 Hz.

## Network topology (fixed)

- **Orin** (`marcop-desktop`) = `192.168.50.10` on the sensor LAN (eth1).
- **Pi PLC** = `192.168.50.40`. Its battery-backed RTC is the persistent UTC/NTP authority
  (`time_sync.PLC_NTP_SERVER`); the Orin syncs to it via chrony.
- **OAK cameras**: docking `192.168.50.21`, cutting `192.168.50.22` (`camera_config.CAMERAS`).
- Live Pi script lives at `/home/marcop/plc-sensors/sensor_plc_publisher.py` (its git repo tracks
  `github.com/aiharvester2025/sim_sensors`).

## Payload envelope (keep it backward-compatible)

```json
{
  "schema": "harvester.sensor-telemetry.v2",
  "sequence": 0,
  "timestamp_unix_s": 1234.567,
  "simulation": { "phase": "BOOM_EXTEND", "boom_angle_deg": 45.0, "boom_extension_m": 5.0,
                  "platform_roll_deg": 0.0, "platform_pitch_deg": 0.0, "docked": false,
                  "target_tree_height_m": 12.0, "center_bark_distance_m": 2.5,
                  "forward_speed_mps": 0.0, "trunk_in_c_channel": false },
  "sensors": {
    "center_line": {"distance_m": 0.120, "valid": true},
    "diagonal_left_45deg": {"distance_m": 9.999, "valid": false},
    "diagonal_right_45deg": {"distance_m": 9.999, "valid": false},
    "c_channel_left": {"distance_m": 0.153, "valid": true},
    "c_channel_right": {"distance_m": 0.153, "valid": true}
  },
  "derived": { "entry_alignment_error_m": 0.0, "lateral_offset_estimate_m": 0.0,
               "equivalent_diameter_estimate_m": 0.0 }
}
```

Backward compatibility is a first-class concern: keep `schema`, `topic`, `sequence`,
`timestamp_unix_s`, `sensors.*`, and `derived.*` unchanged when extending; add new state under the
`simulation` block so existing consumers (`sensor_viewer.py`) keep working. The schema string is
`harvester.sensor-telemetry.v2` (NOT `.v1`); `sensor_viewer.parse_telemetry` rejects anything else.

## State machine pattern

- `DockingSequence.PHASES` = list of `(name, duration_s)` tuples, looped forever:
  `BOOM_RAISE (4.0) → BOOM_EXTEND (4.0) → PLATFORM_LEVEL (3.0) → BOOM_LOWER (3.0) →
  ENTRY_GATE_ALIGNMENT (3.0) → SIDE_CLEARANCE_VERIFY (3.0) → FINAL_DEPTH_STOP (2.0) → DOCKED (3.0)`.
- `DOCKING_PHASES` = `frozenset({'ENTRY_GATE_ALIGNMENT','SIDE_CLEARANCE_VERIFY','FINAL_DEPTH_STOP','DOCKED'})`.
- `tick()` computes `progress ∈ [0,1]` via `_phase_progress()` (elapsed/duration from
  `time.monotonic()`), derives per-phase state via `_boom_state()` + `_approach_distance()`, returns
  the payload dict, then `_advance_if_done()` wraps the phase when `progress >= 1.0`.
- Separate **state derivation** from the **publisher loop** (`main()`) so the sequence is
  unit-testable without a ZMQ socket.

## Geometric sensor synthesis (ray–circle intersection)

Distance sensors read a trunk = a circle of radius `TRUNK_RADIUS_M` in plan view. Use
`ray_circle_range(origin, direction, circle_center, radius)` → nearest forward intersection or
`None`. A reading is `valid: true` only when the ray intersects; otherwise `distance_m =
MAX_RANGE_M = 9.999` with `valid: false`.

Sensor geometry (plan view, metres forward/lateral):
- `center_sensor = (0.0, 0.0)`, beam `(1.0, 0.0)` (forward).
- `left_gate_sensor = (L1_GATE_X_M=0.650, -CHANNEL_HALF_WIDTH_M)`, `right_gate_sensor = (0.650, +0.400)`,
  beams at 45°: `(cos45°, ±sin45°)`.
- `left_side_sensor = (SIDE_SENSOR_X_M=0.250, -0.400)`, `right_side_sensor = (0.250, +0.400)`,
  beams lateral `(0, ±1)`.

Key realism nuance: the **45° "entry-gate" diagonal sensors go out-of-range at final dock** — their
forward-and-outward beams pass the trunk once the platform closes past ~0.2 m. This is physically
correct, not a bug. The sensors that matter for "safely docked" (center + the two lateral C-channel
sensors) converge to their targets: `center_line → 0.120`, `c_channel_left/right → 0.153`.

## Maneuver setpoints (degrees + metres, per operator)

```
BOOM_ANGLE_MAX_DEG = 45.0      BOOM_ANGLE_LOWER_DEG = 12.0
TREE_HEIGHT_M = 12.0           BOOM_BASE_HEIGHT_M = 2.0
PLATFORM_ROLL_MAX_DEG = 2.4    PLATFORM_PITCH_MAX_DEG = -1.8
START_DISTANCE_M = 2.50        INITIAL_OFFSET_M = 0.060
FINAL_CLEARANCE_M = 0.120      TRUNK_RADIUS_M = 0.300
CHANNEL_HALF_WIDTH_M = 0.400   MAX_RANGE_M = 9.999
```

Boom extension to reach a tree at angle θ uses trigonometry (`boom_extension_full_m()`):
`extension = (TREE_HEIGHT_M - BOOM_BASE_HEIGHT_M) / sin(θ)` = `10 / sin(45°) ≈ 14.14 m`. Do NOT
use an arbitrary clamp; the review in the source session flagged that exact bug.

## Deploying to the Pi (if applicable)

SSH may require a password — use `pexpect` from system Python rather than brute-forcing; **ask the
user for credentials** (do not iterate usernames/passwords). Back up the existing script (`.bak`)
before uploading, then deploy via `scp` through `pexpect`. Quoting via `{cmd!r}` (Python repr)
mangles nested quotes — prefer writing a temp script to the Pi or base64-encoding to avoid
shell-quoting issues.

To commit to `sim_sensors` from the Pi: stage **only** `sensor_plc_publisher.py` (leave `.bak` and
`__pycache__/` untracked), commit, push via the existing remote. Watch for a GitHub PAT embedded in
the remote URL (`https://ghp_...@github.com/...`) — do not exfiltrate/echo it further, and flag to
the user that it should be **rotated** (revoke old + issue new) and moved to SSH or a credential
helper.

## Verify

- Unit-test `tick()` with a **fake clock** (monkeypatch `time.monotonic`) — a fast no-sleep loop
  barely advances wall-clock, so real durations make phases appear "stuck" (a common confusion).
- Confirm the full cycle converges: `BOOM_RAISE (0→45°) → BOOM_EXTEND (→14.14 m) → PLATFORM_LEVEL
  (roll/pitch→0) → BOOM_LOWER (45°→12°) → ENTRY_GATE_ALIGNMENT → SIDE_CLEARANCE_VERIFY →
  FINAL_DEPTH_STOP → DOCKED`, with `center_line → 0.120` and `docked: true`.
- Compile-check on target: `python3 -m py_compile sensor_plc_publisher.py`; run with `python3 -u`
  to avoid stdout buffering when capturing via SSH.
- `zmq` must be present on the target (Pi has `zmq 26.4.0`).

## Guardrails

- Don't guess SSH credentials; ask. Don't brute-force usernames/passwords.
- Don't commit backups (`*.bak`) or `__pycache__/`; stage only the intended file.
- Preserve the payload envelope's backward compatibility unless explicitly told to break it.
- Avoid dead code: if a helper (e.g. an unused trig function) is replaced, remove it and make the
  replacement actually used, rather than leaving both (the session's review flagged this).
- Degrees for angles, metres for distances, per the operator's locked-in decision.
