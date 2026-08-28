# PLC Docking Sequence Simulation → Canonical Gateway → HUD

**Goal.** Replace the Raspberry Pi's current "constant-speed forward crawl"
sensor simulation with a full, repeating **docking maneuver** that walks through
the real machine sequence — boom raise, boom extend to tree height, platform
leveling, boom lower toward the trunk, then the five distance sensors converging
on the trunk until a safe dock — and then publish that state (a) into the
canonical ZeroMQ telemetry gateway on the Orin and (b) onto the operator HUD as
a step-by-step guide for the human operator.

## 1. Current state (verified by reading the code)

**Hosts and networks**
- **Pi PLC** at `192.168.50.40` on the sensor LAN (`eth0`), reachable from the
  Orin via `192.168.50.10` (`eth1`). It runs
  `~/plc-sensors/sensor_plc_publisher.py`.
- **Orin** `marcop-desktop` at `192.168.50.10` (`eth1`, sensor LAN) and
  `10.108.137.58` (`eth0`, wifi/LAN). This repo (`harvester_vision`) lives here.
- The current Pi script (`/tmp/sensor_plc_publisher.py` is a stale copy; the
  live one is on the Pi at `~/plc-sensors/`) publishes a **single-part** ZMQ
  stream `tcp://192.168.50.40:5555`, topic `harvester.sensors.v1`, schema
  `harvester.sensor-telemetry.v2`. It has a `ray_circle_range` trunk model and
  five sensors (`center_line`, `diagonal_left_45deg`, `diagonal_right_45deg`,
  `c_channel_left`, `c_channel_right`) plus derived alignment/offset/diameter —
  but it only ever lowers `center_line` at a fixed 35 mm/s; it does **not** model
  the boom/leveling sequence at all.

**Canonical telemetry gateway (already built, already running)**
- `canonical_zmq/harvester_telemetry_contract/protocol.py` — frozen v1 contract.
  Channels already defined include `v1/range/docking`, `v1/range/cutter`,
  `v1/docking/trunk_estimate`, `v1/system/status`, `v1/calibration/status`.
- `canonical_zmq/canonical_zmq_publisher/aggregator.py` — the Orin canonical
  PUB (`tcp://*:5590`) + REP (`tcp://*:5600`), with an **ingest PULL**
  (`tcp://*:5570`) where adapters PUSH canonical three-frame packets.
- `canonical_zmq/canonical_zmq_publisher/oak_capture.py` — the reference adapter
  pattern: build a canonical header, `pack_message(...)`, `push_socket.send_multipart`,
  PUSH into `tcp://127.0.0.1:5570`. The aggregator re-owns sequence/source_id.
- `canonical_zmq/canonical_zmq_publisher/ingest.py` — the synthetic source shows
  the **exact** `v1/range/docking` payload shape the dashboard already consumes:
  a JSON list of records `{telemetry_key, distance_m, valid, frame_id,
  acquisition_timestamp_ns, calibration_id, min_range_m, max_range_m}`.
- **Gap:** there is no `range_ingest` adapter yet (README marks it "deferred").
  Nothing currently connects the Pi's `harvester.sensors.v1` stream into `5590`.
  This is the missing link this task adds.

**Dashboard / HUD (already built)**
- `harvester_dashboard/` Qt Quick app; system `/usr/bin/python3`.
- `HudOverlay.qml` (toggle with key `3`) already renders: a `SensorPanel.qml`
  with the five docking-range rows (`bridge.dockingRangeRows`) + cutter line; a
  trunk/calibration/capabilities block; and a stream-errors panel.
- `model/telemetry_model.py` decodes `v1/range/docking` (JSON list) via
  `snapshot_ranges()` and `v1/docking/trunk_estimate` via `snapshot_trunk()`.
- `bridge.py` exposes `dockingRangeRows`, `cutterRangeLine`, `trunkLine`, etc.
- **Gap:** there is no HUD element that shows the *boom angle / extension /
  leveling / phase* state, and no operator "guide" text. `SensorPanel` is
  range-only. This task adds a phase/guide HUD.

**Telemetry key contract** (`calibration/frames.*.json`) — the five range keys are
`diagonal_left_45deg`, `diagonal_right_45deg`, `center_line`, `c_channel_left`,
`c_channel_right`, mapped to frames `sensor_diagonal_left_frame` etc.

## 2. What we will build

### Part A — Pi: full docking-sequence simulation (rewrite `sensor_plc_publisher.py`)

Replace the fixed-speed crawl with a state machine that loops the maneuver. The
script keeps publishing on `tcp://192.168.50.40:5555` topic `harvester.sensors.v1`
(same envelope as today, so the existing `sensor_viewer.py` still works) but adds
the boom/leveling/phases. New/changed fields in the JSON payload:

- `simulation.phase` — one of:
  `BOOM_RAISE` → `BOOM_EXTEND` → `PLATFORM_LEVEL` → `BOOM_LOWER` →
  `ENTRY_GATE_ALIGNMENT` → `SIDE_CLEARANCE_VERIFY` → `FINAL_DEPTH_STOP` →
  `DOCKED` → (brief hold) → loop back to `BOOM_RAISE`.
- `simulation.boom_angle_deg` — 0 → 45 degrees (see §3 note 1).
- `simulation.boom_extension_m` — grows to the reach for a 12 m tree.
- `simulation.platform_roll_deg` / `platform_pitch_deg` — converge to ~0 in
  `PLATFORM_LEVEL`, then hold.
- `simulation.docked` — bool, true at `DOCKED`/`FINAL_DEPTH_STOP`.
- keep existing `sensors.*` (five ranges) and `derived.*` (alignment, offset,
  diameter); during pre-docking phases the ranges are out-of-range/`valid:false`
  and become valid as the boom lowers and the platform approaches the trunk.
- `sequence`, `timestamp_unix_s` unchanged.

The five range sensors keep the existing `ray_circle_range` trunk geometry (C-channel
half-width 0.400 m, trunk radius 0.300 m) but their readings are now **driven by
the sequence**: they only "see" the trunk during the docking phases and converge
to the final clearance (0.120 m) as the platform approaches.

### Part B — Orin: `range_ingest` adapter (new)

New module `canonical_zmq/canonical_zmq_publisher/range_ingest.py` (mirrors
`oak_capture.py`), run under the depthai-env python:

- SUB to `tcp://192.168.50.40:5555`, topic `harvester.sensors.v1` (single-part
  JSON — this is the *legacy* envelope, not canonical).
- Parse the JSON, map the five `sensors.*` keys → the canonical
  `v1/range/docking` record list (telemetry_key + `frame_id` from
  `calibration` `sensor_telemetry_bindings`), and PUSH a canonical three-frame
  packet into the aggregator ingest `tcp://127.0.0.1:5570`.
- Also emit `v1/docking/trunk_estimate` (the simulated trunk pose from the
  payload's trunk geometry) so the existing trunk HUD block updates.
- **New canonical channels** for the boom/leveling/phase state — see §3. Since
  the frozen `CANONICAL_CHANNELS` set has no boom/leveling channel, we extend
  `protocol.py` with a minimal JSON channel `v1/boom/state` (and, if preferred,
  `v1/range/docking` already carries the ranges). This is a contract change and
  must be coordinated with the dashboard shim (both import the same contract).
- Header: `source_mode: 'hardware'`, `source_id: 'pi_plc'`, `clock_domain:
  'plc_rtc_utc'`, `frame_id` from calibration, `calibration_id` marked provisional.

### Part C — Dashboard HUD: boom/leveling + operator guide

- Extend `SensorPanel.qml` (or add a sibling panel) to render
  `bridge.boomAngleLine`, `bridge.boomExtensionLine`, `bridge.levelLine`, and a
  prominent **phase/guide** line (`bridge.phaseGuideLine`), so the operator sees
  the current step ("RAISING BOOM … 12°", "EXTENDING TO 12 m", "LEVELING …",
  "LOWERING TO TRUNK", "DOCKING — S_C 0.18 m", "DOCKED").
- Extend `bridge.py` + `telemetry_model.py` to decode `v1/boom/state` (and to
  surface phase from `v1/range/docking`/`v1/docking/trunk_estimate` headers).
- The guide text is **render-only** (matches the existing safety boundary: no
  actuation, no command output).

### Part D — Wiring + docs

- Add `range_ingest` to `run_all.sh` (foreground + tmux) and `stop_all.sh` as
  needed, so the Pi stream flows into the canonical bus automatically.
- Update `docs/orin_canonical_zmq.md` (§ "Hardware adapters") to mark
  `range_ingest` implemented and document the new `v1/boom/state` channel.
- Add a unit test for the phase state machine (pure Python, no ZMQ) and for the
  `v1/range/docking` record mapping.

## 3. Decisions (confirmed with operator)

1. **Units.** Boom angle in **degrees** (0°→45°); all distances in **meters**
   (extension to ~12 m reach, final clearance 0.120 m). No percent scale.

2. **New canonical channel.** Add **`v1/boom/state`** — one JSON channel carrying
   `{phase, boom_angle_deg, boom_extension_m, platform_roll_deg,
   platform_pitch_deg, docked}`. Extend `CANONICAL_CHANNELS` + contract tests.

3. **Source badge.** Tag the simulated stream `source_mode: 'hardware'`,
   `source_id: 'pi_plc'` so the HUD badge reads `HARDWARE pi_plc`.

4. **Deployment target.** The simulation rewrite lives **on the Pi**
   (`~/plc-sensors/sensor_plc_publisher.py`); I will SSH to `192.168.50.40` and
   edit it in place (with confirmation at edit time). The Orin-side changes
   (Parts B/C/D) live in this repo.

## 4. Delivery order

1. **Contract** — add `v1/boom/state` (and any needed capability keys) to
   `protocol.py`; update `CANONICAL_CHANNELS`; extend contract tests.
2. **Pi simulation** — rewrite `sensor_plc_publisher.py` as a looping state
   machine (Part A); verify it runs standalone and prints the full phase cycle.
3. **Orin `range_ingest`** — new adapter (Part B) with the record mapping +
   `v1/boom/state` + `v1/docking/trunk_estimate`; unit test the mapping.
4. **Dashboard HUD** — `bridge.py`, `telemetry_model.py`, `SensorPanel.qml`
   (Part C) for phase/guide + boom/leveling lines.
5. **Wiring + docs** — `run_all.sh`/`stop_all.sh`, `docs/orin_canonical_zmq.md`.
6. **End-to-end verify** — start the Pi script, aggregator (with ingest), the
   `range_ingest` adapter, and the dashboard; confirm the HUD shows the full
   BOOM_RAISE→…→DOCKED loop and the five ranges converge at dock.

## 5. Verification checklist

- [ ] Pi script prints the full repeating sequence with sane numbers (boom 0→45°,
      extension to 12 m reach, level to ~0°, lower, ranges converge to 0.120 m).
- [ ] `python3 -m unittest discover -s canonical_zmq/test -v` green (contract +
      mapping tests).
- [ ] `range_ingest` forwards canonical `v1/range/docking`, `v1/boom/state`,
      `v1/docking/trunk_estimate` into `5590` (visible in dashboard streams panel).
- [ ] Dashboard HUD (key `3`) shows phase guide + boom/leveling + five ranges
      that go INVALID→converging→DOCKED each cycle.
- [ ] Safety: no socket ever writes an actuation/command (observation-only).
