---
name: harvester-canonical-zmq
description: "Extend the harvester_vision canonical ZeroMQ telemetry bus: add a channel, write an ingest adapter, wire a source, or reason about the aggregator/contract."
disable-model-invocation: true
argument-hint: "the channel to add, adapter to write, or source to ingest"
metadata:
  author: harvester
  version: "2.0.0"
  status: stable
---

# Harvester Canonical ZMQ

The Orin-side canonical ZeroMQ v1 telemetry bus. This skill is a self-contained reference for
extending it: add a channel, write an ingest adapter, or reason about the frozen contract.

The contract, endpoints, queue policy, and REP status shape are **frozen** and shared with the
Xavier gateway (protocol authority: the Xavier repo's `docs/canonical_zmq_v1.md`). The Orin has
no ROS/Gazebo, so its "canonical publisher" is a **ROS-independent aggregator**, not an `rclpy`
gateway.

## System architecture (memorize this)

```
Mosquitto MQTT (192.168.50.40:1883, topic harvester/sensors/v1)  [default PLC source]
        ──SUB──►  mqtt_ingest.py  ──PUSH──►
Pi PLC (192.168.50.40, tcp://*:5555, topic harvester.sensors.v1)  [legacy, opt-in]
        ──SUB──►  range_ingest.py ──PUSH──►   (WITH_RANGE_INGEST=1)
OAK docking  (192.168.50.21)  ──►  oak_capture.py  ──PUSH──►   aggregator PULL 5570
OAK cutting  (192.168.50.22)  ──►  oak_capture.py  ──PUSH──►        │ (re-owns seq/source_id)
MID-360 LiDAR (future)        ──►  lidar_ingest (deferred) ──►      ▼
                                                          PUB tcp://*:5590 ──SUB──► dashboard
                                                          REP tcp://*:5600  ◄──REQ── status client
```

Hard rules (never violate):

- The **aggregator** (`canonical_zmq/canonical_zmq_publisher/aggregator.py`) is the **sole owner**
  of the `5590` PUB and `5600` REP endpoints. Adapters **never bind `5590`/`5600`**; they PUSH
  canonical three-frame packets into the ingest PULL (`tcp://*:5570`, `--ingest tcp://*:5570`).
- The **contract** (`canonical_zmq/harvester_telemetry_contract/protocol.py`) is pure Python (no
  ROS/ZMQ import). `pack_message(channel, header, payload)` returns exactly three frames
  `[channel_utf8, msgpack(header), payload]`; `unpack_message(frames)` validates and returns
  `(channel, header, payload)`.
- The aggregator **re-owns** `sequence`, `source_id`, and `gateway_monotonic_ns` on ingest
  (`CanonicalAggregator.publish`); an adapter's header may omit them.
- **Safety boundary: observation-only.** Neither the aggregator nor any adapter ever emits a
  joint, velocity, PLC, solenoid, or motion command; the REP is read-only.

## Key files

| Path | Role |
|---|---|
| `canonical_zmq/harvester_telemetry_contract/protocol.py` | Frozen contract: `CANONICAL_CHANNELS`, `validate_header`, `pack_message`, `unpack_message`, `ProtocolError`, `SCHEMA_VERSION=1`. |
| `canonical_zmq/canonical_zmq_publisher/aggregator.py` | `CanonicalAggregator` — PUB/REP bind, ingest PULL, relay SUB, per-channel sequence + newest-wins queues + drop counts, `v1/system/status`. |
| `canonical_zmq/canonical_zmq_publisher/main.py` | CLI entry: `python -m canonical_zmq_publisher.main` (`--pub/--status/--ingest/--relay/--synthetic/--record-dir`). |
| `canonical_zmq/canonical_zmq_publisher/ingest.py` | `SyntheticSource` — emits every dashboard channel for no-hardware validation. |
| `canonical_zmq/canonical_zmq_publisher/oak_capture.py` | `OakCapture` — OAK RGB/depth/camera_info/imu adapter (the model for a new adapter). |
| `canonical_zmq/canonical_zmq_publisher/range_ingest.py` | `RangeIngest` — Pi PLC stream → `v1/range/docking` + `v1/boom/state` + `v1/docking/trunk_estimate`. |
| `canonical_zmq/canonical_zmq_publisher/lidar_capture.py` | `LidarCapture` — MID-360 adapter; `timing_snapshot()` picks acquisition time + provenance. |
| `canonical_zmq/canonical_zmq_publisher/recording.py` | `PacketRecorder`, `load_recording`, `iter_recordings`. |
| `canonical_zmq/canonical_zmq_publisher/replay.py` | Replays a recorded directory on a PUB (default `tcp://*:5591`, avoids live `5590`). |
| `canonical_zmq/test/` | `test_protocol.py`, `test_aggregator.py`, `test_oak_capture.py`, `test_lidar_capture.py`, `test_range_ingest.py`. |
| `lidar/livox_source.py` | Livox-SDK2 adapter; `decode_packet_time`, `timing_snapshot`, `timestamp_status`. |
| `deploy/ptp/`, `deploy/systemd/ptp4l-master@.service` | Orin-as-PTP-master for the LiDAR (verified working). |

## Canonical channels (frozen list)

`CANONICAL_CHANNELS` in `protocol.py` is a `frozenset`. Current members:

```
v1/camera/cutter/rgb|depth|camera_info|imu
v1/camera/docking/rgb|depth|camera_info|imu
v1/lidar/raw
v1/range/docking
v1/range/cutter
v1/boom/state
v1/docking/trunk_estimate
v1/calibration/status
v1/system/status
v1/operator/target_selection
```

Codec rules enforced by `validate_header`:
- `.../rgb` → `jpeg|h264|h265` (+ requires `pixel_encoding` str); `.../depth` → `depth_uint16_le`;
  `v1/lidar/raw` → `lidar_xyz_f32` (+ `point_count`, `point_stride_bytes`, non-empty `point_fields`);
  everything else (incl. `v1/boom/state`, `v1/docking/trunk_estimate`) → `json`.
- Every channel except `v1/system/status` requires a non-empty `frame_id`; `capabilities` must be a
  non-empty dict of `str -> bool`; `source_mode` ∈ `{simulation, hardware}`; `clock_domain` ∈
  `{ros_sim_time, utc_host, plc_rtc_utc, orin_realtime, lidar_ptp_utc}`; `schema_version == 1`.
  `orin_realtime` = this Orin's `CLOCK_REALTIME` (used when a sensor has no absolute clock of its
  own); `lidar_ptp_utc` = the LiDAR's own PTP/GPS-synchronized timestamp. **Caveat: the MID-360
  has NO UTC clock by default** — its timestamps are boot-relative until it slaves to a PTP master
  on the sensor NIC (`deploy/ptp/`). Never label a boot-relative counter `plc_rtc_utc`.
  `validate_header` ignores unknown extra fields, so adapters may add provenance fields (e.g.
  `timestamp_source`, `lidar_time_sync`, `lidar_time_type`, `lidar_time_source`) without a contract
  change; only the fields in `_GLOBAL_FIELDS` are required and type-checked.

The required header fields (`_GLOBAL_FIELDS`): `schema_version`, `source_mode`, `source_id`,
`sequence`, `frame_id`, `acquisition_timestamp_ns`, `clock_domain`, `gateway_monotonic_ns`,
`calibration_id`, `capabilities`.

## Adding a JSON channel + adapter (the `range_ingest` pattern)

`range_ingest.py` is the canonical model for a new observation-only adapter. To add a source:

1. **Contract**: add the channel name to `CANONICAL_CHANNELS` in `protocol.py`, and add a
   round-trip test in `canonical_zmq/test/test_protocol.py`.
2. **Adapter** (`canonical_zmq/canonical_zmq_publisher/<name>_ingest.py`):
   - SUB/connect to the source; map payloads to canonical headers + JSON.
   - A `_base_header(channel, acquisition_timestamp_ns, frame_id, capabilities)` helper producing
     `schema_version: 1`, `source_mode: 'hardware'`, `source_id`, `frame_id`,
     `acquisition_timestamp_ns: time.time_ns()`, `clock_domain`, `calibration_id`,
     `capabilities`, `codec: 'json'`.
   - `pack_message(channel, header, json.dumps(obj, separators=(',', ':')).encode())` then
     `push_socket.send_multipart(frames)`.
   - Keep pure **mapping functions** (e.g. `map_docking_records`, `map_boom_state`,
     `map_trunk_estimate`) separate from the socket loop so they are unit-testable without ZMQ.
   - Set `zmq.LINGER=0`, `RCVHWM=8`/`SNDHWM=8` on sockets. Wrap `pack_message` in try/except
     `ProtocolError` → log + drop, never crash.
3. **Test**: `canonical_zmq/test/test_<name>_ingest.py` for the pure mapping functions.
4. **Wire**: add a launch line to `run_all.sh` (a `<NAME>_CMD` variable + foreground & tmux
   sections) and a pattern to `stop_all.sh` `PATTERNS` array; mark the adapter "implemented" in
   `docs/orin_canonical_zmq.md` under "Hardware adapters".

`range_ingest.py` constants to reuse: `SOURCE_ID='pi_plc'`, `CLOCK_DOMAIN='plc_rtc_utc'`,
`CALIBRATION_ID='pi_plc_provisional_v0'`, `SENSOR_BINDINGS` (five `(telemetry_key, frame_id)`
pairs: `center_line`→`sensor_center_line_frame`, `diagonal_left_45deg`→`sensor_diagonal_left_frame`,
`diagonal_right_45deg`→`sensor_diagonal_right_frame`, `c_channel_left`→`sensor_c_channel_left_frame`,
`c_channel_right`→`sensor_c_channel_right_frame`).

The MQTT adapter (`mqtt_ingest.py`) uses the same `c_channel_left`/`c_channel_right` keys for the
two **ultrasonic side trunk-detection sensors** on the PLC MQTT stream: wire keys
`Ultrasonic Distance Left` and `Ultrasonic DIstance Right` (the PLC misspells the right key — both
spellings are accepted via its `_WIRE_KEY_ALIASES` table), alongside the three laser distances
(`Laser Distance Left/Center/Right`, the right one also tolerated misspelled as `Laser Distanec
Right`). Its `SENSOR_BINDINGS` entries are 3-tuples `(telemetry_key, frame_id, wire_key)`.

## Aggregator behavior (read before touching it)

- `publish()` increments the per-channel sequence, overwrites `source_id`/`gateway_monotonic_ns`,
  validates via `pack_message`, appends to a bounded `deque(maxlen=queue_depth)` (drop-oldest
  increments `drop_counts`), and records `last_stream_status`.
- `flush_one_packet()` pops the newest packet for the first non-empty channel (sorted) and
  `send_multipart(NOBLOCK)`; `ZMQ_CONFLATE` is **never** used.
- `publish_system_status()` emits `v1/system/status` every ~1 s from `main()`; `handle_status_request()`
  answers the REP `5600`.
- `_relay_loop()` forwards a remote canonical PUB (e.g. Xavier) without re-owning sequence
  (preserves remote `source_id`/`source_mode` so the dashboard badge stays correct).

## Two Python interpreters (mandatory)

| Role | Interpreter | Why |
|---|---|---|
| Aggregator / adapters / replay / recording | `depthai-env` (`/home/marcop/depthai-env/bin/python3`) | `zmq 27`, `msgpack`, `numpy`, `cv2`, `depthai`. |
| Dashboard | system `/usr/bin/python3` (3.8.10) | apt PySide2 5.14 QtQuick + `zmq`/`msgpack`/`numpy`/`PIL`. |

Both import the contract when `canonical_zmq` is on `PYTHONPATH`. The dashboard's
`protocol_shim.py` auto-locates it via `_CANDIDATES` (checks `../canonical_zmq` etc.).

## Run / verify

```bash
cd ~/harvester_vision

# Synthetic (no hardware) — emits every dashboard channel
PYTHONPATH=canonical_zmq /home/marcop/depthai-env/bin/python3 \
  -m canonical_zmq_publisher.main --synthetic --synthetic-period-s 0.2

# Relay Xavier Gazebo (gateway at 10.108.137.233:5590) onto local 5590
PYTHONPATH=canonical_zmq /home/marcop/depthai-env/bin/python3 \
  -m canonical_zmq_publisher.main --relay tcp://10.108.137.233:5590

# Full stack (aggregator + OAK adapters + mqtt_ingest + dashboard) — see run_all.sh
./run_all.sh foreground        # or default tmux mode
WITH_RANGE_INGEST=1 ./run_all.sh foreground   # also launch the legacy Pi range_ingest

# Tests (contract + aggregator + adapters)
PYTHONPATH=canonical_zmq /home/marcop/depthai-env/bin/python3 \
  -m unittest discover -s canonical_zmq/test -v
```

For a live adapter, run the aggregator with `--ingest tcp://*:5570`, run the adapter, then SUB to
`tcp://127.0.0.1:5590` and confirm the channel packets arrive.

## Guardrails

- Never bind the canonical `5590`/`5600` from an adapter; the aggregator owns them.
- Adding/renaming a channel is a contract change — update `CANONICAL_CHANNELS` + tests, and confirm
  the name is frozen with the operator before inventing one.
- Keep mapping logic pure and socket logic separate so tests never need a live ZMQ peer.
- Never weaken the observation-only safety boundary.
- Replay binds a **separate** PUB (`tcp://*:5591` by default) so it never collides with live `5590`.
- **Timestamps must state their real clock domain.** The MID-360 especially: it has no UTC clock of
  its own (`time_type==0` = ns since power-on), so label it `orin_realtime` (host arrival) or
  `lidar_ptp_utc` (PTP/GPS-synced device time), never `plc_rtc_utc`. The Orin is the PTP master for
  the LiDAR (`deploy/ptp/`, `ptp4l-master@eth1.service`; verified working, `time_type=1`).
  **Stamp a scan from ONE `timing_snapshot()` call** (source) / `capture.timing_snapshot()` — it
  returns the timestamp and its provenance together. Reading the timestamp and the status from two
  separate calls can label a stale host-clock value `lidar_ptp_utc`. See the README "MID-360 LiDAR
  time synchronization" section.
