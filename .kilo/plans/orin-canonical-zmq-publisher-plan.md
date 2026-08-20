# Orin — Canonical ZeroMQ Publisher + Source-Agnostic Dashboard

**Goal.** Port the Xavier canonical ZeroMQ telemetry stack to the Orin so that
the same source-agnostic operator dashboard runs here and displays, first, the
Gazebo data published by the Xavier (both hosts are on the same `10.108.136.0/23`
network), and later real hardware (OAK, LiDAR, range sensors) agnostically. The
hardware adapters (OAK with H.264/H.265 + Jetson hardware decode, MID-360, range
sensors) are a **later** task; this task delivers the canonical publisher
contract, the Orin-side canonical publisher (aggregator), and the dashboard, plus
a verification path that proves Xavier→Orin Gazebo data flows end-to-end.

## Reference (authoritative, already read)

- `ros2_ws` on Xavier (`https://github.com/aiharvester2025/ros2_ws`):
  - `docs/canonical_zmq_v1.md` — wire contract (three-frame multipart, headers,
    channels, queue policy, recording/replay, compatibility rules).
  - `src/harvester_telemetry_contract/` — pure-Python pack/validate (no ROS/ZMQ).
  - `src/harvester_telemetry_gateway/` — **ROS 2** read-only gateway (the Xavier
    "canonical publisher"): subscribes to Gazebo topics, publishes canonical v1.
  - `src/harvester_dashboard/` — source-agnostic Qt Quick dashboard (no ROS).
- `sim_sensors` (`sensor_plc_publisher.py`) — the legacy single-part ZMQ Pi/PLC
  sensor publisher (not canonical; will be normalized later).

## Key architectural facts (drive the port)

1. **The Xavier `harvester_telemetry_gateway` is a ROS 2 (rclpy) node.** The Orin
   has **no ROS/Gazebo** (verified: `/opt/ros` absent). Therefore the Orin's
   "canonical ZeroMQ publisher" is the **ROS-independent canonical aggregator**
   described in the forward plan (§3 of `1787018525986-*.md`), NOT a copy of the
   ROS gateway. It binds `tcp://*:5590` (PUB) + `tcp://*:5600` (REP status) on the
   Orin, uses the identical contract/recorder/REP shape, and accepts canonical
   packets from ingest adapters over PUSH/PULL (added later for OAK/LiDAR/range).
2. **Wire contract is identical and frozen** (`schema_version=1`). The dashboard
   is source-agnostic: it renders whatever canonical packets arrive, tagged
   `source_mode: simulation` (Xavier) or `hardware` (Orin later).
3. **To show Xavier Gazebo data on Orin now, the Orin dashboard subscribes
   directly to the Xavier gateway** at `tcp://10.108.137.233:5590` (the Xavier
   gateway already binds `tcp://*:5590`). The Orin aggregator additionally offers
   an optional **relay mode** that forwards Xavier's stream to the local `5590`
   so a single Orin endpoint serves both the local hardware aggregator and the
   remote Xavier source (dashboard points at one local endpoint).
4. **Interpreter split.** On Orin there is currently no PySide2; the active
   `python3` is `/usr/bin/python3` (3.8.10). The depthai-env (`/home/marcop/depthai-env`)
   has `zmq 27.1.0`, `msgpack 1.1.1`, `numpy 1.24.4`, `cv2`, `depthai` (no PIL).
   We match the Xavier convention:
   - Dashboard → **system `/usr/bin/python3`** with apt `python3-pyside2.qtquick`,
     `python3-zmq`, `python3-msgpack`, and QML modules.
   - Canonical aggregator / adapters / replay → the depthai-env python (has
     zmq/msgpack/numpy), or system python once apt zmq/msgpack are installed.
5. **Jetson R35 (L4T)** present → hardware decode path exists for later; this task
   keeps H.264/H.265 as clear stub errors (identical to Xavier) since hardware
   adapters are deferred.

## Scope (this task)

### A. Port the canonical contract (pure Python)
Copy `harvester_telemetry_contract` verbatim (it is dependency-free) into the
Orin repo. This is the single source of truth shared by publisher, adapters, and
dashboard. Include its tests.

### B. Orin canonical ZeroMQ publisher (aggregator) — new, no ROS
New package `canonical_zmq_publisher/` (pure Python + ZeroMQ + MessagePack, no
rclpy) that reproduces the Xavier gateway's *publisher half* with the same
behavior, but takes canonical packets from local ingest adapters rather than ROS
subscriptions:
- Binds `tcp://*:5590` PUB + `tcp://*:5600` REP (same response shape).
- Owns per-channel `sequence` counters, bounded newest-wins queues
  (`queue_depth`), drop counting, `ZMQ_CONFLATE` never used.
- Exact three-frame MessagePack recorder (same `recording.py` format) + replay.
- Periodic `v1/system/status` and read-only REP status (profile `hardware`,
  `source_id: orin`).
- Ingest side: PULL socket(s) (e.g. `tcp://*:5570`) where future adapters
  (`oak_capture`, `lidar_ingest`, `range_ingest`) PUSH canonical packets. For
  this task, a minimal **synthetic ingest** (`--synthetic`) emits sample
  canonical packets so the local Orin publisher + dashboard can be validated
  without any hardware.
- Optional **relay mode** (`--relay tcp://10.108.137.233:5590`): a SUB socket
  forwards Xavier's canonical packets onto the local `5590` PUB so the dashboard
  can use a single local endpoint. Relay preserves the original `source_id`
  (`xavier`) and `source_mode` (`simulation`) so the badge stays correct.

### C. Port the source-agnostic dashboard (no ROS)
Copy `harvester_dashboard` verbatim (it has zero ROS imports and already resolves
the contract via `protocol_shim.py`). Run under system `/usr/bin/python3` with
apt Qt/PySide2. Default `--pub` points at the Orin endpoint; to display Xavier
Gazebo data directly, pass `--pub tcp://10.108.137.233:5590 --status
tcp://10.108.137.233:5600`.

### D. Provision the runtime
Install (apt, requires sudo — prompt interactively, never embed the password):
`python3-pyside2.qtquick python3-zmq python3-msgpack qml-module-qtquick2
qml-module-qtquick-window2 qml-module-qtquick-layouts`. Also `python3-numpy
python3-pil` if the dashboard decoders need them under system python (the
reference dashboard's JPEG/depth/LiDAR decoders import numpy + PIL).

### E. Verification
- Contract + aggregator + recorder/replay tests green (pure python).
- Dashboard smoke test skips cleanly without GUI, or runs with Qt installed.
- Live end-to-end: with Xavier gateway running, Orin dashboard `--pub
  tcp://10.108.137.233:5590` shows the cutter/docking camera, LiDAR, ranges,
  trunk estimate, calibration/status, SIMULATION badge.
- Orin aggregator synthetic mode + relay mode: local dashboard shows the same
  canonical channels from the local `5590`.

## Out of scope (later tasks, explicitly deferred)
- OAK hardware capture adapter with H.264/H.265 + Jetson hardware decode.
- MID-360 LiDAR UDP ingest (deskew), PLC/range-sensor Modbus ingest, Pi key
  normalization.
- World-fixed pose exporter and `v1/pose/*` channels.
- Maintenance stream controls (only visible when a hardware control endpoint is
  defined).

## File layout (additive, inside `harvester_vision`)

```
harvester_vision/
  canonical_zmq/
    harvester_telemetry_contract/   # ported verbatim + tests
    canonical_zmq_publisher/
      __init__.py
      aggregator.py                 # binds 5590/5600, queues, status, recorder
      ingest.py                     # PULL ingest + synthetic source
      relay.py                      # Xavier -> local 5590 forwarder
      recording.py                  # exact three-frame msgpack recorder (ported)
      replay.py                     # replay publisher (default 5591)
      config.py                     # CLI/config dataclass
    test/                           # ported + new aggregator/relay tests
  harvester_dashboard/              # ported verbatim + qml/ + test/
  docs/
    orin_canonical_zmq.md           # Orin operational handoff (this plan distilled)
```

## Delivery order
1. Port `harvester_telemetry_contract` + its tests; run them (depthai-env python
   has msgpack).
2. Port recorder/replay helpers + tests.
3. Build `canonical_zmq_publisher` aggregator + synthetic ingest + relay + tests.
4. Port `harvester_dashboard` + qml + tests.
5. Provision apt packages (sudo) and run the dashboard.
6. End-to-end verification: synthetic local, relay, and direct Xavier subscription.
