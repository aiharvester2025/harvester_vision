# Orin — Canonical ZeroMQ Publisher + Source-Agnostic Dashboard

Operational handoff for the Orin side of the canonical ZeroMQ telemetry bus.
The protocol authority is the Xavier repo's `docs/canonical_zmq_v1.md`; the
Orin uses the **identical** frozen v1 contract, endpoints, queue policy,
recorder format, and REP status shape. The Orin has no ROS/Gazebo, so its
"canonical publisher" is a **ROS-independent aggregator**, not the Xavier
`rclpy` gateway.

## Components (in this repo)

| Path | Role |
|---|---|
| `canonical_zmq/harvester_telemetry_contract/` | Pure-Python pack/validate (ported verbatim; no ROS/ZMQ). |
| `canonical_zmq/canonical_zmq_publisher/` | Orin canonical aggregator (PUB/REP), recorder, replay, synthetic source, relay. |
| `canonical_zmq/test/` | Contract + aggregator tests. |
| `harvester_dashboard/` | Source-agnostic Qt Quick dashboard (ported verbatim; no ROS). |

## Endpoints and queue policy (identical to Xavier)

- One PUB endpoint per source, default `tcp://*:5590`; all `v1/*` channels
  multiplexed (first frame = subscription prefix). The **Orin aggregator owns
  `5590` on the Orin**; the Xavier gateway owns `5590` on the Xavier.
- Read-only REP status, default `tcp://*:5600`; same response shape on both.
- Bounded newest-wins queues; drop complete old packets; `ZMQ_CONFLATE` never
  used.

## Two Python interpreters (same rule as Xavier)

| Role | Interpreter | Why |
|---|---|---|
| Canonical aggregator / adapters / replay | `depthai-env` python (`/home/marcop/depthai-env/bin/python3`) | Has `zmq 27`, `msgpack`, `numpy`, `cv2`, `depthai`. |
| **Dashboard** | system `/usr/bin/python3` (3.8.10) | Has apt PySide2 5.14 QtQuick + `zmq`/`msgpack`/`numpy`/`PIL`. |

Both can import the contract when `canonical_zmq` is on `PYTHONPATH` (or the
dashboard's `protocol_shim.py` auto-locates it).

## Run: Orin canonical publisher (synthetic, no hardware)

```bash
cd ~/harvester_vision
PYTHONPATH=canonical_zmq /home/marcop/depthai-env/bin/python3 \
  -m canonical_zmq_publisher.main --synthetic --synthetic-period-s 0.2
```

This binds `tcp://*:5590` (PUB) + `tcp://*:5600` (REP) and emits synthetic
`source_mode: hardware` packets for every dashboard channel.

## Run: relay Xavier Gazebo data to the Orin's local 5590

When the Xavier gateway is running (binds `tcp://*:5590` on Xavier at
`10.108.137.233`), the Orin aggregator can forward that stream onto its own
local `5590` so a single local endpoint serves both sources:

```bash
cd ~/harvester_vision
PYTHONPATH=canonical_zmq /home/marcop/depthai-env/bin/python3 \
  -m canonical_zmq_publisher.main --relay tcp://10.108.137.233:5590
```

Relay preserves each packet's original `source_id`/`source_mode`, so the
dashboard badge stays correct (`SIMULATION` for Xavier, `HARDWARE` for the
later Orin adapters).

## Run: dashboard

**To display Xavier Gazebo data directly** (no relay; Xavier gateway must be
running on the Xavier):

```bash
cd ~/harvester_vision
DISPLAY=:1 PYTHONPATH=harvester_dashboard /usr/bin/python3 \
  -m harvester_dashboard.main \
  --pub tcp://10.108.137.233:5590 --status tcp://10.108.137.233:5600
```

**To display the local Orin publisher** (synthetic or relayed):

```bash
cd ~/harvester_vision
DISPLAY=:1 PYTHONPATH=harvester_dashboard /usr/bin/python3 \
  -m harvester_dashboard.main --pub tcp://127.0.0.1:5590 --status tcp://127.0.0.1:5600
```

Controls: `1` cutter view, `2` docking view (render-only), `3` sensor HUD,
`4` LiDAR inset, `5` LiDAR projection, `0`/`Esc` clear annotation, click to
annotate. All actions are non-actuating annotations only.

## Tests

```bash
cd ~/harvester_vision
# Contract + aggregator (either interpreter)
PYTHONPATH=canonical_zmq /home/marcop/depthai-env/bin/python3 \
  -m unittest discover -s canonical_zmq/test -v
# Dashboard (system python)
PYTHONPATH=harvester_dashboard /usr/bin/python3 \
  -m unittest discover -s harvester_dashboard/test -v
```

## Hardware adapters (deferred — later task)

The Orin aggregator already exposes the ingest boundary (`--ingest
tcp://*:5570`, a PULL socket) and the relay/synthetic modes. The future
adapters PUSH canonical packets into that PULL socket and never bind the
canonical `5590` themselves:

- `oak_capture` — OAK DepthAI v3, H.264/H.265 primary + Jetson hardware decode,
  MJPEG fallback (header `codec`/`pixel_encoding`/`keyframe`).
- `lidar_ingest` — MID-360 UDP, XYZ (deskew later), `v1/lidar/raw`.
- `range_ingest` — Pi/PLC range sensors normalized to canonical `telemetry_key`
  set, `v1/range/docking` / `v1/range/cutter`.
- `cutter_range_ingest` — `v1/range/cutter`.

Safety boundary (unchanged): observation-only. The aggregator never emits a
joint, velocity, PLC, solenoid, or motion command; its REP is read-only.
