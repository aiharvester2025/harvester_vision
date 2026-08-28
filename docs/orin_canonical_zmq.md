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
`4` LiDAR inset, `5` LiDAR projection, `6` camera point-cloud inset,
`0`/`Esc` clear annotation, click to annotate (shows depth + camera-frame XYZ
when the OAK depth stream is on). All actions are non-actuating annotations
only. See `docs/oak_depth_pointcloud.md` for the depth/point-cloud feature.

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

## Hardware adapters

The Orin aggregator exposes the ingest boundary (`--ingest tcp://*:5570`, a
PULL socket). Adapters PUSH canonical packets into that PULL socket and never
bind the canonical `5590` themselves.

- `oak_capture` — **implemented** (see `docs/oak_depth_pointcloud.md`). OAK
  DepthAI v3 RGB H.264/H.265 primary + MJPEG fallback, plus stereo **depth**
  (`v1/camera/<name>/depth`, `depth_uint16_le`) and **camera_info** intrinsics
  (`v1/camera/<name>/camera_info`, `json`). Launched by `run_all.sh`
  (default `CODEC=jpeg`, depth on; `DEPTH=0` disables depth).
- `lidar_ingest` — **deferred** (MID-360 UDP, XYZ, `v1/lidar/raw`). See
  `.kilo/plans/mid360-lidar-integration.md`.
- `range_ingest` — **deferred** (Pi/PLC range sensors → `v1/range/docking`,
  `v1/range/cutter`).
- `cutter_range_ingest` — **deferred** (`v1/range/cutter`).

Safety boundary (unchanged): observation-only. The aggregator never emits a
joint, velocity, PLC, solenoid, or motion command; its REP is read-only.

## Codec decode notes (JPEG slow-motion fix)

The dashboard decodes RGB payloads by the header `codec` field, never by camera
identity. H.264/H.265 use the stateful `nvv4l2decoder` path
(`decoders/jetson_decode.py`); JPEG uses the `nvjpegdec` path
(`decoders/jetson_jpeg.py`). All three fall back to a clear error when the
Jetson hardware decoder is unavailable, never a crash.

**JPEG live-view was slow (human motion lagged).** The JPEG path originally
decoded *synchronously*: `JetsonJpegSession.decode()` pushed each JPEG and then
blocked on `appsink try-pull-sample` (up to 300 ms) waiting for that frame to
decode (~65–125 ms per frame of NVMM conversion + GPU `nvvidconv` + memory
copy). Because the drain worker is single-threaded, this serialized decode
stalled the loop, backed up the ZMQ `RCVHWM` buffer, dropped frames, and
accumulated latency — so live motion looked like slow motion even though each
individual frame was correct (no green screen, good quality). H.265 did not
show this because it decodes *asynchronously*: `decode()` pushes a buffer and
immediately returns the newest frame already cached by the `appsink
new-sample` signal.

**Fix (in `jetson_jpeg.py`).** The JPEG path now mirrors the H.264/H.265 async
model:

- `decode()` pushes the self-contained JPEG and returns `_latest_rgb`
  immediately (non-blocking), instead of waiting on `try-pull-sample`.
- Decoded frames arrive via the `appsink new-sample` signal handler
  (`_on_new_sample`), which does **not** require a running GLib main loop —
  GStreamer dispatches it synchronously from the streaming thread, exactly as
  the working H.264/H.265 decoder already does.
- The redundant `jpegparse` element was removed: `nvjpegdec` consumes
  `image/jpeg` directly.
- appsrc streaming flags changed from `is-live=true do-timestamp=true` to
  `is-live=false do-timestamp=false`. `is-live=true` made GStreamer apply
  live-latency/dropping logic that fought a stateless per-frame JPEG feed;
  `is-live=false` lets each self-contained JPEG flow straight through.

The `nvvidconv` RGBA conversion and the per-`frame_id` pipeline reuse are
unchanged. The NVDEC green-concealment guard (`_is_green_concealment`) applies
**only to the JPEG/`nvjpegdec` path** (`jetson_jpeg.py`) — H.264/H.265 NVDEC
does not emit that artifact, so `jetson_decode.py` has no such check.

**H.264/H.265 frame-path optimization.** `jetson_decode.py` was later tuned to
reduce per-frame CPU at 1080p (see `docs/oak_depth_pointcloud.md`):

- `appsrc` switched to `is-live=false do-timestamp=false` (was `is-live=true`).
- The decoded frame is produced as a **single contiguous HxWx3 copy** (drop the
  RGBA alpha byte in one `[:, :, :3].copy()`), avoiding a second
  `ascontiguousarray` copy in the image provider.

The dashboard also decodes only the **active** camera's RGB + depth streams
(active-camera-only decode), halving the `nvv4l2decoder` driver-thread CPU
(two 1080p H.265 decodes → one).
