# OAK Camera Adapter — H.264/H.265 + Jetson Hardware Decode

**Goal.** Stream the two OAK cameras (docking `192.168.50.21`, cutter
`192.168.50.22`) through the canonical telemetry bus to the source-agnostic
dashboard, with OAK-side H.264/H.265 hardware encode and Jetson-side hardware
decode. Operator can switch between the two cameras (keys `1`/`2`, render-only).
Range sensors are still dummy (the existing Pi/PLC script); LiDAR is not ready —
the design must leave clean slots for both.

## Verified environment facts

- **OAK cameras**: `docking_camera` `192.168.50.21`, `cutting_camera`
  `192.168.50.22` (from `camera_config.py`). DepthAI v3.1.0 in the
  `depthai-env` python.
- **OAK encoder**: DepthAI v3 supports `H264_MAIN`, `H264_HIGH`,
  `H264_BASELINE`, `H265_MAIN` (verified on `dai.VideoEncoderProperties.Profile`).
- **Jetson R35 (L4T)**: `nvidia-l4t-gstreamer` + `gst-plugins-bad` provide
  `nvv4l2decoder` (hardware H.264/H.265 decode) and `nvjpegdec` (hardware JPEG).
- **GStreamer Python bindings**: system `/usr/bin/python3` has `gi` + `Gst`
  1.16.3 (verified). The depthai-env python does **not** have `gi`.
- **Canonical bus**: already ported — `canonical_zmq/` (contract + aggregator)
  and `harvester_dashboard/` (dashboard). The aggregator exposes `--ingest`
  (PULL) for adapters; the dashboard renders `image://frames/{cutter,docking}`
  and switches views on keys `1`/`2` (render-only).

## Architecture

```
OAK (hardware H.264/H.265 encode)
   │  DepthAI v3 (depthai-env python)
   ▼
oak_capture adapter (per camera role)  ──PUSH canonical v1/camera/<role>/rgb──▶
   │                                                                         │
   ▼                                                                         ▼
Canonical aggregator (canonical_zmq_publisher)  ──PUB tcp://*:5590──▶  dashboard SUB
                                                                          │  system python
                                                                          │  nvv4l2decoder (hw decode)
                                                                          ▼
                                                                     RGB frame → HUD
```

- The OAK adapter **never binds** the canonical `5590`; it PUSHes canonical
  three-frame packets into the aggregator's `--ingest tcp://*:5570` PULL socket.
- OAK encodes H.264 (primary) with H.265 available via a flag. Each keyframe
  carries SPS/PPS so the Jetson decoder can (re)initialize mid-stream.
- The dashboard decodes H.264/H.265 with `nvv4l2decoder` (stateful per camera),
  falling back to JPEG/`nvjpegdec` when a stream declares `jpeg`.

## Scope

### 1. OAK capture adapter — `canonical_zmq/canonical_zmq_publisher/oak_capture.py`
- DepthAI v3 pipeline per camera role (reuse `camera_config.CAMERAS`).
- `H264_MAIN` encode (default), `--codec h265`/`--codec jpeg` switch.
- Encode metadata into canonical header: `codec`, `pixel_encoding`,
  `width`/`height`, `keyframe` (from `ImgFrame`/encoder flags when available),
  `source_mode: hardware`, `source_id: orin`, `clock_domain: plc_rtc_utc`,
  `acquisition_timestamp_ns` from `time_sync.capture_timestamp_us()*1000`,
  `frame_id` per camera role.
- PUSH the three canonical frames into the aggregator PULL socket
  (`--ingest-endpoint tcp://127.0.0.1:5570`).
- Per-camera enable/disable via existing control PULL (5566/5567) so switching
  fully stops the inactive pipeline (optional, matches legacy behavior).

### 2. Jetson hardware decode in the dashboard — `harvester_dashboard/.../decoders/`
- Replace the H.264/H.265 stubs with a stateful GStreamer `nvv4l2decoder`
  decoder: accumulate Annex-B NAL units per camera channel, feed to a
  `nvv4l2decoder ! nvvidconv (or videoconvert) ! video/x-raw,format=RGB`
  pipeline, pull frames via `appsink`, return RGB numpy arrays.
- Keep the existing `decode(header, payload) -> ndarray` entry point, but make
  decoders **stateful per channel** (keyed by `frame_id`/channel) via a small
  registry so `cutter` and `docking` have independent sessions.
- Hardware decode must degrade gracefully: if `nvv4l2decoder`/GStreamer is
  unavailable, fall back to a clear error (never crash).

### 3. Wire the decoder registry into `zmq_source.decode_frame`
- `decode_frame` currently returns one array per packet with no state. Add a
  per-channel decoder cache so H.264/H.265 sessions persist across packets.

### 4. Camera switching + HUD (already present, verify)
- Keys `1`/`2` switch the rendered camera (render-only); both subscriptions stay
  live. HUD overlay already shows ranges/trunk/calibration. Confirm nothing new
  is required; the range dummy and (later) LiDAR flow through the same bus.

### 5. Standby slots for range + LiDAR
- The adapter architecture (PUSH → aggregator) already supports additional
  ingests; document the range (`v1/range/docking`, `v1/range/cutter`) and
  LiDAR (`v1/lidar/raw`) channels as the slots they will fill later. No code
  change yet, but keep `oak_capture` single-purpose so range/LiDAR become
  separate small modules.

## Out of scope (later)
- MID-360 LiDAR UDP ingest + deskew.
- Real range sensors (current dummy values from the Pi/PLC script stay as-is).
- Depth channels (OAK depth not enabled yet).

## File layout (additive)
```
canonical_zmq/canonical_zmq_publisher/oak_capture.py   # new adapter
canonical_zmq/test/test_oak_capture.py                 # header/encode tests (no hw)
harvester_dashboard/harvester_dashboard/decoders/
  h264_decoder.py  (replace stub -> Jetson GStreamer)
  h265_decoder.py  (replace stub -> Jetson GStreamer)
  jetson_decode.py (shared GStreamer stateful decode helper)
harvester_dashboard/test/test_decoders.py (extend for h264/h265 happy path)
docs/orin_canonical_zmq.md (run instructions)
```

## Delivery order
1. OAK capture adapter + header/encode tests (no hardware).
2. Jetson GStreamer stateful decoder + tests.
3. Wire decoder registry into `zmq_source`.
4. End-to-end: start aggregator (`--ingest`), start two OAK adapters, start
   dashboard, verify live camera + switching + HUD.
