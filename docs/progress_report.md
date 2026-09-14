# Harvester Vision — Progress Report (~50% Complete)

## By: AP Dr Megat Norulazmi / AI Section

## Executive Summary

The Orin-side harvester telemetry stack is approximately **50% complete**.
The core data path from OAK cameras through the canonical ZeroMQ bus to the
hardware-decoded dashboard is operational, and the LiDAR/PLC foundations are
in place. The remaining work concentrates on hardware integration details
(Modbus map, Livox SDK), pipeline-level optimisations, and touch-UI tooling.

---

## Completed

### OAK Camera Pipeline
- Dual OAK capture adapters (`docking_camera`, `cutting_camera`) publishing
  canonical `v1/camera/<name>/rgb` over ZeroMQ.
- H.264 / H.265 hardware encoding on-device with quality-based rate control;
  MJPEG fallback preserved.
- Additive stereo depth (`v1/camera/<name>/depth`) and IMU
  (`v1/camera/<name>/imu`) streams, both throttled to avoid dashboard OOM.
- Keyframe detection from Annex-B NAL units; supervisor auto-restarts on
  native DepthAI firmware crashes.

### Canonical Telemetry Bus (Orin)
- Aggregator, PULL ingest, recorder, replay, and status REP endpoints
  (`canonical_zmq/`).
- Frozen v1 contract with schema validation, newest-wins queue policy, and
  `ZMQ_CONFLATE`-free design.
- Range/boom ingest adapter that bridges the Pi PLC stream into canonical
  `v1/range/docking` + `v1/boom/state`.

### Dashboard
- Source-agnostic Qt Quick dashboard (`harvester_dashboard/`) running under
  system Python with PySide2.
- Jetson hardware decoding (`nvv4l2decoder` for H.264/H.265, `nvjpegdec` for
  JPEG); active-camera-only decode halves decoder driver CPU.
- Camera switching, telemetry HUD (boom angles, docking range sensors), and
  memory-trim timer to bound RSS.

### Time Synchronisation
- PLC battery-backed RTC is the persistent UTC authority; Orin synchronises
  via chrony.
- DepthAI v3 host-clock sync; publisher maps `ImgFrame.getTimestamp()` to
  PLC-RTC UTC microseconds with `time_authority`, `time_quality`, and
  `chrony_offset_us` in every payload.

### Calibration & Kinematics
- Coordinate-frame contract and commissioning sequence
  (`calibration/README.md`).
- Nominal simulation frames + blank deployment template.
- `lidar/boom_kinematics.py` converts PLC pivot/extension/tilt readings to
  LiDAR height above ground (dependency-free).

### MID-360 LiDAR (Foundations)
- `lidar/leveling.py` — gravity-aligned rotation-only leveling.
- `lidar/livox_source.py` — SDK interface stub.
- `mid360_publisher.py` — synthetic-mode ZMQ publisher with leveling
  plumbing and canonical timestamp contract.

### PLC / Modbus Bridge (Skeleton)
- `plc_sensor_bridge.py` publishes clearly-marked provisional orientation
  and range payloads so downstream code can be developed before the register
  map is available.

### Example & Tooling
- `examples/estimate_tree_height.py` — end-to-end pipeline from leveled cloud
  to absolute tree height.
- `sensor_viewer.py` — Raspberry Pi docking-sensor telemetry viewer.
- Deployment artefacts: systemd template, chrony config, NetworkManager
  connection profile, `run_all.sh` / `stop_all.sh`.
- Unit tests: `test_boom_kinematics.py`, `test_leveling.py`,
  `test_time_sync.py`, `test_transforms.py`, `test_tree_height.py`.

---

## In Progress / Remaining (~50%)

### PLC / Modbus Integration
- Real Modbus register map (addresses, scaling, validity flags) from the PLC
  program is not yet filled in.

### Livox MID-360 Live Integration
- `livox_source.py` adapter needs real SDK2 UDP point-stream and IMU wiring.

### OAK Pipeline Optimisation
- Inactive camera DepthAI pipeline is paused at the stream level but not
  stopped; full pipeline shutdown on camera switch is pending.

### Touch UI & Measurement Tools
- Current camera switching is keyboard/mouse based; PyQt5 touch-button
  replacement and distance-sensor widgets are pending.
- Two-click horizontal-line distance measurement tool is pending.

### Perception
- Trunk/canopy classifier in `estimate_tree_height.py` is a placeholder.

### Simulation
- RViz / Gazebo twin on Xavier with URDF primitives is pending.

### Timestamp Registers (Deferred)
- PTP/Livox and PLC/Modbus hardware timestamp registers are intentionally
  deferred per the phased plan.

---

## Risk / Next Steps

1. **Modbus map** is the critical path for LiDAR leveling via platform tilt /
   boom angle and for range-sensor values.
2. **Inactive pipeline stop** is the highest-ROI CPU optimisation before
   adding full-rate LiDAR rendering.
3. **Livox SDK integration** should be validated in synthetic mode first, then
   against hardware with the leveling unit tests as a regression gate.
