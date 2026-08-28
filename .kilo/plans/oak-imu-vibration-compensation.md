# Plan: OAK IMU + Vibration Compensation for the Camera Point Cloud

## Goal

The OAK-D-Pro-POE-FF cameras (currently RGB + stereo depth) sit on a
hydraulically-actuated, human-operated harvester that is subject to mechanical
vibration. The depth-unprojected point cloud rendered in the dashboard is
therefore jittering in ways that are **not** real scene motion. This work:

1. **Reads the onboard IMU** from each OAK camera (DepthAI v3 `IMU` node).
2. **Publishes the IMU data** on the canonical bus as a new channel.
3. **Uses that IMU data to compensate** (de-rotate / de-vibrate) the point cloud
   **before** it is sent to the dashboard, so the dashboard renders a
   vibration-stabilized cloud.

The dashboard's existing point-cloud path (`decoders/pointcloud.py` →
`bridge.latest_pointcloud` → `PointCloudInset.qml`) is preserved; the
stabilization is applied at the source so the wire stays RGB + depth +
camera_info + a **new** IMU channel, and the dashboard either consumes the
pre-compensated cloud or applies the same compensation as a render-time view.

---

## Key grounding facts (verified against live docs, not memory)

Per the Luxonis skill mandate (`luxonis-build-poc` / `luxonis-troubleshoot`),
every DepthAI v3 API call must be tied to the docs/`oak-examples`, never
invented. Confirmed from `https://docs.luxonis.com/software-v3/...`:

- **IMU node (v3):** `pipeline.create(dai.node.IMU)`; enable reports with
  `imu.enableIMUSensor([dai.IMUSensor.ACCELEROMETER_RAW, dai.IMUSensor.GYROSCOPE_RAW], rate)`;
  batching via `imu.setBatchReportThreshold(N)` / `imu.setMaxBatchReports(M)`;
  output queue via `imu.out.createOutputQueue(maxSize=..., blocking=False)`.
- **IMUData message:** `imu_q.get()` → `IMUData.packets[]`; each `IMUPacket`
  carries `acceleroMeter` / `gyroscope` / `magneticField` / `rotationVector`
  (each with `.x/.y/.z`) plus timestamps (`ts`, `tsDevice`, `sequenceNum`).
- **Hardware:** RVC2 devices (OAK-D Pro family) use **BNO08X** (9-axis) or
  **BMI270** (6-axis). OAK-D Pro PoE ships with BNO08X.
- **Reference frame — the decisive fact:** the `*_UNCALIBRATED` outputs are
  already rotated into the **Luxonis RDF frame = +X right, +Y down, +Z
  forward** — which is **exactly the camera optical frame** already used by
  `decoders/pointcloud.py` and `calibration/README.md`. So
  `ACCELEROMETER_UNCALIBRATED` / `GYROSCOPE_UNCALIBRATED` (and the RAW streams
  rotated through `getImuToCameraExtrinsics(CAM_A, False)`) produce gyro/accel
  in the **same frame the cloud is already expressed in**, removing any
  separate IMU→camera rotation in the compensation math.
- **IMU→camera extrinsics:** `device.readCalibration().getImuToCameraExtrinsics(dai.CameraBoardSocket.CAM_A, False)`
  returns the 4×4; its rotation block rotates RAW IMU vectors into the camera
  frame. This is the fallback if we choose RAW instead of UNCALIBRATED.
- **Rates:** recommended high-rate raw streams are `ACCELEROMETER_RAW@480 Hz`,
  `GYROSCOPE_RAW@400 Hz`; BNO08X gyro >400 Hz jitters, so ~200–400 Hz is safe.

---

## Architecture decision: where does compensation happen?

Two viable placements, settled as follows:

- **Option A — compensate in the OAK adapter (producer).** The adapter already
  runs numpy (depthai-env) and produces the depth. It would compute the
  IMU-derived roll/pitch delta and rotate the depth-derived point cloud before
  publishing a new `pointcloud_xyz_rgb_f32` channel. Pro: uniform wire, dashboard
  just draws. Con: adds a large new channel, changes producer/consumer split,
  more contract surface.
- **Option B — compensate in the dashboard (consumer), at render time.** The
  dashboard already derives the cloud from rgb+depth+camera_info. It receives
  the **IMU channel** and applies a per-frame attitude delta (a 3×3 rotation in
  the optical frame) to the cloud just before flattening for QML. Pro: no new
  heavy wire channel, reuse of existing `unproject_depth`, the point cloud stays
  a "render-time UI-only view" exactly as the prior plan concluded. Con: the
  dashboard must maintain a small attitude state.

**Decision: Option B (dashboard-side render-time compensation),** with the
rotation math implemented in a **pure, dependency-free, unit-tested** module
(`decoders/imustab.py`) so it can be reused and so the same math can later be
moved to the producer without rework. This is the lowest-risk path and
preserves the existing "point cloud is a UI-only view" contract decision from
`oak-depth-pointcloud.md`. The new wire addition is **only** the IMU stream.

---

## What already exists (reuse, do not redo)

- `oak_capture.py` already builds RGB + stereo depth + `camera_info` and PUSHes
  into the aggregator; the additive-pipeline pattern (v3 `Camera` mono pair,
  `StereoDepth`) is the exact template the IMU node follows.
- `canonical_zmq/harvester_telemetry_contract/protocol.py` is the single frozen
  validator; new channel + codec must be added here and mirrored in the
  dashboard `protocol_shim.py` (which just re-exports it).
- `harvester_dashboard/harvester_dashboard/decoders/pointcloud.py:unproject_depth`
  produces the N×3 optical-frame cloud; the compensation is a single matrix
  multiply appended to this path.
- `harvester_dashboard/harvester_dashboard/bridge.py:latest_pointcloud` and
  `_get_camera_pointcloud` are the sole call sites.
- `geometry/transforms.py` already has `rotation_from_quaternion`,
  `rotation_from_rpy`, and `level_points_rotation_only` — the last is the exact
  "apply a rotation-only transform to a point cloud" primitive needed.
- `ingest.py:SyntheticSource.emit_once` is the synthetic test harness; the IMU
  synthetic packet slots in here.
- `zmq_source.py:SocketDrainer` already dispatches JSON vs depth vs rgb vs
  lidar; IMU will be a JSON (or a small binary) channel.

---

## Contract changes (single new channel + one codec)

Add to `CANONICAL_CHANNELS` in `protocol.py`:

- `v1/camera/cutter/imu`
- `v1/camera/docking/imu`

Add a codec branch in `_validate_image_header` (or a dedicated validator):
codec `imu_json` (or reuse `json`). **Decision: use `json`** for the IMU stream
to match `camera_info` and avoid a new codec branch — the payload is small
(~1 sample × {accel, gyro, attitude}) and the dashboard already parses JSON
channels. Rationale: IMU samples are tiny and low-rate (≤ 200 Hz, batched), so
a JSON packet per batch is negligible; JSON also keeps the contract simple and
reuses `JSON_CHANNELS` parsing.

The IMU payload JSON schema (producer → dashboard):

```json
{
  "frame_id": "cutter_camera_optical_frame",
  "accel_ms2": [ax, ay, az],          // m/s^2, optical frame (+X right, +Y down, +Z fwd)
  "gyro_rad_s": [gx, gy, gz],          // rad/s, optical frame
  "attitude_rpy_rad": [roll, pitch, yaw], // gravity-derived tilt (see below)
  "accel_norm_ms2": 9.806,             // |a| magnitude (vibration sanity check)
  "sample_rate_hz": 200,
  "n_samples": 1,                      // number of samples averaged in this packet
  "stamp": { "acquisition_timestamp_ns": ..., "clock_domain": "plc_rtc_utc" }
}
```

Capability flag `camera.<name>.imu` added to `build_capabilities` and the
aggregator `capabilities` dict (default `False`).

> The `attitude_rpy_rad` field is what the dashboard consumes directly. It is
> the small-angle tilt (roll/pitch from gravity, yaw optionally from gyro
> integration or left 0 for a leveling-only compensation). Derivation is
> described below.

---

## Implementation

### 1. OAK adapter: add the IMU node (`oak_capture.py`)

Additive path, mirroring `_build_depth_path`; RGB/depth untouched.

**Pipeline** (inside `_open_pipeline`, guarded by a new `imu=True` flag):
```python
imu = pipeline.create(dai.node.IMU)
imu.enableIMUSensor([
    dai.IMUSensor.ACCELEROMETER_UNCALIBRATED,
    dai.IMUSensor.GYROSCOPE_UNCALIBRATED,
], imu_rate)   # imu_rate default 200 Hz
imu.setBatchReportThreshold(1)
imu.setMaxBatchReports(10)      # ~50 ms worth at 200 Hz
self._imu_queue = imu.out.createOutputQueue(maxSize=32, blocking=False)
```
Use `*_UNCALIBRATED` so the vectors arrive already in the camera optical frame
(no per-sample rotation). Store `self._imu_enabled`, `self._imu_queue`.

**Attitude derivation (in the adapter, pure numpy):**
For each IMU batch, compute the gravity direction from the low-passed
accelerometer (`a_lp = lowpass(a)`; gravity ≈ `-a_lp / |a_lp|` at rest) and
derive roll/pitch tilt relative to a reference pose captured at startup
(or a running low-pass "home" attitude). A first-order complementary estimate
is enough for vibration de-jitter:

- `roll  = atan2(ay, az)` , `pitch = atan2(-ax, sqrt(ay^2+az^2))` (optical frame).
- Store the running low-pass tilt; the **delta** vs the long-term average is
  the vibration term to remove.

**Crucially, keep the compensation math as a shared function** — implement the
gravity→RPY and the "apply delta rotation to N×3 points" in
`harvester_dashboard/harvester_dashboard/decoders/imustab.py` (pure, tested),
and have the adapter call the same function (the adapter already imports from
the repo root; add `harvester_dashboard` to its path or duplicate the ~15 lines
in a small shared `geometry/imu.py` — **decision: put the math in
`geometry/imu.py`** so both the adapter (depthai-env) and the dashboard import
the identical, tested code with zero Qt/numpy-version coupling concerns beyond
numpy).

Wait — `geometry/` is at repo root and is dependency-free except numpy; the
adapter runs with `PYTHONPATH=canonical_zmq` per `run_all.sh`. To avoid
changing the adapter's import environment, place the shared math in
`canonical_zmq/canonical_zmq_publisher/imu.py` (already on the adapter's path)
AND have the dashboard import it via the existing `protocol_shim`-style path,
OR simply **duplicate the small, tested rotation helper** into the dashboard's
`decoders/imustab.py` and unit-test both against the same fixture. 

**Final decision on code placement:** implement the compensation math once in
`harvester_dashboard/harvester_dashboard/decoders/imustab.py` (pure numpy,
unit-tested). The **dashboard** is the sole consumer of that math (Option B).
The **adapter** only *publishes raw IMU* (accel+gyro in optical frame, plus the
gravity-derived `attitude_rpy_rad` computed inline with ~6 lines of the same
math) — it does **not** import the dashboard module. The ~6-line RPY-from-gravity
snippet is trivially duplicated and covered by a small test in the adapter's
test file. This keeps the adapter's environment unchanged and avoids a new
shared-module import graph.

**Emitting IMU packets** (mirror `_emit_depth`, throttled to `imu_fps` default
~50 Hz of *batched* samples): in `run()`, alongside the depth drain, add a
non-blocking `_drain_imu()` that:
- reads all queued `IMUData`, accumulates accel/gyro over the last window,
- computes `attitude_rpy_rad` from the low-passed gravity,
- packs `imu_json` via a new `build_imu_header(...)` + `pack_message` and
  `push_socket.send_multipart`.

New header builder `build_imu_header(camera_role, acquisition_timestamp_ns, frame_id)`
returning `codec='json'`, `width/height` omitted (IMU has no image geometry) —
**note:** the current `_validate_image_header` requires positive `width/height`
for every `v1/camera/` channel; the IMU channel must therefore **not** be
classified as an image channel. Give it its own `_validate_imu_header` (codec
`json`, no width/height) so it is not forced into the image-validator branch.

New CLI flags (additive):
- `--imu` (default on) / `--no-imu`.
- `--imu-rate` (default `200`, Hz, gyro/accel report rate).
- `--imu-fps` (default `50`, Hz, published packet rate).

### 2. Contract (`protocol.py`)

- Add `v1/camera/cutter/imu` and `v1/camera/docking/imu` to `CANONICAL_CHANNELS`.
- Add a `_validate_imu_header` (codec `json`, no width/height requirement) and
  route `channel.endswith('/imu')` to it **before** the generic image branch in
  `validate_header`.
- Add `camera.<name>.imu` to the `build_capabilities` default in `oak_capture.py`
  and to `aggregator.capabilities` (default `False`).

### 3. Dashboard consumption + compensation

- **`zmq_source.py`:** no structural change. Add `v1/camera/<name>/imu` to
  `JSON_CHANNELS` in `telemetry_model.py` so the JSON payload lands in
  `last_json` (the worker already routes any JSON codec channel there). The IMU
  packets are low-rate; no active-camera gating needed, but gating is harmless.
- **`telemetry_model.py`:** add the two IMU channels to `JSON_CHANNELS`; add
  `snapshot_imu(camera)` returning `self._json_of('v1/camera/{}/imu'.format(camera))`.
- **New `decoders/imustab.py` (pure numpy):**
  - `gravity_to_rpy(accel_ms2) -> (roll, pitch, yaw=None)` (optical frame).
  - `tilt_delta_rotation(current_rpy, reference_rpy) -> 3x3` — the small-angle
    rotation that removes the vibration relative to a reference attitude.
  - `stabilize_points(points, current_rpy, reference_rpy) -> points'` — applies
    the delta rotation to an N×3 array (reuses `geometry.transforms` math or an
    inline matrix multiply).
- **`bridge.py`:** in `latest_pointcloud`, after `unproject_depth`, fetch
  `self.model.snapshot_imu(camera)` and, if present and fresh (compare the IMU
  `acquisition_timestamp_ns` to the depth frame's within a tolerance), apply
  `stabilize_points`. Maintain a per-camera reference attitude (`self._imu_reference[camera]`)
  captured on first IMU sample (or a slow-moving average). Expose a read-only
  `imuAttitudeLine` property for the HUD (roll/pitch in degrees) and a
  `imuActive` bool for the inset title.

### 4. Synthetic source (`ingest.py`)

Add `synthetic_imu_payload()` (accel `[0,0,-9.806]` + small gyro noise + zero
attitude) and publish `v1/camera/cutter/imu` and `v1/camera/docking/imu` in
`SyntheticSource.emit_once`, so the full path (including the new compensation
no-op at zero tilt) is testable without hardware.

### 5. Dashboard HUD (small)

- `PointCloudInset.qml` title gains an "IMU: ±X.X° roll ±Y.Y° pitch" suffix
  when `bridge.imuActive`, to show the operator that stabilization is engaged.
- Optional: a `bridge.imuEnabled` toggle (key `7`) to A/B the stabilization —
  default on. Keeps the feature non-disruptive and gives a visual rollback.

### 6. `run_all.sh`

Add `IMU=${IMU:-1}` knob; `IMU=0` appends `--no-imu` to the `oak_capture` launch
command (mirrors the existing `DEPTH=0` knob).

---

## Tests

### `canonical_zmq/test/test_oak_capture.py`

- `test_build_imu_header_is_canonical` — round-trips `build_imu_header` through
  `pack_message`/`unpack_message`; verifies `codec=='json'`, no width/height,
  `capabilities['camera.docking.imu'] is True`.
- `test_imu_capability_default_off` — `build_capabilities` without imu flag has
  `camera.cutter.imu is False`.
- `test_gravity_to_rpy_optical_frame` — pure check of the inline RPY-from-gravity
  snippet (accel `[0,0,-9.8]` → ~0 roll/pitch; accel `[0, 9.8, 0]` → roll=90°).
- `test_imu_channel_validated` — the new channel passes `validate_header`
  without requiring width/height.

### `harvester_dashboard/test/test_imustab.py` (new)

- `test_gravity_to_rpy_identity` — accel `[0,0,-g]` → zero tilt.
- `test_gravity_to_rpy_roll` / `_pitch` — known tilts map to expected angles.
- `test_stabilize_identity` — reference == current → points unchanged.
- `test_stabilize_removes_roll` — a cloud tilted by +roll with reference
  attitude of −roll (or vice-versa) is restored to the level orientation;
  distances from the optical axis are preserved (rotation-only, no scaling).
- `test_stabilize_preserves_distance` — `||p'|| == ||p||` for arbitrary tilt.

### `harvester_dashboard/test/test_pointcloud.py`

- Extend with `test_unproject_then_stabilize` — constant-depth cloud stabilized
  under a synthetic tilt still has every point at the original Euclidean
  distance (no scale introduced).

### `harvester_dashboard/test/test_model.py` / `helpers.py`

- Add `imu_packet()` helper and assert `snapshot_imu('cutter')` returns the
  parsed payload.

---

## Validation (manual, post-merge)

1. Unit: `PYTHONPATH=canonical_zmq /home/marcop/depthai-env/bin/python3 -m unittest discover -s canonical_zmq/test -v`
   and `PYTHONPATH=harvester_dashboard /usr/bin/python3 -m unittest discover -s harvester_dashboard/test -v`.
2. Synthetic: run `--synthetic`, confirm the dashboard HUD shows "IMU" roll/pitch
   ≈ 0 and the point cloud is unchanged.
3. Hardware: `./run_all.sh foreground` — confirm IMU packets arrive at
   `tcp://*:5590`; with the harvester vibrating, toggle key `7` and observe the
   point-cloud jitter reduce when stabilization is on vs off.
4. Time-sync regression: `python3 scripts/validate_camera_time_sync.py` still
   passes (no change to sync).

---

## Risks + rollback

- **IMU absent on some units** (all OAK-D-Lite lack IMU; OAK-D Pro has it).
  The adapter must detect `enableIMUSensor` availability / a missing IMU at
  pipeline build and **gracefully disable** the IMU path (log once) rather than
  crash — the cloud then simply renders uncompensated (current behaviour).
- **Frame mismatch** between IMU `*_UNCALIBRATED` and the camera optical frame:
  both are documented as +X right / +Y down / +Z forward, but this must be
  confirmed on hardware in validation step 3; if the sign of an axis is
  flipped, the fix is a sign constant in `gravity_to_rpy` (single place).
- **Gravity-vs-vibration separation** is imperfect at low frequencies (a slow
  hydraulic pitch also moves the gravity vector). The compensation targets
  **vibration de-jitter** (higher-frequency), not absolute leveling; the
  long-term attitude is intentionally left intact so real boom motion is not
  erased. This is an explicit, documented scope boundary.
- **Rollback:** `--no-imu` (and `IMU=0`) disables the whole feature; the
  dashboard's `imuEnabled` toggle (key `7`) reverts to uncompensated rendering
  without restarting anything. No existing channel or RGB/depth path is touched.

---

## Out of scope

- World-fixed / boom-pose fusion of the camera cloud (still
  `v1/pose/arm`-dependent; see prior plan).
- Absolute leveling that removes the *operator-intended* boom pitch — only
  vibration-frequency de-jitter is compensated.
- LiDAR (MID-360) vibration compensation — separate sensor with its own IMU
  strategy.
- Recording IMU streams to the audit recorder (recorded as a side effect of
  `--record-dir`, no new code).

---

## Files touched

| File | Change |
|---|---|
| `canonical_zmq/canonical_zmq_publisher/oak_capture.py` | Add v3 `IMU` node + queue; `build_imu_header`; `_drain_imu`; `--imu/--no-imu`, `--imu-rate`, `--imu-fps`; inline gravity→RPY; `camera.<name>.imu` capability; graceful IMU-absent fallback. RGB/depth path untouched. |
| `canonical_zmq/harvester_telemetry_contract/protocol.py` | Add `v1/camera/{cutter,docking}/imu` to `CANONICAL_CHANNELS`; add `_validate_imu_header` (json, no width/height). |
| `canonical_zmq/canonical_zmq_publisher/aggregator.py` | Add `camera.<name>.imu: False` to `capabilities`. |
| `canonical_zmq/canonical_zmq_publisher/ingest.py` | `synthetic_imu_payload()` + emit cutter/docking IMU packets. |
| `harvester_dashboard/harvester_dashboard/telemetry_model.py` | Add IMU channels to `JSON_CHANNELS`; `snapshot_imu(camera)`. |
| `harvester_dashboard/harvester_dashboard/decoders/imustab.py` (new) | `gravity_to_rpy`, `tilt_delta_rotation`, `stabilize_points`. |
| `harvester_dashboard/harvester_dashboard/bridge.py` | Apply `stabilize_points` in `latest_pointcloud`; per-camera reference attitude; `imuActive`/`imuEnabled`/`imuAttitudeLine` properties; toggle slot. |
| `harvester_dashboard/qml/PointCloudInset.qml` | IMU status suffix in title; bind to `bridge.imuActive`. |
| `harvester_dashboard/qml/Dashboard.qml` | Key `7` binding for `imuEnabled` toggle. |
| `harvester_dashboard/test/test_imustab.py` (new) | Unit tests for the compensation math. |
| `harvester_dashboard/test/test_pointcloud.py` | Extend with unproject→stabilize test. |
| `harvester_dashboard/test/helpers.py` | `imu_packet()` helper. |
| `canonical_zmq/test/test_oak_capture.py` | IMU header/capability/gravity tests. |
| `run_all.sh` | `IMU=${IMU:-1}` knob. |
| `docs/oak_depth_pointcloud.md` | Document the new IMU channel + compensation. |
