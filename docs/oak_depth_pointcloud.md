# OAK-D Pro Stereo Depth + Camera Point Cloud

Operational and design reference for the stereo-depth and camera-relative
point-cloud feature on the Orin. Complements `docs/orin_canonical_zmq.md`.

## Overview

The OAK-D Pro cameras (RVC2, USB) now produce **stereo depth** and **camera
intrinsics** alongside the existing RGB H.264/H.265/JPEG stream, on the frozen
canonical `v1` contract with **no contract changes**. The dashboard shows:

1. **Click-to-depth** — clicking the live camera image shows the depth in
   metres plus the back-projected camera-frame `(x, y, z)` at that pixel.
2. **Point-cloud inset** — a colored, depth-unprojected 3-D scatter of the
   active camera's scene (toggle with key `6`).

RGB streaming is **always on** for both cameras; depth is additive and
parallel, never touching the RGB path.

## Channels (frozen contract, no new codecs)

| Channel | Codec | Produced by | Notes |
|---|---|---|---|
| `v1/camera/<name>/rgb` | `h264`/`h265`/`jpeg` | OAK adapter | unchanged |
| `v1/camera/<name>/depth` | `depth_uint16_le` | OAK adapter | millimetre plane, independently decodable (no `pixel_encoding`) |
| `v1/camera/<name>/camera_info` | `json` | OAK adapter | intrinsics for back-projection |
| `v1/camera/<name>/imu` | `json` | OAK adapter | accel/gyro + gravity-derived attitude for vibration compensation |

`<name>` is `cutter` or `docking`. The capability flags `camera.<name>.depth`,
`camera.<name>.camera_info`, and `camera.<name>.imu` flip to `true` on the
channels that carry them.

## Producer: `oak_capture.py`

### Stereo pipeline (DepthAI v3)

- Mono pair on `CAM_B`/`CAM_C` (OV9282) at the native **1280×800 (800P)**,
  built with the v3 `Camera` node (`ColorCamera`/`MonoCamera` are deprecated
  in v3). The RGB sensor on `CAM_A` stays on the existing `ColorCamera`
  (kept untouched to guarantee zero RGB regression).
- `StereoDepth` with `setDepthAlign(CAM_A)` (RGB perspective) and
  `setOutputSize(width//2, height//2)` → depth is delivered at **half the RGB
  resolution** (960×540 for 1080p), same 16:9 aspect.
- `setDefaultProfilePreset(...)` default `fast_density`; `setLeftRightCheck`
  default on.

### Why half-resolution depth (not 1080p)

1. **Memory/OOM**: a 1080p depth plane (≈8 MB float32/frame) was the main
   contributor to the dashboard OOM. Half-res (960×540 ≈ 2 MB/frame) is
   bounded.
2. **Exact 2× mapping**: half-res with identical aspect ratio gives a clean,
   integer scale between RGB pixel `(u,v)` and depth pixel `(u/2, v/2)` — no
   fractional remap.

### camera_info (intrinsics)

Read from the device calibration via `getCameraIntrinsics(CAM_A, w, h)` —
**RGB camera** intrinsics at the delivered depth resolution (not the mono
camera). Distortion coefficients from `getDistortionCoefficients(CAM_A)` and
rectification rotation from `getStereoLeftRectificationRotation()`
(depthai 3.1.0 has no `getRectifiedRotation`).

`camera_info` is emitted **periodically (every 5 s)**, not once, because ZMQ
PUB drops messages to late subscribers (a dashboard started later, or a
`--supervise` restart, would otherwise never receive intrinsics).

### CLI flags (additive; RGB unaffected)

| Flag | Default | Purpose |
|---|---|---|
| `--no-depth` | (off) | RGB-only; byte-for-byte pre-depth behaviour |
| `--stereo-profile` | `fast_density` | `fast_density`/`fast_accuracy`/`high_detail`/`robotics`/`default` |
| `--stereo-lr-check` / `--no-stereo-lr-check` | on | left-right check |
| `--depth-fps` | `5.0` | depth publish rate (RGB stays at `--fps`) |

`run_all.sh` knob: `DEPTH=0` appends `--no-depth`.

### IMU (vibration compensation)

The hydraulic harvester vibrates, which jitters the camera point cloud. The
adapter also reads the onboard IMU (BNO08X on the OAK-D Pro) and publishes it on
`v1/camera/<name>/imu` (JSON). The dashboard de-rotates the point cloud by the
IMU tilt **relative to a reference attitude** before rendering, removing
vibration-frequency jitter while preserving the operator's intended (slow) boom
motion.

- Pipeline: v3 `IMU` node with `ACCELEROMETER_RAW` + `GYROSCOPE_RAW` (the
  universally available raw streams on the installed depthai build). The raw
  sensor-frame vectors are rotated into the camera optical frame (+X right, +Y
  down, +Z forward) on the host using the factory `imuExtrinsics` rotation
  (`getImuToCameraExtrinsics(CAM_A, ...)`), so the published accel/gyro and the
  derived attitude share the point cloud's frame. The node is only created
  after `device.getConnectedIMU()` confirms an IMU is present, so a unit without
  an IMU degrades to RGB+/-depth instead of failing to start.
- Payload: `accel_ms2`, `gyro_rad_s`, `attitude_rpy_rad` (roll/pitch from the
  gravity direction), `accel_norm_ms2`, `sample_rate_hz`, `n_samples`.
- CLI flags: `--no-imu` (off), `--imu-rate` (50 Hz default), `--imu-fps`
  (5 Hz default, matching depth). `run_all.sh` knob `IMU=0` appends `--no-imu`.
  Vibration compensation only needs a few Hz (the depth it stabilizes is 4-5
  Hz); the **future VIO arm-pose path uses a separate ~200 Hz IMU bridge**
  (`imu_bridge.py` → `v1/imu/arm`) rather than this channel — see
  `.kilo/plans/1787482633337-vio-arm-pose-marker-tracking.md`.
- Dashboard: `decoders/imustab.py` (`gravity_to_rpy`, `stabilize_points`) does
  the compensation; toggle with key `7` / "7 IMU" button (A/B on/off). The
  point-cloud inset shows the live roll/pitch and stab state. Only the
  **active** camera's IMU is processed (the inactive camera's IMU is dropped,
  matching the existing active-camera-only depth decode).

## Dashboard

### Click-to-depth (accuracy-critical)

RGB is 1920×1080; depth is 960×540. `bridge._depth_pixel_for(camera, u, v)`
maps the clicked RGB pixel to the depth pixel using the ratio of the two
delivered header sizes (robust to any resolution, not just 2×). The crosshair
shows the RGB pixel the operator clicked; back-projection uses the mapped
**depth** pixel with the depth-resolution intrinsics (`AnnotationState.build`
takes separate `backproject_u/v`).

### Point cloud (render-time, UI-only)

`decoders/pointcloud.py:unproject_depth(depth_m, rgb, camera_info,
max_points, rgb_width, rgb_height)`:

- Back-projects valid depth pixels into the camera optical frame
  (`+X` right, `+Y` down, `+Z` forward).
- Samples RGB colour at the corresponding RGB pixel (depth→RGB scale).
- **Subsamples the valid-pixel grid before unprojecting** (never allocates a
  full-resolution Nx3 array), capped at `--pointcloud-max-points` (default
  2000).
- **No new wire channel** — the cloud is derived entirely in the dashboard
  from rgb + depth + camera_info.

Rendered by `qml/PointCloudInset.qml` (top-right of the camera view), toggled
by key `6` / "6 Pts" button, and recomputed only when visible and for the
active camera.

### Active-camera-only decode

`zmq_source.TelemetryWorker` decodes only the **active** camera's `/rgb` and
`/depth` (the inactive camera's packets are ingested for freshness counters
but not decoded). On switch, the inactive camera's stateful NVDEC decoder is
closed. This halves the dashboard's `nvv4l2decoder` driver-thread CPU
(two 1080p H.265 decodes → one).

### Frame-path optimization (decode CPU)

`decoders/jetson_decode.py`:

- `appsrc` uses `is-live=false do-timestamp=false` (was `is-live=true`), and
  produces a **single contiguous HxWx3 copy** (drop alpha in one `[:, :, :3]
  .copy()` instead of a full RGBA copy + a later `ascontiguousarray`).
- The green-concealment check (`_is_green_concealment`) was **removed** from
  the H.264/H.265 path — that guard is a JPEG/`nvjpegdec` artifact only and
  does not affect H.264/H.265.

## Runtime

```bash
cd ~/harvester_vision
CODEC=h265 ./run_all.sh foreground       # depth on by default
DEPTH=0 ./run_all.sh foreground          # RGB-only
```

Controls: `1`/`2` switch camera, `3` HUD, `4` LiDAR inset, `5` LiDAR view,
`6` point cloud, `0`/`Esc` clear, click to annotate (shows depth + XYZ).

Verify the depth stream (dimensions, valid pixels, intrinsics):

```bash
PYTHONPATH=canonical_zmq:. /home/marcop/depthai-env/bin/python3 \
  scripts/validate_depth_stream.py --duration 10
```

## Known hardware facts

- Cameras are **OAK-D Pro** (RVC2, USB), mono OV9282 1280×800, baseline 7.5 cm,
  ideal range ~0.8–12 m (min ~0.4 m at 800P).
- The `NvMMLiteOpen ... BlockType = 279` line in logs is the H.265 NVDEC block
  initializing — **not an error**. `BlockType = 261` is H.264, `277` is MJPEG.

## Tests

- `canonical_zmq/test/test_oak_capture.py` — depth/camera_info header builders,
  capability flags, round-trips.
- `harvester_dashboard/test/test_pointcloud.py` — `unproject_depth`
  (constant-depth distance, invalid pixels, downsampling, colour mapping).
- `harvester_dashboard/test/test_target_model.py` — back-project depth-pixel vs
  RGB-pixel separation; `unproject_depth` ↔ `back_project` agreement.
