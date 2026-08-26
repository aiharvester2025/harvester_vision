# Plan: Add OAK-D Stereo Depth + Click-to-Depth + Point-Cloud View to the Dashboard

## Goal

Extend the canonical ZeroMQ stack so the OAK-D-PRO-POE-FF cameras (currently
RGB-only) also publish stereo depth and camera intrinsics, and so the
dashboard click handler shows the depth + camera-frame 3-D point at the
clicked pixel and can additionally render a colored point cloud of the
scene.

Concrete user-facing outcome:

1. A click on the live camera view shows the depth in metres at the pixel
   plus the back-projected camera-frame 3-D point, matching the existing
   `Annotation.qml` crosshair and label.
2. A new toggle renders a colored, depth-derived 3-D point cloud for the
   active camera using the same projection / scatter pattern as the
   existing LiDAR inset.
3. Depth and `camera_info` stream from the OAK at the same FPS as RGB,
   gated by the same active-camera selection so the inactive camera does
   not waste OAK CPU/USB/Ethernet.

The contract (`v1/camera/<name>/depth` with codec `depth_uint16_le`,
`v1/camera/<name>/camera_info` with codec `json`) is already in the
frozen canonical schema; no contract changes are needed.

---

## Hard constraint: do not break the existing RGB h265/jpeg path

The current RGB streaming (H.264/H.265 hardware encode on the OAK +
Jetson hardware decode, and MJPEG/JPEG lossless fallback) is working and
must **not regress**. This is a non-negotiable constraint:

- The RGB pipeline — `ColorCamera(CAM_A)` → `VideoEncoder`
  (`H264_MAIN`/`H265_MAIN`/`MJPEG` profile, `setQuality` rate control,
  `setDefaultProfilePreset`) → `bitstream` output queue → `build_rgb_header`
  → `pack_message('v1/camera/<name>/rgb', ...)` → PUSH to the aggregator —
  is preserved **verbatim**. Depth is added as a **parallel, additive**
  path; no existing RGB node, queue, codec mapping, keyframe detection,
  header builder, or CLI flag is removed or renamed.
- The three RGB codecs (`jpeg`, `h264`, `h265`) and their
  `pixel_encoding` (`RGB8`, `H264`, `H265`) keep working unchanged. The
  existing `CODEC=${CODEC:-jpeg}` env knob in `run_all.sh` keeps working.
- The dashboard RGB decode path (`zmq_source.py` `decode_frame` →
  `JetsonJpegDecoder` / `JetsonDecoder`, `FrameImageProvider.publish_rgb`,
  `image://frames/<camera>`) is **not** modified.
- Depth is added behind a `--no-depth`-style opt-out and the default
  RGB-only behavior is preserved if depth is disabled. Depth streams on
  **new** channels (`v1/camera/<name>/depth`, `.../camera_info`); it never
  reuses or shadows the `.../rgb` channel.

The only change to shared code in `oak_capture.py` that touches the RGB
path is migrating the deprecated `ColorCamera` to the v3 `Camera` node
(required by the Luxonis skills); this migration must be done such that
the color sensor's 1080p resolution, FPS, 3A/EV/brightness/contrast
controls, and encoder output are byte-for-byte equivalent in behavior.
If the v3 `Camera` migration risks any RGB regression, the fallback is to
keep `ColorCamera` for the RGB sensor (it still works in depthai 3.1.0)
and use the v3 `Camera` node **only** for the two new mono sensors — RGB
correctness outranks the deprecation cleanup. This fallback is explicitly
allowed and preferred over breaking RGB.

---

## What already exists (do not redo)

The following pieces are already implemented and will be reused:

- `canonical_zmq/harvester_telemetry_contract/protocol.py` already
  defines `v1/camera/cutter/depth`, `v1/camera/docking/depth`,
  `v1/camera/cutter/camera_info`, `v1/camera/docking/camera_info` with
  the right codec and capability flags.
- `canonical_zmq/canonical_zmq_publisher/ingest.py:synthetic_depth_payload`
  / `synthetic_camera_info` already produce valid canonical packets for
  testing.
- `harvester_dashboard/harvester_dashboard/decoders/depth_decoder.py`
  decodes `depth_uint16_le` to float32 metres, and `DepthDecoder.depth_at`
  already does a nearest-valid-window lookup for the click path.
- `harvester_dashboard/harvester_dashboard/image_provider.py:colorize_depth`
  / `publish_depth_colored` already build a depth heatmap and serve it
  via `image://frames/<camera>_depth`.
- `harvester_dashboard/harvester_dashboard/zmq_source.py:SocketDrainer.decode_frame`
  already dispatches `/depth` channels to `decode_depth` (stateless, no
  extra wiring needed).
- `harvester_dashboard/harvester_dashboard/bridge.py:annotate_click` and
  `harvester_dashboard/harvester_dashboard/model/target_model.py:back_project`
  already do the click → depth → camera-frame 3-D point path using
  `camera_info` intrinsics, and the `Annotation.qml` crosshair already
  shows the camera-relative label.
- `harvester_dashboard/harvester_dashboard/projection.py:project_points`
  and `qml/LidarInset.qml` already provide a 3-D → 2-D scatter renderer
  (top/front/left/right/iso). The camera-relative point cloud will reuse
  the same projection math (camera optical frame convention) with a new
  panel.
- `harvester_dashboard/harvester_dashboard/decoders/lidar_decoder.py`
  shows the schema-driven point-field decoding pattern; a new
  `pointcloud_xyz_rgb_f32` decoder will follow it.

## What is missing (the actual work)

1. **OAK-D adapter** (`canonical_zmq/canonical_zmq_publisher/oak_capture.py`)
   currently creates only `ColorCamera` on `CAM_A` + `VideoEncoder`. It
   does not create `MonoCamera` left/right, `StereoDepth`, a depth output
   queue, a `camera_info` source, or emit any `/depth` or `/camera_info`
   packets. `capabilities` is hard-coded with `camera.<name>.depth: False`
   and `camera.<name>.camera_info: False`.
2. **Click depth path** is wired but has no live source. Once the OAK-D
   adapter emits depth + camera_info, this just works.
3. **Camera-relative point cloud panel**: no UI exists to render a
   depth-unprojected colored point cloud. The LiDAR inset exists, but
   its point convention is the world/vehicle frame; a new panel
   re-uses its Canvas/projection pattern with camera-optical frame
   conventions.

---

## Implementation

### 1. Extend the OAK-D adapter (`oak_capture.py`)

> **Luxonis skill constraint (mandatory).** Per
> `luxonis-build-poc` / `luxonis-troubleshoot`: in DepthAI v3,
> `ColorCamera` **and** `MonoCamera` are deprecated on **all** platforms
> — use the v3 `Camera` node. The current `oak_capture.py` still uses the
> deprecated `ColorCamera` (already flagged in `calibration/README.md`
> line 73). The two **new** mono sensors must use the v3 `Camera` node,
> and must source the **exact** v3 `Camera` + `StereoDepth` usage from
> `~/.luxonis/agent-context/oak-examples` (cloned from `main`, tracks v3)
> and/or `https://docs.luxonis.com/llms.txt` — **never from memory**. If a
> live example is not available at implementation time, the work pauses at
> that point rather than inventing the v3 API.
>
> **RGB-safety override:** the existing color `ColorCamera` is migrated to
> the v3 `Camera` node **only if** the RGB output stays equivalent
> (1080p/FPS/3A/EV/brightness/contrast/encoder). If any risk of RGB
> regression appears, keep `ColorCamera` for `CAM_A` (still functional in
> depthai 3.1.0) and use the v3 `Camera` node only for the mono `CAM_B` /
> `CAM_C` sensors. RGB correctness outranks the deprecation cleanup.

Verified local facts (depthai **3.1.0**, `/home/marcop/depthai-env/bin/python3`):

- `dai.node.Camera` exists (v3 replacement for `ColorCamera`/`MonoCamera`).
  It uses `initialControl`/`inputControl`, `setSensorType(...)`,
  `setBoardSocket(...)`, `requestOutput(...)`,
  `requestFullResolutionOutput(...)`, and exposes `raw`/`mockIsp` outputs.
- `dai.node.StereoDepth` exposes `left`/`right` inputs, `depth` /
  `disparity` / `rectifiedLeft` / `rectifiedRight` / `confidenceMap`
  outputs, `setDepthAlign(...)`, `setOutputSize(...)`,
  `setDefaultProfilePreset(...)`, `setLeftRightCheck(...)`,
  `setSubpixel(...)`, and `initialConfig`/`inputConfig`/`outConfig`.
- `StereoDepth.PresetMode` members are `FAST_ACCURACY`, `FAST_DENSITY`,
  `DEFAULT`, `FACE`, `HIGH_DETAIL`, `ROBOTICS` (v3 renamed the v2
  `HIGH_DENSITY`/`HIGH_ACCURACY` presets). `StereoDepth.Properties` has
  `depthAlignCamera`, `outWidth`, `outHeight`, `focalLengthFromCalibration`,
  `enableRectification`, `baseline`.

Pipeline additions (v3 `Camera` + `StereoDepth`):

- One v3 `Camera` node per **new mono sensor**: left mono on `CAM_B`,
  right mono on `CAM_C`, each configured with the v3
  `CameraConfig`/resolution + FPS via `requestOutput(...)` and
  `setSensorType(...)`. The exact v3 `Camera` config calls are taken from
  the `oak-examples` stereo/color example, not assumed. Mono resolution
  is the OAK-D-Pro-POE-FF native mono size (400P) and FPS matched to
  `self.fps`.
- The **color sensor on `CAM_A` stays exactly as-is** (existing
  `ColorCamera`, 1080p, `VideoEncoder` H.264/H.265/JPEG) unless the
  optional v3 migration is proven RGB-equivalent; see the RGB-safety
  override above.
- `dai.node.StereoDepth` wired `left.out → stereo.left`, `right.out →
  stereo.right`. `setDefaultProfilePreset(PresetMode.FAST_DENSITY)` (the
  v3 closest to "high density"; `--stereo-profile` overrides).
  `setDepthAlign(...)` to the color camera so the depth map is
  pixel-aligned to the RGB stream; `setOutputSize(self.width,
  self.height)`. `setLeftRightCheck(True)` + `setSubpixel(False)` for
  sane defaults.
- Consume only `stereo.depth` (a `depth_uint16_le` millimetre map) to
  keep CPU/bandwidth bounded; `rectifiedLeft`/`confidenceMap` are not
  consumed in this phase.
- `device.getCalibration()` is queried once on connect to build the
  `camera_info` JSON for the delivered depth size. The v3 calibration
  exposes rectified intrinsics for the requested output; map them into
  the existing `camera_info` JSON schema (k, d, r, p,
  `distortion_model: 'plumb_bob'`, `width`, `height`, `binning_x=1`,
  `binning_y=1`, `roi`). Cache per camera role. (Intrinsics are read
  from the device, never hard-coded — per `calibration/README.md`.)

New CLI flags (added to `_arguments`):

- `--stereo-profile {fast_density,fast_accuracy,high_detail,robotics,default}` (default `fast_density`).
- `--stereo-lr-check` (default on) / `--no-lr-check`.
- `--no-depth` to disable the `/depth` channel entirely (camera can
  still publish RGB + camera_info) so the existing low-CPU mode is
  preserved.

New header builders (mirror `build_rgb_header`):

- `build_depth_header(camera_role, width, height, acquisition_timestamp_ns, frame_id)`
  returning `codec='depth_uint16_le'`, `width`, `height`,
  `keyframe=True` (every depth frame is independently decodable; no
  stateful decoder).
- `build_camera_info_header(camera_role, width, height, frame_id)`
  returning `codec='json'`, `width`, `height`.
- Add a single helper `build_capabilities(camera_role, *, depth, camera_info)`
  so the existing `build_rgb_header` and the new builders share the
  same `capabilities` dict (and `camera.<name>.depth` / `camera.<name>.camera_info`
  flip to `True` when the OAK publishes them).

The `run()` loop becomes a small fan-out: one blocking `get()` on the
RGB queue + one non-blocking `tryGet()` poll on the depth queue. Depth
is the larger data path but is on the local OAK bus; we throttle depth
emission to `self.fps` (one per RGB frame) using the depth queue's
own timestamp. `camera_info` is emitted once on connect and on every
calibration refresh (the OAK calibration is static for one device), so
the adapter emits it once at startup and then on `--recalibrate` /
device reconnect.

### 2. Add a point-cloud channel (new codec, no contract break)

We add a new canonical codec by extending the contract header validator
and the dashboard decoder. The new channel is `v1/camera/<name>/pointcloud`
with codec `pointcloud_xyz_rgb_f32` (xyz float32 metres, rgb uint8
samples from the RGB stream at the same pixel). The OAK does not have
to compute this — the **dashboard** is the right place to unproject depth
using the live camera_info, because the dashboard already has the RGB
pixels in `latest_rgb(camera)`.

Wait — that mixes producer and consumer concerns. The contract is the
canonical wire, so the unprojected point cloud should be produced where
the OAK is reachable. In this project the OAK adapter runs in the
`depthai-env` python (which has numpy) and can do the unprojection
cheaply. Decision: **produce the point cloud in the OAK-D adapter** to
keep the wire contract uniform and let the dashboard just draw.

- Adapter: build a `Nx3` float32 `points` array and a `Nx3` uint8 `colors`
  array from the depth map and the latest RGB frame. Pack as one payload
  using a new codec `pointcloud_xyz_rgb_f32` declared in
  `protocol.py:_validate_image_header` (it is an image-side codec, not
  `lidar_xyz_f32`, because it has a `pixel_encoding` of `XYZ+RGB`).
- A new `PointcloudDecoder` mirrors `LidarDecoder` schema-driven decode
  (point fields, stride, point count).
- The dashboard's `image_provider` already gives us a `cutter` / `docking`
  RGB frame; the OAK adapter is the producer of choice here.

Actually, re-evaluating: the simplest, lowest-risk path is to keep the
**dashboard** as the only point-cloud producer (it already has RGB and
the live depth once both streams arrive). The canonical wire stays
RGB + depth + camera_info; the point cloud is a **render-time** view
derived in the dashboard from those three. This avoids extending the
contract and re-validating every consumer. **Adopt this approach.**

So the only new wire channels are `/rgb`, `/depth`, `/camera_info`. The
point cloud is a UI-only view derived in the dashboard.

### 3. Wire `/depth` and `/camera_info` through the aggregator

No aggregator changes: `_ingest_loop` already accepts any canonical
channel and re-sequences it; the existing bounded-queue newest-wins
policy already handles a 15 Hz depth stream at `queue_depth=2`. The
only change is that the aggregator's per-channel `drop_counts` will now
grow for `/depth` if the dashboard falls behind, which is the correct
behaviour (the errors panel already shows drops per channel).

The OAK adapter's capabilities dict must be updated to declare
`camera.<name>.depth=True` and `camera.<name>.camera_info=True`. The
existing `aggregator.capabilities` defaults are still
`False`; the dashboard reads capabilities from per-channel
`v1/system/status` payloads, which is correct.

### 4. Dashboard side

#### 4.1 `zmq_source.py`

No change. The existing `SocketDrainer.decode_frame` already dispatches
`/depth` to `decode_depth` (stateless). `camera_info` falls into
`JSON_CHANNELS` (`telemetry_model.py:JSON_CHANNELS` already includes
`v1/camera/cutter/camera_info` and `v1/camera/docking/camera_info`) and
is parsed into the `last_json` of the channel state, which is what
`model.snapshot_camera_info(camera)` already returns. The click path
already uses this.

#### 4.2 `bridge.py` (small additions)

- `latest_pointcloud(camera)` slot/property returning the current
  depth-unprojected point cloud + colors derived from
  `latest_rgb(camera)` + `latest_depth(camera)` + `snapshot_camera_info(camera)`.
- A new `@Slot(str, result='QVariantList') get_camera_pointcloud(camera)`
  exposes the unprojected colored cloud for the active camera.
- `latest_pointcloud` is recomputed on every depth frame, capped to
  `--pointcloud-max-points` (default 4096, uniform downsample using
  `np.linspace` like `LidarDecoder.limit`).

The unprojection math is in `decoders/pointcloud.py` (new, dependency-
free):

```python
def unproject_depth(depth_m, rgb, camera_info, max_points):
    """Back-project valid depth pixels to camera-frame XYZ + sample RGB."""
```

Returns `{'points': Nx3 float32 (m), 'colors': Nx3 uint8}` or
`{'points': [], 'colors': []}` when depth is missing. Drops zero /
NaN depth pixels. Uses the same `back_project` math as
`target_model.py`.

#### 4.3 `image_provider.py`

No structural change. `publish_depth_colored` already serves
`image://frames/<camera>_depth`. The new panel reads this URL.

#### 4.4 New QML: `qml/PointCloudInset.qml`

A new `Rectangle` panel (200x200, right side of the camera view) that:

- Mirrors `LidarInset.qml` exactly: Canvas scatter, project function
  for camera-optical frame (X right, Y down, Z forward — note this
  differs from the LiDAR inset's vehicle frame, see below).
- Renders the cloud from a new `bridge.cameraPointcloud` list (Nx3
  flattened as `[x, y, z, r, g, b]` for QML consumption).
- Renders the same crosshair / click target as a colored marker.
- The point convention is the **camera optical frame** (X image-right,
  Y image-down, Z forward through the lens), the same convention used
  in `calibration/README.md`. The projection helper in this QML file
  inverts Y for the screen up-axis (a small per-file projection, kept
  identical in shape to the LiDAR one).

The panel is hidden when `bridge.cameraPointcloud` is empty (no depth
yet) and fades in on the first valid frame.

#### 4.5 Wire the panel in `Dashboard.qml`

- Add `PointCloudInset` under the active `CameraView` (anchor right
  side, margin 8 px).
- A new bridge property `pointcloudVisible` (default `True`) toggled by
  key `6`.

#### 4.6 Annotation click behavior

No change required. The existing `annotate_click` already does
`DepthDecoder.depth_at()` + `back_project` and the existing
`Annotation.qml` shows the label. After this work, the depth is no
longer always `None`; the user sees a real depth (e.g. "2.45 m") and
camera-frame XYZ on click.

We add a one-line improvement: when the click is rejected for
`NO DEPTH`, show a red toast with the exact pixel location
(already implemented — just verify against the new error path).

### 5. `run_all.sh` and supervisor

No CLI change for `oak_capture` defaults. The existing
`CODEC=${CODEC:-jpeg}` env var keeps the existing knob. Add one knob:

- `DEPTH=${DEPTH:-1}` env var so depth can be turned off on
  resource-constrained runs by exporting `DEPTH=0`.

The launch command gains `--no-depth` (default off) and the OAK adapter
gains stereo pipeline construction guarded by the same flag.

### 6. Synthetic source

`canonical_zmq/canonical_zmq_publisher/ingest.py:SyntheticSource.emit_once`
already emits `v1/camera/cutter/depth` and
`v1/camera/cutter/camera_info`. It does **not** currently emit
`v1/camera/docking/depth` or `v1/camera/docking/camera_info`. Add
those two so the `--synthetic` path tests the full pipeline
(including the new panel) without OAK hardware.

### 7. Tests

Add to `canonical_zmq/test/test_oak_capture.py`:

- `test_build_depth_header_is_canonical` — round-trips the new
  `build_depth_header` through `pack_message` / `unpack_message`,
  verifies `codec == 'depth_uint16_le'`, no `pixel_encoding` for depth
  channels (contract currently requires `pixel_encoding` for `/rgb`
  only; depth is a separate branch).
- `test_build_camera_info_header_is_canonical` — round-trips the JSON
  camera_info header.
- `test_depth_capabilities_flag` — `capabilities['camera.cutter.depth']`
  is `True` when the adapter is configured to publish depth.

Add to `harvester_dashboard/test/`:

- `test/test_pointcloud.py` — pure-Python test of
  `unproject_depth(depth_m, rgb, camera_info, max_points)`:
  - identity: a depth map of constant 1.0 m with a synthetic
    camera_info produces a cloud of distance 1.0 m from the optical
    center.
  - downsampling: a 16x16 depth map with 50 % valid pixels clamps the
    output to `max_points` (or fewer).
  - invalid depth = NaN / 0: those pixels are absent from the output.
  - missing rgb: returns points only, no crash.
- Extend `test/test_target_model.py` with a test that the existing
  `back_project` math matches the new `unproject_depth` math (same
  intrinsics, same depth → same XYZ).
- Extend `test/helpers.py` with a `docking_depth_packet()` and
  `docking_camera_info_packet()` for end-to-end tests.

### 8. Validation

End-to-end manual sequence (will be run after the work is merged):

1. `python3 scripts/validate_camera_time_sync.py --duration 60
   --require-synchronized` still passes (no change to time sync).
2. `PYTHONPATH=canonical_zmq /home/marcop/depthai-env/bin/python3
   -m unittest discover -s canonical_zmq/test -v` passes (new depth /
   camera_info header tests included).
3. `PYTHONPATH=harvester_dashboard /usr/bin/python3 -m unittest
   discover -s harvester_dashboard/test -v` passes (new pointcloud
   tests included).
4. Synthetic run: `PYTHONPATH=canonical_zmq
   /home/marcop/depthai-env/bin/python3 -m canonical_zmq_publisher.main
   --synthetic --synthetic-period-s 0.2` then
   `DISPLAY=:1 PYTHONPATH=harvester_dashboard /usr/bin/python3
   -m harvester_dashboard.main --pub tcp://127.0.0.1:5590`. The
   dashboard shows the depth heatmap, click on a pixel shows
   "2.0 m" depth + camera-frame XYZ, and the new point-cloud panel
   is populated.
5. Hardware run: `./run_all.sh foreground`. The OAK-D adapters now run
   the stereo pipeline alongside RGB; the depth stream arrives at
   `tcp://*:5590`; the dashboard click returns real depth.

### 9. Out of scope

- World-fixed pose fusion of the camera point cloud (the calibration
  README is the source of truth; the operator annotations remain
  camera-relative until `v1/pose/arm` exists — see
  `.kilo/plans/1787482633337-vio-arm-pose-marker-tracking.md`).
- The MID-360 LiDAR fusion into a single point cloud. That is a
  separate work item (see `mid360_publisher.py` / `examples/estimate_tree_height.py`).
- OAK-D S3 (solid-state, not Pro) variants — current calibration and
  pipeline config are for the OAK-D-PRO-POE-FF.
- Recording depth streams to the audit recorder. The recorder already
  records every canonical packet; depth is recorded as a side effect
  of turning it on with `--record-dir`. No new code is needed.

### 10. Files touched

| File | Change |
|---|---|
| `canonical_zmq/canonical_zmq_publisher/oak_capture.py` | Add v3 `Camera` mono nodes (CAM_B/CAM_C), `StereoDepth`, depth output queue, `build_depth_header`, `build_camera_info_header`, capability flags; emit depth + camera_info alongside RGB. RGB `ColorCamera`/`VideoEncoder`/codec path kept unchanged (or migrated to v3 `Camera` only if proven equivalent). |
| `canonical_zmq/test/test_oak_capture.py` | Add depth / camera_info header + capability tests. |
| `canonical_zmq/canonical_zmq_publisher/ingest.py` | Add docking-camera depth + camera_info synthetic packets. |
| `harvester_dashboard/harvester_dashboard/decoders/pointcloud.py` (new) | `unproject_depth(depth_m, rgb, camera_info, max_points) -> {points, colors}`. |
| `harvester_dashboard/harvester_dashboard/bridge.py` | `latest_pointcloud(camera)`, `cameraPointcloud` property, `pointcloudVisible` toggle, `get_camera_pointcloud` slot. |
| `harvester_dashboard/qml/PointCloudInset.qml` (new) | Mirror `LidarInset.qml` for the camera optical frame; render colored points + click marker. |
| `harvester_dashboard/qml/Dashboard.qml` | Mount the new panel; bind `pointcloudVisible` to key `6`. |
| `harvester_dashboard/test/helpers.py` | Add docking depth + camera_info packet helpers. |
| `harvester_dashboard/test/test_pointcloud.py` (new) | `unproject_depth` unit tests. |
| `harvester_dashboard/test/test_target_model.py` | Cross-check `unproject_depth` against existing `back_project`. |
| `harvester_dashboard/test/test_gui_acceptance.py` | Extend to assert the new `cameraPointcloud` is populated when depth + rgb + camera_info arrive. |
| `run_all.sh` | `DEPTH=${DEPTH:-1}` env knob. |

### 11. Risk + rollback

- **Risk**: running StereoDepth alongside the H.265 hardware encoder
  on the OAK-D-PRO-POE-FF may exceed the Myriad-X budget at 1920x1080.
  The pipeline already runs at `1920x1080`; the new mono cameras are
  set to `MONO_400_P` (640x400) and the depth output is resized to
  `self.width x self.height` so the on-device stereo runs at 640x400.
  Validate under `tegrastats` (CPU < 80 %, no thermal throttling) at
  15 FPS. If we see pressure, drop to 1280x720 RGB and re-test; the
  `--no-depth` knob is the in-stack fallback.
- **Risk**: the `v1/camera/<name>/camera_info` payload is JSON, parsed
  on every packet. The OAK adapter emits it once on connect and on
  recalibration, not per-frame, so the cost is negligible.
- **Rollback**: the new `--no-depth` flag turns the whole feature
  off without touching the RGB path. The point-cloud panel is purely
  a QML view bound to a bridge property; if depth is missing, the
  panel hides itself. The contract (`v1/camera/<name>/depth`,
  `v1/camera/<name>/camera_info`) is unchanged from the existing
  frozen schema, so no consumer is broken.
