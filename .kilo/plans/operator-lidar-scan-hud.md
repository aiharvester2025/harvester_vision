# Operator LiDAR Scan HUD — Full-Screen Overlay, Guided Scan → Estimate

> **Implemented.** This plan is the design record; the shipped behaviour differs
> from the draft in these ways, driven by later operator requirements:
>
> - Scan is **operator-driven with a 15 s default window** (`scan_seconds`,
>   admin-overridable) and three buttons: **SCAN** (LiDAR standby → normal, timer
>   zeroed), **STOP** (end early, keep data, estimate), **CANCEL** (discard,
>   standby). There is no `guiding` phase — SCAN goes straight to `scanning`, and
>   the crosshair shows while scanning.
> - The estimate runs on the **accumulated** cloud over the window, not the last
>   frame; a live point count with a sparse/filling/dense hint tells the operator
>   when to press STOP.
> - **STOP and the timeout both return the LiDAR to standby.** This requires the
>   dashboard to control the sensor, so `lidar_control.py` was added as the one
>   **opt-in, disabled-by-default** exception to the render-only rule (sends only
>   `{"enabled": bool}`, reachable from the three scan slots).
> - `scanPhase ∈ {idle, scanning, complete, no_data}`; a cloud that never arrives
>   goes `no_data` after a 5 s grace period (the sensor needs time to spin up).
>
> Tests: `test_scan_estimate.py`, `test_livox_imu.py`, `test_lidar_control.py`,
> `test_lidar_scan_overlay.py`, plus updates in `test_no_emit_proof.py`,
> `test_smoke_gui.py`, `test_gui_acceptance.py`, `test_hud_config.py`,
> `test_projection.py`, `test_lidar_capture.py`.

## Goal

Redesign the dashboard's LiDAR HUD into a **single full-screen operator overlay**
that the human operator drives during tree scanning, replacing the current small
right-column inset. The overlay:

1. **Fills the dashboard** and is **anchored at the dashboard centre**, so the
   camera image and the LiDAR point cloud are **spatially overlaid** (the trunk
   the operator sees through the camera is the trunk the cloud draws).
2. Shows **scanning guide instructions** while the scan is in progress
   ("RAISE/EXTEND/LOWER boom…", "hold still…", "scanning 5.0 s…").
3. When the scan completes, **hides the scanning instructions and shows the
   estimate values** (tree height, trunk-end/docking height, boom distance,
   boom angle, boom extension, plus the existing safety/range panels).
4. Is **hidden by default** and toggled by the existing LiDAR button / key `4`
   (and hide/unhide without disturbing the other HUD layers).

It also adds the **missing IMU compensation for the MID-360 point cloud**, reusing
the exact OAK-camera IMU stabilization contract that already exists
(`decoders/imustab.py`), so vibration does not smear the scan.

This is the Orin, human-operated counterpart of the ROS 2 / Gazebo automated
tree-scan and boom-planning work in `aiharvester2025/ros2_ws`. There is **no
simulation on the Orin**: the operator performs the scan by hand, and the HUD is
the safety guide. No actuator command is ever issued (render-only, as today).

## Verified current state

- `harvester_dashboard/qml/Dashboard.qml:136-172` — `CameraView` shrinks its right
  edge to `lidar_inset.left` when `bridge.lidarVisible`; `LidarInset` is a 300 px
  right column; `PointCloudInset` is a 260 px top-right box on the camera view.
  `hudVisible` gates `HudOverlay` (key `3`).
- `harvester_dashboard/qml/LidarInset.qml` — Canvas scatter, origin-centred,
  `range_limit_m 8.0`, five views (`top/front/left/right/iso`) with an explicit
  axis legend; point convention `+x forward / +y left / +z up`.
- `harvester_dashboard/qml/HudOverlay.qml` — MIXED warning, `SensorPanel`,
  trunk/calibration column, diagnostic stream errors. Hosts the operator panels.
- `harvester_dashboard/qml/SensorPanel.qml` — Boom (bottom-right), Docking Ranges
  (bottom-left), Dock Guidance (bottom-centre), Cutter Range + Cutter Guidance
  (cutter view). Bottom-row layout reserved so panels never overlap.
- `harvester_dashboard/harvester_dashboard/bridge.py` — one context property
  `bridge`; `toggle_lidar()` / `lidarVisible` (key `4`), `cycle_lidar_view()` /
  `lidarView` (key `5`), `toggle_pointcloud()` / `pointcloudVisible` (key `6`),
  `toggle_hud()` / `hudVisible` (key `3`), `toggle_operator_huds()` (key `2`),
  `toggle_diagnostic()` (777+Enter), `hudLayout` (config-driven panel layout),
  `lidarPoints` (≤`--lidar-max-points`, default 2000), `_set_lidar_points()` on
  `v1/lidar/raw`.
- `harvester_dashboard/harvester_dashboard/decoders/imustab.py` — **existing**
  OAK IMU stabilization: `gravity_to_rpy`, `tilt_delta_rotation`,
  `stabilize_points(points, current_rpy, reference_rpy)` (rotation-only about a
  slowly-tracked reference). The OAK cloud path already applies it in
  `bridge.latest_pointcloud` / `_stabilize_cloud`; the LiDAR path does **not**.
- `harvester_dashboard/harvester_dashboard/model/telemetry_model.py:30-31` —
  `JSON_CHANNELS` includes `v1/camera/{cutter,docking}/imu`; there is **no**
  `v1/imu/lidar` / `v1/lidar/imu` channel, and `bridge.on_json_packet` only
  handles `channel.endswith('/imu')` by camera name.
- `lidar/livox_source.py` — decodes the MID-360 IMU into
  `self._latest_quaternion` (`(x,y,z,w)`, gravity-leveling, yaw identity) and
  exposes it via `_orientation()`; `lidar/livox_capture` levels the cloud at the
  **producer** via `lidar/leveling.py` before publishing `v1/lidar/raw`.
- `.claude/skills/harvester-dashboard-hud/SKILL.md` — render-only invariant, Qt
  types only in `bridge.py`, QtQuick 2 primitives only (PySide2 5.14, no
  Controls2), `telemetry_model` stays Qt-free, known headless NVDEC failures.
- `docs/orin_canonical_zmq.md:85-131` — control map + "HUD layers" section.
- `README.md` — LiDAR on-demand operation, PTP (`livox_ptp`), 120° forward sector,
  `lidar/boom_kinematics.py` (`lidar_height_above_ground`), and
  `examples/estimate_tree_height.py` (raw cloud → absolute tree height and
  trunk-end height by combining leveling + boom kinematics).

## Refined idea (what I changed and why)

The proposal is sound; these refinements make it safe, testable, and consistent
with the existing architecture.

1. **Keep an explicit two-phase HUD state machine, not a "the overlay shows
   whatever is available" rule.** Introduce `scan_phase ∈ {idle, guiding,
   scanning, complete, no_data}` in the bridge. Phase `guiding`/`scanning` render
   instruction text; `complete` renders the estimate rows; `no_data`/`idle`
   render a "press SCAN" prompt. This is what makes "hide scan info, show
   estimates" deterministic rather than emergent.

2. **Never remove or hide the existing safety panels during a scan.** The
   Docking/Cutter guidance panels are safety-critical. The reserved bottom band
   must stay reserved while the LiDAR overlay is visible, so the full-screen
   *point-cloud* layer grows **into the free camera area only**, and the safety
   panels always paint on top. The LiDAR overlay is a background layer, not a
   replacement for the HUD.

3. **Anchor the cloud to the camera's *displayed image rect*, not the raw item
   rect.** `CameraView` already computes `display_x/y/w/h` for the letterboxed
   `PreserveAspectFit` image. The overlay must project through that same rect, or
   the "object visible through the camera is overlaid by the point cloud" promise
   silently breaks whenever the aspect ratio changes.

4. **Do not attempt pixel-level camera↔LiDAR fusion on the Orin.** The OAK and
   the MID-360 are different sensors with different mount points; the SDK
   extrinsic is not surveyed yet (`calibration/frames.deployment.template.json`
   is intentionally blank). The honest, buildable version is: draw the LiDAR
   cloud in the **camera-optical screen frame using a named, calibratable
   extrinsic**, with the extrinsic supplied from the calibration file and clearly
   labelled unverified until the commissioning survey is done. This is
   *overlay by projection*, not *sensor fusion* — the plan and the UI must say so.

5. **MID-360 IMU compensation must be a new canonical channel, not a refactor of
   the OAK path.** Add `v1/imu/lidar` (JSON, `attitude_rpy_rad` + `frame`) from
   `lidar_capture`, mirroring `v1/camera/*/imu`, and apply `imustab` to the LiDAR
   cloud in the bridge. Do not overload `v1/camera/*/imu` to carry two IMUs.

6. **Reuse the existing estimator math, do not fork it.** The estimate display
   consumes the *same* quantities the ROS 2 side publishes: tree height, docking
   height (`H − offset`), boom angle/extension, platform level, boom distance,
   and the `INFEASIBLE_DOCK_HEIGHT` flag. Put the closed-form math in a new
   Qt-free module (`lidar/scan_estimate.py` or dashboard `model/`), unit-tested
   against the `ros2_ws` acceptance numbers (`θ_b ≈ 1.172 rad`, `Σe ≈ 8.27 m`
   for `H=11.90`, `d_horiz=4.87`; `θ_b ≈ 44.45°`, `Σe ≈ 9.296 m`, docking error
   0.000 m for the deployment geometry).

7. **Be explicit that the Orin cannot port `LivoxViewer2`.** The viewer binary is
   x86_64-only; the Orin is aarch64. This dashboard is the Orin's viewer. The
   LiDAR settings learned on the laptop (frame-time as accumulation window,
   point size 1–2 px, FOV/ROI masking, `r.ScreenPercentage 100`) map onto
   `--lidar-max-points`, the projection zoom, and the capture sector.

## Design

### A. `v1/imu/lidar` — MID-360 IMU as a canonical channel

Producer (`canonical_zmq/canonical_zmq_publisher/lidar_capture.py`, using
`lidar/livox_source.py`):

- Publish a JSON packet on `v1/imu/lidar` at the LiDAR IMU rate (bounded, e.g.
  ~10–20 Hz publish cap) with:
  `attitude_rpy_rad`, `quaternion_xyzw`, `frame_id: 'mid360_link'`,
  `accel_ms2` (optional), and the usual `header` (timestamp provenance from the
  existing `timing_snapshot`, `clock_domain`, `time_quality`).
- Only publish while the LiDAR is enabled (respect the existing on-demand
  control channel), matching the OAK publisher contract.

Model/bridge (`harvester_dashboard`):

- Add `v1/imu/lidar` to `JSON_CHANNELS`.
- In `bridge.on_json_packet`, route `channel == 'v1/imu/lidar'` to a new
  `_on_lidar_imu(decoded)` that stores `(roll, pitch)` (from `attitude_rpy_rad`)
  and latches a reference on first sample — the **same** contract as
  `_on_imu` for cameras, but on separate state (`_lidar_imu_attitude`,
  `_lidar_imu_reference`).
- Apply stabilization in `_set_lidar_points()`: if `imuEnabled` (reuse the same
  key `7` toggle so there is one "stabilization" concept), call
  `imustab.stabilize_points(raw_points, current, reference)` before storing
  `_lidar_points`, then emit `lidar_points_changed`.
- Expose `lidarImuAttitudeLine` and `lidarImuActive` mirroring `imuAttitudeLine` /
  `imuActive`, and show them in the LiDAR overlay's diagnostic corner.

Frame/sign note to verify during commissioning: the MID-360 reports `+X forward /
+Y left / +Z up`, while `imustab` was written for the OAK **optical** frame
(`+Z forward`). The LiDAR path must convert the LiDAR IMU roll/pitch into the
convention `_rpy_to_rotation` expects (or use `lidar/leveling.py`'s
`level_from_tilt_rpy`-equivalent with the LiDAR frame). This is a **single
documented conversion constant/function**, unit-tested; getting it wrong tilts
the cloud instead of leveling it. Do not silently reuse the optical-frame
assumption.

### B. Full-screen LiDAR scan overlay (new QML)

New component `qml/LidarScanOverlay.qml`:

- **Geometry**: `anchors.fill: camera_view` (the free camera area only — see A/B
  layering below), so it inherits the camera's letterboxed `display_*` rect.
- **Projection**: lift `projection.py`'s math into a shared helper and add a
  **camera-optical view** (`+x image-right, +y image-down, +z forward`) in
  addition to the existing vehicle-frame views, so the cloud can be drawn in the
  camera's screen frame. Reuse the existing `LidarInset.project()` conventions
  for the vehicle views; add `project_camera(x,y,z, K, extrinsic, scale)` for the
  optical view.
- **Anchor at centre**: origin of the projection is the **image-rect centre**
  (`display_x + display_w/2`, `display_y + display_h/2`), not the item centre,
  guaranteeing the cloud rotates/scales about the same point the camera image is
  centred on.
- **Zoom**: keep a `range_limit_m` property (default larger than the inset's 8 m,
  e.g. 12–15 m for whole-tree scans) plus `+`/`-` on-screen buttons (render-only)
  and a `--lidar-hud-range-m` default.
- **View cycle**: reuse key `5` / `cycle_lidar_view()`; add a `camera` view to
  `_LIDAR_VIEWS` so the operator can overlay directly on the camera image.
  `projection.VIEWS`/`VIEW_LABELS`/`AXIS_LABELS` extend accordingly, with tests.
- **Drawing**: Canvas (QtQuick 2 primitive; no Controls2) drawing points as small
  sprites (1–2 px, matching the LiDAR guidance), range-coloured, with the
  existing axis legend reduced to a corner chip while scanning.

Layering in `Dashboard.qml` (order matters, bottom → top):

1. `CameraView` (full width now when the overlay is visible).
2. `LidarScanOverlay` (full camera area, visible when `bridge.lidarVisible`).
3. `HudOverlay` (operator HUD, gated by `hudVisible`) — safety panels always on
   top of the cloud.
4. Transient toast (top-most).

The old right-column `LidarInset` behaviour becomes the overlay's responsibility;
when `lidarVisible` is true the camera no longer shrinks — it stays full-frame
and the overlay paints over it. When `lidarVisible` is false, behaviour is
unchanged. `PointCloudInset` (OAK depth cloud) stays as-is, gated independently
(key `6`).

### C. Two-phase guidance overlay (scan guide → estimate)

Extend `hud_config.py` with a new panel, `lidar_scan`, alongside the existing
five, with the same override contract (`anchor`, `caption`, `width`, `height`,
fonts, `opacity`, `visible`, `rows`, `margin_px`) plus scan-specific fields:
`guide_font_px`, `estimate_font_px`, `zoom_default_m`, `zoom_min_m`,
`zoom_max_m`, `hold_still_*` thresholds. Ship defaults in
`harvester_dashboard/hud_config.json`.

Bridge state machine (new, render-only):

- `scanPhase` (`idle|guiding|scanning|complete|no_data`) and `scanGuideText`
  (the current instruction line), `scanProgress` (0–1), `scanCountdownS`,
  `scanEstimateRows` (list of `{key,label,value,valid}`), `scanQualityText`.
- Operator actions: `begin_scan()` (button / a key), `cancel_scan()`.
  `begin_scan` starts the guided sequence; the phase advances to `scanning`,
  then to `complete` when the estimator has enough valid geometry (or to
  `no_data` on timeout). This is driven by the bridge's 200 ms `refresh()` plus
  a monotonic deadline; it **never writes a socket**.

Phase content:

| Phase | Overlay content |
|---|---|
| `idle` | "LIDAR READY — press SCAN to start" + tips (aim at trunk, keep crown in view) |
| `guiding` | Step-by-step boom instruction: raise/extend/lower to framed targets; live "trunk in view?" indicator; "centre the trunk" crosshair guidance |
| `scanning` | Same guide plus `scanning… N.N s` countdown and point-count/coverage quality bar |
| `complete` | **Scan instructions hidden**; estimate rows shown (see D) + "press SCAN again to rescan" |
| `no_data` | "No usable geometry — check LiDAR enabled / aim at trunk" |

Guardrails: the scanning phase must not fabricate confidence. If the session has
not produced a usable cloud (LiDAR disabled, all points out of sector, cloud
count below a floor), the phase is `no_data`, never `complete` with dash values
presented as measurements. Mirror the docking-guidance rule that a stalled
stream must not render a reassuring state.

### D. Estimate computation and rows (reuse the ros2_ws math)

New Qt-free module (dashboard `model/scan_estimate.py`, pure numpy) computing,
from the leveled cloud + boom state + the trunk docking sensor:

- `trunk_axis` — median XY in a mid-trunk band (encoder-free, as validated in
  `analyze_tree_scan_v2.py`: `(8.35, 0)` / `(8.28, 0)` vs true `(8.5, 0)`).
- `trunk_top_z` — **highest point within a tight cylinder** about the axis
  (`r < 0.35 m`), which beat the canopy-99th-percentile estimate on the
  simulation (`~11.83–11.93 m` vs `12.0 m`).
- `crown_base_z` — first histogram bin where the canopy annulus exceeds the
  density threshold (cross-check; ~9.0–9.25 m vs `9.2 m`).
- `docking_height_m` — `trunk_top_z − docking_offset_below_trunk_top_m`
  (deployment decision: **2.0 m** below trunk top → `H_dock = 10.0 m` for the
  reference tree).
- `boom_angle_rad`, `boom_extension_total_m`, `platform_level_rad`,
  `boom_horizontal_distance_m`, `docking_lower_angle_rad` — closed-form IK from
  the URDF constants (`h_p`, `L_fixed`, `L_plat`, stroke), with the **as-built
  corrections** from `ros2_ws`: `leveling = +θ_b`, `BOOM_PIVOT_WORLD_Z = 1.81 m`.
- `feasible` + `deficit_m` — `INFEASIBLE_DOCK_HEIGHT` when
  `L_needed > L_fixed + 9.6`.

`scanEstimateRows` renders (from `hud_config.lidar_scan.rows`):

| key | default label |
|---|---|
| `tree_height_m` | Tree Height |
| `trunk_end_height_m` | Trunk End Height |
| `docking_height_m` | Docking Height |
| `boom_angle_deg` | Boom Angle |
| `boom_extension_m` | Boom Extension |
| `platform_level_deg` | Platform Level |
| `boom_distance_m` | Boom Distance |
| `docking_lower_deg` | Docking Lower Angle |
| `status` | Status (`READY` / `INFEASIBLE_DOCK_HEIGHT` / `NO DATA`) |

These are **operator safety guidance**, explicitly advisory: the HUD must state
"advisory only — verify before moving" and must never imply the estimate is a
measured contact. The measured contact remains the five docking-range sensors.

### E. Keyboard / button map (render-only)

- Key `4` / "4 LiDAR" toolbar button: toggle the full-screen LiDAR scan overlay
  (existing `bridge.toggle_lidar()`), hidden by default.
- Key `5` / "5 View": cycle projection views, now including the camera-overlay
  view (existing `cycle_lidar_view()`).
- New on-screen "SCAN" button inside the overlay: `bridge.begin_scan()`
  (render-only; starts the guide/estimate state machine).
- New on-screen "RESCAN"/"CANCEL": `bridge.cancel_scan()`.
- Key `7` keeps its single meaning: IMU stabilization A/B — now covering **both**
  the OAK cloud and the LiDAR cloud.
- Keys `1`,`2`,`3`,`6`,`0/Esc`, and `777+Enter` keep their existing meanings.

## Files to change

1. **New** `harvester_dashboard/qml/LidarScanOverlay.qml` — full-screen scan
   overlay (Canvas, two-phase content, zoom/view/scan buttons).
2. **New** `harvester_dashboard/harvester_dashboard/model/scan_estimate.py` —
   Qt-free estimator + IK (numpy only).
3. **New** `harvester_dashboard/harvester_dashboard/decoders/livox_imu.py` (or a
   documented conversion helper in `decoders/imustab.py`) — LiDAR-frame →
   `imustab`-frame roll/pitch conversion.
4. `harvester_dashboard/qml/Dashboard.qml` — stop shrinking `CameraView` for
   `lidarVisible`; add `LidarScanOverlay` between `CameraView` and `HudOverlay`;
   keep `LidarInset` only if retained as a small diagnostic thumbnail (decision:
   remove it and route key `4` to the overlay).
5. `harvester_dashboard/qml/projection` (via `harvester_dashboard/harvester_dashboard/projection.py`)
   — add the `camera` view + `project_camera` helper; extend `VIEWS`,
   `VIEW_LABELS`, `AXIS_LABELS`.
6. `harvester_dashboard/harvester_dashboard/bridge.py` — `lidar_scan` layout
   accessors; scan state machine (`scanPhase`, `scanGuideText`, `scanProgress`,
   `scanEstimateRows`, `scanQualityText`, `lidarImuAttitudeLine`,
   `lidarImuActive`); slots `begin_scan`, `cancel_scan`; LiDAR IMU routing +
   stabilization in `_set_lidar_points`; add `camera` to `_LIDAR_VIEWS`.
7. `harvester_dashboard/harvester_dashboard/hud_config.py` — `LIDAR_SCAN` panel
   name, dataclass fields, defaults, merge, `to_qml`.
8. `harvester_dashboard/hud_config.json` — shipped `lidar_scan` block; **remove
   the pasted prose at lines 1–9** (the file is JSON and currently starts with
   the idea text before the object, which the strict `parse_constant` loader will
   reject — this is a real latent failure worth fixing in the same change).
9. `harvester_dashboard/harvester_dashboard/config.py` — add
   `--lidar-hud-range-m` (default overlay zoom), `--docking-offset-below-top-m`
   (default 2.0), and the boom-constant overrides if not read from the URDF file.
10. `harvester_dashboard/harvester_dashboard/model/telemetry_model.py` — add
    `v1/imu/lidar` to `JSON_CHANNELS`.
11. `canonical_zmq/canonical_zmq_publisher/lidar_capture.py` — publish
    `v1/imu/lidar` (bounded rate, enabled-gated).
12. `canonical_zmq/harvester_telemetry_contract/protocol.py` — register the
    `v1/imu/lidar` channel/schema (mirroring `v1/camera/*/imu`).
13. `docs/orin_canonical_zmq.md` — controls, HUD layers, new `lidar_scan` config
    block, `v1/imu/lidar` provenance.
14. `.claude/skills/harvester-dashboard-hud/SKILL.md` — new overlay, new keys,
    `v1/imu/lidar`, scan state machine, layered-painting invariant.
15. `README.md` — MID-360 IMU compensation, Orin scan workflow, advisory-only
    estimate note, and the x86-Viewer2/Orin note.
16. `calibration/README.md` + `calibration/frames.deployment.template.json` —
    document the camera↔LiDAR extrinsic required by the overlay (survey pending).

## Tests to add / update

- `harvester_dashboard/test/test_scan_estimate.py` (new) — trunk axis, tight-
  cylinder trunk top, crown-base transition, `H_dock = H − 2.0`, IK against the
  `ros2_ws` acceptance numbers (`θ_b≈1.172 rad`, `Σe≈8.27 m`; and the as-built
  `44.45°`/`9.296 m`/0.000 m docking error), `INFEASIBLE_DOCK_HEIGHT` firing.
- `harvester_dashboard/test/test_imustab.py` (extend) — LiDAR-frame conversion
  and `stabilize_points` on a LiDAR-convention cloud (rotation-only, distances
  preserved, identity at reference).
- `harvester_dashboard/test/test_hud_config.py` (extend) — `lidar_scan` panel
  defaults + override; and a regression test that the shipped
  `hud_config.json` **parses** (guards against the prose-prefix bug).
- `harvester_dashboard/test/test_no_emit_proof.py` (extend) — `begin_scan`,
  `cancel_scan`, `cycle_lidar_view` (incl. `camera`), `toggle_lidar` write zero
  socket traffic; `scanPhase` transitions.
- `harvester_dashboard/test/test_model.py` (extend) — `v1/imu/lidar` ingest.
- `harvester_dashboard/test/test_smoke_gui.py` / `test_gui_acceptance.py`
  (extend) — overlay visibility by key `4`, phase text swap (guide → estimate),
  camera view in the cycle.
- `canonical_zmq/test/test_lidar_capture.py` (extend) — `v1/imu/lidar`
  published only while enabled; schema round-trip.
- Confirm the two known pre-existing NVDEC JPEG failures remain the only
  failures.

## Verification

```bash
cd ~/harvester_vision
PYTHONPATH=harvester_dashboard /usr/bin/python3 \
  -m unittest discover -s harvester_dashboard/test -v
QT_QPA_PLATFORM=offscreen PYTHONPATH=harvester_dashboard /usr/bin/python3 \
  harvester_dashboard/test/test_gui_acceptance.py
# canonical producer
PYTHONPATH=canonical_zmq:. python3 -m canonical_zmq_publisher.lidar_capture \
  --sdk-mode synthetic --enabled     # expect v1/lidar/raw AND v1/imu/lidar
```

Manual (with display): start the dashboard, `ROTATION`/synthetic source; press
`4` → full-screen overlay centred on the camera; press SCAN → guide text; after
completion → instructions hide, estimate rows appear; toggle `7` → cloud
de-jitters; safety panels remain painted on top throughout.

## Risks / open decisions

1. **Camera↔LiDAR extrinsic is unsurveyed.** The overlay is only spatially
   correct once the deployment extrinsic is measured. Ship it as a named
   calibration entry with an "unverified" badge; the operator must not treat the
   overlay as a calibrated measurement. This is the single biggest correctness
   risk in the idea and must not be papered over.
2. **Yaw.** The MID-360 IMU cannot observe yaw; leveling is rotation-only and
   yaw-agnostic by design (`lidar/leveling.py`). The overlay must be explicit
   that horizontal alignment uses the camera mount / operator framing, not IMU
   yaw.
3. **Host CPU.** The dashboard already runs near CPU limits with two OAK streams;
   a 2000-point Canvas redraw at frame rate is heavy. Cap the overlay redraw
   (e.g. 10–15 Hz) and only when visible, matching the existing
   active-camera-only decode discipline. The full-screen overlay is *larger* than
   the old 300 px inset, so this cap is load-bearing.
4. **Removing `LidarInset.qml`** vs keeping it as a thumbnail. Removing is
   cleaner (one surface) but is a bigger change; if kept, key `4` should drive
   the overlay and the inset should be diagnostic-only. Decision needed before
   implementation.
5. **`hud_config.json` currently has the idea prose pasted before the JSON
   object** — the loader will `SystemExit`. Fixing it is folded into this change;
   flagging it because it means the shipped HUD config may already be failing to
   load on the Orin.
6. **Estimate vs measurement boundary.** The estimate rows are drawn from one
   scan and advisory IK. Keep the existing five-range measured docking safety as
   the authoritative contact guard; the overlay must state this in one visible
   line.

## Out of scope

- Writing any actuator/joint/PLC command (render-only, unchanged).
- Porting `LivoxViewer2` to the Orin (x86_64-only; this dashboard is the viewer).
- Full sensor fusion / pixel-accurate camera-LiDAR registration.
- Changes to the ROS 2 `ros2_ws` packages (this plan consumes their math/shape,
  not their code).
- Live hardware tuning, which is only possible once the machine is built.
