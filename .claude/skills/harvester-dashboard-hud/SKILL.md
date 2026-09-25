---
name: harvester-dashboard-hud
description: "Extend the harvester_vision Qt Quick dashboard: surface a channel through the model/bridge/QML, or change HUD/overlay visibility and keyboard toggles."
disable-model-invocation: true
argument-hint: "the HUD element to add/change, or the channel to surface"
metadata:
  author: harvester
  version: "2.0.0"
  status: stable
---

# Harvester Dashboard HUD

Extend or modify the harvester's source-agnostic Qt Quick dashboard (the "operator HUD"):
surface a new telemetry channel, add a HUD panel, or change visibility/keyboard toggles.

The dashboard is under `harvester_dashboard/`, driven by a single QObject context property
`bridge`. The render path is **wire channel → `TelemetryModel` → `DashboardBridge` Qt property →
QML**, and it is strictly **render-only** (no socket writes from view/toggle controls).

## Layered architecture

- **Model** `harvester_dashboard/harvester_dashboard/model/telemetry_model.py` — pure-Python,
  Qt-free. `TelemetryModel` holds one `StreamState` per channel; JSON channels listed in
  `JSON_CHANNELS`. Add a `snapshot_<x>()` accessor returning `self._json_of('<channel>')`.
- **Bridge** `.../harvester_dashboard/bridge.py` — `DashboardBridge(QObject)`. Add a
  `_get_<x>_line()` method, a `Signal`, a `Property(...)` binding, and emit the signal in
  `refresh()`.
- **QML** `harvester_dashboard/qml/`:
  - `Dashboard.qml` — root layout + keyboard handling + toolbar.
  - `HudOverlay.qml` — MIXED-SOURCES warning, the operator sensor panels host
    (`SensorPanel`), right trunk/calibration, bottom stream-errors panel
    (diagnostic-gated).
  - `SensorPanel.qml` — hosts the five config-driven operator panels in Loaders,
    anchored from `bridge.hudLayout`: **Boom** (bottom-right; PLC MQTT values:
    boom angle, boom length, slew angle, platform tilt X1/Y1, prime mover tilt
    X2/Y2), **Docking Ranges** (bottom-left, + phase guide), **Cutter Range**
    (cutter view only), **Docking Safety** (bottom-center, docking view only),
    and **Cutter Safety** (bottom-center, cutter view only).
  - `SensorHudPanel.qml` — generic panel (caption + caption/value rows) driven by
    a `bridge.hudLayout.<name>` config object and a rows model.
  - `DockGuidanceHud.qml` — docking safety-guidance panel (state banner,
    stop-bar, metrics), driven by `bridge.hudLayout.dock_guidance` and the
    `bridge.dock*` properties (see `safety_guidance.py`).
  - `CutterGuidanceHud.qml` — cutter safety-guide panel (cut-step banner,
    clearance alert, clearance bar, metrics, CONFIRM STEP prompt), driven by
    `bridge.hudLayout.cutter_guidance` and the `bridge.cutter*` properties (see
    `cutter_safety_guidance.py`).
  - `CameraView.qml` — camera image (image://frames), stale ring + timestamp line
    (diagnostic-gated), click annotation, crosshair.
  - `LidarScanOverlay.qml` — **full-screen** LiDAR scan overlay: draws the cloud
    over the camera image (centred on the camera's displayed image rect), shows
    guided scan instructions while scanning, then swaps to the estimate rows on
    completion. Toggled by key `4` (hidden by default). Created BEFORE
    `HudOverlay` in `Dashboard.qml`, so the safety panels always paint on top —
    the overlay is a background layer, not a replacement for the HUD.

    The information sits in **three short cards pinned to the bottom edge** so
    the cloud above them stays visible (one tall card used to block the trunk):
    scan state / guidance (bottom-left), **TREE ESTIMATE** (bottom-centre) and
    **BOOM / DOCK ESTIMATE** (bottom-right). The two estimate cards are hidden
    until `complete`, and split the rows by the `group` field the estimator sets
    (`tree` / `boom`) via `rowsInGroup(name)` — data-driven, not hardcoded keys.

    **Exclusive scan mode.** While the overlay is up, `bridge.lidarScanActive`
    (= `lidarVisible`) hides the unrelated HUD: `Dashboard.qml` hides
    `HudOverlay` + `PointCloudInset`, and every `SensorPanel.qml` Loader's
    `active` adds `&& !bridge.lidarScanActive`. Gating the *Loader active* (not
    just `visible`) matters — a hidden Loader stays loaded and its panel keeps
    rendering; deactivating frees it. This is the "exclusive view" rule, so keep
    the new loader's `active` condition in sync when adding one.
  - `PointCloudInset.qml`, `Annotation.qml` — inset overlays.

## Two-layer HUD (operator vs. diagnostics)

The HUD is split into two independently-toggled layers:

- **Operator HUD** (key `3` → `bridge.hudVisible`): phase guide, trunk +
  calibration, MIXED-SOURCES warning, source badge.
- **Operator sensor panels** (key `2` → `bridge.operatorHudsVisible` + key `1`
  → `bridge.cutterHudVisible`): Boom + Docking Ranges on the docking camera, and
  the Cutter Range panel on the cutter camera. Key `1` switches to cutter and
  shows only the Cutter HUD (Boom/Docking hide); pressed again on cutter it
  hides/shows the Cutter HUD. Key `2` toggles Boom+Docking on the docking
  camera, and on the cutter camera it returns to docking, hides the Cutter HUD,
  and shows Boom+Docking. Layout/captions/sizes are admin-set via `--hud-config`
  (see `harvester_dashboard/hud_config.py`).
- **Docking safety guidance** (`bridge.dock*`, docking view only): an advisory
  speed/gap envelope over the `center_line` range record. `safety_guidance.py`
  (pure Python) maps (closing speed, gap) onto `safe`/`warn`/`danger`/`no_data`
  using the stopping-distance + TTC model. `bridge.py` derives the closing speed
  (`-d(gap)/dt` least-squares slope over a 1.5 s window, EMA-smoothed), applies
  the staleness TTL and hysteresis debounce, and exposes `dockSafetyState/
  dockGuidanceText/dockSafetyRow/dockSpeedCmS/dockSpeedSmoothedCmS/
  dockCenterDistanceM/dockStopDistanceM/dockMaxSpeedCmS/
  dockRecommendedSpeedCmS/dockTtcS`. Thresholds come from `--safety-config`
  (`config/safety_guidance.json`). `DockGuidanceHud.qml` renders it; its layout
  is the `dock_guidance` panel in the same `--hud-config` file. It is advisory
  only and render-only.

  Two invariants worth protecting: (1) the HUD must **never** show `safe` when
  the range is absent, stale, non-finite, or has no fittable closing speed — a
  stalled stream must not invent a 0 cm/s speed and flip DANGER to a reassuring
  green (`evaluate` takes `speed_cm_s=None` → `no_data`, and the bridge holds a
  WARN/DANGER while the gap is fresh but unfittable); (2) the displayed message
  must never contradict the shown state (a WARN banner always leads with
  `SLOW`). **A non-finite (`NaN`/`Inf`) gap or speed is `no_data` at the model
  level** — `NaN` compares false against every threshold, so an unguarded `NaN`
  would fall through all DANGER/WARN tests and render a false-green `SAFE`; guard
  finiteness in `evaluate()`, not only at the ingest boundary. `SafetyConfig.load`
  falls back to defaults for a missing file (valid), but **logs a warning** for an
  existing file that is unreadable/malformed, so a broken tuning file is not
  mistaken for an applied one. The `NO_DATA`/`SAFE` message text has a **single
  definition** (`_no_data_message`/`_safe_message`), used by both `evaluate()`
  and `state_message()`, so a debounced hold can never display text that
  disagrees with the banner colour; keep them shared rather than re-inlining the
  strings. The `Docking Ranges` panel rows for the two
  ultrasonic side sensors are labelled `Left`/`Right`
  (`c_channel_left`/`c_channel_right`).

- **Cutter safety guidance** (`bridge.cutter*`, cutter view only): the sibling
  of the docking guidance, for the cutting arm. A pure-Python model
  (`cutter_safety_guidance.py`) applies the sensor→tip offset
  (`tip_clearance = raw_range − sensor_to_tip_offset_m`, ~0.19 m; the sensor sits
  behind the tip) and maps (closing speed, clearance) onto
  `safe`/`warn`/`danger`/`no_data` with the same stopping-distance scheme, plus a
  cut-sequence phase machine (`idle → approach → align → open → advance → cut`).
  `bridge.py` derives the closing speed from the `cutter_forward` record on
  `v1/range/cutter` (least-squares slope over ~1.5 s, EMA-smoothed), tracks
  staleness, advances the measured `approach → align` step, applies the debounce,
  and exposes `cutterSafetyState/cutterPhase/cutterGuidanceText/cutterPhaseText/
  cutterClearanceM/cutterRawRangeM/cutterSpeedSmoothedCmS/cutterMaxSpeedCmS/
  cutterStopDistanceM/cutterTtcS/cutterCanConfirm/cutterSafetyRow`. Thresholds
  come from `--cutter-config` (`config/cutter_safety_guidance.json`).
  `CutterGuidanceHud.qml` renders it; its layout is the `cutter_guidance` panel
  in the same `--hud-config` file. Advisory only and render-only. It is gated on
  the same key-1 cutter HUD flag (`bridge.cutterHudVisible`) as the Cutter Range
  panel, so key 1 hides/shows both cutter-view HUDs together — keep those two
  loaders' `active` conditions in sync.

  Placement: on the cutter view the Cutter Range panel is positioned by its own
  configured `anchor` (default bottom-left, but movable to either edge or the
  center via `--hud-config`); the guidance panel is positioned per its own
  `anchor` inside the reserved free band. A `center` anchor centers it on the
  **screen** (not in the leftover gap, which would sit visibly off-center), and
  the panel is shrunk — then hidden — rather than allowed to overlap the Cutter
  Range HUD. Keep `layoutCutterRow` reserving whichever side the range panel is
  anchored to, and hide when a centered range would collide (see the invariants
  below). The docking-view bottom row (`layoutBottomRow`) is unchanged.

  Invariants worth protecting: (1) the phase may enter `align` ("STOP — ready to
  cut") **only** when the tip is inside the ready band, settled
  (`speed ≤ warn_margin·v_max`), **and** not inside the danger floor. The ready
  band sits inside `warn_clearance` by design (the tip settles in a stable WARN
  "ready-to-cut" state), so `align` legitimately coincides with `warn` but must
  **never** coincide with `danger`/`no_data` — keep the danger-floor guard in
  `next_phase`; (2) `align → open → advance → cut` are operator-confirmed
  via the bridge slot `cutter_confirm_phase()`, which is a **no-op while
  DANGER/NO_DATA** (the CONFIRM STEP button is disabled and relabelled "WAIT —
  tip too close" for danger, "WAIT — no range" for no_data); (3) a range dropout
  **holds** an in-progress operator phase rather than resetting it; (4) `no_data`
  is grey and never a false green, and non-finite ranges are `no_data` at the
  model level; (5) the cutter guidance loader must share key-1's
  `bridge.cutterHudVisible` gate with the Cutter Range loader, and the cutter row
  layout must reserve space on whichever side the Cutter Range panel is anchored
  (it can be moved to the right or center via `--hud-config`) and hide rather than
  overlap — a centered range panel confines the guidance panel to a free side band
  and hides it if the guidance is also centered.
  The depth camera/LiDAR do **not** feed the clearance (they are on the arm base,
  off the extension) — never imply fusion.

- **Developer-diagnostic HUD** (`7` `7` `7` + `Enter` → `bridge.diagnosticVisible`): bottom
  stream table, toolbar status line, active-camera timestamp line, stale-camera ring.

Both toggles are **render-only** — they never write to a telemetry/control socket (the safety
boundary). Gate each diagnostic element in QML on `bridge.diagnosticVisible` (see
`HudOverlay.qml` errors_panel, `CameraView.qml` stale ring + timestamp, `Dashboard.qml` status
line).

## Keyboard sequences without breaking single-key shortcuts

`Dashboard.qml` uses a `key_buffer` + 800 ms flush `Timer`. Only the **ambiguous** key is buffered:

- Buffer **only `7`** (ambiguous with the single-key `7` IMU toggle). All other keys (`1`–`6`,
  `0`) act immediately.
- The flush timer replays a lone buffered `7` as `toggle_imu()` after ~800 ms so the single-key
  shortcut still works; `777` + `Enter` commits `toggle_diagnostic()`.

Buffering **every** digit would add an 800 ms delay to all primary controls — do not do that.

Full key map (render-only unless noted): `1` cutter view / toggle Cutter Range
HUD, `2` toggle Boom+Docking HUDs (on cutter: back to docking + show them), `3`
operator HUD, `4` LiDAR scan overlay (full-screen, hidden by default), `5` LiDAR
view cycle (opens in the **front (x-z)** working view; cycles
front → left → right → iso → camera → top, and includes the `camera` overlay
view), `6` point cloud, `7` IMU
stabilization A/B (covers **both** the OAK cloud and the MID-360 cloud), `0`/`Esc`
clear annotation, `777`+`Enter` diagnostic layer, click = annotate (depth +
camera-frame XYZ). The overlay also has on-screen SCAN/STOP/CANCEL/VIEW/ZOOM
buttons. Toolbar buttons show the `#4fc3f7` blue outline while their layer is
active (LiDAR button keys on `bridge.lidarVisible`).

## Operator LiDAR scan HUD (`lidar_scan`)

The full-screen overlay is driven by a **state machine** in `bridge.py`:
`scanPhase ∈ {idle, scanning, complete, no_data}`. The operator presses SCAN
(`bridge.begin_scan()`), which puts the LiDAR into normal mode, zeroes the timer,
clears the accumulated buffer, and enters `scanning`; `refresh()` (200 ms)
advances it via `bridge.advance_scan()`. The scan ends when:

- the operator presses **STOP** (`bridge.stop_scan()`) — ends early, keeps data; or
- `scan_seconds` (default **15 s**, admin-overridable) elapses.

Both paths return the LiDAR to **standby**, keep the accumulated cloud, and run
the estimate once. **CANCEL** (`bridge.cancel_scan()`) discards the data, zeroes
the timer, and stands the LiDAR down without estimating. Only `complete` renders
`scanEstimateRows` — the instructions and the estimates never render together. A
missing/empty/stale cloud is `no_data`, never a confident-looking `complete` (the
same "never a false green" rule the docking guidance uses).

- **Accumulated buffer.** Points are accumulated across the window
  (`_scan_accumulated`, capped at `_SCAN_ACCUM_MAX_POINTS`), and the estimate runs
  on the accumulation — not the latest frame — so a long window genuinely
  densifies the measurement. `scanQualityText` shows a live point count +
  sparse/filling/dense hint so the operator can judge when to press STOP.
- **LiDAR control is the ONE render-only exception.** `lidar_control.py` PUSHes
  only `{"enabled": bool}` to the producer's PULL endpoint, is **disabled by
  default** (`--lidar-control`, off ⇒ dashboard stays render-only), and is wired
  in `main.py` (also sends `enabled: false` on exit). `bridge.lidarControlEnabled`
  tells the QML whether the buttons actually control the sensor; when false the
  overlay says so on screen. It never touches the boom/platform/PLC — the LiDAR
  is a sensor, not an actuator.
- Estimator: `model/scan_estimate.py` (Qt-free, numpy) — the Orin counterpart of
  the validated `ros2_ws` `harvester_dock/live_height_estimator.py`:
  - **Trunk axis** must be radius-corrected for the single-sided (half-cylinder)
    scan: `median_x + (2/pi)*trunk_radius`.  Without it the axis is biased ~0.22 m
    toward the sensor (recording check: raw median 8.283 -> 8.506 vs true 8.5).
  - **Docking height comes from the CROWN BASE (trunk-end), NOT the tree top:**
    `H_dock = crown_base - docking_offset_below_trunk_top_m` (~7.2 m for the
    reference tree).  The legacy `tree_top - 2.0` = 10.0 m lands inside the
    frond/FFB zone (fronds ~9.45 m, FFBs ~9.55 m) and is why the platform crashed
    into them.  When the crown-base detector fails the target is `NO DATA` — never
    the unsafe fallback.
  - **Crown base** is the canopy-annulus density jump (0.25 m bins, fixed
    `z_min = 5 m` above harvester self-clutter).  The threshold is
    `max(crown_density_threshold, ratio * trunk-clutter median)` so it adapts to
    the sweep density (a dense 40-step recording has ~28k canopy pts/bin vs
    ~100-150 trunk clutter; a sparse 5-step sweep is far lower).
  - **Frame matters:** the live MID-360 cloud is sensor-origin, the recorded
    Gazebo cloud is `frame_id: world`; the bridge picks `frame` from the packet
    header so height is absolute in both.
  - **`d_horiz` is derived from the scanned trunk axis** when no explicit
    distance is given, so a LiDAR-only scan needs no camera channel.  It is
    boom-pivot-relative: `d_horiz = hypot(axis - (base_x + BOOM_PIVOT_BASE_X_OFFSET_M))`
    with offset -0.87 m, so the reference trunk at x=8.5 gives 9.37 m
    (ros2_ws parity).  `--base-x` overrides `base_x`; a camera
    `v1/docking/trunk_estimate` overrides the derivation when present.
  - Closed-form boom IK with the as-built `ros2_ws` corrections
    (`leveling = +theta_b`, `BOOM_PIVOT_WORLD_Z = 1.81 m`), plus `theta_d`
    (`docking_lower_angle`, the extra boom-lower angle to descend onto the dock;
    0 at the solved configuration).
  - Parity tests against the real `tree_scan_002` recording live in
    `test_scan_estimate.RecordedScanParityTest` (skipped without the fixture).
- Layout/config: the `lidar_scan` panel in the same `--hud-config` file as the
  other sensor HUDs (anchor/caption/size/fonts + `zoom_default_m`, `scan_seconds`,
  `redraw_hz`, `scrim_opacity`, `guide_font_px`, `estimate_font_px`).
- MID-360 IMU: `v1/imu/lidar` (JSON) is routed to `bridge._on_lidar_imu`, which
  converts the sensor-frame attitude in `decoders/livox_imu.py` and applies the
  **same** `decoders/imustab.stabilize_points` the OAK cloud uses. Two payload
  shapes are accepted: the flat `attitude_rpy_rad` this repo's producer emits,
  and the ROS-style `orientation` quaternion used by the `tree_scan_002`
  recordings (`quaternion_to_rpy`). Key `7` toggles stabilization for both
  clouds; the LiDAR reference is latched separately so the two IMUs never mix.
- ADVISORY ONLY: the estimate is geometry from one scan, not a measured contact.
  The overlay must always show the advisory line; the five measured range sensors
  remain the authoritative docking guard. The camera↔LiDAR extrinsic is
  unsurveyed, so the camera overlay is an aid, not a calibrated measurement.

## Surface a new JSON channel (worked example: `v1/boom/state`)

1. **Model**: add the channel to `JSON_CHANNELS`; add
   `def snapshot_boom(self): return self._json_of('v1/boom/state')`.
2. **Bridge**: add `boom_changed = Signal()`, emit it in `refresh()`; add getters
   `_get_boom_angle_line`, `_get_phase_guide_line`, etc. reading `self.model.snapshot_boom()`; add
   `Property(str, _get_boom_angle_line, notify=boom_changed)` etc.
3. **QML**: render the new properties in `SensorPanel.qml`.
4. **Test**: extend `harvester_dashboard/test/test_no_emit_proof.py` to drive the new slot and
   assert zero socket traffic, plus a property-flip assertion.

The bridge's `refresh()` (QTimer 200 ms) re-emits `ranges_changed`, `boom_changed`,
`trunk_changed`, `calibration_changed`, `stream_rows_changed`, `status_summary_changed`,
`frame_tick` on every tick.

## Verify

```bash
cd ~/harvester_vision
# Dashboard (system python, headless)
PYTHONPATH=harvester_dashboard /usr/bin/python3 \
  -m unittest discover -s harvester_dashboard/test -v
```

Known pre-existing failures: `test_decoders` and `test_zmq_source` **JPEG** cases fail headless
because the Jetson NVDEC hardware decoder is unavailable ("NvMMLiteOpen … Unsupported Codec").
These are unrelated to model/bridge/QML changes — confirm they fail before your change too, and do
NOT "fix" them.

## Runtime / memory facts (do not regress)

- Dashboard runs under **system `/usr/bin/python3`** with PySide2 5.14 QtQuick (no Controls2 — the
  root is a plain `Item` wrapped by `QQuickView`, not `QQmlApplicationEngine`).
- `run_all.sh` launches it with `MALLOC_ARENA_MAX=2` (caps glibc malloc arenas; the dominant
  contributor to the ~3 GB steady-state RSS) and a 15 s `gc.collect()+malloc_trim` timer
  (`main._install_memory_trim_timer`).
- Depth is delivered at **half RGB resolution** (960×540 for 1080p); `bridge._depth_pixel_for`
  maps RGB→depth pixel via the delivered-size ratio. Point-cloud unprojection is cached
  (`_raw_cloud_cache`) and flattened with a single vectorized `tolist()`.
- Only the **active** camera is decoded (`zmq_source.TelemetryWorker` active-camera-only decode);
  the inactive camera's packets are ingested for freshness but not decoded.

## Guardrails

- Never add a bridge slot that writes to a socket; view switching and HUD toggles are render-only.
  The **one** exception is the LiDAR standby/normal control (`lidar_control.py`), which is opt-in,
  disabled by default, sends only `{"enabled": bool}`, and is reachable from exactly three slots
  (`begin_scan`/`stop_scan`/`cancel_scan`). Any new socket write needs the same explicit scoping.
- When removing a QML binding, remove the now-unused bridge getter + `Property` too (avoid dead
  code), but leave valid `TelemetryModel` APIs and their tests in place if reusable.
- `telemetry_model` must stay Qt-free; keep all Qt types (`Signal`, `Property`, `Slot`, `QTimer`,
  `QObject`) in `bridge.py` (and `zmq_source.py` for the worker).
- Use only QtQuick 2 primitives (PySide2 5.14 has no Controls2).

## Docs to update

When keyboard/HUD behavior changes, update `docs/orin_canonical_zmq.md` (dashboard controls /
"HUD layers" subsection) and `docs/oak_depth_pointcloud.md` (Controls summary line). The README
documents the legacy `oak_rgb_viewer.py`, not this dashboard, so it usually needs no change.
