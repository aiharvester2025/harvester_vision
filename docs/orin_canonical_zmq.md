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

## Run: replay a recorded tree scan on the Orin (no hardware, no Xavier)

The Xavier recorded canonical telemetry with `PacketRecorder` during the Gazebo
tree-scan runs (`~/harvester_audits/tree_scan_001`, `tree_scan_002`).  Those are
`.msgpack` recordings in this repo's own format, so `replay.py` feeds the real
dashboard and the real estimator with **real simulated palm geometry** — no
synthetic fixture needed.  `tree_scan_002` additionally carries `v1/imu/lidar`,
so the IMU compensation path is exercised too.

Pull just the two channels onto the Orin (gitignored `testdata/`):

```bash
cd ~/harvester_vision
mkdir -p testdata/tree_scan_002
rsync -az ubuntu@10.108.137.233:~/harvester_audits/tree_scan_002/{v1_lidar_raw,v1_imu_lidar} \
      testdata/tree_scan_002/
```

Replay in a loop while the dashboard runs (the recordings are a single scan
session, so loop them for a live-looking stream):

```bash
PYTHONPATH=canonical_zmq /home/marcop/depthai-env/bin/python3 \
  -m canonical_zmq_publisher.replay testdata/tree_scan_002 \
  --endpoint tcp://*:5590 --speed 2
```

Then point the dashboard at that endpoint (its default `--pub tcp://127.0.0.1:5590`
already matches when the replay binds locally).  Open the LiDAR overlay (key `4`)
and press SCAN: the estimate is taken from the accumulated cloud, and the
recorded cloud is `frame_id: world`, so the bridge estimates it as world-frame
(height is absolute, not ground-subtracted).

Reference geometry (`ros2_ws/src/oil_palm_tree_description/config/tree_targets.yaml`):
trunk at world (8.5, 0), trunk top 12.0 m, crown base 9.2 m.  The parity tests in
`harvester_dashboard/test/test_scan_estimate.py` (`RecordedScanParityTest`) assert
the estimator recovers these from the recording, and are skipped when the
fixture is absent.

**Docking height comes from the CROWN BASE, not the tree top.**
`H_dock = crown_base - docking_offset_below_trunk_top_m` (~7.2 m for the
reference tree).  The legacy `tree_top - 2.0` = 10.0 m lands inside the
frond/FFB zone (fronds ~9.45 m, FFBs ~9.55 m) and is why the platform crashed
into them.  When the crown-base detector fails, the target is `NO DATA` — it
never docks on the unsafe fallback.

**The boom rows are computed from the scan alone.**  `d_horiz` is derived from
the scanned trunk axis (boom-pivot-relative: `axis_x - (base_x - 0.87)`, so the
reference trunk at x=8.5 gives 9.37 m), so a LiDAR-only scan with `CAMERAS=0`
populates boom angle, extension, distance, platform level and docking lower
angle without the camera trunk channel.  A `v1/docking/trunk_estimate` pose, when
present, overrides the derivation.

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

Controls: `1` cutter view / toggle the Cutter Range HUD, `2` toggle the Boom +
Docking HUDs (on the cutter view it returns to the docking camera and shows
them), `3` operator sensor HUD, `4` full-screen LiDAR scan overlay, `5` LiDAR
projection (opens in the **front (y-z)** working view; cycles
front → left → right → iso → camera → top, and includes the `camera` overlay
view), `6` camera point-cloud inset,
`7` IMU stabilization A/B (covers both the OAK and MID-360 clouds), `0`/`Esc`
clear annotation, click to annotate (shows depth + camera-frame XYZ when the OAK
depth stream is on). All actions are non-actuating annotations only. See
`docs/oak_depth_pointcloud.md` for the depth/point-cloud feature.

### HUD layers (operator vs. developer diagnostics)

The sensor HUD is split into two independently-toggled layers:

- **Operator HUD** (key `3` / "3 HUD" button) — the operational guidance an
  operator needs: the operator sensor panels (below), docking phase guide, trunk
  estimate, calibration status, the MIXED-SOURCES warning, and the source badge.

- **Operator sensor panels** (keys `1` and `2`) — three separate, individually
  configurable panels, shown per the active camera:

  | Panel | Default location | Contents |
  |---|---|---|
  | **Boom** | bottom-right | the seven PLC MQTT values: boom angle, boom length, slew angle, platform tilt X1/Y1, prime mover tilt X2/Y2 |
  | **Docking Ranges** | bottom-left | the docking ranges: laser distances (45° left, center, 45° right) and the left/right ultrasonic trunk-detection side ranges + phase guide |
  | **Cutter Range** | bottom-left | the cutter range reading |
  | **Docking Safety** | bottom-center | the docking approach safety guidance: state banner, stop-bar, and speed/gap/TTC metrics (docking view only; see below) |

  On startup the docking camera is the default view and the Boom + Docking
  Range panels are displayed. The three bottom panels (Docking Ranges
  bottom-left, Docking Safety bottom-center, Boom bottom-right) are laid out
  edge-to-edge so they never overlap; the Docking Safety panel occupies the gap
  between its neighbours (capped at its configured width) and hides if the
  screen is too narrow to show all three.
  - Pressing `1` switches to the cutter camera and shows **only** the Cutter
    Range HUD (Boom and Docking hide immediately). Pressing `1` again while on
    the cutter view hides/shows the Cutter Range HUD; the camera stays on
    cutter.
  - Pressing `2` toggles the Boom + Docking HUDs on the docking camera. On the
    cutter camera, `2` always returns to the docking camera, hides the Cutter
    Range HUD, and shows the Boom + Docking HUDs.

  The Boom HUD values are sourced from the PLC MQTT subscriber (the
  `harvester/sensors/v1` topic mapped by `mqtt_ingest` onto `v1/boom/state`):
  boom angle, boom length, slew angle, platform tilt X1/Y1, and prime mover
  tilt X2/Y2.

- **Operator LiDAR scan overlay** (key `4`, "4 LiDAR" button; hidden by default)
  — a **full-screen** overlay that draws the MID-360 point cloud over the live
  camera image, centred on the camera's displayed image rect so the trunk seen
  through the camera is where the cloud draws. It is created *before*
  `HudOverlay` in `Dashboard.qml`, so the safety panels always paint on top of
  the cloud: the overlay is a background layer, not a replacement for the HUD.

  The overlay runs an operator-driven scan state machine
  (`scanPhase ∈ {idle, scanning, complete, no_data}`). On-screen
  SCAN/STOP/CANCEL/VIEW/ZOOM buttons (and keys `4`/`5`) drive it:
  - `idle` — "press SCAN" prompt, LiDAR in standby;
  - **SCAN** — LiDAR standby → normal, timer zeroed, buffer cleared, `scanning`;
  - **STOP** — end early, keep the accumulated data, estimate now;
  - **CANCEL** — discard the scan, zero the timer, LiDAR → standby;
  - timeout — 15 s `scan_seconds` (admin-overridable) behaves like STOP;
  - `complete` — the scan instructions hide and the **estimate rows** appear,
    split across two short cards at the bottom edge (the tree estimate at
    bottom-centre, the boom/dock estimate at bottom-right) so the point cloud
    above them stays visible: tree height, trunk top, crown base, uncertainty,
    docking height, boom angle, boom extension, platform level, boom distance,
    docking lower angle, status;
   - `no_data` — no usable cloud; the estimate is never shown from bad geometry.
     A **completed** scan also drops back to `no_data` ("stream stopped — estimate
     is stale") once `v1/lidar/raw` is stale past a 3 s grace, so a replay that
     reached the end of its file cannot leave a stale READY on screen.


  The estimate runs on the **accumulated** cloud over the whole window (capped),
  not the last frame, and a live point count with a sparse/filling/dense hint
  tells the operator when to press STOP. Both STOP and the timeout return the
  LiDAR to standby so the motor does not idle between scans.

  **LiDAR control is the one opt-in exception to the dashboard's render-only
  rule** (`harvester_dashboard/lidar_control.py`). It is disabled unless
  `--lidar-control <producer PULL endpoint>` is passed (off by default, so an
  existing deploy is unchanged; `run_all.sh` passes it when `LIDAR=1`), PUSHes
  only `{"enabled": bool}`, and is reachable from exactly the three scan buttons.
  `bridge.lidarControlEnabled` lets the overlay state on screen when the buttons
  do not control the sensor.

  The estimate (`harvester_dashboard/model/scan_estimate.py`, Qt-free) is the
  Orin counterpart of the `ros2_ws` tree-scan/boom-plan math, including the
  as-built corrections (`leveling = +theta_b`, boom pivot `1.81 m`). It is
  **advisory only** — geometry from one scan, not a measured contact; the five
  measured range sensors remain the authoritative docking guard. The
  camera↔LiDAR extrinsic is unsurveyed until the commissioning survey, so the
  `camera` overlay view is an aid, not a calibrated measurement.

  The overlay's MID-360 IMU compensation comes from the `v1/imu/lidar` channel
  (JSON, published by `lidar_capture` alongside the cloud). `bridge._on_lidar_imu`
  converts the sensor-frame attitude in `decoders/livox_imu.py` and applies the
  same `decoders/imustab.stabilize_points` the OAK cloud uses; key `7` toggles
  stabilization for both clouds. The overlay layout/config is the `lidar_scan`
  panel in the same `--hud-config` file as the other sensor HUDs (see below).

  **Exclusive scan mode.** While the overlay is visible, the unrelated HUD hides
  so the scan view is uncluttered: the operator sensor panels (Boom, Docking
  Ranges, Docking/Cutter Safety, Cutter Range), the trunk/calibration column, the
  MIXED-SOURCES warning, and the camera point-cloud inset all hide; they return
  the moment the overlay is hidden (key `4`). The estimate card carries its own
  advisory line, and the measured five-range docking safety returns as soon as
  scanning ends.

- **Developer-diagnostic HUD** (key sequence **`7` `7` `7` then `Enter`**) — the
  health overlays useful only when debugging: the bottom per-channel stream
  table ("streams recv … drops …" + age/gaps/decode-errors per channel), the
  toolbar status line ("status … | drops … | rec on/off"), the active-camera
  timestamp line, and the stale-camera red ring.

#### Configuring the operator sensor panels

An admin can move the panels, rename their captions, resize them, and change the
value/caption font sizes via a JSON file, passed with `--hud-config`:

```bash
PYTHONPATH=harvester_dashboard /usr/bin/python3 -m harvester_dashboard.main \
  --pub tcp://127.0.0.1:5590 --status tcp://127.0.0.1:5600 \
  --hud-config harvester_dashboard/hud_config.json
```

Without `--hud-config` (or when the file is absent) the built-in defaults are
used: boom bottom-right, docking ranges bottom-left, large fonts (46 px values,
30 px captions) sized to be read from ~2 feet. A supplied path that exists but is
malformed is fatal, so an operator display never boots with a half-applied
configuration. The shipped `harvester_dashboard/hud_config.json` mirrors the
defaults and documents every accepted key:

```jsonc
{
  "boom": {
    "visible": true,                 // show this panel at startup
    "anchor": "bottom-right",        // top|bottom + left|center|right
    "caption": "BOOM",
    "width": 520,
    "height": 0,                     // 0 = auto-size to content
    "value_font_px": 46,
    "caption_font_px": 30,
    "margin_px": 12,
    "opacity": 0.85,
    "rows": { "boom_angle_deg": "Boom Angle" }
  },
  "docking": { "anchor": "bottom-left", "rows": { "center_line": "Center" } },
  "cutter_range": { "anchor": "bottom-left", "caption": "CUTTER RANGE" },
  "dock_guidance": { "anchor": "bottom-center", "caption": "DOCKING SAFETY" },
  "cutter_guidance": { "anchor": "bottom-center", "caption": "CUTTER SAFETY" },
  "lidar_scan": {
    "anchor": "bottom-center",
    "caption": "LIDAR SCAN",
    "guide_font_px": 34,             // scan-instruction text size
    "estimate_font_px": 34,          // estimate-row text size
    "zoom_default_m": 12.0,          // overlay half-width of the projected cloud
    "zoom_min_m": 3.0,
    "zoom_max_m": 40.0,
    "scan_seconds": 15.0,            // guided scan window (s)
    "redraw_hz": 12.0,               // overlay repaint cap (CPU guard)
    "scrim_opacity": 0.15,           // scan-mode camera dim (0 disables)
    "rows": { "tree_height": "Tree Height", "boom_angle": "Boom Angle" }
  }
}
```

Unknown keys are ignored and any omitted key falls back to its default.

#### Docking safety guidance (advisory, operator-facing)

The **Docking Safety** panel (default bottom-center, docking view only) tells the
operator, during the c-channel platform's final approach to the trunk, whether
the closing speed is safe and at what speed to close. It reads the forward gap
from the `center_line` record on `v1/range/docking` and derives the **platform
closing speed** dashboard-side as the least-squares slope of the gap over ~1.5 s
(`-d(gap)/dt`), EMA-smoothed. It is **advisory only**: the dashboard never
commands motion.

The panel shows a colour-coded **state banner** (`APPROACH OK` green /
`SLOW …` orange / `STOP NOW` red / grey `NO RANGE DATA`), a **stop-bar** that
compares the current gap (fill) with the required stopping distance
`d_stop(v) = v·latency + v²/(2·a_max)` (white marker; the bar turns red once the
marker meets the fill), and a **metrics row** (closing speed · gap · TTC · max
safe speed). `NO DATA` is grey and distinct from a green `SAFE`; `danger` and
`no_data` are adopted immediately while `safe`/`warn` transitions are debounced
to prevent flicker.

Two safety invariants hold regardless of tuning: the panel **never shows `SAFE`
on absent, stale, unfittable, or non-finite data** (a stalled range stream cannot
invent a 0 cm/s speed and clear a DANGER approach; while the gap is fresh but no
closing speed can be fitted, a held WARN/DANGER is retained, and staleness falls
through to `NO DATA`), and the banner text never contradicts its colour (a WARN
banner always leads with `SLOW`). A **non-finite** gap or speed (`NaN`/`Inf`) is
treated as `NO DATA` at the model level, not just at the ingest boundary: `NaN`
compares false against every threshold, so an unguarded `NaN` gap would otherwise
fall through every DANGER/WARN test and render a false-green `SAFE`. Thresholds
are clamped on load, including the `danger` bands (pulled inside the
corresponding `warn` bands) and a strictly positive `warn_margin`.

The thresholds live in `harvester_dashboard/config/safety_guidance.json`, loaded
with `--safety-config` (default: the shipped file). A missing file falls back to
the built-in defaults silently (a missing tuning file is a valid configuration),
but an **existing** file that is unreadable, malformed, or not a JSON object also
falls back to the defaults **and logs a warning**, so a broken tuning file on the
operator display is not mistaken for an applied one; out-of-range values are
clamped:

```bash
PYTHONPATH=harvester_dashboard /usr/bin/python3 -m harvester_dashboard.main \
  --pub tcp://127.0.0.1:5590 --status tcp://127.0.0.1:5600 \
  --safety-config harvester_dashboard/config/safety_guidance.json
```

`a_max_m_s2` is the maximum safe platform deceleration and `latency_s` the
operator reaction + actuation latency; `warn_margin` (0.7) fires WARN at 70% of
the physical stop speed. Do not change `a_max_m_s2`/`latency_s` without operator
sign-off.

The guidance panel's **location, caption, size, fonts, opacity, stop-bar scale
and per-row captions** are configured in the **same** `--hud-config` file as the
other sensor HUDs, under the `dock_guidance` key:

```jsonc
{
  "dock_guidance": {
    "visible": true,
    "anchor": "bottom-center",       // top|bottom + left|center|right
    "caption": "DOCKING SAFETY",
    "width": 620,
    "height": 0,                     // 0 = auto-size to content
    "value_font_px": 46,
    "caption_font_px": 30,
    "margin_px": 12,
    "opacity": 0.85,
    "stopbar_range_m": 1.5,          // stop-bar full scale (m)
    "banner_font_px": 40,
    "pulse_danger": true,
    "rows": { "gap": "Gap", "max_speed": "Max Safe" }
  }
}
```

#### Cutter safety guidance (advisory, operator-facing)

The **Cutter Safety** panel (default bottom-center, **cutter view only**) is the
sibling of the docking guidance, for the cutting arm. It is shown/hidden together
with the Cutter Range HUD by key `1` on the cutter view. It (1) warns the operator
before the cutter **tip** crashes into the trunk / FFB / frond, and (2) prompts
the **cut sequence**: approach → stop/ready → open scissors wide → advance →
cut. It is **advisory only**: the dashboard never commands motion, and the
scissors/gripper are hydraulic operator actions with no sensing.

The **only** clearance measurement is the single forward range sensor on
`v1/range/cutter` (telemetry key `cutter_forward`). The depth camera and LiDAR
are mounted on the arm base and do **not** follow the extension, so they cannot
see the tip — there is **no fusion**. The sensor sits **behind** the tip, so the
raw range overstates the true clearance; the model applies the tunable offset
`tip_clearance = raw_range − sensor_to_tip_offset_m` (URDF-derived ≈ 0.19 m).
The closing speed is derived dashboard-side as the least-squares slope of the
raw range over ~1.5 s (`-d(range)/dt`), EMA-smoothed.

The panel shows a colour-coded **cut-step banner** (the current operator prompt),
a **clearance alert line** (`TIP CLEAR` green / `SLOW …` orange / `STOP` red /
grey `NO DATA`), a **clearance bar** comparing the tip clearance (fill) with the
required stopping distance `d_stop(v)` (white marker; red once the marker meets
the fill), a **metrics row** (clearance · closing speed · TTC · max safe), and a
**CONFIRM STEP** button that advances the operator-confirmed prompts. As with
docking, `NO DATA` is grey (never a false green), `danger`/`no_data` bypass the
debounce, and non-finite ranges are treated as `NO DATA`.

The cut sequence runs `approach → align` automatically, but **only** when the tip
is inside the ready band, has settled (closing speed ≤ `warn_margin·v_max`), and is
**not** inside the danger floor. The ready band sits inside the warn clearance by
design, so a stable WARN "ready-to-cut" state is expected (the panel shows the
orange `SLOW` clearance line together with the ready prompt); ALIGN never
coincides with `danger`/`no_data`, so the "ready to cut" prompt cannot appear while
the tip is inside the danger band. The remaining steps
(`align → open → advance → cut`) have no sensors and are advanced by the
**CONFIRM STEP** button, which is **disabled and relabelled** ("WAIT — tip too
close" for `danger`, "WAIT — no range" for `no_data`) so the sequence cannot be
advanced into a collision or with no measurement. A brief range dropout
**holds** an in-progress operator step rather than resetting it.

Thresholds live in `harvester_dashboard/config/cutter_safety_guidance.json`,
loaded with `--cutter-config` (default: the shipped file; same fallback/sanitize
contract as `--safety-config`):

```bash
PYTHONPATH=harvester_dashboard /usr/bin/python3 -m harvester_dashboard.main \
  --pub tcp://127.0.0.1:5590 --status tcp://127.0.0.1:5600 \
  --cutter-config harvester_dashboard/config/cutter_safety_guidance.json
```

Key thresholds: `sensor_to_tip_offset_m` (0.19), `warn_clearance_m` (0.30),
`danger_clearance_m` (0.10), `ready_standoff_m` (0.20, deliberately clear of the
danger band), `advance_distance_m` (0.05) and `align_tolerance_m` (0.05).
`a_max_m_s2`/`latency_s` share the docking values; do not change them without
operator sign-off. The panel's **location, caption, size, fonts, opacity,
clearance-bar scale, CONFIRM-button visibility and per-row captions** are
configured in the **same** `--hud-config` file as the other sensor HUDs, under
the `cutter_guidance` key (see the example above; its `stopbar_range_m` defaults
to `1.0` m because cutter clearances are small). A `center` anchor centers the
panel on the **screen** (not merely in the gap right of the Cutter Range panel),
and the panel shrinks — then hides — rather than overlap the Cutter Range
distance HUD on a narrow display. The layout reserves the side the Cutter Range
panel is anchored to, so moving the range panel to the right (or a side) still
keeps the two apart; a centered range panel confines the guidance panel to a free
side band (and hides it if the guidance is also centered).

Notes on the `777`+`Enter` sequence:

- Only the `7` key is buffered (it is ambiguous with the single-key `7` IMU
  toggle). A lone `7` still toggles IMU stabilization after a short (~800 ms)
  idle, while a fast `7`-`7`-`7` followed by `Enter` toggles the diagnostic
  layer.
- The two layers are independent: `3` shows/hides the operator HUD regardless
  of the diagnostic state, and `777`+`Enter` shows/hides the diagnostic
  overlays regardless of the operator HUD.
- Both toggles are render-only: neither writes to any telemetry or control
  socket (the safety boundary is unchanged).

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
  (`v1/camera/<name>/depth`, `depth_uint16_le`), **camera_info** intrinsics
  (`v1/camera/<name>/camera_info`, `json`), and an onboard **IMU** stream
  (`v1/camera/<name>/imu`, `json`) for point-cloud vibration compensation.
  Launched by `run_all.sh` (default `CODEC=h265`, depth + imu on; `DEPTH=0`
  disables depth, `IMU=0` disables the IMU stream).
- `lidar_capture` — **implemented**. Reads the MID-360 through Livox-SDK2
  (`lidar/livox_source.py`), levels to gravity, clips to a forward sector and
  range, and PUSHes `v1/lidar/raw` (`lidar_xyz_f32`). Alongside the cloud it
  publishes the MID-360 IMU attitude on `v1/imu/lidar` (`json`), which the
  dashboard uses for point-cloud vibration compensation. Starts idle and is
  enabled over a PULL control socket (`{"enabled": true}`) — the operator scan
  HUD drives this to switch the MID-360 between normal and standby (see the
  operator scan-overlay section above). Launched by
  `run_all.sh` (`LIDAR=1` default, `LIDAR_MODE=livox-sdk` for the real sensor).
  See `.kilo/plans/mid360-lidar-integration.md`.

### MID-360 IMU provenance (`v1/imu/lidar`)

`v1/imu/lidar` is a JSON channel mirroring the camera IMU channels
(`v1/camera/<name>/imu`), carrying `attitude_rpy_rad` (gravity-referenced
roll/pitch about the sensor's own axes), `quaternion_xyzw`, `accel_ms2` and
`accel_norm_ms2` for the newest sample, with `frame_id: mid360_link`. It is
published only while the LiDAR is enabled (same on-demand contract as the
cloud), so a disabled sensor stays silent. Its timing header uses the same
`timing_snapshot()` provenance as the cloud (see below), so an unsynchronized
LiDAR is never labelled `lidar_ptp_utc`.

The dashboard converts the sensor-frame attitude to its stabilization frame in
`harvester_dashboard/decoders/livox_imu.py` and then applies the **same**
`decoders/imustab.stabilize_points` the OAK cloud uses, so one stabilization
implementation serves both sensors. The LiDAR IMU state is kept separate from
the camera IMU state, so the two never mix.

**Only the vibration band is removed (`imustab.VIBRATION_BAND_DEG`, 8°).**  A
larger tilt is machine motion, not vibration: rotating the cloud by it collapses
the scene (the Gazebo recordings' IMU is pure attitude and swings to ~60° pitch,
which folds a 9 m trunk to ~4 m).  Outside the band the cloud is shown raw and
`bridge.lidarImuMotion` is set, so the HUD reports "motion — stab withheld"
rather than silently distorting the view.  Real hydraulic vibration is a few
degrees, so it stays inside the band.  The reference RE-BASES when the tilt
leaves the band, so one big swing does not leave stabilization permanently
withheld, and `begin_scan` clears the reference so a scan never inherits a
withheld state.  This is why the recorded replay looks correct without the
operator having to press `7`.

### LiDAR timestamp provenance (`v1/lidar/raw`)

The MID-360 has **no UTC clock of its own**: its packet timestamps are
nanoseconds since power-on until it is slaved to a PTP/gPTP master (or GPS). The
LiDAR is always the PTP *slave*, so the Orin is the master
(`deploy/ptp/`, `deploy/systemd/ptp4l-master@.service`). **Verified working on
the deployment Orin NX**: `ptp4l` is grandmaster on `eth1` and the MID-360
reports `time_type=1`; measured LiDAR↔Orin agreement is ~1–2 ms (σ ≈ 0.5 ms,
software timestamping).

`lidar_capture` decodes the per-packet `time_type` and never mislabels the
origin. It stamps each scan from a single `timing_snapshot()` call that returns
the chosen time **and** its provenance together, so the header can never pair a
host-clock value with a PTP label (two separate reads could disagree on a stale
sample). Cases:

| Condition | `clock_domain` | `timestamp_source` |
|---|---|---|
| `time_type` 1/2 **and** sample fresh | `lidar_ptp_utc` | `livox_ptp` / `livox_gps` |
| not synchronized, or absolute sample stale | `orin_realtime` | `host_arrival` |
| no packet ever arrived | `orin_realtime` | `host_now` |

Extra header fields `lidar_time_sync`, `lidar_time_type`, and `lidar_time_source`
state the raw device state (`ptp`/`gps`/`device_uptime`/`unknown`/`no_data`).
`lidar_time_sync` is true only for the first row — a stale absolute sample is
published as host time with `lidar_time_sync=false`, even though the device's
`time_type` may still read 1. The contract's `clock_domain` allowlist includes
both new domains.

- `range_ingest` — **retained but not launched by default**. Subscribes to the
  Pi PLC's single-part JSON stream (`tcp://192.168.50.40:5555`, topic
  `harvester.sensors.v1`) and PUSHes canonical packets for
  `v1/range/docking` (the five docking ranges), `v1/boom/state` (boom angle,
  extension, leveling, phase), and `v1/docking/trunk_estimate`. `run_all.sh` no
  longer starts it (the MQTT subscriber supplies the boom/range data); set
  `WITH_RANGE_INGEST=1` to launch it. See
  `.kilo/plans/plc-docking-sequence-simulation.md`.
- `mqtt_ingest` — **implemented**. Subscribes to a Mosquitto MQTT broker
  (`mqtt://192.168.50.40:1883`, topic `harvester/sensors/v1`) that carries the
  Node-RED PLC sensor stream and PUSHes canonical packets for `v1/boom/state`
  (boom angle/extension, platform/primemover tilt, slew angle) and
  `v1/range/docking` (the three laser docking distances plus the left/right
  ultrasonic trunk-detection side ranges, mapped to `c_channel_left` /
  `c_channel_right`). Launched by `run_all.sh`
  (configurable via `MQTT_HOST`/`MQTT_PORT`/`MQTT_TOPIC`).
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
(two 1080p H.265 decodes → one). The inactive camera's **IMU** stream is
similarly dropped (only the active camera's point cloud is stabilized).

### CPU / rate budget

The Orin runs 4 online CPU cores (cores 4-7 are offline by default; see the
MID-360/VIO plan before enabling them). The harvester stack's software cost is
bounded by matching low-rate streams to their consumers:

- IMU publishes at `--imu-fps 5` Hz (matching depth), not the raw sensor rate —
  vibration compensation only needs a few Hz.
- The point cloud is derived **in the dashboard** at render time, subsampled to
  `--pointcloud-max-points` (default 2000), and its flatten-to-QML conversion
  is vectorized (single numpy `tolist()`), keeping the dashboard main thread in
  the low single-digit percent.
- The remaining dashboard cost is the **irreducible NVDEC hardware H.265 decode
  thread** (~30% of one core); everything else is a few percent each.

On the deployed unit (headless, no remote desktop) the stack uses roughly a
third of the 4-core budget. Remote-access tooling (Anydesk screen streaming,
VS Code/kilo server) is CPU-hungry and will push a 4-core box toward 100% when
active; that overhead is not part of the product and does not reflect the
headless deployment.

### OAK device-disconnect resilience

The OAK devices can drop their XLink connection when the host CPU is briefly
saturated (the DepthAI monitor thread misses a keepalive ping). `oak_capture`
treats this as a clean exit (no traceback) so `--supervise` restarts it, and
`_open_pipeline` retries discovery for 60 s with progress logging (not 20 s),
so a restarted child waits out the device's ~10-15 s reconnection window
instead of churning through "cannot find device" restarts.
