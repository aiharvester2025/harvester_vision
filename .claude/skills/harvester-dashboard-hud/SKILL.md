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
  - `HudOverlay.qml` — MIXED-SOURCES warning, left `SensorPanel`, right trunk/calibration, bottom
    stream-errors panel (diagnostic-gated).
  - `SensorPanel.qml` — phase guide (`phaseGuideLine`), boom angle/extension/leveling
    (`boomAngleLine`/`boomExtensionLine`/`levelLine`), docking range rows (`dockingRangeRows`),
    cutter range (`cutterRangeLine`).
  - `CameraView.qml` — camera image (image://frames), stale ring + timestamp line
    (diagnostic-gated), click annotation, crosshair.
  - `LidarInset.qml`, `PointCloudInset.qml`, `Annotation.qml` — inset overlays.

## Two-layer HUD (operator vs. diagnostics)

The HUD is split into two independently-toggled layers:

- **Operator HUD** (key `3` → `bridge.hudVisible`): phase guide, boom/leveling, docking ranges,
  cutter range, trunk + calibration, MIXED-SOURCES warning, source badge.
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

Full key map (render-only unless noted): `1` cutter view, `2` docking view, `3` HUD, `4` LiDAR
inset, `5` LiDAR view cycle, `6` point cloud, `7` IMU stabilization A/B, `0`/`Esc` clear
annotation, `777`+`Enter` diagnostic layer, click = annotate (depth + camera-frame XYZ).

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
- When removing a QML binding, remove the now-unused bridge getter + `Property` too (avoid dead
  code), but leave valid `TelemetryModel` APIs and their tests in place if reusable.
- `telemetry_model` must stay Qt-free; keep all Qt types (`Signal`, `Property`, `Slot`, `QTimer`,
  `QObject`) in `bridge.py` (and `zmq_source.py` for the worker).
- Use only QtQuick 2 primitives (PySide2 5.14 has no Controls2).

## Docs to update

When keyboard/HUD behavior changes, update `docs/orin_canonical_zmq.md` (dashboard controls /
"HUD layers" subsection) and `docs/oak_depth_pointcloud.md` (Controls summary line). The README
documents the legacy `oak_rgb_viewer.py`, not this dashboard, so it usually needs no change.
