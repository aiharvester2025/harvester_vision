# Docking Safety-Guidance HUD on the Orin (no simulation)

## Goal

Port the **docking safety-guidance** system from the `ros2_ws` skill/plan
(`docking-safety-guidance-plan.md`, `.claude/skills/docking-safety-guidance`) into
**this** repo (`harvester_vision`, the Orin ZeroMQ + PySide2 dashboard), **excluding the
simulation** (`harvester_dock` approach driver, Gazebo/RViz, ROS2 approach scripts).

The result: a physics-based **stopping-distance + TTC** guidance model driven by the **real
sensor data** the Orin already receives, plus an operator-glanceable HUD panel whose
**location, size, captions, fonts, opacity and rows are configurable in the same config file
as the other sensor HUDs** (`harvester_dashboard/hud_config.json`, `--hud-config`).

## Scope correction (carried from the source plan)

The guidance is about the **c-channel platform docking onto the trunk** (boom elevation +
extension + leveling), **not** the prime mover driving. Inputs are the **forward gap**
(`center_line`) and the **platform closing speed** (`-d(gap)/dt`). The model is
**advisory/operator-facing only** — the dashboard never commands motion. Hard-contact
authority stays with the real machine's PLC/safety layer, which is **not present in this
repo** and is out of scope.

## Current state (verified in this repo)

- **Orin stack, no ROS2.** The dashboard is `harvester_dashboard/` (PySide2 5.14, QtQuick 2
  primitives only — no Controls2). Path: wire channel → `TelemetryModel` → `DashboardBridge`
  Qt property → QML, strictly **render-only**.
- **Data already present.** `v1/range/docking` (JSON list) carries records
  `{telemetry_key, distance_m, valid}`; `center_line` is the forward gap
  (`canonical_zmq/.../range_ingest.py:31-37`). `bridge._get_docking_range_rows()`
  (`bridge.py:649`) already reads it and `refresh()` (QTimer 200 ms, `bridge.py:585`) emits
  `ranges_changed` each tick.
- **Reusable HUD config system already exists.** `harvester_dashboard/harvester_dashboard/
  hud_config.py` (`HudPanelConfig`/`HudLayoutConfig`/`load_hud_config`), shipped
  `harvester_dashboard/hud_config.json`, `--hud-config` in `config.py:63`, generic
  `qml/SensorHudPanel.qml`, and the three-panel host `qml/SensorPanel.qml`
  (Boom, Docking Ranges, Cutter Range) anchored from `bridge.hudLayout`.
- **No safety-guidance model exists here yet** (`find . -name '*safety*'` → nothing). The
  source `safety_guidance.py` is pure Python (no Qt/ROS) and ports verbatim.

## Design

### 1. Guidance model — new `harvester_dashboard/harvester_dashboard/safety_guidance.py`

Port the source module **verbatim** (pure Python, Qt-free, no new deps): `SafetyConfig`
(frozen dataclass with `load()` + sanitization), `Guidance`, `evaluate()`, `state_message()`,
`SAFE/WARN/DANGER/NO_DATA`, `default_config_path()`. Exact formulas:

```
d_stop(v) = v·latency + v²/(2·a_max)                        # v in m/s
v_max(d)  = -a_max·latency + sqrt((a_max·latency)² + 2·a_max·d)   # cm/s
```

State order (first match wins): `no_data` (missing/stale/`d<=0`) → `danger`
(`d<=danger_distance` floor, or `d_stop>=d` while approaching, or `TTC<=danger_ttc`) →
`warn` (`d<=warn_distance` or `TTC<=warn_ttc` or `v>warn_margin·v_max`) → `safe`.
`evaluate()` is stateless; hysteresis lives in the bridge.

### 2. Tuning file — new `harvester_dashboard/config/safety_guidance.json`

Ships the documented defaults (`a_max_m_s2=0.10`, `latency_s=0.30`, `warn_margin=0.7`,
`warn_ttc_s=4.0`, `danger_ttc_s=1.5`, `warn_distance_m=1.0`, `danger_distance_m=0.3`,
`stationary_epsilon_cm_s=0.5`, `stale_s=2.0`, `debounce_s=0.4`, `speed_ema_alpha=0.35`) with
the units/meaning comments. Loader is fallback-safe (missing/malformed → defaults) and
**sanitizes** out-of-range values (`a_max>=1e-3`, `latency>=0`, margins clamped).

Add `DashboardConfig.safety_config_path` + `--safety-config` (default points at the shipped
file via `default_config_path()`; empty/missing → built-in defaults), mirroring
`hud_config_path`.

### 3. Bridge wiring — `bridge.py`

- Accept `safety_config` (default `SafetyConfig.load(default_config_path())`) in
  `__init__`, so existing tests constructing `DashboardBridge(config, model, annotation)`
  keep working.
- **Closing-speed derivation** `_update_dock_speed()`: rolling `deque` of
  `(receipt_monotonic_s, distance_m)` from the `center_line` record, prune > 1.5 s,
  least-squares slope → `closing = -slope·100` cm/s. Record
  `_center_range_recv_monotonic` for staleness. None/invalid → age out and set `None`.
- **Hysteresis** `_recompute_safety_guidance()`: EMA-smooth speed, call `evaluate(...,
  stale=...)`, adopt the candidate **immediately** for `danger`/`no_data`, otherwise only
  after `debounce_s`. When the held state differs from the candidate, display
  `state_message(held, candidate)` so text and colour never disagree.
- Call both from `refresh()` (200 ms tick); emit a new `dock_safety_changed` signal.
- **QML properties** (all float getters return `nan` when absent): `dockSpeedCmS`,
  `dockCenterDistanceM`, `dockSpeedSmoothedCmS`, `dockSafetyState` (init `'no_data'`),
  `dockGuidanceText`, `dockRecommendedSpeedCmS`, `dockMaxSpeedCmS`, `dockStopDistanceM`,
  `dockTtcS`. Also `dockSafetyRow` (`QVariantList` of `{key,label,value,valid}`) so the
  existing generic `SensorHudPanel` renders the state/metrics rows uniformly.
- `render-only`: no socket writes from this path.

### 4. HUD panel configurable in the **same** file (explicit requirement)

Extend the existing config system rather than adding a parallel one:

- `hud_config.py`: add `DOCK_GUIDANCE = 'dock_guidance'`; new
  `_DEFAULT_DOCK_GUIDANCE_ROWS` (`state`→`State`, `speed`→`Closing Speed`,
  `gap`→`Gap`, `ttc`→`TTC`, `max_speed`→`Max Safe`); include the panel in
  `HudLayoutConfig`, `default_hud_layout()`, `to_qml()`, `load_hud_config()`, and
  `__all__`.
- Defaults for `dock_guidance` (sized for ~2 ft, 1080p): anchor **`top-center`**,
  `visible: true`, `caption: "DOCKING SAFETY"`, `width: 700`, `height: 0`,
  `value_font_px: 46`, `caption_font_px: 30`, `opacity: 0.85`, and dedicated keys for the
  guidance extras: `stopbar_range_m: 1.5`, `banner_font_px: 40`, `pulse_danger: true`.
  (The generic `SensorHudPanel` ignores the extras; the dedicated `DockGuidanceHud.qml`
  uses them.)
- `harvester_dashboard/hud_config.json`: mirror the new panel, fully self-documenting
  (location, size, caption, fonts, opacity, per-row captions, stop-bar scale), so an admin
  edits **one file** for every sensor HUD including the guidance panel.
- `DashboardConfig`/`--hud-config` unchanged; the loader keeps the fail-fast contract for a
  malformed explicit path and forward-compatible unknown-key handling.

### 5. QML — `qml/DockGuidanceHud.qml` + host in `SensorPanel.qml`

**New `DockGuidanceHud.qml`** (QtQuick 2 primitives only), driven by
`bridge.hudLayout.dock_guidance` for anchor/caption/size/fonts/opacity/stop-bar scale:

1. **State banner** — `bridge.dockGuidanceText`, coloured by `bridge.dockSafetyState`;
   `danger` pulses (`NumberAnimation` opacity 1.0→0.55, `InOutSine`, `Infinite`), border
   width 3 in danger else 2.
2. **Stop-bar** — track `#2a3a4a`; fill = `gap/stopbar_range_m` clamped 0..1, red when
   `d_stop >= gap` else the state colour; white 4 px marker at `d_stop/stopbar_range_m`;
   marker hidden and `—` shown when non-finite.
3. **Metrics row** — `speed · gap · TTC · max` from the bridge properties, `—` when NaN.

Colour map (must match): `danger` `#e23c3c`/`#3a1414`, `warn` `#f0a030`/`#3a2a10`,
`no_data` `#9fb4c7`/`#1a1f24`, `safe` `#40c040`/`#102a18`.

**`SensorPanel.qml`**: add a `dock_guidance` loader anchored from
`bridge.hudLayout.dock_guidance` via the existing `applyAnchor`, active when
`bridge.operatorHudsVisible && hudLayout.dock_guidance.visible && bridge.view !== "cutter"`
(shown on the docking view, hidden on the cutter view). Keep the three existing panels
unchanged.

### 6. Docs & skills

- `docs/orin_canonical_zmq.md`: document the guidance panel (states/colours, stop-bar),
  the `--safety-config` tuning file, and the new `dock_guidance` block in the
  `--hud-config` example.
- `.claude/skills/harvester-dashboard-hud/SKILL.md`: add `DockGuidanceHud.qml` + the
  `dock_guidance` panel to the QML/layer map and the config example.

## Files to Change

1. **New** `harvester_dashboard/harvester_dashboard/safety_guidance.py` — ported model.
2. **New** `harvester_dashboard/config/safety_guidance.json` — tuning defaults.
3. **New** `harvester_dashboard/qml/DockGuidanceHud.qml` — state banner + stop-bar + metrics.
4. `harvester_dashboard/harvester_dashboard/config.py` — `safety_config_path` + `--safety-config`.
5. `harvester_dashboard/harvester_dashboard/main.py` — load safety config, pass to bridge.
6. `harvester_dashboard/harvester_dashboard/bridge.py` — speed derivation, staleness,
   debounce, `dock_*` properties, `dock_safety_changed`, `dockSafetyRow`.
7. `harvester_dashboard/harvester_dashboard/hud_config.py` — `dock_guidance` panel + extras.
8. `harvester_dashboard/hud_config.json` — mirror the new panel.
9. `harvester_dashboard/qml/SensorPanel.qml` — host the guidance loader.
10. `harvester_dashboard/test/test_safety_guidance.py` — **new**, ported model tests.
11. `harvester_dashboard/test/test_hud_config.py` — `dock_guidance` defaults/overrides/extras.
12. `harvester_dashboard/test/test_smoke_gui.py` — guidance property/layout assertions.
13. `harvester_dashboard/test/test_no_emit_proof.py` — extend the render-only proof.
14. `docs/orin_canonical_zmq.md`, `.claude/skills/harvester-dashboard-hud/SKILL.md`.

## Tests to Add / Update

- **New `test_safety_guidance.py`** (unittest, no pytest in the system interpreter): missing/
  stale → `no_data`; stationary/moving-away → `safe`; TTC warn/danger; stopping-distance
  danger; `recommended_speed` decreases with distance; `v_max` monotonic; `warn_margin`
  scaling; config fallback + sanitization (`a_max<=0`, bad strings); `state_message` matches
  the held state (never "STOP" on an orange banner).
- **`test_hud_config.py`**: `dock_guidance` present in defaults/`to_qml`; partial override of
  anchor/caption/size/fonts/rows; the guidance extras (`stopbar_range_m`, `banner_font_px`,
  `pulse_danger`) honored; unknown keys ignored.
- **`test_smoke_gui.py`**: `dockSafetyState` boots `'no_data'` (never `'safe'`); feeding a
  shrinking `center_line` series drives `safe→warn→danger` and a finite
  `dockRecommendedSpeedCmS`; the new loader activates on the docking view and not on cutter;
  `hudLayout` includes `dock_guidance`.
- **`test_no_emit_proof.py`**: drive the guidance path (ranges + refresh) and assert zero
  socket traffic (extends the existing render-only proof).

## Verification

```bash
cd ~/harvester_vision
# Dashboard (system python, headless) — no pytest in this interpreter
PYTHONPATH=harvester_dashboard /usr/bin/python3 \
  -m unittest discover -s harvester_dashboard/test -v
# QML load / GUI smoke (offscreen)
QT_QPA_PLATFORM=offscreen PYTHONPATH=harvester_dashboard /usr/bin/python3 \
  -m unittest harvester_dashboard.test.test_smoke_gui -v
```

Confirm the two known pre-existing NVDEC JPEG failures remain the only failures. Manual (if a
display is available): feed a real/synthetic `center_line` series on `--pub`, watch
green→orange→red and the stop-bar crossing red; edit `hud_config.json` (move the panel,
rename the caption, grow the fonts, change `stopbar_range_m`) and relaunch to confirm the
guidance panel follows the **same** config file as the other HUDs.

## Notes / Guardrails

- **No simulation**: skip the approach driver, Gazebo/RViz, ROS2, and
  `harvester_boom_plan/safety.py`. The hard-contact authority is the real machine's PLC and
  is not in this repo; this model stays advisory.
- **Render-only**: never add a bridge slot that writes a socket; keep `telemetry_model`
  Qt-free and all Qt types in `bridge.py`.
- **QtQuick 2 primitives only** (PySide2 5.14, no Controls2).
- **Units fixed**: speed cm/s, distance m, TTC s.
- `no_data` is a real state (grey), never a false green; `danger`/`no_data` bypass debounce.
- Do not silently change `a_max_m_s2`/`latency_s` without operator sign-off.
- Reuse the existing `hud_config`/`SensorHudPanel` machinery; do not create a second config
  file or a second anchoring convention.
