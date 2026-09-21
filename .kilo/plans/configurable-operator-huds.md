# Configurable Operator HUDs: Boom, Docking Ranges, Cutter Range

## Goal

Replace the single monolithic operator `SensorPanel` with **separate, individually
configurable HUD panels**, each shown based on the active camera/state:

1. **Boom HUD** — boom angle, boom extension, platform level. Default anchor **bottom-right**.
2. **Docking Range HUD** — the five docking ranges (center, 45° left, 45° right, side left,
   side right). Default anchor **bottom-left**.
3. **Cutter Range HUD** — the cutter range reading, shown when the **cutting camera** view is
   active.

All three HUDs must be **large and legible from ~2 feet** (large panels, large value text, large
captions).

Behavior changes:

- On dashboard startup, the **docking camera is the default view** (already true from the previous
  change) and the **Boom + Docking HUDs are displayed first**.
- Pressing **key `2`** toggles the Boom + Docking HUDs, and:
  - if the camera is on **cutter**, it switches the camera **back to docking** and shows the HUDs;
  - if the HUDs are currently visible, `2` hides them;
  - if hidden, `2` shows them (switching to docking if needed).

Admin configuration via a **JSON file** (`--hud-config` flag), covering per-HUD: location,
caption names, panel size, value font size, caption font size, per-row captions, and
visibility-by-default.

## Current State (verified)

- `harvester_dashboard/qml/SensorPanel.qml` — one `Column` containing:
  - phase guide (`bridge.phaseGuideLine`),
  - boom block (`bridge.boomAngleLine` / `boomExtensionLine` / `levelLine`),
  - a "docking ranges" block (title + `Repeater` over `bridge.dockingRangeRows`) plus
    `bridge.cutterRangeLine` at the bottom.
- `harvester_dashboard/qml/HudOverlay.qml` — MIXED warning (top center), `SensorPanel` at
  `anchors.left/top`, right column with trunk+calibration and MQTT diagnostic panel, bottom
  stream-errors panel (diagnostic-gated).
- `harvester_dashboard/qml/Dashboard.qml:159` — `HudOverlay { visible: bridge.hudVisible }`
  (operator HUD, toggled by key `3`). Key `2` = `bridge.set_view("docking")`
  (`Dashboard.qml:55`), toolbar button "2 Docking".
- `harvester_dashboard/harvester_dashboard/bridge.py`:
  - `_get_docking_range_rows()` (line 568), `_get_cutter_range_line()` (582),
    `_get_boom_angle_line()` (595), `_get_boom_extension_line()` (602), `_get_level_line()`
    (609), `_get_phase_guide_line()` (618).
  - Properties at lines 819-828; signals `ranges_changed`, `boom_changed` at 43/45;
    `refresh()` (515) emits them every 200 ms.
  - `hudVisible` / `toggle_hud()` (key `3`).
- `harvester_dashboard/harvester_dashboard/config.py` — pure dataclass + argparse.
- `harvester_dashboard/harvester_dashboard/main.py` — builds config, bridge; QQuickView root;
  context property `bridge`.
- Range rows come from `model.snapshot_ranges()`; `v1/range/docking` records carry
  `telemetry_key` values `center_line`, `diagonal_left_45deg`, `diagonal_right_45deg`,
  `c_channel_left`, `c_channel_right` (`range_ingest.py:40`).
- Docs: `docs/orin_canonical_zmq.md` (Controls + "HUD layers" section), plus
  `.claude/skills/harvester-dashboard-hud/SKILL.md`.

## Design

### 1. HUD configuration (`harvester_dashboard/hud_config.json` + loader)

New module `harvester_dashboard/harvester_dashboard/hud_config.py` (pure Python, Qt-free, no new
third-party deps — use `json`):

```python
@dataclass
class HudPanelConfig:
    visible: bool
    anchor: str            # e.g. 'bottom-right', 'bottom-left', 'top-right', ...
    caption: str           # panel title text
    width: int
    height: int            # 0 = auto-size to content
    value_font_px: int
    caption_font_px: int
    rows: Dict[str, str]   # optional per-row / per-field display names
    margin_px: int
    opacity: float

@dataclass
class HudLayoutConfig:
    boom: HudPanelConfig
    docking: HudPanelConfig
    cutter_range: HudPanelConfig
```

- `load_hud_config(path: Optional[str]) -> HudLayoutConfig`:
  - No path / missing file → built-in defaults (below), no error.
  - A path is given but the file is malformed / unreadable → raise a clear `SystemExit`
    (fail fast, never silently show a broken HUD on an operator display).
  - Unknown keys are ignored (forward-compatible); missing per-HUD keys fall back to defaults.
- Anchor parsing: accept `top`/`bottom` + `left`/`right` (e.g. `bottom-right`) plus a
  `center`-horizontal variant `bottom-center`. Store as a normalized string; QML maps it.

**Default values (sized for ~2 ft readability, 1080p):**

| HUD | anchor | width | value font | caption font | caption | default visible |
|---|---|---|---|---|---|---|
| boom | `bottom-right` | 520 | 46 | 30 | `BOOM` | true |
| docking | `bottom-left` | 560 | 46 | 30 | `DOCKING RANGES` | true |
| cutter_range | `bottom-left` | 460 | 46 | 30 | `CUTTER RANGE` | true |

`height: 0` means auto-fit content. Per-row caption defaults:

- docking rows: `center_line`→`Center`, `diagonal_left_45deg`→`45° Left`,
  `diagonal_right_45deg`→`45° Right`, `c_channel_left`→`Side Left`,
  `c_channel_right`→`Side Right`.
- boom fields: `boom_angle_deg`→`Boom Angle`, `boom_extension_m`→`Boom Extension`,
  `platform_level`→`Platform Level`.

### 2. `DashboardConfig` / CLI wiring

In `config.py`:
- Add `hud_config_path: Optional[str] = None`.
- Add argparse `--hud-config` (default `None`).
- Pass through in `from_args`.

In `main.py`:
- `hud = load_hud_config(config.hud_config_path)`.
- Expose to QML. Two options: set a second context property, or add a `schemaGroup`-free
  `QVariantMap` property on the bridge. Simplest and least intrusive: **add
  `hudLayout` (QVariantMap) plus per-HUD nested maps as a `bridge` property**, built from
  `HudLayoutConfig`. The bridge already receives `config`; pass `hud` into `DashboardBridge`
  (new optional `hud_config` arg) and expose `hudLayout = Property('QVariantMap', ...)`.

The bridge is the single context property already; keep it that way.

### 3. Bridge changes (`bridge.py`)

- Import/accept `hud_config` (default built-in defaults when `None`, so existing tests that
  construct `DashboardBridge(config, model, annotation)` keep working).
- New state + slot:
  - `self._operator_huds_visible` (default from `hud_config` — boom/docking visible).
  - `@Slot() toggle_operator_huds()`:
    - if currently visible → hide (`_operator_huds_visible = False`), emit signal;
    - else → show, and **if `self._view != 'docking'` call `self.set_view('docking')`**
      (this reuses the existing render-only view switch and its `view_changed` handling).
  - `_get_operator_huds_visible()`, signal `operator_huds_visible_changed`.
  - Property `operatorHudsVisible`.
- New convenience getter for the cutter-range HUD rows so QML can render it uniformly:
  reuse `_get_cutter_range_line()`; add a `cutterRangeRow` property (or a single-row list) —
  keep it minimal: expose `cutterRangeLine` (exists) and let the Cutter HUD render one row.
- Expose `hudLayout` as `Property('QVariantMap', _get_hud_layout)` returning a plain dict:
  `{'boom': {...}, 'docking': {...}, 'cutter_range': {...}}` with keys `anchor`, `caption`,
  `width`, `height`, `valueFontPx`, `captionFontPx`, `marginPx`, `opacity`, `visible`, `rows`.
- Keep `dockingRangeRows` as-is; the QML applies config captions by mapping
  `modelData.key` → `hudLayout.docking.rows[modelData.key]` with fallback to the key.
- **Do not remove** `boomAngleLine`/`boomExtensionLine`/`levelLine`/`phaseGuideLine` getters if
  still used for value-only rows; refactor them into a structured boom-rows list so the Boom HUD
  can render caption/value pairs:
  - Add `_get_boom_rows()` → `[{'key': 'boom_angle_deg', 'label': ..., 'value': '+12.3°'}, ...]`
    and Property `boomRows` (`notify=boom_changed`). Remove the three `*Line` getters/Properties
    **only if** no QML references them after the refactor (guardrail: no dead code).
- `hudVisible` (key `3`) remains the master operator-HUD toggle for the whole `HudOverlay`
  (phase guide, trunk/calibration, MIXED warning). The new `operatorHudsVisible` gates the
  new Boom/Docking/Cutter panels. Key `2` drives `operatorHudsVisible`; key `3` continues to
  drive `hudVisible`. This keeps the two controls independent and preserves `3`'s existing tests.

### 4. QML changes

**New `BoomHud.qml`, `RangeHud.qml` (reusable for docking + cutter), or a single generic
`SensorHudPanel.qml`** driven by a config object + rows model. Prefer one generic component:

`SensorHudPanel.qml`:
- inputs: `caption`, `rows` (list of `{label, value, valid?}`), `panelWidth`, `panelHeight`,
  `valueFontPx`, `captionFontPx`, `opacity`.
- Renders a `Rectangle` with caption `Text` (caption font) and a `Column`/`Repeater` of rows,
  each a caption `Text` (caption font, muted color) + value `Text` (value font, large, colored
  by `valid`).
- Auto-height when `panelHeight === 0`.

**Rewrite `SensorPanel.qml`** into three separately anchored panels (or delete `SensorPanel.qml`
and place the panels directly in `HudOverlay.qml`):

- **RangeHud (docking)** anchored per `hudLayout.docking.anchor` (default bottom-left):
  `caption = hudLayout.docking.caption`, rows built from `bridge.dockingRangeRows` with labels
  from `hudLayout.docking.rows`.
- **BoomHud** anchored per `hudLayout.boom.anchor` (default bottom-right): rows from
  `bridge.boomRows`, labels from `hudLayout.boom.rows`.
- **CutterHud** anchored per `hudLayout.cutter_range.anchor`, `visible` only when
  `bridge.view === 'cutter'` AND `hudLayout.cutter_range.visible` AND
  `bridge.operatorHudsVisible`.
- **Remove** the cutter-range line from the old docking panel (it moves to CutterHud) and
  **remove** the old boom block from the left panel (it moves to BoomHud).
- The **phase guide** row: keep it, but move it to the docking HUD caption area (or a small
  separate panel) so boom/docking/cutter split is clean. Decision: keep phase guide as the
  docking HUD's subtitle row (config caption + phase line beneath).
- All three panels gated by `bridge.operatorHudsVisible` **and** the per-HUD `visible` config.
- Anchor helper: a small QML function `anchorFor(name)` returning which `anchors.*` to set;
  implement with a JS function + conditional `anchors` assignments (QtQuick 2 primitives only,
  no Controls2).

**`Dashboard.qml`**:
- Key `2` → `bridge.toggle_operator_huds()` (replaces `bridge.set_view("docking")`).
- Toolbar "2 Docking" button → same slot; update its active-border binding to
  `bridge.operatorHudsVisible`.
- Keep key `1` (cutter view), key `3` (whole operator HUD) unchanged.

### 5. Legibility

- Default `value_font_px: 46`, `caption_font_px: 30`, panel widths ~520-560, generous row
  spacing and padding; semi-opaque dark background (`opacity 0.85`) for contrast. These are
  config-overridable so an admin can tune for the actual monitor.

## Files to Change

1. **New** `harvester_dashboard/harvester_dashboard/hud_config.py` — dataclasses + JSON loader +
   defaults + validation.
2. **New** `harvester_dashboard/hud_config.json` — shipped default config (mirrors built-in
   defaults, self-documenting for admins).
3. `harvester_dashboard/harvester_dashboard/config.py` — `hud_config_path` field + `--hud-config`.
4. `harvester_dashboard/harvester_dashboard/main.py` — load config, pass to bridge.
5. `harvester_dashboard/harvester_dashboard/bridge.py` — `hud_config` arg, `operatorHudsVisible`
   state/slot/property, `hudLayout` property, `boomRows` property (replace `*Line` boom getters
   if unused).
6. `harvester_dashboard/qml/SensorHudPanel.qml` — **new** generic panel component.
7. `harvester_dashboard/qml/SensorPanel.qml` — rewrite into Boom + Docking (+ phase guide) panels,
   anchored from config; **or** delete and inline into `HudOverlay.qml`.
8. `harvester_dashboard/qml/HudOverlay.qml` — place Boom / Docking / Cutter panels, gate on
   `bridge.operatorHudsVisible`.
9. `harvester_dashboard/qml/Dashboard.qml` — key `2` + toolbar button wiring.
10. `docs/orin_canonical_zmq.md` — update Controls (key `2` behavior) and "HUD layers" section;
    document the `--hud-config` file and its schema.
11. `.claude/skills/harvester-dashboard-hud/SKILL.md` — update HUD layout/key map description.

## Tests to Add / Update

- **New** `harvester_dashboard/test/test_hud_config.py`:
  - defaults when path is `None`;
  - load a valid JSON override (location, captions, sizes, per-row names, visibility);
  - malformed JSON / missing explicit file → clear failure;
  - unknown keys ignored; partial override falls back to defaults.
- **New/extend** bridge tests:
  - `operatorHudsVisible` starts per config (true for default);
  - `toggle_operator_huds()` hides then shows;
  - showing while `view == 'cutter'` switches the view to `docking` and emits `view_changed`;
  - render-only: no socket traffic (extend `test_no_emit_proof.py`).
- **Update** `test_smoke_gui.py` — it asserts `hudVisible` toggling (key `3`); add
  `operatorHudsVisible` + `toggle_operator_huds` assertions; confirm `boomRows`/`hudLayout`
  render.
- **Update** `test_gui_acceptance.py` — key `2` now toggles HUD / forces docking (the current
  test uses key `2`/`1` to switch views; rework that step to use the new semantics and still
  leave `cutter` active for the annotation/stale steps, or switch via a direct `set_view`).
- Confirm the two known pre-existing NVDEC JPEG failures remain the only failures.

## Verification

```bash
cd ~/harvester_vision
PYTHONPATH=harvester_dashboard /usr/bin/python3 -m unittest discover -s harvester_dashboard/test -v
# QML syntax / GUI smoke (offscreen)
QT_QPA_PLATFORM=offscreen PYTHONPATH=harvester_dashboard /usr/bin/python3 \
  harvester_dashboard/test/test_gui_acceptance.py
```

Manual (if display available): start dashboard → docking view with Boom HUD bottom-right and
Docking Range HUD bottom-left at large font; press `2` to hide, again to show; switch to cutter
(`1`) → Boom/Docking HUDs hidden + Cutter Range HUD shown; press `2` → returns to docking and
shows Boom/Docking HUDs; edit `hud_config.json` (move anchors, rename captions, grow fonts) and
relaunch to confirm.

## Notes / Guardrails

- Keep `bridge` as the single QML context property; no new socket writes (render-only).
- Keep `telemetry_model` Qt-free; all Qt types stay in `bridge.py`.
- QtQuick 2 primitives only (PySide2 5.14, no Controls2).
- When replacing `SensorPanel` contents, remove now-unused bridge getters/Properties to avoid
  dead code, but keep valid `TelemetryModel` APIs and their tests.
- The prior plan `dev-diagnostic-hud-toggle.md` remains valid; `diagnosticVisible` gating is
  untouched.

## Open Defaults Chosen (adjustable during implementation)

- Phase guide stays with the Docking HUD as a subtitle row.
- `operatorHudsVisible` default = `hud_layout.boom.visible && hud_layout.docking.visible`
  (true by default), so startup shows Boom + Docking HUDs as requested.
- Cutter Range HUD is always gated on the cutter view being active, regardless of key `2`.
