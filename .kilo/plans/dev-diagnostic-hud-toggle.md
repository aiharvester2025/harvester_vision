# Developer-Diagnostic HUD Toggle (777 + Enter)

**Goal.** Separate the dashboard HUD into two independently-controlled layers:

1. **Operator HUD** — the operational guidance (phase guide, boom/leveling,
   docking ranges, cutter range, trunk + calibration, MIXED-SOURCES warning).
   Already toggled by key `3` (and the "3 HUD" toolbar button) via
   `bridge.hudVisible` / `HudOverlay`.
2. **Developer-diagnostic HUD** — the health/diagnostic overlays useful only
   when debugging (per-channel stream table, packet recv/drop counters, the
   status/recording line, the active-camera timestamp line, the stale ring).

We add a **new toggle** for layer 2, bound to a **numeric keypad-like sequence
`777` followed by `Enter`**, so a developer can hide/show the diagnostics
without affecting the operator HUD (which keeps its `3` toggle).

## 1. Current state (verified by reading the code)

- `harvester_dashboard/qml/Dashboard.qml` — root; owns all `Keys.onPressed`
  handling (lines 14–25) and the toolbar (lines 28–92). Keys are matched one
  at a time via `event.key` against single `Qt.Key_*` values; there is **no**
  multi-key sequence state today.
- `harvester_dashboard/harvester_dashboard/bridge.py` — `DashboardBridge`
  exposes `hudVisible` (operator layer) with `toggle_hud()` slot. No
  diagnostic-layer state exists.
- `harvester_dashboard/qml/HudOverlay.qml` — contains BOTH layers today:
  - Operator: `SensorPanel` (left), trunk+calibration panel (right), MIXED
    warning (top-center).
  - Diagnostic: the bottom **stream errors panel** (lines 60–134: "streams
    recv … drops …" + per-channel `Repeater`).
- `harvester_dashboard/qml/CameraView.qml` — operator camera image, plus two
  diagnostic overlays: the **stale ring** (lines 45–52) and the **active
  timestamp line** (lines 79–86).
- `harvester_dashboard/qml/Dashboard.qml` toolbar — a `Text` (lines 85–91)
  showing `bridge.sourceBadge` (operator: mode + source id) and
  `bridge.statusLine` (diagnostic: "status: … | drops … | rec on/off").

## 2. What is developer-diagnostic (to gate behind 777) vs operator

| Element | Location | Layer |
|---|---|---|
| Phase guide + boom/leveling + ranges | `SensorPanel.qml` | operator |
| Trunk + calibration panel | `HudOverlay.qml` right | operator |
| MIXED SOURCES warning | `HudOverlay.qml` top | operator |
| **Stream errors panel** (recv/drops + rows) | `HudOverlay.qml` bottom | **diagnostic** |
| **Status line** (status/drops/rec) | `Dashboard.qml` toolbar | **diagnostic** |
| **Active timestamp line** (seq/clock) | `CameraView.qml` bottom | **diagnostic** |
| **Stale ring** | `CameraView.qml` | **diagnostic** |
| Source badge (mode + source id) | `Dashboard.qml` toolbar | operator |

## 3. Design

### 3.1 Bridge: new diagnostic-layer state (Python)
Add to `DashboardBridge`:
- `diagnostic_visible_changed = Signal()`
- `self._diagnostic_visible = True` (default on, matching today)
- `@Slot() toggle_diagnostic()` — flips the flag, emits the signal.
- `_get_diagnostic_visible() -> bool`
- Property `diagnosticVisible = Property(bool, _get_diagnostic_visible,
  notify=diagnostic_visible_changed)`

(The `777`+Enter key *sequence* is handled in QML; the bridge only needs the
toggle slot + property.)

### 3.2 QML: multi-key sequence detection (Dashboard.qml)
Qt Quick `Keys.onPressed` reports single keys only, so accumulate digits in a
small buffer and match on `Enter`/`Return`:

- Add `property string key_buffer: ""`.
- In `Keys.onPressed`, before the existing single-key checks:
  - If `event.key` is a digit `Qt.Key_0..Qt.Key_9`, append the digit to
    `key_buffer`, cap length at ~8, `event.accepted = true`, and return.
  - If `event.key === Qt.Key_Enter || Qt.Key_Return`:
    - if `key_buffer === "777"` → `bridge.toggle_diagnostic()`;
    - clear `key_buffer`; `event.accepted = true`; return.
  - Otherwise (any non-digit, non-enter key) clear `key_buffer` and fall
    through to the existing single-key handling.
- Guard against the buffer accumulating stale digits: clear it on any non-digit
  keypress (including the existing `1/2/3/4/...` handlers).

Note: the existing single-key bindings for `0..9` include key `0` (clear
annotation) and key `7` (IMU). We must ensure the digit-buffering branch runs
first and `return`s so pressing `7` then `7` then `7` then `Enter` toggles
diagnostics, while a bare `7` (with no Enter) still toggles IMU. This means
**decouple** single-key `7`/`0` handling from the buffer: buffer digits first;
only treat `7`/`0` as their single-key actions when they are not part of a
pending `777` sequence. Simplest correct rule: buffer *all* digit presses and
only act on Enter; but that would break the existing single-key `0` (clear) and
`7` (IMU) shortcuts.

**Resolution (keeps all existing behavior):** treat the `777`+Enter sequence as
requiring an *uninterrupted* run of digits ending in Enter. Specifically:
- Pressing a digit **does not immediately** trigger the digit's normal action.
- On `Enter`, if the buffer is exactly `777`, toggle diagnostics.
- On `Enter` with any other buffer, discard.
- If the operator wants the *old* single-key `7` (IMU) or `0` (clear) behavior,
  they press those keys **without** following with Enter — but since digits are
  buffered, we need a timeout-free decision.

To avoid regressing `7`/`0`, the cleanest approach is a **short key-buffer
timeout** OR to require Enter to commit any digit sequence. Given the user
explicitly asked for "777 + Enter", we adopt: **digits accumulate; Enter commits
the sequence; any non-digit key flushes the buffer and is handled normally.**
The single-key `0` (clear) and `7` (IMU) remain reachable because those actions
are bound to the *keyboard* `0`/`7` too — but we must not silently break them.

**Final, unambiguous behavior:**
- Add a dedicated QML `Timer` (`key_buffer_timer`, ~800 ms) that flushes
  `key_buffer` to empty. This lets a lone `7` (paused ≥800 ms) fall through to
  the IMU toggle, while a fast `7-7-7-Enter` commits the diagnostic toggle.
- Digits set/append the buffer and restart the timer; Enter commits and clears.
- A non-digit key flushes the buffer immediately and proceeds to its own action.

Because this adds a small timer, we also keep the existing single-key `7`/`0`
handlers intact for robustness (they only run when the buffer was empty because
the timer flushed it).

### 3.3 QML: gate the diagnostic elements
Wrap each diagnostic element's `visible` with `bridge.diagnosticVisible`:
- `HudOverlay.qml` bottom **stream errors panel** → `visible:
  bridge.diagnosticVisible`.
- `CameraView.qml` **active timestamp line** → `visible:
  bridge.diagnosticVisible`.
- `CameraView.qml` **stale ring** → `visible: bridge.activeCameraStale &&
  bridge.diagnosticVisible`.
- `Dashboard.qml` toolbar **status line** — split the combined Text so the
  source badge (operator) stays and the status/drops/rec part becomes a separate
  Text gated on `bridge.diagnosticVisible`.

### 3.4 Keep `hudVisible` (key `3`) scoped to the operator layer only
The operator `HudOverlay` already contains the diagnostic stream panel; after
this change the stream panel inside `HudOverlay` is additionally gated by
`diagnosticVisible`, so:
- `3` toggles the whole operator HUD (and the stream panel hides with it).
- `777`+Enter toggles only the diagnostic sub-panels (stream table, status line,
  timestamp line, stale ring), independent of `hudVisible`.

## 4. Files to change

1. `harvester_dashboard/harvester_dashboard/bridge.py`
   - Add `diagnostic_visible_changed` signal, `_diagnostic_visible` state,
     `toggle_diagnostic()` slot, `_get_diagnostic_visible()`, and the
     `diagnosticVisible` Property.
2. `harvester_dashboard/qml/Dashboard.qml`
   - Add `key_buffer` property + `key_buffer_timer`.
   - Extend `Keys.onPressed` to buffer digits and commit `777`+Enter.
   - Split the toolbar status `Text` into source-badge (operator) and
     status-line (diagnostic, gated).
3. `harvester_dashboard/qml/HudOverlay.qml`
   - Gate the bottom stream-errors `Rectangle` on `bridge.diagnosticVisible`.
4. `harvester_dashboard/qml/CameraView.qml`
   - Gate the stale ring and the active-timestamp line on
     `bridge.diagnosticVisible`.

## 5. Verification

- `bash -n` n/a; instead:
  - Python: `python3 -c "import ast; ast.parse(...)"` on `bridge.py`.
  - Dashboard headless test suite still green
    (`PYTHONPATH=harvester_dashboard /usr/bin/python3 -m unittest discover -s
    harvester_dashboard/test -v`); the 2 pre-existing NVDEC JPEG failures are
    unrelated.
- Manual (if GUI available): run dashboard; press `3` toggles operator HUD
  (phase/ranges/trunk); press `7`,`7`,`7`,`Enter` toggles only the stream
  table + status + timestamp + stale ring; verify `1/2/4/5/6/7/0/Esc` still work
  (including single `7` IMU and single `0` clear after the 800 ms buffer flush).
- Confirm no socket is written by the new toggle (render-only, matches the
  existing safety boundary).

## 6. Delivery order

1. Bridge: diagnostic state + property + slot.
2. Dashboard.qml: key buffer + timer + Enter commit + split status line.
3. HudOverlay.qml + CameraView.qml: gate diagnostic elements.
4. Verify (syntax + headless tests + manual GUI if available).
