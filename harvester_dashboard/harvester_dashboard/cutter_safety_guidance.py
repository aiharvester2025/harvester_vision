"""Cutter safety-guidance model (pure Python, no Qt).

Encodes the operator-facing guidance for the **cutting arm** as a small,
unit-testable pure function suite.  It mirrors the docking safety-guidance
model (see ``safety_guidance.py``) but governs the **cutter tip's** approach to
the cutting object (trunk / FFB / frond) and the subsequent cut sequence.

Measurement
-----------
The **only** clearance measurement is the single forward range sensor
``cutting_tool_left_range`` (a child of ``cutting_tool_link``, so it follows the
rail yaw, arm lift, and arm extension).  The depth camera and LiDAR are mounted
on ``cutting_arm_base_link`` and do **not** follow the extension, so they cannot
see the tip -- there is no fusion input.

The sensor sits **behind** the cutter tip, so the raw reading overstates the
true tip clearance.  The model applies an explicit, tunable offset:

    tip_clearance_m = range_m - sensor_to_tip_offset_m

where ``sensor_to_tip_offset_m`` is the forward distance from the sensor face to
the cutter tip (URDF-derived; ~0.19 m for the reference tool).

Model (stopping distance + advisory time-to-collision)
------------------------------------------------------
Two inputs drive the state:

  * ``speed_cm_s``     -- tip closing speed toward the object (positive =
                          clearance shrinking), derived from -d(clearance)/dt.
  * ``clearance_m``    -- offset-corrected tip-to-object distance.

  d_stop(v) = v*t_latency + v^2 / (2*a_max)
  v_max(d)  = -a_max*t_latency + sqrt((a_max*t_latency)^2 + 2*a_max*d)

The state is driven by the **clearance floor and the stopping-distance bound**.
TTC (``clearance / speed``) is computed and reported as a glanceable advisory
number only; it does not itself latch a state (unlike the docking model, which
also has advisory TTC bands).

States (mirrors docking): SAFE / WARN / DANGER / NO_DATA.

A clearance at or below the danger floor -- including a **zero or negative**
clearance (raw range smaller than the sensor->tip offset, i.e. the tip pressing
on the object) -- is DANGER, unlike the docking model where a non-positive gap is
NO_DATA.  The cutter measures *clearance from the tip*, so contact/pressing is a
real, dangerous measurement rather than a missing one.

Cut sequence
------------
A small guidance phase machine runs on top of the clearance state.  It prompts
the operator through the cut; it never actuates anything (the scissors and
gripper are hydraulic operator actions with no simulated joints):

  APPROACH -> ALIGN (STOP, ready to cut) -> OPEN -> ADVANCE -> CUT

APPROACH -> ALIGN fires when the tip is inside the ready band and settled (not
warn-speed) and **not** inside the danger floor.  The ready band sits inside the
WARN clearance by design, so ALIGN legitimately coincides with a stable WARN
"ready-to-cut" state; it must never coincide with DANGER.

This is the **advisory/operator-facing** layer only: it never commands motion.
"""

from __future__ import annotations

import json
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Optional

SAFE = 'safe'
WARN = 'warn'
DANGER = 'danger'
NO_DATA = 'no_data'

# Cut-sequence phases.
PHASE_IDLE = 'idle'
PHASE_APPROACH = 'approach'
PHASE_ALIGN = 'align'      # tip at the ready standoff: STOP, ready to cut
PHASE_OPEN = 'open'        # prompt: open the scissors wide
PHASE_ADVANCE = 'advance'  # prompt: move the cutter forward the tuned distance
PHASE_CUT = 'cut'          # prompt: perform the cut (gripper holds the FFB)

PHASES = (PHASE_IDLE, PHASE_APPROACH, PHASE_ALIGN, PHASE_OPEN, PHASE_ADVANCE,
          PHASE_CUT)


@dataclass(frozen=True)
class CutterConfig:
    """Tunable thresholds (loaded from cutter_safety_guidance.json)."""

    # Sensor -> tip forward offset (m).  tip_clearance = range - offset.
    sensor_to_tip_offset_m: float = 0.19

    # Physical model.
    a_max_m_s2: float = 0.10          # max safe cutter deceleration (m/s^2)
    latency_s: float = 0.30           # operator reaction + actuation latency (s)
    warn_margin: float = 0.7          # WARN fires at this fraction of v_max(d)

    # Absolute clearance floors.
    warn_clearance_m: float = 0.30
    danger_clearance_m: float = 0.10

    # Cut-sequence geometry.
    ready_standoff_m: float = 0.20    # tip-to-object distance to "STOP, ready"
    advance_distance_m: float = 0.05  # forward move before cutting
    align_tolerance_m: float = 0.05   # band around ready_standoff = aligned

    # Motion / staleness handling.
    stationary_epsilon_cm_s: float = 0.5
    stale_s: float = 2.0
    debounce_s: float = 0.4
    speed_ema_alpha: float = 0.35

    @classmethod
    def load(cls, path: Optional[Path] = None) -> 'CutterConfig':
        """Load thresholds from a JSON file, falling back to defaults.

        Missing keys keep their dataclass defaults; malformed/absent files
        return the default configuration rather than raising (the HUD must
        never crash over a tuning file).  Out-of-range values are sanitized so
        a bad tuning file can never make the HUD divide-by-zero or produce a
        non-monotonic envelope.
        """
        if path is None:
            return cls()
        try:
            raw = json.loads(Path(path).read_text(encoding='utf-8'))
        except (OSError, json.JSONDecodeError):
            return cls()
        if not isinstance(raw, dict):
            return cls()
        values = {k: raw[k] for k in (
            'sensor_to_tip_offset_m', 'a_max_m_s2', 'latency_s', 'warn_margin',
            'warn_clearance_m', 'danger_clearance_m', 'ready_standoff_m',
            'advance_distance_m', 'align_tolerance_m',
            'stationary_epsilon_cm_s', 'stale_s', 'debounce_s',
            'speed_ema_alpha') if k in raw}
        try:
            cfg = cls(**values)
            cfg = cls(
                sensor_to_tip_offset_m=float(cfg.sensor_to_tip_offset_m),
                # a_max must be positive (division); warn_margin in [0,1].
                a_max_m_s2=max(1e-3, float(cfg.a_max_m_s2)),
                latency_s=max(0.0, float(cfg.latency_s)),
                warn_margin=min(1.0, max(0.0, float(cfg.warn_margin))),
                warn_clearance_m=max(0.0, float(cfg.warn_clearance_m)),
                # The danger band must sit inside the warn band, otherwise the
                # danger-by-clearance floor becomes unreachable and the advisory
                # silently loses that escalation.
                danger_clearance_m=min(
                    max(0.0, float(cfg.danger_clearance_m)),
                    max(0.0, float(cfg.warn_clearance_m))),
                ready_standoff_m=max(0.0, float(cfg.ready_standoff_m)),
                advance_distance_m=max(0.0, float(cfg.advance_distance_m)),
                align_tolerance_m=max(0.0, float(cfg.align_tolerance_m)),
                stationary_epsilon_cm_s=float(cfg.stationary_epsilon_cm_s),
                stale_s=max(0.0, float(cfg.stale_s)),
                debounce_s=max(0.0, float(cfg.debounce_s)),
                speed_ema_alpha=min(1.0, max(0.0, float(cfg.speed_ema_alpha))),
            )
        except (TypeError, ValueError):
            return cls()
        return cfg


@dataclass(frozen=True)
class CutterGuidance:
    """One evaluation result (clearance state + cut-sequence phase)."""

    state: str                 # SAFE / WARN / DANGER / NO_DATA
    phase: str                 # cut-sequence phase
    clearance_m: Optional[float]     # offset-corrected tip-to-object distance
    speed_cm_s: Optional[float]      # smoothed tip closing speed (+ = closing)
    ttc_s: Optional[float]     # None when speed ~0 / no valid TTC
    stop_distance_m: Optional[float]
    max_speed_cm_s: Optional[float]
    recommended_speed_cm_s: Optional[float]
    message: str               # human guidance line
    phase_message: str         # operator prompt for the current cut phase

    @property
    def approaching(self) -> bool:
        return self.speed_cm_s is not None and self.speed_cm_s > 0.0


def tip_clearance_m(range_m: float, cfg: CutterConfig) -> float:
    """Offset-corrected tip-to-object clearance from a raw range reading.

    The sensor sits behind the tip, so the raw range overstates the true
    clearance; subtract the sensor-to-tip forward offset.
    """
    return float(range_m) - cfg.sensor_to_tip_offset_m


def _clamp_speed_for_ttc(speed_cm_s: float, min_speed_cm_s: float = 0.05) -> float:
    """Clamp speed to a floor so TTC is finite and monotonic near zero."""
    return max(speed_cm_s, min_speed_cm_s)


def _stop_distance_m(speed_cm_s: float, cfg: CutterConfig) -> float:
    """Stopping distance (m) from closing speed v = speed_cm_s (cm/s)."""
    v = max(0.0, speed_cm_s) / 100.0
    a_max = max(1e-3, cfg.a_max_m_s2)  # guard a hand-built zero config
    return v * cfg.latency_s + (v * v) / (2.0 * a_max)


def _max_safe_speed_cm_s(clearance_m: float, cfg: CutterConfig) -> float:
    """v_max(d) in cm/s: the max closing speed that can still stop within d."""
    a = max(1e-3, cfg.a_max_m_s2)      # guard a hand-built zero config
    t = cfg.latency_s
    d = max(0.0, clearance_m)
    v = -a * t + math.sqrt((a * t) * (a * t) + 2.0 * a * d)
    return v * 100.0


def phase_message(phase: str, cfg: CutterConfig) -> str:
    """Operator prompt for a cut-sequence phase."""
    if phase == PHASE_APPROACH:
        return 'approaching — move cutter to the object'
    if phase == PHASE_ALIGN:
        return 'STOP — ready to cut (open scissors wide next)'
    if phase == PHASE_OPEN:
        return 'OPEN the cutting scissors WIDE'
    if phase == PHASE_ADVANCE:
        return 'ADVANCE cutter forward {:.0f} cm'.format(
            cfg.advance_distance_m * 100.0)
    if phase == PHASE_CUT:
        return 'CUT now (gripper holds the FFB)'
    return 'cut guidance idle'


def state_message(state: str, g: CutterGuidance) -> str:
    """A human message consistent with a *held* clearance state.

    Used by the bridge during hysteresis debounce: when the displayed state is
    still the old one but the candidate has advanced, the message must match
    the shown colour (e.g. never "STOP" against an orange WARN banner).
    """
    if state == DANGER:
        return 'STOP — cutter tip too close'
    if state == WARN:
        if g.clearance_m is not None:
            rec = g.recommended_speed_cm_s
            return (f'SLOW — {rec:.0f} cm/s max ({g.clearance_m:.2f} m)'
                    if rec is not None else
                    f'SLOW ({g.clearance_m:.2f} m)')
        return 'SLOW'
    if state == NO_DATA:
        return _no_data_message(g)
    return (f'tip clear ({g.clearance_m:.2f} m)'
            if g.clearance_m is not None else 'tip clear')


def _no_data_message(g) -> str:
    """The NO_DATA message (single definition).

    ``evaluate()`` and ``state_message()`` both call this so a debounced hold can
    never show NO_DATA text that disagrees with what ``evaluate()`` produced.  A
    guidance with no usable clearance (missing, zero, or non-finite) reports the
    clearance as unavailable; otherwise the range has simply not arrived yet.

    Accepts either a ``CutterGuidance`` or a raw clearance, so both callers share
    it (mirrors ``safety_guidance._no_data_message``).
    """
    if isinstance(g, CutterGuidance):
        clearance = g.clearance_m
    else:
        clearance = g
    if clearance is None:
        return 'cutter: awaiting range'
    try:
        usable = math.isfinite(float(clearance)) and float(clearance) > 0.0
    except (TypeError, ValueError, OverflowError):
        usable = False
    return 'cutter: awaiting range' if usable else 'cutter: range unavailable'


def evaluate(
        speed_cm_s: Optional[float],
        clearance_m: Optional[float],
        cfg: CutterConfig,
        *,
        stale: bool = False,
        phase: str = PHASE_IDLE,
        min_ttc_speed_cm_s: float = 0.05,
) -> CutterGuidance:
    """Evaluate the cutter clearance state for a (speed, clearance) reading.

    ``clearance_m`` is the **offset-corrected** tip-to-object distance.  Either
    input may be None, or ``stale`` may be set, to yield NO_DATA.

    A **non-finite** (NaN/Inf) reading is also NO_DATA: JSON accepts the bare
    ``NaN``/``Infinity`` literals, and every comparison against NaN is false, so
    an unguarded NaN clearance would fall through every DANGER/WARN test and be
    reported as SAFE.  A reassuring green banner on unreadable range data is the
    one failure this model must never produce.

    ``phase`` is the caller-held cut-sequence phase (the state machine advances
    in the bridge); it is echoed into the result with its operator message.
    Hysteresis is intentionally NOT applied here (stateless evaluation); the
    bridge holds the debounce across frames.
    """
    pmsg = phase_message(phase, cfg)

    if speed_cm_s is None or clearance_m is None or stale:
        return CutterGuidance(NO_DATA, phase, clearance_m, None, None, None,
                              None, None, _no_data_message(clearance_m), pmsg)

    speed = float(speed_cm_s)
    clear = float(clearance_m)

    # Reject non-finite readings before any comparison: NaN compares false
    # against every bound, so it would otherwise be reported as SAFE.
    if not (math.isfinite(speed) and math.isfinite(clear)):
        return CutterGuidance(NO_DATA, phase, clear, speed, None, None, None,
                              None, _no_data_message(clear), pmsg)

    # Absolute floor: the tip is inside the danger clearance (even stationary).
    if clear <= cfg.danger_clearance_m:
        return CutterGuidance(
            DANGER, phase, clear, speed, None,
            _stop_distance_m(speed, cfg), _max_safe_speed_cm_s(clear, cfg), 0.0,
            f'STOP — tip {clear:.2f} m from object', pmsg)

    approaching = speed > cfg.stationary_epsilon_cm_s

    ttc_s: Optional[float] = None
    if approaching:
        v = _clamp_speed_for_ttc(speed, min_ttc_speed_cm_s)
        ttc_s = clear / (v / 100.0)

    stop_dist = _stop_distance_m(speed, cfg)
    v_max = _max_safe_speed_cm_s(clear, cfg)

    # DANGER by stopping distance: cannot stop within the remaining clearance.
    if approaching and stop_dist >= clear:
        v_warn = cfg.warn_margin * v_max
        return CutterGuidance(
            DANGER, phase, clear, speed, ttc_s, stop_dist, v_max, v_warn,
            f'STOP NOW — {clear:.2f} m clearance, need {stop_dist:.2f} m',
            pmsg)

    # WARN: inside the warn clearance or exceeding the warning speed fraction
    # of the physical stop limit.
    warn_by_clearance = clear <= cfg.warn_clearance_m
    warn_by_speed = approaching and speed > (cfg.warn_margin * v_max)
    if warn_by_clearance or warn_by_speed:
        v_warn = cfg.warn_margin * v_max
        if warn_by_speed and ttc_s is not None:
            return CutterGuidance(
                WARN, phase, clear, speed, ttc_s, stop_dist, v_max, v_warn,
                f'SLOW to {v_warn:.0f} cm/s ({ttc_s:.1f}s to impact)', pmsg)
        return CutterGuidance(
            WARN, phase, clear, speed, ttc_s, stop_dist, v_max, v_warn,
            f'SLOW — {v_warn:.0f} cm/s max ({clear:.2f} m)', pmsg)

    return CutterGuidance(SAFE, phase, clear, speed, ttc_s, stop_dist, v_max,
                          None, f'tip clear ({clear:.2f} m)', pmsg)


def next_phase(phase: str, clearance_m: Optional[float],
               speed_cm_s: Optional[float],
               cfg: CutterConfig, *, stale: bool = False) -> str:
    """Advance the cut-sequence phase from the current observation.

    The measured quantity drives APPROACH -> ALIGN; the remaining phases are
    operator actions with no sensor, so they advance only when explicitly
    requested (``advance_phase``).  A brief stale/missing range must NOT wipe an
    operator's in-progress ALIGN/OPEN/ADVANCE/CUT steps, so those phases are held
    through a dropout.

    APPROACH -> ALIGN requires ALL of:

      * the tip is inside the ready band
        (``clearance <= ready_standoff + align_tolerance``);
      * the tip has **settled** and is not being warned to slow: the closing
        speed is at or below ``warn_margin * v_max(clearance)``; and
      * the clearance is **not** inside the danger floor.

    The ready band upper bound (``ready_standoff + align_tolerance``, 0.25 m by
    default) deliberately sits *inside* ``warn_clearance_m`` (0.30 m): the tip is
    expected to settle in a stable WARN "ready-to-cut" state, so ALIGN and WARN
    legitimately coincide (the shipped tuning validated this).  ALIGN must never
    coincide with DANGER, however, so the danger floor is an explicit guard: the
    "ready to cut" prompt can never appear while the tip is inside the danger
    band.  An unknown/non-finite closing speed counts as *not settled*.
    """
    if phase not in (PHASE_IDLE, PHASE_APPROACH):
        # Operator-confirmed phases hold; a range dropout does not reset them.
        return phase
    if stale or clearance_m is None:
        return PHASE_APPROACH if phase != PHASE_IDLE else PHASE_IDLE
    # Never claim ready-to-cut inside the danger floor.
    if clearance_m <= cfg.danger_clearance_m:
        return PHASE_APPROACH
    if speed_cm_s is None:
        return PHASE_APPROACH
    try:
        speed = float(speed_cm_s)
    except (TypeError, ValueError, OverflowError):
        speed = float('inf')
    if not math.isfinite(speed):
        # An unknown closing speed is not "settled": do not claim ready-to-cut.
        return PHASE_APPROACH
    in_ready_band = clearance_m <= cfg.ready_standoff_m + cfg.align_tolerance_m
    settled = speed <= cfg.warn_margin * _max_safe_speed_cm_s(clearance_m, cfg)
    if in_ready_band and settled:
        return PHASE_ALIGN
    return PHASE_APPROACH


def advance_phase(phase: str) -> str:
    """Advance the operator-confirmed part of the cut sequence by one step.

    ``align -> open -> advance -> cut`` (terminal at ``cut``).  Any other phase
    (idle/approach) is treated as not-yet-aligned and starts the confirmed
    sequence at ``align``.
    """
    order = (PHASE_ALIGN, PHASE_OPEN, PHASE_ADVANCE, PHASE_CUT)
    if phase in order:
        idx = order.index(phase)
        return order[idx + 1] if idx + 1 < len(order) else PHASE_CUT
    return PHASE_ALIGN


def default_config_path() -> Path:
    """Best-effort path to the tuning file next to this module."""
    return Path(__file__).resolve().parent.parent / 'config' / \
        'cutter_safety_guidance.json'


__all__ = [
    'SAFE', 'WARN', 'DANGER', 'NO_DATA',
    'PHASE_IDLE', 'PHASE_APPROACH', 'PHASE_ALIGN', 'PHASE_OPEN',
    'PHASE_ADVANCE', 'PHASE_CUT', 'PHASES',
    'CutterConfig', 'CutterGuidance',
    'tip_clearance_m', 'evaluate', 'next_phase', 'advance_phase',
    'phase_message', 'state_message', 'default_config_path',
]