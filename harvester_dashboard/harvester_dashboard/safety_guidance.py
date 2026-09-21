"""Docking approach safety-guidance model (pure Python, no Qt).

Encodes the operator guidance logic as a small, unit-testable pure function
suite.  The model maps (closing speed, distance-to-trunk) onto a discrete
safety state (SAFE / WARN / DANGER / NO_DATA) plus a human guidance message,
a recommended (maximum safe) speed, and the stopping-distance geometry used
by the HUD.

Model (stopping distance + time-to-collision)
---------------------------------------------
The guidance is for the **c-channel platform** approaching the **trunk** during
the final docking phase (boom elevation + extension + leveling).  The two inputs
are:

  * ``speed_cm_s``  -- platform closing speed toward the trunk (positive =
                       gap shrinking), derived from -d(center_range)/dt.
  * ``distance_m``  -- forward gap from the c-channel to the trunk surface
                       (the ``center_line`` range record on ``v1/range/docking``).

Two independent physical quantities drive the state:

  Stopping distance (the authoritative speed bound)
    d_stop(v) = v*t_latency + v^2 / (2*a_max)

    The gap needed to stop from closing speed ``v`` given a maximum safe
    deceleration ``a_max`` and reaction/actuation latency ``t_latency``.
    Solving d = d_stop(v) for v gives the maximum safe closing speed at a gap:

      v_max(d) = -a_max*t_latency + sqrt((a_max*t_latency)^2 + 2*a_max*d)

    This is the physically correct, distance-aware "slow to X cm/s" figure:
    closer -> slower, and it honours the platform's actual deceleration
    capability (unlike ``d / warn_ttc_s``).

  Time-to-collision (the glanceable advisory)
    TTC = d / v   (seconds to impact at the current speed)

State assignment (with hysteresis applied by the caller via ``debounce_s``):

  NO_DATA   no valid range, or range stale (age > stale_s)          (grey)
  SAFE      moving away (v < 0), OR stationary outside danger distance,
            OR (d > warn_distance AND TTC > warn_ttc AND v <= v_warn(d))
  WARN      d <= warn_distance  OR  TTC <= warn_ttc  OR  v > v_warn(d)
  DANGER    d <= d_stop(v)      OR  d <= danger_distance

where ``v_warn(d) = warn_margin * v_max(d)`` so WARN fires *before* the physical
stop limit, leaving the operator headroom.  ``warn_margin`` in (0, 1).

A stationary platform (|v| < epsilon) is never DANGER by stopping distance
(no motion), but one already inside ``danger_distance_m`` is DANGER (too close
to linger).  A platform moving away is always SAFE.

This is the **advisory/operator-facing** layer only: it never commands motion.
The hard-contact authority is the real machine's PLC/safety layer, outside this
dashboard.
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


@dataclass(frozen=True)
class SafetyConfig:
    """Tunable thresholds (loaded from safety_guidance.json)."""

    # Physical model.
    a_max_m_s2: float = 0.10          # max safe platform deceleration (m/s^2)
    latency_s: float = 0.30           # operator reaction + actuation latency (s)
    warn_margin: float = 0.7          # WARN fires at this fraction of v_max(d)

    # Advisory TTC bands.
    warn_ttc_s: float = 4.0
    danger_ttc_s: float = 1.5

    # Absolute standoff floors.
    warn_distance_m: float = 1.0
    danger_distance_m: float = 0.3

    # Motion / staleness handling.
    stationary_epsilon_cm_s: float = 0.5
    stale_s: float = 2.0              # range age beyond which -> NO_DATA
    debounce_s: float = 0.4           # hysteresis debounce (anti-flicker)
    speed_ema_alpha: float = 0.35

    @classmethod
    def load(cls, path: Optional[Path] = None) -> 'SafetyConfig':
        """Load thresholds from a JSON file, falling back to defaults.

        Missing keys keep their dataclass defaults; malformed/absent files
        return the default configuration rather than raising (the HUD must
        never crash over a tuning file).
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
            'a_max_m_s2', 'latency_s', 'warn_margin',
            'warn_ttc_s', 'danger_ttc_s', 'warn_distance_m',
            'danger_distance_m', 'stationary_epsilon_cm_s', 'stale_s',
            'debounce_s', 'speed_ema_alpha') if k in raw}
        cfg = cls(**values)
        # Sanitize out-of-range values so a bad tuning file can never make the
        # HUD divide-by-zero or produce a non-monotonic envelope.  ``a_max``
        # and ``latency`` must be positive; ``warn_margin`` and ``speed_ema_alpha``
        # must lie in (0, 1].
        try:
            a_max = max(1e-3, float(cfg.a_max_m_s2))
            latency = max(0.0, float(cfg.latency_s))
            # warn_margin must stay strictly positive: a 0 fraction would make
            # the WARN speed bound and the recommended speed 0 cm/s.
            warn_margin = min(1.0, max(1e-3, float(cfg.warn_margin)))
            warn_ttc = max(0.0, float(cfg.warn_ttc_s))
            danger_ttc = max(0.0, float(cfg.danger_ttc_s))
            warn_distance = max(0.0, float(cfg.warn_distance_m))
            danger_distance = max(0.0, float(cfg.danger_distance_m))
            # The danger bands must sit inside the warn bands, otherwise the
            # danger-by-TTC / danger-by-distance branches become unreachable and
            # the advisory silently loses those escalations.
            cfg = cls(
                a_max_m_s2=a_max,
                latency_s=latency,
                warn_margin=warn_margin,
                warn_ttc_s=warn_ttc,
                danger_ttc_s=min(danger_ttc, warn_ttc),
                warn_distance_m=warn_distance,
                danger_distance_m=min(danger_distance, warn_distance),
                stationary_epsilon_cm_s=float(cfg.stationary_epsilon_cm_s),
                stale_s=max(0.0, float(cfg.stale_s)),
                debounce_s=max(0.0, float(cfg.debounce_s)),
                speed_ema_alpha=min(1.0, max(0.0, float(cfg.speed_ema_alpha))),
            )
        except (TypeError, ValueError):
            return cls()
        return cfg


@dataclass(frozen=True)
class Guidance:
    """One evaluation result."""

    state: str                 # SAFE / WARN / DANGER / NO_DATA
    ttc_s: Optional[float]     # None when speed is ~0 / no valid TTC
    speed_cm_s: float          # smoothed closing speed (positive = closing)
    distance_m: Optional[float]
    stop_distance_m: Optional[float]     # d_stop(v) at the current speed
    max_speed_cm_s: Optional[float]      # v_max(d): physical speed limit at gap
    recommended_speed_cm_s: Optional[float]  # None when SAFE / NO_DATA
    message: str               # human guidance line


def state_message(state: str, g: Guidance) -> str:
    """A human message consistent with a *held* guidance state.

    Used by the bridge during hysteresis debounce: when the displayed state is
    still the old one but the candidate has advanced, the message must match
    the shown colour rather than the candidate's (e.g. never show a "STOP NOW"
    message against an orange WARN banner).
    """
    if state == DANGER:
        return 'STOP — collision risk'
    if state == WARN:
        if g.distance_m is not None:
            rec = g.recommended_speed_cm_s
            return (f'SLOW — {rec:.0f} cm/s max ({g.distance_m:.2f} m)'
                    if rec is not None else
                    f'SLOW ({g.distance_m:.2f} m)')
        return 'SLOW'
    if state == NO_DATA:
        return 'approach: awaiting center range'
    return (f'approach clear ({g.distance_m:.2f} m)'
            if g.distance_m is not None else 'approach clear')


def _clamp_speed_for_ttc(speed_cm_s: float, min_speed_cm_s: float = 0.05) -> float:
    """Clamp speed to a floor so TTC is finite and monotonic near zero."""
    return max(speed_cm_s, min_speed_cm_s)


def _stop_distance_m(speed_cm_s: float, cfg: SafetyConfig) -> float:
    """Stopping distance (m) from closing speed v = speed_cm_s (cm/s).

    d_stop = v*t_latency + v^2/(2*a_max), with v in m/s.
    """
    v = max(0.0, speed_cm_s) / 100.0
    return v * cfg.latency_s + (v * v) / (2.0 * cfg.a_max_m_s2)


def _max_safe_speed_cm_s(distance_m: float, cfg: SafetyConfig) -> float:
    """v_max(d) in cm/s: the max closing speed that can still stop within d."""
    a = cfg.a_max_m_s2
    t = cfg.latency_s
    d = max(0.0, distance_m)
    v = -a * t + math.sqrt((a * t) * (a * t) + 2.0 * a * d)
    return v * 100.0


def evaluate(
        speed_cm_s: Optional[float],
        distance_m: Optional[float],
        cfg: SafetyConfig,
        *,
        stale: bool = False,
        min_ttc_speed_cm_s: float = 0.05,
) -> Guidance:
    """Evaluate the safety state for a (speed, distance) reading.

    ``speed_cm_s`` / ``distance_m`` may be None (no valid sensor reading);
    ``stale`` marks data older than ``cfg.stale_s``.  Either yields NO_DATA.

    Hysteresis is intentionally NOT applied here (this is a stateless
    evaluation); the bridge holds the debounce across frames.
    """
    if speed_cm_s is None or distance_m is None or stale:
        return Guidance(NO_DATA, None, 0.0, distance_m, None, None, None,
                        'approach: awaiting center range')

    speed = float(speed_cm_s)
    dist = float(distance_m)
    if dist <= 0.0:
        return Guidance(NO_DATA, None, speed, dist, None, None, None,
                        'approach: distance unavailable')

    # Absolute standoff floor: inside the danger distance is DANGER regardless
    # of speed (even stationary).
    if dist <= cfg.danger_distance_m:
        return Guidance(
            DANGER, None, speed, dist, _stop_distance_m(speed, cfg),
            _max_safe_speed_cm_s(dist, cfg), 0.0,
            f'STOP — collision imminent ({dist:.2f} m)')

    # Determine whether we are closing on the trunk.  A stationary platform
    # (|v| <= epsilon) is neither approaching nor moving away; it only becomes
    # DANGER via the absolute distance floor above.
    approaching = speed > cfg.stationary_epsilon_cm_s

    ttc_s: Optional[float] = None
    if approaching:
        v = _clamp_speed_for_ttc(speed, min_ttc_speed_cm_s)
        ttc_s = dist / (v / 100.0)   # m / (m/s)

    stop_dist = _stop_distance_m(speed, cfg)
    v_max = _max_safe_speed_cm_s(dist, cfg)

    # DANGER by stopping distance: cannot stop within the remaining gap.
    if approaching and stop_dist >= dist:
        v_warn = cfg.warn_margin * v_max
        return Guidance(
            DANGER, ttc_s, speed, dist, stop_dist, v_max, v_warn,
            f'STOP NOW — {dist:.2f} m gap, need {stop_dist:.2f} m to stop')

    # DANGER by advisory TTC floor (fast-but-far).
    if ttc_s is not None and ttc_s <= cfg.danger_ttc_s:
        v_warn = cfg.warn_margin * v_max
        return Guidance(
            DANGER, ttc_s, speed, dist, stop_dist, v_max, v_warn,
            f'STOP NOW — {ttc_s:.1f}s to impact, slow to {v_warn:.0f} cm/s')

    # WARN: inside the warn standoff, inside the warn TTC, or exceeding the
    # warning speed fraction of the physical stop limit.
    warn_by_distance = dist <= cfg.warn_distance_m
    warn_by_ttc = ttc_s is not None and ttc_s <= cfg.warn_ttc_s
    warn_by_speed = approaching and speed > (cfg.warn_margin * v_max)
    if warn_by_distance or warn_by_ttc or warn_by_speed:
        v_warn = cfg.warn_margin * v_max
        if warn_by_ttc or warn_by_speed:
            # Always lead with "SLOW" so the message matches the orange WARN
            # banner; the TTC suffix is only a detail.
            if ttc_s is not None and ttc_s > 0.0:
                return Guidance(
                    WARN, ttc_s, speed, dist, stop_dist, v_max, v_warn,
                    f'SLOW {v_warn:.0f} cm/s max ({ttc_s:.1f} s to impact)')
            return Guidance(
                WARN, ttc_s, speed, dist, stop_dist, v_max, v_warn,
                f'SLOW to {v_warn:.0f} cm/s')
        return Guidance(
            WARN, ttc_s, speed, dist, stop_dist, v_max, v_warn,
            f'SLOW — {v_warn:.0f} cm/s max ({dist:.2f} m)')

    # Otherwise: safe (moving away is always SAFE; stationary outside danger).
    return Guidance(SAFE, ttc_s, speed, dist, stop_dist, v_max, None,
                    f'approach clear ({dist:.2f} m)')


def default_config_path() -> Path:
    """Best-effort path to the tuning file next to this module."""
    return Path(__file__).resolve().parent.parent / 'config' / \
        'safety_guidance.json'


__all__ = [
    'SAFE', 'WARN', 'DANGER', 'NO_DATA',
    'SafetyConfig', 'Guidance',
    'evaluate', 'state_message', 'default_config_path',
]
