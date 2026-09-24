"""Admin-configurable layout for the operator HUD panels.

The dashboard ships sensible defaults (large panels/text so the HUD is legible
from ~2 feet) and an admin can point ``--hud-config`` at a JSON file to move the
panels, rename their captions, resize them, and change the value/caption font
sizes.  This module is Qt-free so it can be unit tested headlessly.
"""

from __future__ import annotations

import json
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Dict, Optional


# Panel names recognised in the config file.
BOOM = 'boom'
DOCKING = 'docking'
CUTTER_RANGE = 'cutter_range'
DOCK_GUIDANCE = 'dock_guidance'
CUTTER_GUIDANCE = 'cutter_guidance'
LIDAR_SCAN = 'lidar_scan'

# Anchors accepted for a panel.  ``value`` is the position name used in JSON
# (``<vertical>-<horizontal>``); ``horizontal``/``vertical`` below drive QML.
_ANCHORS = {
    'top-left', 'top-center', 'top-right',
    'bottom-left', 'bottom-center', 'bottom-right',
}

# Default display names for the docking range rows, keyed by the canonical
# ``telemetry_key`` (the MQTT subscriber maps the PLC laser distances onto
# these; see canonical_zmq mqtt_ingest.SENSOR_BINDINGS).  The side left/right
# names are retained for channels that carry them.
_DEFAULT_DOCKING_ROWS = {
    'center_line': 'Center',
    'diagonal_left_45deg': '45\u00b0 Left',
    'diagonal_right_45deg': '45\u00b0 Right',
    'c_channel_left': 'Left',
    'c_channel_right': 'Right',
}

# Default display names for the boom fields (PLC MQTT subscriber values on
# v1/boom/state; see canonical_zmq mqtt_ingest.map_boom_state).
_DEFAULT_BOOM_ROWS = {
    'boom_angle_deg': 'Boom Angle',
    'boom_extension_m': 'Boom Length',
    'slew_angle_deg': 'Slew Angle',
    'platform_tilt_x1_deg': 'Platform Tilt X1',
    'platform_tilt_y1_deg': 'Platform Tilt Y1',
    'primemover_tilt_x2_deg': 'Prime Mover Tilt X2',
    'primemover_tilt_y2_deg': 'Prime Mover Tilt Y2',
}

_DEFAULT_CUTTER_ROWS = {
    'cutter_range': 'Cutter',
}

# Default display names for the docking safety-guidance metrics rows (see
# safety_guidance.py / the DockGuidanceHud panel).  Only the rows the dedicated
# panel renders are named here.
_DEFAULT_DOCK_GUIDANCE_ROWS = {
    'state': 'State',
    'speed': 'Closing Speed',
    'gap': 'Gap',
    'ttc': 'TTC',
    'max_speed': 'Max Safe',
}

# Default display names for the cutter safety-guide metrics rows (see
# cutter_safety_guidance.py / the CutterGuidanceHud panel).
_DEFAULT_CUTTER_GUIDANCE_ROWS = {
    'state': 'State',
    'phase': 'Cut Step',
    'clearance': 'Clearance',
    'speed': 'Closing Speed',
    'ttc': 'TTC',
    'max_speed': 'Max Safe',
}

# Default display names for the LiDAR scan overlay's estimate rows (see
# model/scan_estimate.py / LidarScanOverlay.qml).  The overlay is a single
# full-screen operator HUD: it shows scan instructions while scanning and swaps
# to these estimate rows when the scan completes.
_DEFAULT_LIDAR_SCAN_ROWS = {
    'tree_height': 'Tree Height',
    'trunk_top': 'Trunk Top',
    'crown_base': 'Crown Base',
    'uncertainty': 'Uncertainty',
    'docking_height': 'Docking Height',
    'boom_angle': 'Boom Angle',
    'boom_extension': 'Boom Extension',
    'platform_level': 'Platform Level',
    'boom_distance': 'Boom Distance',
    'docking_lower': 'Docking Lower Angle',
    'status': 'Status',
}


@dataclass
class HudPanelConfig:
    """One configurable HUD panel."""

    name: str
    visible: bool = True
    anchor: str = 'bottom-left'
    caption: str = ''
    width: int = 520
    height: int = 0  # 0 => auto-size to content
    value_font_px: int = 46
    caption_font_px: int = 30
    margin_px: int = 12
    opacity: float = 0.85
    rows: Dict[str, str] = field(default_factory=dict)
    # Dock/cutter-guidance-only extras (ignored by the generic SensorHudPanel,
    # used by DockGuidanceHud.qml / CutterGuidanceHud.qml).  Kept on the shared
    # dataclass so both guidance panels are configured in the same file/loader
    # as every other sensor HUD.
    stopbar_range_m: float = 1.5      # full-scale of the clearance/stop bar (m)
    banner_font_px: int = 40          # state-banner text size
    pulse_danger: bool = True         # pulse the banner while DANGER
    show_confirm_button: bool = True  # cutter: show the CONFIRM STEP button
    # LiDAR scan-overlay-only extras (ignored by the other panels).  The overlay
    # is full-screen and centred; ``zoom_*`` set the default/min/max half-width
    # of the projected cloud window in metres, and ``guide_font_px`` /
    # ``estimate_font_px`` size the instruction text and the estimate rows.
    guide_font_px: int = 34
    estimate_font_px: int = 34
    zoom_default_m: float = 12.0
    zoom_min_m: float = 3.0
    zoom_max_m: float = 40.0
    scan_seconds: float = 15.0        # guided scan duration (s)
    redraw_hz: float = 12.0           # overlay repaint cap (CPU guard)
    # Darkness of the full-screen scrim the overlay draws over the camera image
    # while LiDAR scan mode is active.  0 disables it (camera image untouched).
    # Only applies in scan mode; the camera view is never tinted otherwise.
    scrim_opacity: float = 0.15

    def to_qml(self) -> Dict[str, Any]:
        """Return a plain dict suitable for a QVariantMap bridge property."""
        return {
            'name': self.name,
            'visible': bool(self.visible),
            'anchor': self.anchor,
            'caption': self.caption,
            'width': int(self.width),
            'height': int(self.height),
            'valueFontPx': int(self.value_font_px),
            'captionFontPx': int(self.caption_font_px),
            'marginPx': int(self.margin_px),
            'opacity': float(self.opacity),
            'rows': dict(self.rows),
            'stopbarRangeM': float(self.stopbar_range_m),
            'bannerFontPx': int(self.banner_font_px),
            'pulseDanger': bool(self.pulse_danger),
            'showConfirmButton': bool(self.show_confirm_button),
            'guideFontPx': int(self.guide_font_px),
            'estimateFontPx': int(self.estimate_font_px),
            'zoomDefaultM': float(self.zoom_default_m),
            'zoomMinM': float(self.zoom_min_m),
            'zoomMaxM': float(self.zoom_max_m),
            'scanSeconds': float(self.scan_seconds),
            'redrawHz': float(self.redraw_hz),
            'scrimOpacity': float(self.scrim_opacity),
        }


@dataclass
class HudLayoutConfig:
    """The full set of operator HUD panels."""

    boom: HudPanelConfig
    docking: HudPanelConfig
    cutter_range: HudPanelConfig
    dock_guidance: HudPanelConfig
    cutter_guidance: HudPanelConfig
    lidar_scan: HudPanelConfig

    def to_qml(self) -> Dict[str, Any]:
        return {
            BOOM: self.boom.to_qml(),
            DOCKING: self.docking.to_qml(),
            CUTTER_RANGE: self.cutter_range.to_qml(),
            DOCK_GUIDANCE: self.dock_guidance.to_qml(),
            CUTTER_GUIDANCE: self.cutter_guidance.to_qml(),
            LIDAR_SCAN: self.lidar_scan.to_qml(),
        }


def default_hud_layout() -> HudLayoutConfig:
    """Built-in defaults: boom bottom-right, docking ranges bottom-left."""
    return HudLayoutConfig(
        boom=HudPanelConfig(
            name=BOOM,
            visible=True,
            anchor='bottom-right',
            caption='BOOM',
            width=520,
            height=0,
            value_font_px=46,
            caption_font_px=30,
            margin_px=12,
            opacity=0.85,
            rows=dict(_DEFAULT_BOOM_ROWS),
        ),
        docking=HudPanelConfig(
            name=DOCKING,
            visible=True,
            anchor='bottom-left',
            caption='DOCKING RANGES',
            width=560,
            height=0,
            value_font_px=46,
            caption_font_px=30,
            margin_px=12,
            opacity=0.85,
            rows=dict(_DEFAULT_DOCKING_ROWS),
        ),
        cutter_range=HudPanelConfig(
            name=CUTTER_RANGE,
            visible=True,
            anchor='bottom-left',
            caption='CUTTER RANGE',
            width=460,
            height=0,
            value_font_px=46,
            caption_font_px=30,
            margin_px=12,
            opacity=0.85,
            rows=dict(_DEFAULT_CUTTER_ROWS),
        ),
        dock_guidance=HudPanelConfig(
            name=DOCK_GUIDANCE,
            visible=True,
            # Bottom-center, between the docking-ranges (bottom-left) and boom
            # (bottom-right) panels.
            anchor='bottom-center',
            caption='DOCKING SAFETY',
            width=620,
            height=0,
            value_font_px=46,
            caption_font_px=30,
            margin_px=12,
            opacity=0.85,
            rows=dict(_DEFAULT_DOCK_GUIDANCE_ROWS),
            stopbar_range_m=1.5,
            banner_font_px=40,
            pulse_danger=True,
        ),
        cutter_guidance=HudPanelConfig(
            name=CUTTER_GUIDANCE,
            visible=True,
            # Bottom-center on the cutter view.  The Cutter Range distance
            # panel is bottom-left on the same view; the host positions this
            # panel in the free gap so the two never overlap.
            anchor='bottom-center',
            caption='CUTTER SAFETY',
            width=620,
            height=0,
            value_font_px=46,
            caption_font_px=30,
            margin_px=12,
            opacity=0.85,
            rows=dict(_DEFAULT_CUTTER_GUIDANCE_ROWS),
            # Cutter clearances are small, so the bar's full-scale is 1 m.
            stopbar_range_m=1.0,
            banner_font_px=40,
            pulse_danger=True,
            show_confirm_button=True,
        ),
        lidar_scan=HudPanelConfig(
            name=LIDAR_SCAN,
            # The overlay itself is full-screen; this panel's anchor/width are
            # advisory (its info card sits at the bottom-centre over the cloud)
            # and remain admin-overridable like every other panel.
            visible=True,
            anchor='bottom-center',
            caption='LIDAR SCAN',
            width=820,
            height=0,
            value_font_px=46,
            caption_font_px=30,
            margin_px=16,
            opacity=0.85,
            rows=dict(_DEFAULT_LIDAR_SCAN_ROWS),
            guide_font_px=34,
            estimate_font_px=34,
            zoom_default_m=12.0,
            zoom_min_m=3.0,
            zoom_max_m=40.0,
            scan_seconds=15.0,
            redraw_hz=12.0,
            scrim_opacity=0.15,
        ),
    )

def _as_int(value: Any, fallback: int) -> int:
    # OverflowError is caught because json accepts the Infinity/NaN literals by
    # default, and int(float('inf')) raises OverflowError rather than ValueError.
    # Without it a config like {"width": Infinity} crashed with a traceback
    # instead of the documented fatal SystemExit.
    try:
        return int(value)
    except (TypeError, ValueError, OverflowError):
        return fallback


def _as_float(value: Any, fallback: float) -> float:
    try:
        return float(value)
    except (TypeError, ValueError, OverflowError):
        return fallback


def _as_bool(value: Any, fallback: bool) -> bool:
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        lowered = value.strip().lower()
        if lowered in ('true', 'yes', '1', 'on'):
            return True
        if lowered in ('false', 'no', '0', 'off'):
            return False
    if isinstance(value, (int, float)):
        return bool(value)
    return fallback


def _merge_panel(base: HudPanelConfig, raw: Any, name: str) -> HudPanelConfig:
    """Return a copy of ``base`` with any recognised keys from ``raw`` applied."""
    if not isinstance(raw, dict):
        return base

    anchor = base.anchor
    if isinstance(raw.get('anchor'), str) and raw['anchor'] in _ANCHORS:
        anchor = raw['anchor']

    rows = dict(base.rows)
    raw_rows = raw.get('rows')
    if isinstance(raw_rows, dict):
        for key, label in raw_rows.items():
            rows[str(key)] = str(label)

    def _text(key: str, fallback: str) -> str:
        value = raw.get(key)
        return str(value) if isinstance(value, (str, int, float)) else fallback

    return HudPanelConfig(
        name=name,
        visible=_as_bool(raw.get('visible'), base.visible),
        anchor=anchor,
        caption=_text('caption', base.caption),
        width=max(1, _as_int(raw.get('width'), base.width)),
        height=max(0, _as_int(raw.get('height'), base.height)),
        value_font_px=max(1, _as_int(raw.get('value_font_px'), base.value_font_px)),
        caption_font_px=max(1, _as_int(raw.get('caption_font_px'), base.caption_font_px)),
        margin_px=max(0, _as_int(raw.get('margin_px'), base.margin_px)),
        opacity=min(1.0, max(0.0, _as_float(raw.get('opacity'), base.opacity))),
        rows=rows,
        stopbar_range_m=max(1e-3, _as_float(
            raw.get('stopbar_range_m'), base.stopbar_range_m)),
        banner_font_px=max(1, _as_int(
            raw.get('banner_font_px'), base.banner_font_px)),
        pulse_danger=_as_bool(raw.get('pulse_danger'), base.pulse_danger),
        show_confirm_button=_as_bool(
            raw.get('show_confirm_button'), base.show_confirm_button),
        guide_font_px=max(1, _as_int(
            raw.get('guide_font_px'), base.guide_font_px)),
        estimate_font_px=max(1, _as_int(
            raw.get('estimate_font_px'), base.estimate_font_px)),
        zoom_default_m=max(1e-3, _as_float(
            raw.get('zoom_default_m'), base.zoom_default_m)),
        zoom_min_m=max(1e-3, _as_float(
            raw.get('zoom_min_m'), base.zoom_min_m)),
        zoom_max_m=max(1e-3, _as_float(
            raw.get('zoom_max_m'), base.zoom_max_m)),
        scan_seconds=max(1e-3, _as_float(
            raw.get('scan_seconds'), base.scan_seconds)),
        redraw_hz=max(1e-3, _as_float(
            raw.get('redraw_hz'), base.redraw_hz)),
        # Clamp to a valid opacity: 0 disables the scrim, >1 would be meaningless.
        scrim_opacity=min(1.0, max(0.0, _as_float(
            raw.get('scrim_opacity'), base.scrim_opacity))),
    )


def _reject_non_finite(literal: str) -> float:
    """Raise for the non-standard ``Infinity``/``-Infinity``/``NaN`` JSON literals."""
    raise ValueError('{} is not a valid number in a hud config'.format(literal))


def default_hud_config_path() -> Path:
    """Best-effort path to the shipped HUD layout overriding the defaults.

    Resolved as ``harvester_dashboard/hud_config.json`` relative to the
    repository root (one level above the package), matching the launch layout.
    A missing file is not an error: ``load_hud_config`` falls back to the
    built-in defaults.
    """
    return Path(__file__).resolve().parent.parent / 'hud_config.json'


def load_hud_config(path: Optional[str]) -> HudLayoutConfig:
    """Load the admin HUD layout, falling back to built-in defaults.

    ``None``/empty path uses the shipped ``hud_config.json`` when present, else
    the built-in defaults; an explicitly supplied path that exists but is
    unreadable or malformed is fatal so an operator display never silently
    boots with a half-applied configuration.
    """
    layout = default_hud_layout()
    if not path:
        path = str(default_hud_config_path())
    if not Path(path).exists():
        return layout

    try:
        # parse_constant rejects the non-standard Infinity/-Infinity/NaN literals
        # that Python's json accepts by default. They are not valid JSON, cannot
        # be laid out, and silently coerced to a default they would hide a broken
        # config, so treat them as malformed and fail fast like any other bad
        # value.
        with open(path, 'r', encoding='utf-8') as handle:
            data = json.load(handle, parse_constant=_reject_non_finite)
    except FileNotFoundError:
        return layout
    except (OSError, ValueError) as exc:
        raise SystemExit('hud config {!r} could not be loaded: {}'.format(path, exc))

    if not isinstance(data, dict):
        raise SystemExit('hud config {!r} must be a JSON object'.format(path))

    return HudLayoutConfig(
        boom=_merge_panel(layout.boom, data.get(BOOM), BOOM),
        docking=_merge_panel(layout.docking, data.get(DOCKING), DOCKING),
        cutter_range=_merge_panel(
            layout.cutter_range, data.get(CUTTER_RANGE), CUTTER_RANGE),
        dock_guidance=_merge_panel(
            layout.dock_guidance, data.get(DOCK_GUIDANCE), DOCK_GUIDANCE),
        cutter_guidance=_merge_panel(
            layout.cutter_guidance, data.get(CUTTER_GUIDANCE), CUTTER_GUIDANCE),
        lidar_scan=_merge_panel(
            layout.lidar_scan, data.get(LIDAR_SCAN), LIDAR_SCAN),
    )


__all__ = [
    'BOOM', 'DOCKING', 'CUTTER_RANGE', 'DOCK_GUIDANCE', 'CUTTER_GUIDANCE',
    'LIDAR_SCAN',
    'HudPanelConfig', 'HudLayoutConfig',
    'default_hud_layout', 'load_hud_config', 'default_hud_config_path',
]
