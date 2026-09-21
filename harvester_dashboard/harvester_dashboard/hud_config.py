"""Admin-configurable layout for the operator HUD panels.

The dashboard ships sensible defaults (large panels/text so the HUD is legible
from ~2 feet) and an admin can point ``--hud-config`` at a JSON file to move the
panels, rename their captions, resize them, and change the value/caption font
sizes.  This module is Qt-free so it can be unit tested headlessly.
"""

from __future__ import annotations

import json
from dataclasses import dataclass, field
from typing import Any, Dict, Optional


# Panel names recognised in the config file.
BOOM = 'boom'
DOCKING = 'docking'
CUTTER_RANGE = 'cutter_range'

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
    'c_channel_left': 'Side Left',
    'c_channel_right': 'Side Right',
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
        }


@dataclass
class HudLayoutConfig:
    """The full set of operator HUD panels."""

    boom: HudPanelConfig
    docking: HudPanelConfig
    cutter_range: HudPanelConfig

    def to_qml(self) -> Dict[str, Any]:
        return {
            BOOM: self.boom.to_qml(),
            DOCKING: self.docking.to_qml(),
            CUTTER_RANGE: self.cutter_range.to_qml(),
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
    )


def _as_int(value: Any, fallback: int) -> int:
    try:
        return int(value)
    except (TypeError, ValueError):
        return fallback


def _as_float(value: Any, fallback: float) -> float:
    try:
        return float(value)
    except (TypeError, ValueError):
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
    )


def load_hud_config(path: Optional[str]) -> HudLayoutConfig:
    """Load the admin HUD layout, falling back to built-in defaults.

    ``None``/empty path or a missing file yields the defaults.  An explicitly
    supplied path that exists but is unreadable or malformed is fatal so an
    operator display never silently boots with a half-applied configuration.
    """
    layout = default_hud_layout()
    if not path:
        return layout

    try:
        with open(path, 'r', encoding='utf-8') as handle:
            data = json.load(handle)
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
    )


__all__ = [
    'BOOM', 'DOCKING', 'CUTTER_RANGE',
    'HudPanelConfig', 'HudLayoutConfig',
    'default_hud_layout', 'load_hud_config',
]
