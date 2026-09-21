"""HUD layout config: defaults, overrides, per-row captions, error handling."""

import json
import os
import tempfile
import unittest

from harvester_dashboard.hud_config import (
    default_hud_layout, load_hud_config)
from harvester_dashboard.hud_config import HudLayoutConfig, HudPanelConfig


def _layout_with(boom_visible: bool, docking_visible: bool,
                 cutter_visible: bool) -> HudLayoutConfig:
    base = default_hud_layout()

    def panel(source, visible):
        fields = dict(source.__dict__)
        fields['visible'] = visible
        return HudPanelConfig(**fields)

    return HudLayoutConfig(
        boom=panel(base.boom, boom_visible),
        docking=panel(base.docking, docking_visible),
        cutter_range=panel(base.cutter_range, cutter_visible),
    )


class HudConfigDefaultsTest(unittest.TestCase):
    def test_defaults_none_path(self):
        layout = load_hud_config(None)
        self.assertEqual(layout.boom.anchor, 'bottom-right')
        self.assertEqual(layout.docking.anchor, 'bottom-left')
        self.assertTrue(layout.boom.visible)
        self.assertTrue(layout.docking.visible)
        # Large enough to read from ~2 feet.
        self.assertGreaterEqual(layout.boom.value_font_px, 40)
        self.assertGreaterEqual(layout.docking.value_font_px, 40)

    def test_defaults_missing_file(self):
        layout = load_hud_config('/nonexistent/hud.json')
        self.assertEqual(layout.boom.anchor, 'bottom-right')
        self.assertEqual(layout.docking.anchor, 'bottom-left')

    def test_default_rows_present(self):
        layout = default_hud_layout()
        self.assertEqual(layout.docking.rows['center_line'], 'Center')
        self.assertIn('diagonal_left_45deg', layout.docking.rows)
        # Boom rows are the seven PLC MQTT subscriber values.
        self.assertEqual(set(layout.boom.rows), {
            'boom_angle_deg', 'boom_extension_m', 'slew_angle_deg',
            'platform_tilt_x1_deg', 'platform_tilt_y1_deg',
            'primemover_tilt_x2_deg', 'primemover_tilt_y2_deg'})
        self.assertEqual(layout.boom.rows['boom_extension_m'], 'Boom Length')
        self.assertEqual(layout.boom.rows['slew_angle_deg'], 'Slew Angle')

    def test_to_qml_shape(self):
        qml = default_hud_layout().to_qml()
        self.assertEqual(set(qml), {'boom', 'docking', 'cutter_range'})
        self.assertEqual(qml['boom']['valueFontPx'],
                         default_hud_layout().boom.value_font_px)
        self.assertEqual(qml['docking']['anchor'], 'bottom-left')
        self.assertIn('rows', qml['docking'])


class HudConfigOverrideTest(unittest.TestCase):
    def _write(self, payload):
        handle = tempfile.NamedTemporaryFile(
            mode='w', suffix='.json', delete=False)
        json.dump(payload, handle)
        handle.close()
        self.addCleanup(os.unlink, handle.name)
        return handle.name

    def test_partial_override_falls_back(self):
        path = self._write({'docking': {'anchor': 'top-right', 'width': 700}})
        layout = load_hud_config(path)
        self.assertEqual(layout.docking.anchor, 'top-right')
        self.assertEqual(layout.docking.width, 700)
        # Unspecified keys keep defaults.
        self.assertEqual(layout.docking.value_font_px,
                         default_hud_layout().docking.value_font_px)
        # Other panels untouched.
        self.assertEqual(layout.boom.anchor, 'bottom-right')

    def test_captions_sizes_rows_visibility(self):
        path = self._write({
            'boom': {
                'visible': False,
                'caption': 'BOOM ARM',
                'value_font_px': 60,
                'caption_font_px': 34,
                'rows': {'boom_angle_deg': 'Angle'},
            },
        })
        layout = load_hud_config(path)
        self.assertFalse(layout.boom.visible)
        self.assertEqual(layout.boom.caption, 'BOOM ARM')
        self.assertEqual(layout.boom.value_font_px, 60)
        self.assertEqual(layout.boom.caption_font_px, 34)
        self.assertEqual(layout.boom.rows['boom_angle_deg'], 'Angle')
        # Unmentioned default rows survive the merge.
        self.assertIn('boom_extension_m', layout.boom.rows)

    def test_unknown_keys_ignored(self):
        path = self._write({'docking': {'anchor': 'bottom-left', 'bogus': 1},
                            'mystery_panel': {'anchor': 'top-left'}})
        layout = load_hud_config(path)
        self.assertEqual(layout.docking.anchor, 'bottom-left')

    def test_invalid_anchor_falls_back(self):
        path = self._write({'docking': {'anchor': 'sideways'}})
        layout = load_hud_config(path)
        self.assertEqual(layout.docking.anchor,
                         default_hud_layout().docking.anchor)

    def test_malformed_json_is_fatal(self):
        handle = tempfile.NamedTemporaryFile(
            mode='w', suffix='.json', delete=False)
        handle.write('{not json')
        handle.close()
        self.addCleanup(os.unlink, handle.name)
        with self.assertRaises(SystemExit):
            load_hud_config(handle.name)

    def test_non_object_json_is_fatal(self):
        path = self._write(['not', 'an', 'object'])
        with self.assertRaises(SystemExit):
            load_hud_config(path)

    def test_non_finite_numbers_are_fatal(self):
        # json accepts Infinity/NaN by default; a layout cannot use them, so they
        # must fail fast rather than silently falling back to a default size.
        for literal in ('Infinity', '-Infinity', 'NaN'):
            with self.subTest(literal=literal):
                handle = tempfile.NamedTemporaryFile(
                    mode='w', suffix='.json', delete=False)
                handle.write('{"boom": {"width": %s}}' % literal)
                handle.close()
                self.addCleanup(os.unlink, handle.name)
                with self.assertRaises(SystemExit):
                    load_hud_config(handle.name)


class OperatorHudVisibilityTest(unittest.TestCase):
    """The key-2 flag gates only the boom and docking panels.

    Each of those is additionally gated on its own ``visible`` config, so the
    toggle is on while either is enabled. cutter_range is excluded because it is
    gated on the cutter view, not this flag.
    """

    def _bridge(self, layout):
        try:
            from harvester_dashboard.bridge import DashboardBridge
            from harvester_dashboard.config import DashboardConfig
            from harvester_dashboard.model.telemetry_model import TelemetryModel
            from harvester_dashboard.model.target_model import AnnotationState
            from harvester_dashboard.protocol_shim import ensure_contract_importable
        except Exception as exc:  # pragma: no cover - Qt not installed
            raise unittest.SkipTest('dashboard bridge unavailable: %s' % exc)
        ensure_contract_importable()
        return DashboardBridge(
            DashboardConfig(status_endpoint='', annotation_endpoint=''),
            TelemetryModel(), AnnotationState(), hud_config=layout)

    def test_cutter_only_does_not_enable_operator_huds(self):
        # With only the cutter panel visible, the boom/docking toggle must start
        # hidden: nothing it controls is on screen, so a "visible" default would
        # make the first key-2 press hide rather than show.
        try:
            bridge = self._bridge(_layout_with(False, False, True))
        except unittest.SkipTest:
            self.skipTest('dashboard bridge unavailable')
        self.assertFalse(bridge.operatorHudsVisible)

    def test_either_boom_or_docking_enables_operator_huds(self):
        for boom, docking in ((True, True), (True, False), (False, True)):
            with self.subTest(boom=boom, docking=docking):
                try:
                    bridge = self._bridge(_layout_with(boom, docking, False))
                except unittest.SkipTest:
                    self.skipTest('dashboard bridge unavailable')
                self.assertTrue(bridge.operatorHudsVisible)

    def test_both_operator_panels_disabled_hides_operator_huds(self):
        try:
            bridge = self._bridge(_layout_with(False, False, True))
        except unittest.SkipTest:
            self.skipTest('dashboard bridge unavailable')
        self.assertFalse(bridge.operatorHudsVisible)


if __name__ == '__main__':
    unittest.main()
