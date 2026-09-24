"""HUD layout config: defaults, overrides, per-row captions, error handling."""

import json
import os
import tempfile
import unittest

from harvester_dashboard.hud_config import (
    default_hud_config_path, default_hud_layout, load_hud_config)
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
        dock_guidance=panel(base.dock_guidance, base.dock_guidance.visible),
        cutter_guidance=panel(
            base.cutter_guidance, base.cutter_guidance.visible),
        lidar_scan=panel(base.lidar_scan, base.lidar_scan.visible),
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

    def test_none_path_resolves_shipped_config(self):
        # With no --hud-config, the shipped file must be picked up so its
        # panels (e.g. dock_guidance, cutter_guidance) reach a normal
        # run_all.sh deploy.
        path = default_hud_config_path()
        self.assertTrue(path.exists(), path)
        self.assertEqual(load_hud_config(None).dock_guidance.caption,
                         load_hud_config(str(path)).dock_guidance.caption)
        self.assertEqual(load_hud_config(None).cutter_guidance.caption,
                         load_hud_config(str(path)).cutter_guidance.caption)

    def test_shipped_config_parses(self):
        # Regression: the shipped file once began with design-note prose before
        # the JSON object, which made the strict loader SystemExit at startup.
        # It must parse cleanly and expose the scan panel.
        path = default_hud_config_path()
        self.assertTrue(path.exists(), path)
        layout = load_hud_config(str(path))     # raises SystemExit if malformed
        self.assertTrue(layout.lidar_scan.visible)
        self.assertEqual(layout.lidar_scan.caption, 'LIDAR SCAN')

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
        self.assertEqual(set(qml), {'boom', 'docking', 'cutter_range',
                                    'dock_guidance', 'cutter_guidance',
                                    'lidar_scan'})
        self.assertEqual(qml['boom']['valueFontPx'],
                         default_hud_layout().boom.value_font_px)
        self.assertEqual(qml['docking']['anchor'], 'bottom-left')
        self.assertIn('rows', qml['docking'])
        self.assertEqual(qml['lidar_scan']['anchor'], 'bottom-center')

    def test_dock_guidance_defaults(self):
        layout = default_hud_layout()
        panel = layout.dock_guidance
        self.assertTrue(panel.visible)
        self.assertEqual(panel.anchor, 'bottom-center')
        self.assertEqual(panel.caption, 'DOCKING SAFETY')
        self.assertGreaterEqual(panel.value_font_px, 40)
        self.assertGreaterEqual(panel.banner_font_px, 30)
        self.assertAlmostEqual(panel.stopbar_range_m, 1.5)
        self.assertTrue(panel.pulse_danger)
        # The metric row captions are named for the guidance columns.
        self.assertEqual(panel.rows['gap'], 'Gap')
        self.assertEqual(panel.rows['max_speed'], 'Max Safe')

    def test_dock_guidance_to_qml_extras(self):
        panel = default_hud_layout().to_qml()['dock_guidance']
        self.assertIn('stopbarRangeM', panel)
        self.assertIn('bannerFontPx', panel)
        self.assertIn('pulseDanger', panel)
        self.assertAlmostEqual(panel['stopbarRangeM'], 1.5)

    def test_cutter_guidance_defaults(self):
        layout = default_hud_layout()
        panel = layout.cutter_guidance
        self.assertTrue(panel.visible)
        # Same config file as the other HUDs, bottom-center on the cutter view.
        self.assertEqual(panel.anchor, 'bottom-center')
        self.assertEqual(panel.caption, 'CUTTER SAFETY')
        self.assertGreaterEqual(panel.value_font_px, 40)
        self.assertGreaterEqual(panel.banner_font_px, 30)
        # Cutter clearances are small: the bar full-scale is 1 m.
        self.assertAlmostEqual(panel.stopbar_range_m, 1.0)
        self.assertTrue(panel.pulse_danger)
        self.assertTrue(panel.show_confirm_button)
        # The metric row captions are named for the cutter columns.
        self.assertEqual(panel.rows['phase'], 'Cut Step')
        self.assertEqual(panel.rows['clearance'], 'Clearance')
        self.assertEqual(panel.rows['max_speed'], 'Max Safe')

    def test_cutter_guidance_to_qml_extras(self):
        panel = default_hud_layout().to_qml()['cutter_guidance']
        for key in ('stopbarRangeM', 'bannerFontPx', 'pulseDanger',
                    'showConfirmButton'):
            self.assertIn(key, panel)
        self.assertAlmostEqual(panel['stopbarRangeM'], 1.0)
        self.assertTrue(panel['showConfirmButton'])

    def test_lidar_scan_defaults(self):
        panel = default_hud_layout().lidar_scan
        self.assertTrue(panel.visible)
        self.assertEqual(panel.anchor, 'bottom-center')
        self.assertEqual(panel.caption, 'LIDAR SCAN')
        # Scan-overlay extras ship with usable defaults.
        self.assertGreater(panel.zoom_default_m, panel.zoom_min_m)
        self.assertLess(panel.zoom_default_m, panel.zoom_max_m)
        # 15 s is the operator default scan window, admin-overridable.
        self.assertAlmostEqual(panel.scan_seconds, 15.0)
        self.assertGreater(panel.redraw_hz, 0.0)
        # The estimate row captions are named.
        self.assertEqual(panel.rows['tree_height'], 'Tree Height')
        self.assertEqual(panel.rows['boom_angle'], 'Boom Angle')

    def test_lidar_scan_to_qml_extras(self):
        panel = default_hud_layout().to_qml()['lidar_scan']
        for key in ('guideFontPx', 'estimateFontPx', 'zoomDefaultM',
                    'zoomMinM', 'zoomMaxM', 'scanSeconds', 'redrawHz'):
            self.assertIn(key, panel)
        self.assertAlmostEqual(panel['zoomDefaultM'], 12.0)


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

    def test_dock_guidance_override(self):
        path = self._write({
            'dock_guidance': {
                'anchor': 'bottom-right',
                'caption': 'DOCK SAFETY',
                'width': 800,
                'visible': False,
                'banner_font_px': 52,
                'stopbar_range_m': 2.0,
                'pulse_danger': False,
                'rows': {'gap': 'Distance'},
            },
        })
        layout = load_hud_config(path)
        panel = layout.dock_guidance
        self.assertEqual(panel.anchor, 'bottom-right')
        self.assertEqual(panel.caption, 'DOCK SAFETY')
        self.assertEqual(panel.width, 800)
        self.assertFalse(panel.visible)
        self.assertEqual(panel.banner_font_px, 52)
        self.assertAlmostEqual(panel.stopbar_range_m, 2.0)
        self.assertFalse(panel.pulse_danger)
        self.assertEqual(panel.rows['gap'], 'Distance')
        # Unmentioned default rows survive the merge.
        self.assertIn('max_speed', panel.rows)

    def test_unknown_keys_ignored(self):
        path = self._write({'docking': {'anchor': 'bottom-left', 'bogus': 1},
                            'mystery_panel': {'anchor': 'top-left'}})
        layout = load_hud_config(path)
        self.assertEqual(layout.docking.anchor, 'bottom-left')

    def test_cutter_guidance_override(self):
        path = self._write({
            'cutter_guidance': {
                'anchor': 'bottom-right',
                'caption': 'CUTTER GUIDE',
                'width': 500,
                'visible': False,
                'banner_font_px': 44,
                'stopbar_range_m': 0.5,
                'pulse_danger': False,
                'show_confirm_button': False,
                'rows': {'phase': 'Step'},
            },
        })
        layout = load_hud_config(path)
        panel = layout.cutter_guidance
        self.assertEqual(panel.anchor, 'bottom-right')
        self.assertEqual(panel.caption, 'CUTTER GUIDE')
        self.assertEqual(panel.width, 500)
        self.assertFalse(panel.visible)
        self.assertEqual(panel.banner_font_px, 44)
        self.assertAlmostEqual(panel.stopbar_range_m, 0.5)
        self.assertFalse(panel.pulse_danger)
        self.assertFalse(panel.show_confirm_button)
        self.assertEqual(panel.rows['phase'], 'Step')
        # Unmentioned default rows survive the merge.
        self.assertIn('clearance', panel.rows)
        # The docking panel is untouched by a cutter-only override.
        self.assertEqual(layout.dock_guidance.caption, 'DOCKING SAFETY')

    def test_invalid_anchor_falls_back(self):
        path = self._write({'docking': {'anchor': 'sideways'}})
        layout = load_hud_config(path)
        self.assertEqual(layout.docking.anchor,
                         default_hud_layout().docking.anchor)

    def test_lidar_scan_override(self):
        path = self._write({
            'lidar_scan': {
                'anchor': 'top-right',
                'caption': 'SCAN TREE',
                'zoom_default_m': 20.0,
                'scan_seconds': 8.0,
                'rows': {'tree_height': 'Height'},
            },
        })
        layout = load_hud_config(path)
        panel = layout.lidar_scan
        self.assertEqual(panel.anchor, 'top-right')
        self.assertEqual(panel.caption, 'SCAN TREE')
        self.assertAlmostEqual(panel.zoom_default_m, 20.0)
        self.assertAlmostEqual(panel.scan_seconds, 8.0)
        self.assertEqual(panel.rows['tree_height'], 'Height')
        # Unmentioned default rows survive the merge.
        self.assertIn('boom_angle', panel.rows)

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
