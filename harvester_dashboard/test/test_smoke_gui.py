"""GUI smoke test: loads Dashboard.qml offscreen and exercises view keys.

Skips cleanly (``skipTest``) when PySide2/QtQuick are unavailable or no
display/offscreen platform exists, so the pure-python suite stays green
in any environment.
"""

import os
import unittest

try:
    import PySide2.QtQuick  # noqa: F401
    _GUI_IMPORTS_OK = True
except ImportError:
    _GUI_IMPORTS_OK = False

if _GUI_IMPORTS_OK:
    os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')
    from PySide2.QtCore import QUrl
    from PySide2.QtGui import QGuiApplication
    from PySide2.QtQuick import QQuickView

from harvester_dashboard.config import DashboardConfig
from harvester_dashboard.model.telemetry_model import TelemetryModel
from harvester_dashboard.model.target_model import AnnotationState
from harvester_dashboard.protocol_shim import ensure_contract_importable

ensure_contract_importable()


@unittest.skipUnless(_GUI_IMPORTS_OK, 'PySide2 QtQuick not installed')
class GuiSmokeTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = QGuiApplication.instance() or QGuiApplication(
            ['harvester-dashboard-smoke'])
        from harvester_dashboard.bridge import DashboardBridge
        from harvester_dashboard.image_provider import FrameImageProvider
        cls.model = TelemetryModel()
        cls.annotation = AnnotationState()
        cls.bridge = DashboardBridge(
            DashboardConfig(status_endpoint='', annotation_endpoint=''),
            cls.model, cls.annotation)
        cls.provider = FrameImageProvider()
        cls.view = QQuickView()
        cls.view.engine().addImageProvider('frames', cls.provider)
        cls.view.rootContext().setContextProperty('bridge', cls.bridge)
        qml_dir = os.path.join(
            os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'qml')
        cls.view.setSource(QUrl.fromLocalFile(os.path.join(qml_dir, 'Dashboard.qml')))
        if cls.view.status() != QQuickView.Null and not cls.root_ok():
            raise unittest.SkipTest('Dashboard.qml failed to load')
        cls.root = cls.view.rootObject()

    @classmethod
    def root_ok(cls):
        return cls.view.status() == QQuickView.Ready

    @classmethod
    def tearDownClass(cls):
        cls.view.deleteLater()
        cls.app.processEvents()
        del cls.view

    def _process(self, ms=50):
        for _ in range(max(1, int(ms / 5))):
            self.app.processEvents()

    def test_root_loads(self):
        self.assertIsNotNone(self.root)

    def test_view_switch_is_render_only_state(self):
        # The dashboard boots on the docking camera by default.
        self.assertEqual(self.bridge.view, 'docking')
        self.bridge.set_view('cutter')
        self.app.processEvents()
        self.assertEqual(self.bridge.view, 'cutter')
        self.bridge.set_view('docking')
        self.app.processEvents()
        self.assertEqual(self.bridge.view, 'docking')

    def test_hud_and_lidar_toggles(self):
        initial_hud = self.bridge.hudVisible
        self.bridge.toggle_hud()
        self.app.processEvents()
        self.assertEqual(self.bridge.hudVisible, not initial_hud)
        self.bridge.toggle_hud()
        initial_lidar = self.bridge.lidarVisible
        self.bridge.toggle_lidar()
        self.app.processEvents()
        self.assertEqual(self.bridge.lidarVisible, not initial_lidar)
        self.bridge.toggle_lidar()

    def test_operator_huds_boot_visible_and_toggle(self):
        # Boom + docking HUDs are shown at startup.
        self.assertTrue(self.bridge.operatorHudsVisible)
        # Key 2 (on the docking view) hides, then shows.
        self.bridge.set_view('docking')
        self.bridge.toggle_operator_huds()
        self.app.processEvents()
        self.assertFalse(self.bridge.operatorHudsVisible)
        self.bridge.toggle_operator_huds()
        self.app.processEvents()
        self.assertTrue(self.bridge.operatorHudsVisible)

    def test_key2_on_cutter_always_returns_to_docking(self):
        # On the cutter view key 2 always switches to docking and shows the
        # Boom + Docking HUDs (even when they were already visible).
        self.bridge.select_cutter_view()
        self.app.processEvents()
        self.assertEqual(self.bridge.view, 'cutter')
        self.assertTrue(self.bridge.operatorHudsVisible)
        self.bridge.toggle_operator_huds()
        self.app.processEvents()
        self.assertEqual(self.bridge.view, 'docking')
        self.assertTrue(self.bridge.operatorHudsVisible)
        self.assertTrue(self.bridge.cutterHudVisible)

    def test_key1_toggles_cutter_hud_on_cutter_view(self):
        # First press from docking -> cutter view, cutter HUD shown.
        self.bridge.set_view('docking')
        self.bridge.select_cutter_view()
        self.app.processEvents()
        self.assertEqual(self.bridge.view, 'cutter')
        self.assertTrue(self.bridge.cutterHudVisible)
        # Press again on the cutter view -> hide, then show.
        self.bridge.select_cutter_view()
        self.app.processEvents()
        self.assertEqual(self.bridge.view, 'cutter')
        self.assertFalse(self.bridge.cutterHudVisible)
        self.bridge.select_cutter_view()
        self.app.processEvents()
        self.assertTrue(self.bridge.cutterHudVisible)
        # Restore for other tests.
        self.bridge.set_view('docking')
        self.bridge._set_cutter_hud_visible(True)

    def test_operator_hud_layout_and_rows(self):
        layout = self.bridge.hudLayout
        self.assertEqual(set(layout), {'boom', 'docking', 'cutter_range'})
        self.assertEqual(layout['boom']['anchor'], 'bottom-right')
        self.assertEqual(layout['docking']['anchor'], 'bottom-left')
        # Boom has the seven PLC MQTT sensor rows.
        keys = [row['key'] for row in self.bridge.boomRows]
        self.assertEqual(keys, [
            'boom_angle_deg', 'boom_extension_m', 'slew_angle_deg',
            'platform_tilt_x1_deg', 'platform_tilt_y1_deg',
            'primemover_tilt_x2_deg', 'primemover_tilt_y2_deg'])
        # Cutter range row is always present (invalid until data arrives).
        cutter = self.bridge.cutterRangeRow
        self.assertEqual(len(cutter), 1)
        self.assertEqual(cutter[0]['key'], 'cutter_range')

    def test_docking_range_rows_have_display_shape(self):
        # The shared SensorHudPanel renders {key, label, value, valid}; a row
        # missing `value`/`label` produced "Unable to assign [undefined] to
        # QString" and an empty docking HUD.
        from helpers import json_packet
        self.model.ingest_frames(json_packet('v1/range/docking', [
            {'telemetry_key': 'diagonal_left_45deg',
             'distance_m': 4.885705947875977, 'valid': True},
            {'telemetry_key': 'center_line',
             'distance_m': 5.1432294845581055, 'valid': True},
            {'telemetry_key': 'diagonal_right_45deg',
             'distance_m': 5.082465171813965, 'valid': True},
        ]))
        self.app.processEvents()
        rows = self.bridge.dockingRangeRows
        self.assertEqual(len(rows), 3)
        by_key = {r['key']: r for r in rows}
        for key in ('diagonal_left_45deg', 'center_line',
                    'diagonal_right_45deg'):
            row = by_key[key]
            for field in ('key', 'label', 'value', 'valid'):
                self.assertIn(field, row)
            self.assertTrue(row['value'].endswith('m'))
            self.assertTrue(row['valid'])
        self.assertEqual(by_key['center_line']['value'], '5.143 m')
        self.assertEqual(by_key['center_line']['label'], 'Center')
        self.assertEqual(by_key['diagonal_left_45deg']['label'], '45° Left')

    def test_lidar_view_cycles_in_order(self):
        # Key 5 cycles top -> front -> left -> right -> iso -> top ...
        expected = ['top', 'front', 'left', 'right', 'iso', 'top']
        self.assertEqual(self.bridge.lidarView, 'top')
        for step, name in enumerate(expected[1:], start=1):
            self.bridge.cycle_lidar_view()
            self.app.processEvents()
            self.assertEqual(self.bridge.lidarView, name)
        # Labels are human-readable and track the mode.
        self.assertEqual(self.bridge.lidarView, 'top')
        self.bridge.cycle_lidar_view()
        self.assertEqual(self.bridge.lidarViewLabel, 'front (x-z)')
        # Reset to top for other tests.
        for _ in range(4):
            self.bridge.cycle_lidar_view()
        self.assertEqual(self.bridge.lidarView, 'top')

    def test_maintenance_hidden_without_hardware_status(self):
        self.assertFalse(self.bridge.maintenanceAvailable)
        self.assertEqual(self.bridge.maintenanceMode, 'unknown')

    def test_annotation_click_without_depth_shows_no_depth(self):
        self.bridge.annotate_click(10, 10)
        self.app.processEvents()
        self.assertFalse(self.bridge.annotationActive)
        self.assertIn('NO DEPTH', self.bridge.toast)

    def test_clear_annotation(self):
        self.bridge.clear_annotation()
        self.app.processEvents()
        self.assertFalse(self.bridge.annotationActive)

    def test_image_provider_serves_published_frame(self):
        import numpy as np
        from PySide2.QtCore import QSize
        frame = np.zeros((12, 16, 3), dtype=np.uint8)
        frame[:, :, 0] = 200
        self.provider.publish_rgb('cutter', frame)
        image = self.provider.requestImage('cutter', QSize(), QSize())
        self.assertEqual(image.width(), 16)
        self.assertEqual(image.height(), 12)

    def test_image_provider_strips_query_string_from_id(self):
        # Regression: QML appends "?n=<counter>" to the image source URL to
        # force re-requests; the query becomes part of requestImage's id and
        # must be stripped before the frame lookup, or the camera view renders
        # the dark placeholder instead of the decoded frame.
        import numpy as np
        from PySide2.QtCore import QSize
        frame = np.zeros((6, 8, 3), dtype=np.uint8)
        frame[:, :, 1] = 255
        self.provider.publish_rgb('cutter', frame)
        image = self.provider.requestImage('cutter?n=42', QSize(), QSize())
        self.assertEqual(image.width(), 8)
        self.assertEqual(image.height(), 6)
        # The green frame must be served, not the 4x4 dark placeholder.
        self.assertEqual(image.pixelColor(4, 3).green(), 255)

    def test_bridge_updates_from_synthetic_packets(self):
        from helpers import json_packet
        frames = json_packet('v1/range/cutter', {
            'telemetry_key': 'cutter_forward', 'distance_m': 1.5,
            'valid': True})
        self.model.ingest_frames(frames)
        self.app.processEvents()
        row = self.bridge.cutterRangeRow[0]
        self.assertIn('1.50 m', row['value'])
        self.assertTrue(row['valid'])
        self.assertEqual(self.bridge.sourceBadge, 'SIMULATION xavier')


@unittest.skipUnless(_GUI_IMPORTS_OK, 'PySide2 QtQuick not installed')
class HudPanelGatingTest(unittest.TestCase):
    """Operator panel gating: per-panel `visible` config and cutter overlap."""

    @classmethod
    def setUpClass(cls):
        from PySide2.QtCore import QObject
        from harvester_dashboard.bridge import DashboardBridge
        from harvester_dashboard.image_provider import FrameImageProvider
        from harvester_dashboard.hud_config import HudLayoutConfig, HudPanelConfig

        cls.app = QGuiApplication.instance() or QGuiApplication(
            ['harvester-dashboard-hud-gating'])
        cls._QObject = QObject
        # Boom panel disabled by config; docking enabled.
        layout = HudLayoutConfig(
            boom=HudPanelConfig(name='boom', visible=False, anchor='bottom-right'),
            docking=HudPanelConfig(name='docking', visible=True, anchor='bottom-left'),
            cutter_range=HudPanelConfig(name='cutter_range', visible=True),
        )
        cls.bridge = DashboardBridge(
            DashboardConfig(status_endpoint='', annotation_endpoint=''),
            TelemetryModel(), AnnotationState(), hud_config=layout)
        cls.view = QQuickView()
        cls.view.engine().addImageProvider('frames', FrameImageProvider())
        cls.view.rootContext().setContextProperty('bridge', cls.bridge)
        qml_dir = os.path.join(
            os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'qml')
        cls.view.setSource(QUrl.fromLocalFile(os.path.join(qml_dir, 'Dashboard.qml')))
        cls.root = cls.view.rootObject()

    @classmethod
    def tearDownClass(cls):
        cls.view.deleteLater()
        cls.app.processEvents()
        del cls.view

    def _loaders(self):
        return [c for c in self.root.findChildren(self._QObject)
                if c.metaObject().className() == 'QQuickLoader']

    def test_disabled_panel_config_is_honored(self):
        # Key-2 toggle is on (docking enabled), boom disabled by config.
        self.assertTrue(self.bridge.operatorHudsVisible)
        self.bridge.set_view('docking')
        for _ in range(4):
            self.app.processEvents()
        active = [l.property('active') for l in self._loaders()]
        # docking active, boom inactive, cutter inactive (docking view).
        self.assertEqual(active, [True, False, False])

    def test_docking_panel_hidden_on_cutter_view(self):
        # On the cutter view the docking panel hides so it cannot overlap
        # the cutter-range panel (both default to bottom-left).
        self.bridge.set_view('cutter')
        for _ in range(4):
            self.app.processEvents()
        active = [l.property('active') for l in self._loaders()]
        self.assertEqual(active, [False, False, True])

    def test_key2_toggle_drives_docking_loader(self):
        # Key 2 must actually drive the QML gating (docking loader active
        # state), not just the bridge property.
        self.bridge.set_view('docking')
        self.bridge._set_operator_huds_visible(True)
        for _ in range(4):
            self.app.processEvents()
        self.assertTrue(self._loaders()[0].property('active'))
        self.bridge.toggle_operator_huds()
        for _ in range(4):
            self.app.processEvents()
        self.assertFalse(self._loaders()[0].property('active'))
        # Restore.
        self.bridge._set_operator_huds_visible(True)
        for _ in range(4):
            self.app.processEvents()
        self.assertTrue(self._loaders()[0].property('active'))

    def test_key1_cutter_hud_flag_drives_loader(self):
        # Key 1's on-cutter hide/show must actually drive the cutter loader.
        self.bridge.set_view('cutter')
        self.bridge._set_cutter_hud_visible(True)
        for _ in range(4):
            self.app.processEvents()
        self.assertTrue(self._loaders()[2].property('active'))
        self.bridge._set_cutter_hud_visible(False)
        for _ in range(4):
            self.app.processEvents()
        self.assertFalse(self._loaders()[2].property('active'))
        # Restore.
        self.bridge.set_view('docking')
        self.bridge._set_cutter_hud_visible(True)
        for _ in range(4):
            self.app.processEvents()


if __name__ == '__main__':
    unittest.main()
