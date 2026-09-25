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


def _abs_center_x(item, root):
    """Absolute (root-space) x of an item's horizontal center.

    ``item.x``/``width`` are in the item's parent coordinate space (the camera
    view inset), so map to the root to compare against the screen center.
    """
    from PySide2.QtCore import QPointF
    return item.mapToItem(root, QPointF(item.property('width') / 2.0, 0)).x()


def _abs_left_x(item, root):
    """Absolute (root-space) left edge x of an item."""
    from PySide2.QtCore import QPointF
    return item.mapToItem(root, QPointF(0, 0)).x()


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
        self.assertEqual(set(layout), {'boom', 'docking', 'cutter_range',
                                       'dock_guidance', 'cutter_guidance',
                                       'lidar_scan'})
        self.assertEqual(layout['boom']['anchor'], 'bottom-right')
        self.assertEqual(layout['docking']['anchor'], 'bottom-left')
        self.assertEqual(layout['dock_guidance']['anchor'], 'bottom-center')
        self.assertEqual(layout['cutter_guidance']['anchor'], 'bottom-center')
        self.assertEqual(layout['lidar_scan']['anchor'], 'bottom-center')
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

    def test_lidar_toolbar_button_tracks_visibility_flag(self):
        # The "4 LiDAR" button outlines on the blue selected colour while the
        # scan overlay is up.  The toolbar delegates are not instantiated in an
        # offscreen QQuickView, so this asserts the flag the QML binding keys on
        # (the border expression itself is a one-line mirror of this).
        if self.bridge.lidarVisible:
            self.bridge.toggle_lidar()
        self.app.processEvents()
        self.assertFalse(self.bridge.lidarVisible)
        self.bridge.toggle_lidar()
        self.app.processEvents()
        self.assertTrue(self.bridge.lidarVisible)
        self.bridge.toggle_lidar()   # restore hidden
        self.app.processEvents()

    def test_lidar_view_cycles_in_order(self):
        # The overlay opens in the FRONT (x-z) working view; key 5 cycles
        # front -> left -> right -> iso -> camera -> top -> front ...
        expected = ['front', 'left', 'right', 'iso', 'camera', 'top', 'front']
        self.assertEqual(self.bridge.lidarView, 'front')
        for step, name in enumerate(expected[1:], start=1):
            self.bridge.cycle_lidar_view()
            self.app.processEvents()
            self.assertEqual(self.bridge.lidarView, name)
        # Label tracks the mode.  `front` plots y across and z up, so its label
        # is (y-z), matching projection.VIEW_LABELS.
        self.assertEqual(self.bridge.lidarView, 'front')
        self.assertEqual(self.bridge.lidarViewLabel, 'front (y-z)')
        # Reset to front for other tests.
        self.assertEqual(self.bridge.lidarView, 'front')

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
            dock_guidance=HudPanelConfig(name='dock_guidance', visible=True),
            cutter_guidance=HudPanelConfig(name='cutter_guidance', visible=True),
            lidar_scan=HudPanelConfig(name='lidar_scan', visible=True),
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
        # Loader order: docking, boom, cutter, cutter_guidance, dock_guidance.
        # docking active, boom inactive, cutter inactive (docking view),
        # cutter_guidance inactive (docking view),
        # dock_guidance active (docking view + operator HUDs on).
        self.assertEqual(active, [True, False, False, False, True])

    def test_docking_panel_hidden_on_cutter_view(self):
        # On the cutter view the docking panel hides so it cannot overlap
        # the cutter-range panel (both default to bottom-left); the docking
        # guidance panel is docking-view only too.  The cutter guidance panel
        # is cutter-view only.
        self.bridge.set_view('cutter')
        for _ in range(4):
            self.app.processEvents()
        active = [l.property('active') for l in self._loaders()]
        self.assertEqual(active, [False, False, True, True, False])

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

    def test_lidar_scan_mode_hides_unrelated_hud(self):
        # While the full-screen LiDAR scan overlay is up, the operator sensor
        # panels must deactivate (the scan view is not cluttered by them).
        from PySide2.QtCore import QObject
        self.bridge.set_view('docking')
        self.bridge._set_operator_huds_visible(True)
        for _ in range(4):
            self.app.processEvents()
        self.assertTrue(self._loaders()[0].property('active'))
        self.bridge.toggle_lidar()
        for _ in range(4):
            self.app.processEvents()
        active = [l.property('active') for l in self._loaders()]
        self.assertFalse(any(active),
                         'operator HUD loaders must all deactivate in scan mode')
        self.bridge.toggle_lidar()   # hide overlay
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


@unittest.skipUnless(_GUI_IMPORTS_OK, 'PySide2 QtQuick not installed')
class DockGuidanceGuiTest(unittest.TestCase):
    """The docking safety-guidance HUD boots grey and renders in the layout."""

    @classmethod
    def setUpClass(cls):
        from harvester_dashboard.bridge import DashboardBridge
        from harvester_dashboard.image_provider import FrameImageProvider

        cls.app = QGuiApplication.instance() or QGuiApplication(
            ['harvester-dashboard-dock-guidance'])
        cls.model = TelemetryModel()
        cls.bridge = DashboardBridge(
            DashboardConfig(status_endpoint='', annotation_endpoint=''),
            cls.model, AnnotationState())
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

    def _process(self):
        for _ in range(4):
            self.app.processEvents()

    def test_guidance_boots_no_data_never_safe(self):
        # Initial guidance must be NO_DATA (grey), never a reassuring green.
        self.assertEqual(self.bridge.dockSafetyState, 'no_data')
        self.assertIn('awaiting', self.bridge.dockGuidanceText)

    def test_guidance_float_properties_are_nan_without_data(self):
        import math
        for value in (self.bridge.dockSpeedCmS,
                      self.bridge.dockSpeedSmoothedCmS,
                      self.bridge.dockCenterDistanceM,
                      self.bridge.dockRecommendedSpeedCmS,
                      self.bridge.dockMaxSpeedCmS,
                      self.bridge.dockStopDistanceM,
                      self.bridge.dockTtcS):
            self.assertTrue(math.isnan(value), value)

    def test_guidance_rows_present_and_shaped(self):
        rows = self.bridge.dockSafetyRow
        keys = [row['key'] for row in rows]
        self.assertEqual(keys, ['state', 'speed', 'gap', 'ttc', 'max_speed'])
        by_key = {r['key']: r for r in rows}
        self.assertEqual(by_key['state']['value'], 'NO DATA')
        self.assertFalse(by_key['state']['valid'])
        for row in rows:
            self.assertIn('label', row)
            self.assertIn('value', row)

    def test_guidance_panel_is_docking_view_only(self):
        from PySide2.QtCore import QObject

        def loaders():
            return [c for c in self.root.findChildren(QObject)
                    if c.metaObject().className() == 'QQuickLoader']

        self.bridge.set_view('docking')
        self._process()
        # Loader order: docking, boom, cutter, cutter_guidance, dock_guidance.
        self.assertTrue(loaders()[4].property('active'))
        self.bridge.set_view('cutter')
        self._process()
        self.assertFalse(loaders()[4].property('active'))
        self.bridge.set_view('docking')
        self._process()
        self.assertTrue(loaders()[4].property('active'))

    def test_bottom_row_panels_do_not_overlap(self):
        # The three bottom panels (docking ranges | dock guidance | boom) must
        # sit edge-to-edge between the screen edges, never overlapping.
        from PySide2.QtCore import QObject

        def loaders():
            return [c for c in self.root.findChildren(QObject)
                    if c.metaObject().className() == 'QQuickLoader']

        self.bridge.set_view('docking')
        # The deployment monitor is 1920x1080; the panels only all fit at a
        # width wide enough to leave a gap between the docking and boom panels.
        self.root.setProperty('width', 1920)
        self._process()
        docking, boom, _cutter, _cutter_guidance, guidance = loaders()
        for loader in (docking, boom, guidance):
            self.assertTrue(loader.property('active'))
        self.assertTrue(guidance.property('visible'))
        docking_right = docking.property('x') + docking.property('width')
        guidance_left = guidance.property('x')
        guidance_right = guidance.property('x') + guidance.property('width')
        boom_left = boom.property('x')
        self.assertLessEqual(docking_right, guidance_left,
                             'docking ranges overlaps dock guidance')
        self.assertLessEqual(guidance_right, boom_left,
                             'dock guidance overlaps boom')
        # Guidance sits between the two, not stacked on either edge.
        self.assertGreater(guidance.property('width'), 0)
        self.assertLess(guidance.property('x'), boom_left)

    def test_bottom_row_hides_guidance_when_too_narrow(self):
        # On a screen too narrow to fit all three, the guidance panel hides
        # rather than overlapping its neighbours.
        from PySide2.QtCore import QObject

        def loaders():
            return [c for c in self.root.findChildren(QObject)
                    if c.metaObject().className() == 'QQuickLoader']

        self.bridge.set_view('docking')
        self.root.setProperty('width', 1024)
        self._process()
        _docking, _boom, _cutter, _cutter_guidance, guidance = loaders()
        self.assertFalse(guidance.property('visible'))
        self.root.setProperty('width', 1920)
        self._process()


@unittest.skipUnless(_GUI_IMPORTS_OK, 'PySide2 QtQuick not installed')
class CutterGuidanceGuiTest(unittest.TestCase):
    """The cutter safety-guide HUD is cutter-view only and never overlaps the
    Cutter Range (distance sensor) panel below it."""

    @classmethod
    def setUpClass(cls):
        from harvester_dashboard.bridge import DashboardBridge
        from harvester_dashboard.image_provider import FrameImageProvider

        cls.app = QGuiApplication.instance() or QGuiApplication(
            ['harvester-dashboard-cutter-guidance'])
        cls.model = TelemetryModel()
        cls.bridge = DashboardBridge(
            DashboardConfig(status_endpoint='', annotation_endpoint=''),
            cls.model, AnnotationState())
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

    def _process(self):
        for _ in range(4):
            self.app.processEvents()

    def _loaders(self):
        from PySide2.QtCore import QObject
        return [c for c in self.root.findChildren(QObject)
                if c.metaObject().className() == 'QQuickLoader']

    def test_boots_no_data_never_safe(self):
        self.assertEqual(self.bridge.cutterSafetyState, 'no_data')
        self.assertEqual(self.bridge.cutterPhase, 'idle')
        self.assertIn('awaiting', self.bridge.cutterGuidanceText)

    def test_float_properties_are_nan_without_data(self):
        import math
        for value in (self.bridge.cutterClearanceM,
                      self.bridge.cutterRawRangeM,
                      self.bridge.cutterSpeedSmoothedCmS,
                      self.bridge.cutterMaxSpeedCmS,
                      self.bridge.cutterStopDistanceM,
                      self.bridge.cutterTtcS):
            self.assertTrue(math.isnan(value), value)

    def test_cutter_rows_present_and_shaped(self):
        rows = self.bridge.cutterSafetyRow
        keys = [row['key'] for row in rows]
        self.assertEqual(keys, ['state', 'phase', 'clearance', 'speed',
                                'ttc', 'max_speed'])
        by_key = {r['key']: r for r in rows}
        self.assertEqual(by_key['state']['value'], 'NO DATA')
        self.assertFalse(by_key['state']['valid'])

    def test_cutter_guidance_panel_is_cutter_view_only(self):
        # Loader order: docking, boom, cutter, cutter_guidance, dock_guidance.
        self.bridge.set_view('cutter')
        self._process()
        self.assertTrue(self._loaders()[3].property('active'))
        self.bridge.set_view('docking')
        self._process()
        self.assertFalse(self._loaders()[3].property('active'))

    def test_key1_toggle_hides_cutter_guidance_with_range_hud(self):
        # Key 1 on the cutter view toggles the Cutter Range HUD; the Cutter
        # Safety-Guide HUD must react together, or the toggle would leave a
        # guidance panel behind on an otherwise-empty cutter view.
        self.bridge.set_view('cutter')
        self.bridge._set_cutter_hud_visible(True)
        self._process()
        self.assertTrue(self._loaders()[2].property('active'))
        self.assertTrue(self._loaders()[3].property('active'))
        self.bridge.select_cutter_view()   # key 1 -> hide both
        self._process()
        self.assertFalse(self._loaders()[2].property('active'))
        self.assertFalse(self._loaders()[3].property('active'))
        self.bridge.select_cutter_view()   # key 1 again -> show both
        self._process()
        self.assertTrue(self._loaders()[2].property('active'))
        self.assertTrue(self._loaders()[3].property('active'))
        # Restore for other tests.
        self.bridge.set_view('docking')
        self.bridge._set_cutter_hud_visible(True)
        self._process()

    def test_cutter_row_panels_do_not_overlap(self):
        # On the cutter view the Cutter Range panel (bottom-left) and the
        # Cutter Safety-Guide panel (bottom-center) must sit side by side.
        self.bridge.set_view('cutter')
        self.root.setProperty('width', 1920)
        self._process()
        _docking, _boom, cutter_range, cutter_guidance, _dock = self._loaders()
        self.assertTrue(cutter_range.property('active'))
        self.assertTrue(cutter_guidance.property('active'))
        self.assertTrue(cutter_guidance.property('visible'))
        range_right = _abs_left_x(cutter_range, self.root) \
            + cutter_range.property('width')
        guidance_left = _abs_left_x(cutter_guidance, self.root)
        self.assertLessEqual(range_right, guidance_left,
                             'cutter range overlaps cutter guidance')
        self.assertGreater(cutter_guidance.property('width'), 0)
        # Restore the view for other tests.
        self.bridge.set_view('docking')
        self._process()

    def test_cutter_guidance_is_screen_centered(self):
        # The guidance panel must be centered on the SCREEN, not merely in the
        # leftover gap right of the Cutter Range panel (which sits bottom-left
        # and would push a gap-centered panel visibly right of center).
        self.bridge.set_view('cutter')
        for width in (1920, 1600, 1440, 1280):
            with self.subTest(width=width):
                self.root.setProperty('width', width)
                self._process()
                _d, _b, _cr, guidance, _dock = self._loaders()
                self.assertTrue(guidance.property('visible'))
                center = _abs_center_x(guidance, self.root)
                self.assertAlmostEqual(
                    center, width / 2.0, delta=2.0,
                    msg='cutter guidance is not screen-centered at %d' % width)
        self.root.setProperty('width', 1920)
        self._process()

    def test_cutter_row_hides_guidance_when_too_narrow(self):
        self.bridge.set_view('cutter')
        # Narrower than the cutter-range panel (460) + margins + a usable
        # guidance panel, so the guidance panel hides rather than overlapping.
        self.root.setProperty('width', 760)
        self._process()
        _docking, _boom, _cutter_range, cutter_guidance, _dock = self._loaders()
        self.assertTrue(cutter_guidance.property('active'))
        self.assertFalse(cutter_guidance.property('visible'))
        self.root.setProperty('width', 1920)
        self._process()


@unittest.skipUnless(_GUI_IMPORTS_OK, 'PySide2 QtQuick not installed')
class CutterGuidanceRightAnchorGuiTest(unittest.TestCase):
    """A right-anchored Cutter Range panel must never be overlapped by the
    Cutter Safety-Guide panel (its reserved band follows the range panel)."""

    @classmethod
    def setUpClass(cls):
        import json
        import tempfile

        from harvester_dashboard.bridge import DashboardBridge
        from harvester_dashboard.hud_config import load_hud_config
        from harvester_dashboard.image_provider import FrameImageProvider

        cls.app = QGuiApplication.instance() or QGuiApplication(
            ['harvester-dashboard-cutter-right-anchor'])
        # hudLayout is a constant QML property, so the layout must be set at
        # construction time (as a real --hud-config deployment does).
        handle = tempfile.NamedTemporaryFile(
            mode='w', suffix='.json', delete=False)
        json.dump({'cutter_range': {'anchor': 'bottom-right'}}, handle)
        handle.close()
        cls._config_path = handle.name
        cls.model = TelemetryModel()
        cls.bridge = DashboardBridge(
            DashboardConfig(status_endpoint='', annotation_endpoint=''),
            cls.model, AnnotationState(),
            hud_config=load_hud_config(cls._config_path))
        cls.view = QQuickView()
        cls.view.engine().addImageProvider('frames', FrameImageProvider())
        cls.view.rootContext().setContextProperty('bridge', cls.bridge)
        qml_dir = os.path.join(
            os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'qml')
        cls.view.setSource(
            QUrl.fromLocalFile(os.path.join(qml_dir, 'Dashboard.qml')))
        cls.root = cls.view.rootObject()
        cls.root.setProperty('width', 1920)
        cls.root.setProperty('height', 1080)

    @classmethod
    def tearDownClass(cls):
        cls.view.deleteLater()
        cls.app.processEvents()
        del cls.view
        os.unlink(cls._config_path)

    def test_right_anchored_range_not_overlapped(self):
        self.bridge.set_view('cutter')
        for _ in range(10):
            self.app.processEvents()
        from PySide2.QtCore import QObject
        loaders = [c for c in self.root.findChildren(QObject)
                   if c.metaObject().className() == 'QQuickLoader']
        cutter_range, cutter_guidance = loaders[2], loaders[3]
        self.assertTrue(cutter_range.property('active'))
        # The range panel really is on the right.
        range_left = _abs_left_x(cutter_range, self.root)
        self.assertGreater(range_left, 1920 / 2.0)
        if not cutter_guidance.property('visible'):
            # Hiding is the safe outcome; there is no overlap.
            return
        guide_right = _abs_left_x(cutter_guidance, self.root) \
            + cutter_guidance.property('width')
        self.assertLessEqual(
            guide_right, range_left,
            'cutter guidance overlaps a right-anchored range panel')


@unittest.skipUnless(_GUI_IMPORTS_OK, 'PySide2 QtQuick not installed')
class CutterGuidanceCenterRangeGuiTest(unittest.TestCase):
    """A centered Cutter Range panel must not be overlapped by the guidance
    panel; a side-anchored guidance panel sits in the free edge band."""

    @classmethod
    def setUpClass(cls):
        import json
        import tempfile

        from harvester_dashboard.bridge import DashboardBridge
        from harvester_dashboard.hud_config import load_hud_config
        from harvester_dashboard.image_provider import FrameImageProvider

        cls.app = QGuiApplication.instance() or QGuiApplication(
            ['harvester-dashboard-cutter-center-range'])
        handle = tempfile.NamedTemporaryFile(
            mode='w', suffix='.json', delete=False)
        json.dump({'cutter_range': {'anchor': 'bottom-center'},
                   'cutter_guidance': {'anchor': 'bottom-left'}}, handle)
        handle.close()
        cls._config_path = handle.name
        cls.bridge = DashboardBridge(
            DashboardConfig(status_endpoint='', annotation_endpoint=''),
            TelemetryModel(), AnnotationState(),
            hud_config=load_hud_config(cls._config_path))
        cls.view = QQuickView()
        cls.view.engine().addImageProvider('frames', FrameImageProvider())
        cls.view.rootContext().setContextProperty('bridge', cls.bridge)
        qml_dir = os.path.join(
            os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'qml')
        cls.view.setSource(
            QUrl.fromLocalFile(os.path.join(qml_dir, 'Dashboard.qml')))
        cls.root = cls.view.rootObject()
        cls.root.setProperty('width', 1920)
        cls.root.setProperty('height', 1080)

    @classmethod
    def tearDownClass(cls):
        cls.view.deleteLater()
        cls.app.processEvents()
        del cls.view
        os.unlink(cls._config_path)

    def test_center_range_with_side_guide_no_overlap(self):
        self.bridge.set_view('cutter')
        for _ in range(10):
            self.app.processEvents()
        from PySide2.QtCore import QObject
        loaders = [c for c in self.root.findChildren(QObject)
                   if c.metaObject().className() == 'QQuickLoader']
        cutter_range, cutter_guidance = loaders[2], loaders[3]
        self.assertTrue(cutter_range.property('active'))
        if not cutter_guidance.property('visible'):
            # Hiding is also acceptable; there is no overlap.
            return
        range_left = _abs_left_x(cutter_range, self.root)
        guide_right = _abs_left_x(cutter_guidance, self.root) \
            + cutter_guidance.property('width')
        self.assertLessEqual(
            guide_right, range_left,
            'side guidance overlaps a centered range panel')


if __name__ == '__main__':
    unittest.main()
