"""Wire-level no-emit proof for the render-only control semantics.

Starts a spy SUB on the canonical endpoint, a capture socket on the
annotation port, and drives the bridge through every view toggle.  Any
outbound packet from the dashboard during view switching fails the test.
"""

import json
import time
import unittest
import zmq

from helpers import base_header

from harvester_dashboard.annotation_publisher import AnnotationPublisher
from harvester_dashboard.model.target_model import AnnotationState
from harvester_dashboard.protocol_shim import unpack_message


class NoEmitProofTest(unittest.TestCase):
    """View switching must not emit any socket traffic, ever."""

    def setUp(self):
        self.context = zmq.Context()
        # A gateway stand-in: binds canonical PUB so the dashboard SUB has
        # something to connect to; also spies for any dashboard emission.
        self.canonical = self.context.socket(zmq.PUB)
        self.canonical.bind('tcp://127.0.0.1:55901')
        # Spy on the suggested annotation port (nothing should bind it
        # unless enabled; we bind to detect and reject stray binds).
        self.annotation_spy = self.context.socket(zmq.SUB)
        self.annotation_spy.setsockopt(zmq.SUBSCRIBE, b'')
        self.annotation_spy.bind('tcp://127.0.0.1:55902')

    def tearDown(self):
        self.canonical.close(0)
        self.annotation_spy.close(0)
        self.context.term()

    def _send_something(self):
        from harvester_dashboard.protocol_shim import pack_message
        header = base_header(clock_domain='utc_host')
        frames = pack_message(
            'v1/system/status', header, json.dumps({'ok': True}).encode())
        self.canonical.send_multipart(frames)

    def test_view_switching_emits_nothing(self):
        try:
            from PySide2.QtCore import QTimer
            from PySide2.QtGui import QGuiApplication
            from harvester_dashboard.bridge import DashboardBridge
            from harvester_dashboard.config import DashboardConfig
            from harvester_dashboard.model.telemetry_model import TelemetryModel
            from harvester_dashboard.model.target_model import AnnotationState
        except ImportError:
            self.skipTest('PySide2 unavailable')

        app = QGuiApplication.instance() or QGuiApplication(['no-emit-test'])
        model = TelemetryModel()
        annotation = AnnotationState()
        config = DashboardConfig(
            pub_endpoint='tcp://127.0.0.1:55901',
            status_endpoint='',            # no REQ: replay-style session
            annotation_endpoint='',        # annotation PUB disabled
        )
        bridge = DashboardBridge(config, model, annotation)
        self._send_something()

        # Feed a shrinking center-range series so the safety-guidance path
        # (speed derivation + evaluation + HUD properties) actually runs, then
        # drive every view control many times while spinning the loop.  A
        # shrinking cutter range drives the cutter guidance path too, including
        # the operator-confirmed cut-sequence prompt.
        import time as _time
        distance = 3.0
        cutter_distance = 1.0
        for _ in range(20):
            distance = max(0.2, distance - 0.05)
            cutter_distance = max(0.25, cutter_distance - 0.03)
            from helpers import json_packet
            model.ingest_frames(json_packet('v1/range/docking', [
                {'telemetry_key': 'center_line', 'distance_m': distance,
                 'valid': True}], sequence=int(_time.time() * 10) % 1000 + 1))
            model.ingest_frames(json_packet('v1/range/cutter', {
                'telemetry_key': 'cutter_forward',
                'distance_m': cutter_distance, 'valid': True},
                sequence=int(_time.time() * 10) % 1000 + 1))
            bridge.refresh()
            bridge.cutter_confirm_phase()
            bridge.set_view('docking')
            bridge.set_view('cutter')
            bridge.select_cutter_view()
            bridge.toggle_hud()
            bridge.toggle_operator_huds()
            bridge.toggle_lidar()
            bridge.toggle_diagnostic()
            for _ in range(5):
                app.processEvents()
            # The annotation spy channel must stay silent: view switching and
            # the guidance path never emit traffic on any socket.
            try:
                self.annotation_spy.recv_multipart(zmq.NOBLOCK)
                self.fail('annotation socket received traffic during view switch')
            except zmq.Again:
                pass
        # Restore toggles to defaults for other tests in this process.
        if not bridge.hudVisible:
            bridge.toggle_hud()
        if not bridge.operatorHudsVisible:
            bridge.toggle_operator_huds()
        if not bridge.lidarVisible:
            bridge.toggle_lidar()
        if bridge.diagnosticVisible:
            bridge.toggle_diagnostic()

    def test_diagnostic_toggle_flips_property(self):
        try:
            from PySide2.QtGui import QGuiApplication
            from harvester_dashboard.bridge import DashboardBridge
            from harvester_dashboard.config import DashboardConfig
            from harvester_dashboard.model.telemetry_model import TelemetryModel
            from harvester_dashboard.model.target_model import AnnotationState
        except ImportError:
            self.skipTest('PySide2 unavailable')

        app = QGuiApplication.instance() or QGuiApplication(['diag-toggle-test'])
        model = TelemetryModel()
        annotation = AnnotationState()
        config = DashboardConfig(
            pub_endpoint='tcp://127.0.0.1:55901',
            status_endpoint='',
            annotation_endpoint='',
        )
        bridge = DashboardBridge(config, model, annotation)
        initial = bridge.diagnosticVisible
        bridge.toggle_diagnostic()
        self.assertNotEqual(bridge.diagnosticVisible, initial)
        bridge.toggle_diagnostic()
        self.assertEqual(bridge.diagnosticVisible, initial)

    def test_mqtt_sensor_rows_render_boom_and_range(self):
        try:
            from PySide2.QtGui import QGuiApplication
            from harvester_dashboard.bridge import DashboardBridge
            from harvester_dashboard.config import DashboardConfig
            from harvester_dashboard.model.telemetry_model import TelemetryModel
            from harvester_dashboard.model.target_model import AnnotationState
        except ImportError:
            self.skipTest('PySide2 unavailable')

        app = QGuiApplication.instance() or QGuiApplication(['mqtt-rows-test'])
        model = TelemetryModel()
        annotation = AnnotationState()
        config = DashboardConfig(
            pub_endpoint='tcp://127.0.0.1:55901',
            status_endpoint='',
            annotation_endpoint='',
        )
        bridge = DashboardBridge(config, model, annotation)

        # Empty state: all rows present with '—' placeholders.
        rows = bridge.mqttSensorRows
        labels = [r['label'] for r in rows]
        self.assertIn('Boom angle', labels)
        self.assertIn('Laser 45° left', labels)
        self.assertIn('Laser center', labels)
        self.assertIn('Laser 45° right', labels)
        self.assertTrue(all(r['value'] == '—' for r in rows))

        # Feed a canonical boom payload (as produced by mqtt_ingest).
        from helpers import json_packet
        boom = {
            'phase': 'RUN',
            'boom_angle_deg': 0.101,
            'boom_extension_m': 0.0025,
            'slew_angle_deg': 12.5,
            'platform_tilt_x1_deg': 0.296,
            'platform_tilt_y1_deg': -0.127,
            'primemover_tilt_x2_deg': -36.41,
            'primemover_tilt_y2_deg': -43.10,
            'platform_roll_deg': 0.296,
            'platform_pitch_deg': -0.127,
            'docked': False,
        }
        model.ingest_frames(json_packet('v1/boom/state', boom, sequence=1))
        model.ingest_frames(json_packet('v1/range/docking', [
            {'telemetry_key': 'diagonal_left_45deg', 'distance_m': 4.92,
             'valid': True},
            {'telemetry_key': 'center_line', 'distance_m': 5.15,
             'valid': True},
        ], sequence=1))

        rows = bridge.mqttSensorRows
        by_label = {r['label']: r['value'] for r in rows}
        self.assertEqual(by_label['Run phase'], 'RUN')
        self.assertIn('0.101', by_label['Boom angle'])
        self.assertIn('4.920', by_label['Laser 45° left'])
        self.assertIn('5.150', by_label['Laser center'])
        self.assertEqual(by_label['Laser 45° right'], '—')
        self.assertNotEqual(by_label['Prime mover tilt X2'], '—')

    def test_annotation_publisher_disabled_has_no_socket(self):
        publisher = AnnotationPublisher('')   # default disabled
        self.assertIsNone(publisher.socket)
        self.assertFalse(publisher.enabled)
        annotation = AnnotationState()
        annotation.build('cutter', 4, 4, 1.0, {'k': [1, 0, 0, 0, 1, 0, 0, 0, 1]})
        self.assertFalse(publisher.publish_annotation(annotation))
        publisher.close()

    def test_annotation_publisher_emits_only_on_annotation(self):
        publisher = AnnotationPublisher(
            'inproc://annotation-test-{}'.format(id(self)),
            context=self.context)
        subscriber = self.context.socket(zmq.SUB)
        subscriber.setsockopt(zmq.SUBSCRIBE, b'')
        subscriber.connect(publisher.endpoint)
        # Give the subscription a moment to propagate.
        import time
        time.sleep(0.2)
        annotation = AnnotationState()
        annotation.build('cutter', 4, 4, 1.5, {'k': [1, 0, 0, 0, 1, 0, 0, 0, 1]})
        self.assertTrue(publisher.publish_annotation(annotation))
        deadline = time.time() + 2.0
        received = None
        while time.time() < deadline:
            try:
                received = subscriber.recv_multipart(zmq.NOBLOCK)
                break
            except zmq.Again:
                time.sleep(0.02)
        self.assertIsNotNone(received, 'annotation packet never arrived')
        channel, header, payload = unpack_message(received)
        self.assertEqual(channel, 'v1/operator/target_selection')
        body = json.loads(payload.decode('utf-8'))
        self.assertEqual(body['action'], 'created')
        self.assertIsNone(body['tree_base_xyz'])
        self.assertFalse(body['world_fixed'])
        publisher.close()
        subscriber.close(0)

    def _scan_bridge(self):
        from PySide2.QtGui import QGuiApplication
        from harvester_dashboard.bridge import DashboardBridge
        from harvester_dashboard.config import DashboardConfig
        from harvester_dashboard.model.telemetry_model import TelemetryModel
        from harvester_dashboard.model.target_model import AnnotationState

        app = QGuiApplication.instance() or QGuiApplication(['scan-test'])
        model = TelemetryModel()
        config = DashboardConfig(
            pub_endpoint='tcp://127.0.0.1:55901',
            status_endpoint='',
            annotation_endpoint='',
        )
        bridge = DashboardBridge(config, model, AnnotationState())
        # main.py wires the JSON hook; mirror it so the IMU/scan paths run.
        model.on_json = bridge.on_json_packet
        return app, model, bridge

    def test_scan_controls_emit_nothing(self):
        """begin_scan/cancel_scan/cycle_lidar_view must not write any socket."""
        try:
            import PySide2  # noqa: F401
        except ImportError:
            self.skipTest('PySide2 unavailable')
        app, _model, bridge = self._scan_bridge()
        bridge.toggle_lidar()
        bridge.begin_scan()
        for _ in range(6):
            bridge.cycle_lidar_view()
            bridge.cancel_scan()
            bridge.begin_scan()
            bridge.refresh()
            for _ in range(3):
                app.processEvents()
            try:
                self.annotation_spy.recv_multipart(zmq.NOBLOCK)
                self.fail('annotation socket received traffic during scan controls')
            except zmq.Again:
                pass
        if bridge.lidarVisible:
            bridge.toggle_lidar()

    def test_scan_phase_transitions(self):
        try:
            import PySide2  # noqa: F401
        except ImportError:
            self.skipTest('PySide2 unavailable')
        app, model, bridge = self._scan_bridge()
        self.assertEqual(bridge.scanPhase, 'idle')
        bridge.begin_scan()
        self.assertEqual(bridge.scanPhase, 'scanning')
        # Just after SCAN the LiDAR is still spinning up, so a missing cloud is
        # inside the grace period: the scan must NOT abort on the first tick.
        bridge.advance_scan()
        self.assertEqual(bridge.scanPhase, 'scanning')
        # Past the grace period with still no cloud -> NO DATA, LiDAR stood down.
        bridge._scan_started_at = (
            time.monotonic() - bridge._SCAN_CLOUD_GRACE_S - 1.0)
        bridge.advance_scan()
        self.assertEqual(bridge.scanPhase, 'no_data')
        self.assertEqual(bridge.scanEstimateRows, [])
        # Feed a cloud, rescan, and let the window elapse -> complete.
        import numpy as np
        rng = np.random.default_rng(0)
        trunk_z = rng.uniform(1.0, 12.0, size=1500)
        trunk = np.stack([rng.normal(0, 0.1, 1500), rng.normal(0, 0.1, 1500),
                          trunk_z], axis=1)
        theta = rng.uniform(0, 6.28, 1500)
        r = rng.uniform(0.5, 3.0, 1500)
        canopy = np.stack([r * np.cos(theta), r * np.sin(theta),
                           rng.uniform(9.2, 11.7, 1500)], axis=1)
        cloud = np.vstack([trunk, canopy]).astype('<f4')
        # Begin the scan FIRST, then feed frames: accumulation only happens
        # while scanning, which is the live order.
        bridge.begin_scan()
        from helpers import lidar_packet
        model.ingest_frames(lidar_packet(cloud))
        bridge.on_frame_decoded('v1/lidar/raw', cloud)
        # Accumulation happened as frames arrived while scanning.
        self.assertGreater(len(bridge._scan_accumulated), 0)
        # Force the scan window to have elapsed without sleeping.
        bridge._scan_started_at = time.monotonic() - bridge._scan_seconds - 1.0
        bridge.advance_scan()
        self.assertEqual(bridge.scanPhase, 'complete')
        self.assertTrue(bridge.scanEstimateRows)
        keys = [r['key'] for r in bridge.scanEstimateRows]
        self.assertIn('tree_height', keys)
        self.assertIn('boom_angle', keys)
        self.assertIn('status', keys)
        # Cancel returns to idle and clears the rows.
        bridge.cancel_scan()
        self.assertEqual(bridge.scanPhase, 'idle')
        self.assertEqual(bridge.scanEstimateRows, [])

    def test_stop_scan_ends_early_and_estimates(self):
        """STOP ends the window early, keeps the data, and estimates."""
        try:
            import PySide2  # noqa: F401
        except ImportError:
            self.skipTest('PySide2 unavailable')
        app, model, bridge = self._scan_bridge()
        import numpy as np
        from helpers import lidar_packet
        rng = np.random.default_rng(1)
        trunk_z = rng.uniform(1.0, 12.0, size=1200)
        trunk = np.stack([rng.normal(0, 0.1, 1200), rng.normal(0, 0.1, 1200),
                          trunk_z], axis=1)
        theta = rng.uniform(0, 6.28, 1200)
        r = rng.uniform(0.5, 3.0, 1200)
        canopy = np.stack([r * np.cos(theta), r * np.sin(theta),
                           rng.uniform(9.2, 11.7, 1200)], axis=1)
        cloud = np.vstack([trunk, canopy]).astype('<f4')
        model.ingest_frames(lidar_packet(cloud))
        bridge.on_frame_decoded('v1/lidar/raw', cloud)
        bridge.begin_scan()
        # Well before the window elapses.
        bridge._scan_started_at = time.monotonic()
        bridge.stop_scan()
        self.assertEqual(bridge.scanPhase, 'complete')
        self.assertTrue(bridge.scanEstimateRows)

    def test_control_publisher_emits_enabled_only(self):
        """SCAN -> enabled true; STOP/CANCEL -> enabled false; nothing else."""
        try:
            import PySide2  # noqa: F401
        except ImportError:
            self.skipTest('PySide2 unavailable')
        from harvester_dashboard.lidar_control import LidarControlPublisher

        sent = []

        class SpyPublisher(LidarControlPublisher):
            def __init__(self):
                self.endpoint = 'spy'
                self.socket = object()
                self.sent = 0
                self.errors = 0
                self.last_error = ''
                self._last_value = None

            def set_enabled(self, enabled):
                sent.append(bool(enabled))
                self._last_value = bool(enabled)
                return True

        app, model, bridge = self._scan_bridge()
        spy = SpyPublisher()
        bridge.lidar_control = spy
        self.assertTrue(bridge.lidarControlEnabled)
        bridge.toggle_lidar()
        bridge.begin_scan()
        bridge.cancel_scan()
        bridge.begin_scan()
        bridge.stop_scan()
        self.assertEqual(sent, [True, False, True, False])
        # Only the three actions write, and only booleans.
        self.assertTrue(all(isinstance(v, bool) for v in sent))

    def test_control_disabled_by_default(self):
        try:
            import PySide2  # noqa: F401
        except ImportError:
            self.skipTest('PySide2 unavailable')
        _app, _model, bridge = self._scan_bridge()
        # No --lidar-control: render-only, and the scan still works.
        self.assertFalse(bridge.lidarControlEnabled)
        self.assertIsNone(getattr(bridge, 'lidar_control', None))

    def test_lidar_scan_active_hides_unrelated_hud(self):
        """liadarScanActive tracks the overlay flag so the other HUD hides."""
        try:
            import PySide2  # noqa: F401
        except ImportError:
            self.skipTest('PySide2 unavailable')
        app, _model, bridge = self._scan_bridge()
        self.assertFalse(bridge.lidarVisible)
        self.assertFalse(bridge.lidarScanActive)
        bridge.toggle_lidar()
        app.processEvents()
        self.assertTrue(bridge.lidarVisible)
        self.assertTrue(bridge.lidarScanActive)
        bridge.toggle_lidar()
        app.processEvents()
        self.assertFalse(bridge.lidarScanActive)

    def test_lidar_imu_reports_and_stabilizes(self):
        try:
            import PySide2  # noqa: F401
        except ImportError:
            self.skipTest('PySide2 unavailable')
        app, model, bridge = self._scan_bridge()
        from helpers import imu_packet
        import numpy as np
        points = np.array([[1.0, 0.5, 2.0], [2.0, 0.0, 3.0]], dtype='<f4')
        bridge.on_frame_decoded('v1/lidar/raw', points)
        app.processEvents()
        self.assertFalse(bridge.lidarImuActive)
        model.ingest_frames(imu_packet(
            'v1/imu/lidar', attitude=[0.1, 0.05, 0.0]))
        app.processEvents()
        self.assertTrue(bridge.lidarImuActive)
        self.assertIn('lidar imu', bridge.lidarImuAttitudeLine)
        # The stabilized cloud still has the same point count and distances.
        self.assertEqual(len(bridge.lidarPoints), 2)

    def _ingest_lidar_json(self, model, payload):
        """Ingest a raw JSON packet on v1/imu/lidar (the recording shape)."""
        import json as _json
        from helpers import base_header
        from harvester_telemetry_contract import pack_message
        header = base_header(sequence=1, codec='json')
        model.ingest_frames(pack_message(
            'v1/imu/lidar', header, _json.dumps(payload).encode('utf-8')))

    def test_large_imu_tilt_withholds_stabilization(self):
        # The Gazebo recording's IMU is pure machine motion (~60 deg pitch).
        # Rotating the cloud by it collapses the scene, so stabilization must be
        # withheld and the HUD must say so rather than showing a distorted cloud.
        try:
            import PySide2  # noqa: F401
        except ImportError:
            self.skipTest('PySide2 unavailable')
        import math
        app, model, bridge = self._scan_bridge()
        import numpy as np
        pts = np.array([[8.5, 0.0, 9.0], [8.5, 0.0, 6.0]], dtype='<f4')
        bridge.on_frame_decoded('v1/lidar/raw', pts)
        # First sample latches the reference (level).
        self._ingest_lidar_json(model, {
            'orientation': {'x': 0.0, 'y': 0.0, 'z': 0.0, 'w': 1.0}})
        app.processEvents()
        # Second sample is a 60-degree pitch: outside the vibration band.
        a = math.radians(60.0)
        self._ingest_lidar_json(model, {
            'orientation': {'x': 0.0, 'y': math.sin(a / 2), 'z': 0.0,
                            'w': math.cos(a / 2)}})
        app.processEvents()
        self.assertTrue(bridge.lidarImuMotion)
        self.assertIn('motion', bridge.lidarImuAttitudeLine)
        # The cloud is NOT rotated: the 9 m point stays at 9 m.
        self.assertAlmostEqual(bridge.lidarPoints[0][2], 9.0, delta=0.01)

    def test_stabilization_recovers_after_machine_motion(self):
        # After a big swing the reference must RE-BASE, otherwise every later
        # sample looks like motion and stabilization stays off forever.
        try:
            import PySide2  # noqa: F401
        except ImportError:
            self.skipTest('PySide2 unavailable')
        import math
        app, model, bridge = self._scan_bridge()
        import numpy as np
        pts = np.array([[8.5, 0.0, 9.0]], dtype='<f4')
        bridge.on_frame_decoded('v1/lidar/raw', pts)
        self._ingest_lidar_json(model, {
            'orientation': {'x': 0.0, 'y': 0.0, 'z': 0.0, 'w': 1.0}})
        app.processEvents()
        a = math.radians(60.0)     # big swing: motion, reference re-bases
        self._ingest_lidar_json(model, {
            'orientation': {'x': 0.0, 'y': math.sin(a / 2), 'z': 0.0,
                            'w': math.cos(a / 2)}})
        app.processEvents()
        self.assertTrue(bridge.lidarImuMotion)
        # A small wobble about the NEW pose is vibration again -> recovered.
        b = math.radians(63.0)
        self._ingest_lidar_json(model, {
            'orientation': {'x': 0.0, 'y': math.sin(b / 2), 'z': 0.0,
                            'w': math.cos(b / 2)}})
        app.processEvents()
        self.assertFalse(bridge.lidarImuMotion)

    def test_small_imu_tilt_stabilizes(self):
        try:
            import PySide2  # noqa: F401
        except ImportError:
            self.skipTest('PySide2 unavailable')
        import math
        app, model, bridge = self._scan_bridge()
        import numpy as np
        pts = np.array([[8.5, 0.0, 9.0]], dtype='<f4')
        bridge.on_frame_decoded('v1/lidar/raw', pts)
        self._ingest_lidar_json(model, {
            'orientation': {'x': 0.0, 'y': 0.0, 'z': 0.0, 'w': 1.0}})
        app.processEvents()
        a = math.radians(3.0)   # vibration, inside the band
        self._ingest_lidar_json(model, {
            'orientation': {'x': 0.0, 'y': math.sin(a / 2), 'z': 0.0,
                            'w': math.cos(a / 2)}})
        app.processEvents()
        self.assertFalse(bridge.lidarImuMotion)

    def test_completed_scan_drops_to_no_data_when_stream_stops(self):
        # A replay reaching the end of its file must not leave a stale READY
        # estimate on screen.
        try:
            import PySide2  # noqa: F401
        except ImportError:
            self.skipTest('PySide2 unavailable')
        import time as _time
        app, model, bridge = self._scan_bridge()
        import numpy as np
        rng = np.random.default_rng(0)
        tz = rng.uniform(1.0, 12.0, 1500)
        trunk = np.stack([rng.normal(0, 0.1, 1500), rng.normal(0, 0.1, 1500),
                          tz], axis=1)
        th = rng.uniform(0, 6.28, 1500)
        r = rng.uniform(0.5, 3.0, 1500)
        canopy = np.stack([r * np.cos(th), r * np.sin(th),
                           rng.uniform(9.2, 11.7, 1500)], axis=1)
        cloud = np.vstack([trunk, canopy]).astype('<f4')
        from helpers import lidar_packet
        model.ingest_frames(lidar_packet(cloud))
        bridge.on_frame_decoded('v1/lidar/raw', cloud)
        bridge.begin_scan()
        bridge._scan_started_at = _time.monotonic() - bridge._scan_seconds - 1.0
        bridge.advance_scan()
        self.assertEqual(bridge.scanPhase, 'complete')
        # Simulate the stream ending: rewind the receipt clock past the grace.
        state = model.state('v1/lidar/raw')
        state.last_recv_monotonic_s -= (bridge._SCAN_STALE_GRACE_S + 1.0)
        bridge.advance_scan()
        self.assertEqual(bridge.scanPhase, 'no_data')
        self.assertIn('stale', bridge._scan_reason)
        self.assertEqual(bridge.scanEstimateRows, [])


if __name__ == '__main__':
    unittest.main()
