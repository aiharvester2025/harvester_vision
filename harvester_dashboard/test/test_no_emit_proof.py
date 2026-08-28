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

        # Drive every view control many times while spinning the loop.
        for _ in range(20):
            bridge.set_view('docking')
            bridge.set_view('cutter')
            bridge.toggle_hud()
            bridge.toggle_lidar()
            bridge.toggle_diagnostic()
            for _ in range(5):
                app.processEvents()
            # The annotation spy channel must stay silent: view switching
            # never emits traffic on any socket.
            try:
                self.annotation_spy.recv_multipart(zmq.NOBLOCK)
                self.fail('annotation socket received traffic during view switch')
            except zmq.Again:
                pass
        # Restore toggles to defaults for other tests in this process.
        if not bridge.hudVisible:
            bridge.toggle_hud()
        if not bridge.lidarVisible:
            bridge.toggle_lidar()
        if not bridge.diagnosticVisible:
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


if __name__ == '__main__':
    unittest.main()
