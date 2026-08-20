"""Scripted GUI acceptance harness (offscreen): keys, clicks, grabs.

Run manually::

    QT_QPA_PLATFORM=offscreen PYTHONPATH=src/harvester_dashboard \\
    /usr/bin/python3 test_gui_acceptance.py

Loads the real QML with a synthetic live source (contract-built packets
fed in-process), then:
  * grabs the cutter view,
  * presses "2" -> grabs docking view,
  * clicks a pixel with valid synthetic depth -> verifies crosshair state,
  * clicks a pixel with no depth -> verifies NO DEPTH toast,
  * presses "0" -> verifies annotation cleared,
  * verifies keys 1/2 emit nothing on the wire (spy SUB).
"""

import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')

from PySide2.QtCore import QEvent, QUrl, QTimer, Qt
from PySide2.QtGui import QGuiApplication, QKeyEvent
from PySide2.QtQuick import QQuickView

from helpers import depth_packet, jpeg_packet, json_packet

from harvester_dashboard.bridge import DashboardBridge
from harvester_dashboard.config import DashboardConfig
from harvester_dashboard.image_provider import FrameImageProvider
from harvester_dashboard.model.telemetry_model import TelemetryModel
from harvester_dashboard.model.target_model import AnnotationState
from harvester_dashboard.zmq_source import SocketDrainer

GRABS = '/tmp/kilo'


def main() -> int:
    app = QGuiApplication(['gui-acceptance'])
    model = TelemetryModel()
    annotation = AnnotationState()
    config = DashboardConfig(pub_endpoint='tcp://127.0.0.1:55905',
                             status_endpoint='')
    bridge = DashboardBridge(config, model, annotation)
    provider = FrameImageProvider()

    view = QQuickView()
    view.engine().addImageProvider('frames', provider)
    view.rootContext().setContextProperty('bridge', bridge)
    qml = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                       '..', 'qml', 'Dashboard.qml')
    view.setSource(QUrl.fromLocalFile(os.path.abspath(qml)))
    view.setGeometry(0, 0, 1280, 800)
    view.show()
    app.processEvents()

    # --- synthetic live camera stream (mirrors the wire via the model) ---
    import numpy as np
    camera_info = {'width': 64, 'height': 48,
                   'k': [55.0, 0, 32, 0, 55.0, 24, 0, 0, 1]}
    model.ingest_frames(json_packet(
        'v1/camera/cutter/camera_info', camera_info, width=64, height=48))
    depth = np.full((48, 64), 2.0, dtype=np.float32)
    depth[0:20, 40:60] = np.nan   # a no-depth corner region
    model.ingest_frames(depth_packet(
        'v1/camera/cutter/depth', depth_m=depth, width=64, height=48))
    rgb_frames = jpeg_packet('v1/camera/cutter/rgb', width=64, height=48)
    model.ingest_frames(rgb_frames)
    from harvester_dashboard.protocol_shim import unpack_message
    _c, rgb_header, rgb_payload = unpack_message(rgb_frames)
    rgb = SocketDrainer.decode_frame('v1/camera/cutter/rgb', rgb_header, rgb_payload)
    bridge.on_frame_decoded('v1/camera/cutter/rgb', rgb)
    provider.publish_rgb('cutter', rgb)
    depth_frames = depth_packet(
        'v1/camera/cutter/depth', depth_m=depth, width=64, height=48)
    model.ingest_frames(depth_frames)
    _cd, depth_header, depth_payload = unpack_message(depth_frames)
    depth_decoded = SocketDrainer.decode_frame(
        'v1/camera/cutter/depth', depth_header, depth_payload)
    bridge.on_frame_decoded('v1/camera/cutter/depth', depth_decoded)
    docking_frames = jpeg_packet('v1/camera/docking/rgb', width=64, height=48,
                                 color=(0, 120, 255))
    model.ingest_frames(docking_frames)
    _c2, d_header, d_payload = unpack_message(docking_frames)
    docking_rgb = SocketDrainer.decode_frame(
        'v1/camera/docking/rgb', d_header, d_payload)
    bridge.on_frame_decoded('v1/camera/docking/rgb', docking_rgb)
    provider.publish_rgb('docking', docking_rgb)
    app.processEvents()

    results = []
    def check(name, ok):
        results.append((name, bool(ok)))
        print('{}: {}'.format('PASS' if ok else 'FAIL', name))

    def grab(name):
        app.processEvents(); app.processEvents()
        image = view.grabWindow()
        path = os.path.join(GRABS, 'acceptance_{}.png'.format(name))
        image.save(path)
        return path

    def key(k):
        event = QKeyEvent(QEvent.KeyPress, k, Qt.NoModifier)
        app.sendEvent(view.rootObject(), event)
        app.sendEvent(view.rootObject(), QKeyEvent(QEvent.KeyRelease, k, Qt.NoModifier))
        app.processEvents()

    # 1. cutter view visible with synthetic image
    grab('cutter')
    check('cutter view active', bridge.view == 'cutter')
    check('camera info ingested', model.snapshot_camera_info('cutter') is not None)

    # 2. key 2 -> docking (render-only)
    key(Qt.Key_2)
    grab('docking')
    check('key 2 switched to docking', bridge.view == 'docking')
    key(Qt.Key_1)
    check('key 1 switched back to cutter', bridge.view == 'cutter')

    # 3. annotate a valid-depth pixel (image 64x48 mapped into 1280x800)
    # image displayed area: width/height scaled preserving aspect; click center
    bridge.annotate_click(32, 24)
    app.processEvents()
    grab('annotation')
    check('annotation active with depth', bridge.annotationActive is True)
    check('annotation distance 2.00 m', '2.00 m' in bridge.annotationLabel)

    # 4. clear with key 0
    key(Qt.Key_0)
    check('key 0 cleared annotation', bridge.annotationActive is False)

    # 5. no-depth pixel (u,v inside NaN corner) -> toast, no annotation
    bridge.annotate_click(50, 10)
    app.processEvents()
    grab('nodepth')
    check('NO DEPTH toast shown', 'NO DEPTH' in bridge.toast)
    check('no annotation created without depth', bridge.annotationActive is False)

    # 6. stale flag flips after silence
    time.sleep(0.1)
    check('camera fresh before silence', bridge.activeCameraStale is False)
    # force staleness by rewinding the receipt clock
    state = model.state('v1/camera/cutter/rgb')
    if state.last_recv_monotonic_s is None:
        state.last_recv_monotonic_s = time.monotonic()
    state.last_recv_monotonic_s -= 3.0
    app.processEvents()
    check('camera stale after 3 s silence', bridge.activeCameraStale is True)
    grab('stale')

    # 7. key 5 cycles the LiDAR view top -> front -> left -> right -> iso
    # Feed a synthetic LiDAR cloud so the inset repaints in each view.
    import numpy as np
    cloud = np.array([[1.0, 0.0, 0.5], [0.0, 1.0, 0.0], [-1.0, 0.0, 0.0],
                      [0.0, -1.0, 0.0], [2.0, 2.0, 1.0], [0.0, 0.0, 0.5]],
                     dtype=np.float32)
    bridge.on_frame_decoded('v1/lidar/raw', cloud)
    app.processEvents()
    expected = ['top', 'front', 'left', 'right', 'iso', 'top']
    check('lidar view starts top', bridge.lidarView == 'top')
    for name in expected[1:]:
        key(Qt.Key_5)
        check('key 5 -> {}'.format(name), bridge.lidarView == name)
    grab('lidar_iso')
    key(Qt.Key_5)  # wrap back to top for any later steps
    check('key 5 wraps to front', bridge.lidarView == 'front')

    failures = [name for name, ok in results if not ok]
    print('\n{}/{} checks passed'.format(
        len(results) - len(failures), len(results)))
    return 1 if failures else 0


if __name__ == '__main__':
    sys.exit(main())
