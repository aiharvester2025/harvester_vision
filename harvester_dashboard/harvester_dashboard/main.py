#!/usr/bin/env python3
"""Dashboard entry point: QApplication + QML engine + telemetry wiring.

Run under the system Python with the Qt Quick modules installed::

    DISPLAY=:1 /usr/bin/python3 -m harvester_dashboard.main \
        --pub tcp://127.0.0.1:5591 --status '' 

Sockets created by this process, exhaustively:

1. One SUB socket to ``--pub`` (read-only drain).
2. Optional REQ to ``--status`` (read-only query).  Empty ``--status``
   disables it entirely.
3. Optional annotation forward PUB to ``--annotation-pub`` (default
   disabled).  It never binds 5590/5591/5600.

View switching (keys 1/2) is render-only and emits no traffic.
"""

from __future__ import annotations

import signal
import sys
from pathlib import Path

from .config import DashboardConfig
from .protocol_shim import ensure_contract_importable

ensure_contract_importable()

from PySide2.QtCore import QUrl, Qt
from PySide2.QtGui import QGuiApplication
from PySide2.QtQuick import QQuickView


def qml_directory() -> Path:
    packaged = Path(__file__).resolve().parent.parent / 'qml'
    if packaged.is_dir():
        return packaged
    raise SystemExit('QML views directory not found next to the package')


def main(argv=None) -> int:
    config = DashboardConfig.from_args(argv)
    app = QGuiApplication(sys.argv[:1] if argv is None else [sys.argv[0]])
    app.setApplicationName('harvester-dashboard')
    app.setOrganizationName('harvester-telemetry')

    from .model.telemetry_model import TelemetryModel
    from .model.target_model import AnnotationState
    from .bridge import DashboardBridge
    from .image_provider import FrameImageProvider
    from .zmq_source import TelemetrySource

    model = TelemetryModel()
    annotation = AnnotationState()
    from .annotation_publisher import AnnotationPublisher
    annotation_publisher = AnnotationPublisher(config.annotation_endpoint)
    bridge = DashboardBridge(config, model, annotation,
                             annotation_publisher=annotation_publisher)
    provider = FrameImageProvider()
    source = TelemetrySource(config, model)
    source.on_frame = _make_frame_sink(bridge, provider)

    # QQuickView (not QQmlApplicationEngine): Controls2 is unavailable on
    # this PySide2 build, so the root is a plain Item that QQuickView wraps
    # in a real window.
    view = QQuickView()
    view.engine().addImageProvider('frames', provider)
    view.rootContext().setContextProperty('bridge', bridge)
    view.setSource(QUrl.fromLocalFile(str(qml_directory() / 'Dashboard.qml')))
    if view.status() != QQuickView.Null:
        if view.status() == QQuickView.Error:
            for error in view.errors():
                print(error.toString(), file=sys.stderr)
            return 3
    view.setTitle('Harvester Telemetry Dashboard')
    view.setColor('#101418')
    view.show()

    source.start()

    signal.signal(signal.SIGINT, signal.SIG_DFL)
    code = app.exec_()

    source.stop()
    if bridge.status_client is not None:
        bridge.status_client.close()
    annotation_publisher.close()
    return code


def _make_frame_sink(bridge, provider):
    """Route decoded frames into the image provider and bridge."""

    def sink(channel: str, decoded) -> None:
        bridge.on_frame_decoded(channel, decoded)
        if channel.endswith('/rgb'):
            camera = 'cutter' if '/cutter/' in channel else 'docking'
            provider.publish_rgb(camera, decoded)
        elif channel.endswith('/depth'):
            from .image_provider import colorize_depth
            camera = 'cutter' if '/cutter/' in channel else 'docking'
            provider.publish_depth_colored(camera + '_depth', decoded)
    return sink


if __name__ == '__main__':
    sys.exit(main())
