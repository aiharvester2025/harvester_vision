"""Optional non-actuating annotation forwarder.

Disabled by default.  When enabled via ``--annotation-pub`` it binds a
**separate** PUB endpoint (suggested ``tcp://127.0.0.1:5592``) and
publishes ``v1/operator/target_selection`` packets when the operator
creates or clears an annotation.  It never binds or writes the canonical
5590/5600 endpoints and never carries any command payload.
"""

from __future__ import annotations

import time
from typing import Optional

from .protocol_shim import ProtocolError, pack_message


class AnnotationPublisher:
    """Publishes operator annotations on a dedicated, non-canonical PUB."""

    def __init__(self, endpoint: str = '', source_id: str = 'dashboard',
                 context=None):
        self.endpoint = endpoint
        self.source_id = source_id
        self.sequence = 0
        self.socket = None
        self.sent = 0
        self.errors = 0
        self.last_error = ''
        self._owns_context = False
        if endpoint:
            import zmq
            if context is None:
                self._context = zmq.Context.instance()
                self._owns_context = True
            else:
                self._context = context
            self.socket = self._context.socket(zmq.PUB)
            self.socket.setsockopt(zmq.LINGER, 0)
            self.socket.bind(endpoint)

    @property
    def enabled(self) -> bool:
        return self.socket is not None

    def publish_annotation(self, annotation, action: str = 'created') -> bool:
        """Forward one annotation event; returns True when sent."""
        if not self.enabled or annotation is None:
            return False
        self.sequence += 1
        payload = {
            'action': action,
            'camera': annotation.camera,
            'pixel': {'u': annotation.pixel[0], 'v': annotation.pixel[1]},
            'depth_m': annotation.depth_m,
            'point_camera_xyz': (list(annotation.point_camera)
                                 if annotation.point_camera else None),
            'frame_id': annotation.frame_id,
            # Phase 1 is camera-relative only; world-fixed stays null until
            # v1/pose/* exists and target.world_fixed is true.
            'tree_base_xyz': None,
            'world_fixed': False,
        }
        header = {
            'schema_version': 1,
            'source_mode': 'simulation',
            'source_id': self.source_id,
            'sequence': self.sequence,
            'frame_id': annotation.frame_id or 'camera_relative',
            'acquisition_timestamp_ns': time.time_ns(),
            'clock_domain': 'utc_host',
            'gateway_monotonic_ns': time.monotonic_ns(),
            'calibration_id': 'none',
            'codec': 'json',
            'capabilities': {'target.world_fixed': False},
        }
        try:
            import json
            frames = pack_message(
                'v1/operator/target_selection', header,
                json.dumps(payload, separators=(',', ':')).encode('utf-8'))
        except ProtocolError as error:
            self.errors += 1
            self.last_error = str(error)
            return False
        try:
            self.socket.send_multipart(frames, flags=1)  # zmq.NOBLOCK == 1
            self.sent += 1
            return True
        except Exception as error:  # pragma: no cover - transport failure
            self.errors += 1
            self.last_error = str(error)
            return False

    def close(self) -> None:
        if self.socket is not None:
            try:
                self.socket.close(0)
            except Exception:
                pass
            self.socket = None


__all__ = ['AnnotationPublisher']
