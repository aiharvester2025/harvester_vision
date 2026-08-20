"""Read-only REQ client for the canonical status REP endpoint.

Sends an empty JSON request body and decodes the response containing
``schema_version``, ``active_profile``, ``calibration_revision``,
``streams``, ``dropped_packets``, ``recording``, and ``capabilities``.
The REQ socket only ever receives; it cannot enable cameras or command
hardware (the endpoint itself is read-only).
"""

from __future__ import annotations

import json
from typing import Dict, Optional

import zmq


class StatusClient:

    def __init__(self, endpoint: str, timeout_ms: int = 1500, context=None):
        self.endpoint = endpoint
        self.timeout_ms = max(100, int(timeout_ms))
        self._context = context or zmq.Context.instance()
        self._socket = None

    def _ensure_socket(self):
        if self._socket is None:
            self._socket = self._context.socket(zmq.REQ)
            self._socket.setsockopt(zmq.LINGER, 0)
            self._socket.setsockopt(zmq.RCVTIMEO, self.timeout_ms)
            self._socket.setsockopt(zmq.SNDTIMEO, self.timeout_ms)
            self._socket.connect(self.endpoint)
        return self._socket

    def query(self) -> Optional[Dict]:
        """Perform one REQ/REP round trip; ``None`` on timeout/error."""
        socket = self._ensure_socket()
        try:
            socket.send(b'{}')
            raw = socket.recv()
        except zmq.ZMQError:
            self._reset()
            return None
        try:
            response = json.loads(raw.decode('utf-8'))
        except (UnicodeDecodeError, ValueError):
            return None
        return response if isinstance(response, dict) else None

    def _reset(self):
        # A timed-out REQ socket enters a state where it must be rebuilt
        # before the next request (ZMQ REQ strict request/reply state).
        if self._socket is not None:
            try:
                self._socket.close(0)
            except Exception:
                pass
            self._socket = None

    def close(self):
        self._reset()

    @staticmethod
    def source_mode_of(response: Optional[Dict]) -> str:
        """Map a REP response to the maintenance-availability mode.

        Only an explicit hardware mode (via ``source_mode``, the
        ``active_profile`` name, or per-stream headers) permits
        maintenance stream controls.  Everything else — simulation,
        unknown, or unreachable — keeps them hidden.
        """
        if not isinstance(response, dict):
            return 'unknown'
        mode = response.get('source_mode')
        if mode in ('hardware', 'simulation'):
            return mode
        profile = str(response.get('active_profile', '')).lower()
        if profile == 'hardware' or 'hardware' in profile:
            return 'hardware'
        if profile == 'simulation' or 'simulation' in profile:
            return 'simulation'
        streams = response.get('streams')
        if isinstance(streams, dict) and streams:
            modes = {
                stream.get('source_mode')
                for stream in streams.values()
                if isinstance(stream, dict)
            }
            modes.discard(None)
            if modes == {'hardware'}:
                return 'hardware'
            if modes == {'simulation'}:
                return 'simulation'
        return 'unknown'
