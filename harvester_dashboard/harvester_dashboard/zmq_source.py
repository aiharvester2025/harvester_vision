"""Canonical telemetry subscriber with a bounded drain-and-decode worker.

One SUB socket subscribes to every canonical channel prefix, drains with
``recv_multipart(NOBLOCK)`` until ``zmq.Again``, decodes payloads in the
worker thread, and posts results to the UI thread through queued Qt
signals.  The socket ``RCVHWM`` bounds in-flight memory; drops are
counted, never accumulated.

Render-only guarantee: this module creates exactly one SUB socket and
never sends anything on it.  The pure-Python :class:`SocketDrainer` core
is shared with the inproc tests.
"""

from __future__ import annotations

import json
from typing import Callable, List, Optional

import zmq

from .protocol_shim import CANONICAL_CHANNELS
from . import decoders
from .decoders.errors import UnsupportedCodecError


class SocketDrainer:
    """Qt-free SUB socket owner; drives callbacks on the caller's thread.

    Used directly by tests against an inproc PUB socket and by the Qt
    worker below.  It only ever calls ``setsockopt(SUBSCRIBE, ...)`` and
    ``recv``: no send call exists on this socket.
    """

    def __init__(self, endpoint: str, hwm: int = 8, context=None,
                 decode_rgb: bool = True):
        self.endpoint = endpoint
        self.decode_rgb_enabled = decode_rgb
        self.context = context or zmq.Context.instance()
        self.socket = self.context.socket(zmq.SUB)
        self.socket.setsockopt(zmq.LINGER, 0)
        self.socket.setsockopt(zmq.RCVHWM, max(1, int(hwm)))
        self.socket.connect(endpoint)
        for channel in sorted(CANONICAL_CHANNELS):
            self.socket.setsockopt(zmq.SUBSCRIBE, channel.encode('utf-8'))
        self.total_received = 0
        self.total_dropped = 0
        self.total_decode_errors = 0
        # Callbacks (channel, header, payload, parsed_or_None)
        self.on_packet: Optional[Callable[[str, dict, bytes, object], None]] = None

    def close(self) -> None:
        try:
            self.socket.close(0)
        except Exception:
            pass

    def drain_once(self, max_packets: int = 512) -> int:
        """Drain every immediately available packet; return count handled."""
        handled = 0
        while handled < max_packets:
            try:
                frames = self.socket.recv_multipart(flags=zmq.NOBLOCK)
            except zmq.Again:
                break
            except zmq.ZMQError:
                break
            self._handle(frames)
            handled += 1
        return handled

    def _handle(self, frames: List[bytes]) -> None:
        from .protocol_shim import ProtocolError, unpack_message
        try:
            channel, header, payload = unpack_message(frames)
        except (ProtocolError, IndexError):
            self.total_decode_errors += 1
            raw = ''
            try:
                raw = bytes(frames[0]).decode('utf-8', 'replace')
            except Exception:
                pass
            if self.on_packet is not None:
                self.on_packet(raw, {}, b'', None)
            return
        parsed = None
        if header.get('codec') == 'json':
            try:
                parsed = json.loads(payload.decode('utf-8'))
            except (UnicodeDecodeError, ValueError):
                parsed = None
        self.total_received += 1
        if self.on_packet is not None:
            self.on_packet(channel, header, payload, parsed)

    # Stateful decoders (H.264/H.265) are cached per (codec, frame_id) so a
    # decode session persists across the access units of one stream.  JPEG,
    # depth, and LiDAR decoders are stateless and created fresh each call.
    _stateful_decoders: dict = {}

    @staticmethod
    def _stateful_decoder(codec: str, frame_id: str):
        key = (codec, frame_id)
        decoder = SocketDrainer._stateful_decoders.get(key)
        if decoder is None:
            decoder = decoders.decoder_for_codec(codec)
            SocketDrainer._stateful_decoders[key] = decoder
        return decoder

    @staticmethod
    def decode_frame(channel: str, header: dict, payload: bytes):
        """Decode image/depth/lidar payloads; ``None`` for JSON channels.

        Raises :class:`UnsupportedCodecError` for h264/h265 when the Jetson
        decoder is unavailable so callers surface a stream error instead of
        crashing.
        """
        if channel.endswith('/rgb'):
            codec = header.get('codec')
            if codec in ('jpeg', 'h264', 'h265'):
                # Stateful hardware decode (jpeg: JetsonJpegDecoder cached
                # per frame_id; h264/h265: JetsonDecoder).  A fresh decoder
                # per call would lose the async-decode-in-flight frame.
                decoder = SocketDrainer._stateful_decoder(
                    codec, header.get('frame_id', ''))
                return decoder.decode(header, payload)
            raise UnsupportedCodecError(
                'unknown rgb codec {!r}'.format(codec))
        if channel.endswith('/depth'):
            return decoders.decode_depth(header, payload)
        if channel == 'v1/lidar/raw':
            return decoders.decode_lidar(header, payload)
        return None


try:
    from PySide2.QtCore import QObject, QThread, QTimer, Signal, Slot
    _QT_AVAILABLE = True
except ImportError:  # pragma: no cover - pure-python test path
    _QT_AVAILABLE = False
    QObject = object
    QThread = None
    QTimer = None

    def Signal(*args, **kwargs):  # type: ignore[misc]
        def _decorated(*_a, **_k):
            raise RuntimeError('PySide2 unavailable')
        return _decorated

    def Slot(*args, **kwargs):  # type: ignore[misc]
        def _decorated(fn):
            return fn
        return _decorated


if _QT_AVAILABLE:

    class TelemetryWorker(QObject):
        """Lives on a QThread; owns the SocketDrainer and decodes there.

        Cross-thread signalling is deliberately batched into a single
        ``Signal(object)`` per drain tick carrying a list of plain Python
        tuples.  Typed signal arguments (dict/QByteArray) would force a
        QVariant conversion per packet on this PySide2 build, which is
        both slow and fragile for nested MessagePack-derived headers.
        """

        batch_ready = Signal(object)          # [(channel, header, payload, parsed_or_frame), ...]
        decode_failed = Signal(str, str)
        counters_updated = Signal(int, int, int)

        _POLL_MS = 16

        def __init__(self, endpoint: str, hwm: int = 8, parent=None):
            super().__init__(parent)
            self._endpoint = endpoint
            self._hwm = hwm
            self.drainer: Optional[SocketDrainer] = None
            self._timer: Optional[QTimer] = None
            self._pending: list = []
            self._pending_errors: list = []
            # Which camera ('cutter'|'docking') is currently displayed.  Only
            # this camera's RGB stream is hardware-decoded; the inactive
            # camera's H.265/H.264 frames are dropped before reaching the
            # stateful NVDEC decoder, halving the host-side decode CPU (the
            # nvv4l2decoder driver threads are ~35% each on the Orin).  Set
            # from the UI thread via the queued ``set_active_camera`` slot.
            self._active_camera = 'cutter'

        def set_active_camera(self, camera: str) -> None:
            """Update the active camera (queued from the UI thread).

            Closes the newly-inactive camera's stateful NVDEC decoder so its
            driver thread fully idles (not just starved of input), and lets a
            fresh decoder be created when that camera becomes active again
            (the OAK re-sends SPS/PPS keyframes on re-subscribe, so the decode
            renegotiates cleanly).
            """
            if camera not in ('cutter', 'docking') or camera == self._active_camera:
                return
            previous = self._active_camera
            self._active_camera = camera
            # Drop the stateful decoder(s) of the camera we just left.
            for (codec, frame_id) in list(SocketDrainer._stateful_decoders.keys()):
                if self._frame_id_is_camera(frame_id, previous):
                    dec = SocketDrainer._stateful_decoders.pop((codec, frame_id), None)
                    if dec is not None:
                        try:
                            dec.close()
                        except Exception:
                            pass

        @staticmethod
        def _frame_id_is_camera(frame_id: str, camera: str) -> bool:
            """True when a frame_id belongs to the named camera."""
            if camera == 'cutter':
                return 'cutter' in frame_id
            if camera == 'docking':
                return 'docking' in frame_id
            return False

        def start(self):
            self.drainer = SocketDrainer(self._endpoint, hwm=self._hwm)
            self.drainer.on_packet = self._collect
            self._timer = QTimer(self)
            self._timer.timeout.connect(self._tick)
            self._timer.start(self._POLL_MS)

        def stop(self):
            if self._timer is not None:
                self._timer.stop()
                self._timer = None
            if self.drainer is not None:
                self.drainer.close()
                self.drainer = None

        def _tick(self):
            self.drainer.drain_once()
            if self._pending_errors:
                errors = self._pending_errors
                self._pending_errors = []
                for channel, message in errors:
                    self.decode_failed.emit(channel, message)
            if self._pending:
                batch = self._pending
                self._pending = []
                self.batch_ready.emit(batch)
            self.counters_updated.emit(
                self.drainer.total_received, self.drainer.total_dropped,
                self.drainer.total_decode_errors)

        def _collect(self, channel, header, payload, parsed):
            if header == {}:
                # Contract-violating packet: counted by the drainer; surface
                # as a stream error with whatever channel prefix arrived.
                self._pending_errors.append(
                    (channel or 'unknown', 'packet failed contract validation'))
                return
            decoded = parsed
            if channel.endswith('/rgb') or channel.endswith('/depth'):
                # Active-camera-only decode: the inactive camera's H.265/H.264
                # RGB frames AND its depth maps are dropped here (before
                # reaching the stateful NVDEC decoder or the float32 depth
                # conversion).  Only one nvv4l2decoder driver thread runs at a
                # time and the inactive camera's ~2 MB/frame depth decode is
                # skipped, roughly halving the dashboard's decode CPU on the
                # Orin without changing resolution or codec.  The packet is
                # still ingested into the model (freshness counters), only the
                # decode is skipped.
                if not self._channel_is_active(channel):
                    self._pending.append((channel, header, payload, None))
                    return
            elif channel.endswith('/imu'):
                # Active-camera-only IMU: drop the inactive camera's parsed IMU
                # JSON so its attitude bookkeeping and any downstream work are
                # skipped.  The packet is still ingested for freshness counters.
                # (The parsed value is nulled here; ingest_packet re-parses the
                # raw payload only for the model's own JSON storage, which is
                # cheap and not wired to the point-cloud path.)
                if not self._channel_is_active(channel):
                    self._pending.append((channel, header, payload, None))
                    return
            if not channel.startswith('v1/operator/'):
                try:
                    frame = SocketDrainer.decode_frame(channel, header, payload)
                except UnsupportedCodecError as error:
                    self._pending_errors.append((channel, str(error)))
                except ValueError as error:
                    self._pending_errors.append((channel, 'decode: {}'.format(error)))
                else:
                    if frame is not None:
                        decoded = frame
            self._pending.append((channel, header, payload, decoded))

        def _channel_is_active(self, channel: str) -> bool:
            """Instance check: is this /rgb channel the active camera's?"""
            if '/cutter/' in channel:
                return self._active_camera == 'cutter'
            if '/docking/' in channel:
                return self._active_camera == 'docking'
            return True

    class TelemetrySource(QObject):
        """UI-thread facade: worker QThread + queued-signal plumbing."""

        # Forwarded to the worker thread (queued) so the worker knows which
        # camera to hardware-decode.
        active_camera_changed = Signal(str)

        def __init__(self, config, model, parent=None):
            super().__init__(parent)
            self.config = config
            self.model = model
            self.worker = TelemetryWorker(config.pub_endpoint, hwm=config.socket_hwm)
            self.thread = QThread(self)
            self.worker.moveToThread(self.thread)
            self.thread.started.connect(self.worker.start)
            self.thread.finished.connect(self.worker.stop)
            self.worker.batch_ready.connect(self._on_batch)
            self.worker.decode_failed.connect(self._on_decode_failed)
            self.worker.counters_updated.connect(self._on_counters)
            # Queued: set_active_camera runs on the worker thread.
            self.active_camera_changed.connect(self.worker.set_active_camera)
            self.on_frame = None   # UI hook (channel, decoded) for provider/bridge

        def start(self):
            self.thread.start()

        def stop(self):
            self.thread.quit()
            self.thread.wait(2000)

        @Slot(str)
        def set_active_camera(self, camera: str) -> None:
            """Set the active camera; the worker picks it up on its thread."""
            self.active_camera_changed.emit(camera)

        # -- UI-thread slots -------------------------------------------------
        def _on_batch(self, batch):
            ingest = self.model.ingest_packet
            for channel, header, payload, decoded in batch:
                ingest(channel, header, payload)
                if self.on_frame is not None and decoded is not None \
                        and not isinstance(decoded, (dict, list)):
                    self.on_frame(channel, decoded)

        def _on_decode_failed(self, channel, error):
            if not channel:
                return
            self.model.ensure_channel(channel).record_decode_error(error)

        def _on_counters(self, received, dropped, decode_errors):
            self.model.source_received = received
            self.model.source_dropped = dropped
            self.model.source_decode_errors = decode_errors


__all__ = ['SocketDrainer', 'TelemetrySource', 'TelemetryWorker', '_QT_AVAILABLE']
