"""Jetson hardware H.264/H.265 decode via GStreamer ``nvv4l2decoder``.

A stateful decoder session per camera channel.  Each incoming ZMQ packet is
one Annex-B encoded access unit (a keyframe carries SPS/PPS + IDR; other
packets carry P-slices).  We push each access unit into an ``appsrc`` and
receive decoded RGBA frames from an ``appsink`` ``new-sample`` callback,
converting on the GPU with ``nvvidconv``.

The module degrades gracefully: if GStreamer or the NVIDIA decoder element is
unavailable, constructing a decoder raises :class:`UnsupportedCodecError` so
the caller surfaces a stream error instead of crashing the dashboard.

Only the system ``/usr/bin/python3`` (which has ``gi``/``Gst``) can use this
module; the depthai-env python has no GStreamer bindings.
"""

from __future__ import annotations

import threading
from typing import Optional

import numpy as np

from .errors import UnsupportedCodecError

try:
    import gi
    gi.require_version('Gst', '1.0')
    from gi.repository import Gst
    _GST_AVAILABLE = True
except (ImportError, ValueError):  # pragma: no cover - non-system python
    _GST_AVAILABLE = False
    Gst = None


def _ensure_gst():
    if not _GST_AVAILABLE:
        raise UnsupportedCodecError(
            'Jetson hardware decode requires GStreamer bindings '
            '(system /usr/bin/python3 with python3-gi)')
    if not Gst.is_initialized():
        Gst.init(None)


def _nvv4l2_available() -> bool:
    if not _GST_AVAILABLE:
        return False
    _ensure_gst()
    return Gst.ElementFactory.find('nvv4l2decoder') is not None


class JetsonDecoder:
    """Stateful hardware decode session for one camera channel.

    ``decode(header, payload)`` feeds one access unit and returns the newest
    decoded RGB ``ndarray``, or ``None`` if no complete frame was produced yet
    (e.g. the first packets may only carry SPS/PPS).
    """

    def __init__(self, codec: str, width: Optional[int] = None,
                 height: Optional[int] = None):
        if codec not in ('h264', 'h265'):
            raise UnsupportedCodecError(
                'JetsonDecoder only supports h264/h265, got {!r}'.format(codec))
        _ensure_gst()
        if not _nvv4l2_available():
            raise UnsupportedCodecError(
                'nvv4l2decoder element is unavailable on this host; '
                'cannot hardware-decode {}'.format(codec))
        self.codec = codec
        self.width = width
        self.height = height
        self._output_format = 'RGBA'
        self._lock = threading.Lock()
        self._latest_rgb: Optional[np.ndarray] = None
        self._pipeline = self._build_pipeline()

    def _build_pipeline(self):
        caps = 'video/x-h264,stream-format=byte-stream' if self.codec == 'h264' \
            else 'video/x-h265,stream-format=byte-stream'
        if Gst.ElementFactory.find('nvvidconv') is not None:
            convert = 'nvvidconv'
            self._output_format = 'RGBA'
        else:
            convert = 'videoconvert'
            self._output_format = 'RGB'
        description = (
            'appsrc name=src is-live=true do-timestamp=true format=time '
            'caps={} ! {}parse ! nvv4l2decoder ! {} ! '
            'video/x-raw,format={} ! appsink name=sink sync=false '
            'max-buffers=2 drop=true emit-signals=true'.format(
                caps, self.codec[:4], convert, self._output_format)
        )
        pipeline = Gst.parse_launch(description)
        self._appsrc = pipeline.get_by_name('src')
        self._appsink = pipeline.get_by_name('sink')
        # Receive decoded frames via the "new-sample" signal instead of
        # polling, which is the reliable path for a continuous live stream.
        self._appsink.connect('new-sample', self._on_new_sample)
        pipeline.set_state(Gst.State.PLAYING)
        return pipeline

    def _on_new_sample(self, appsink):
        sample = appsink.emit('pull-sample')
        if sample is None:
            return Gst.FlowReturn.OK
        buf = sample.get_buffer()
        caps = sample.get_caps()
        if buf is None or caps is None:
            return Gst.FlowReturn.OK
        s = caps.get_structure(0)
        w = s.get_value('width')
        h = s.get_value('height')
        ok, mapinfo = buf.map(Gst.MapFlags.READ)
        if not ok:
            return Gst.FlowReturn.OK
        try:
            raw = np.frombuffer(mapinfo.data, dtype=np.uint8)
            channels = 4 if self._output_format == 'RGBA' else 3
            frame = raw.reshape((h, w, channels)).copy()
            if channels == 4:
                # nvvidconv honours the requested ``video/x-raw,format=RGBA``:
                # bytes are R,G,B,A in order, so only the alpha byte is dropped.
                # (Verified: solid red comes out [255,0,0,255].)
                frame = frame[:, :, :3]
            # Guard against the NVDEC green concealment frame (see
            # JetsonJpegSession._is_green_concealment for the full explanation):
            # keep the previous good frame rather than flashing green.
            if not self._is_green_concealment(frame):
                self._latest_rgb = frame
        finally:
            buf.unmap(mapinfo)
        return Gst.FlowReturn.OK

    @staticmethod
    def _is_green_concealment(rgb: np.ndarray) -> bool:
        """True when ``rgb`` is the NVDEC green concealment frame.

        The frame's exact value is not stable (observed both as a clean
        [0,147,0] and as noisy near-saturated green); the invariant is a
        near-black red channel with strongly dominant green.
        """
        small = rgb[::16, ::16].astype(np.float32)
        r_mean = small[:, :, 0].mean()
        g_mean = small[:, :, 1].mean()
        b_mean = small[:, :, 2].mean()
        if g_mean < 100.0:
            return False
        if r_mean > 0.25 * g_mean:
            return False
        if b_mean > 0.40 * g_mean:
            return False
        return True

    def decode(self, header, payload: bytes) -> Optional[np.ndarray]:
        """Feed one access unit; return the newest decoded RGB frame or None."""
        with self._lock:
            if self._pipeline is None:
                return None
            if not payload:
                return self._latest_rgb
            if self.width is None and header:
                self.width = int(header.get('width', 0)) or None
                self.height = int(header.get('height', 0)) or None
            buffer = Gst.Buffer.new_wrapped(payload)
            self._appsrc.emit('push-buffer', buffer)
            return self._latest_rgb

    def close(self):
        with self._lock:
            if self._pipeline is not None:
                self._pipeline.set_state(Gst.State.NULL)
                self._pipeline = None

    def __del__(self):
        try:
            self.close()
        except Exception:
            pass


def jetson_decoder_for(codec: str, width=None, height=None) -> 'JetsonDecoder':
    """Factory that raises :class:`UnsupportedCodecError` when unavailable."""
    return JetsonDecoder(codec, width=width, height=height)
