"""Jetson hardware JPEG decode via GStreamer ``nvjpegdec``.

The OAK camera encodes MJPEG when ``--codec jpeg`` is used; ``nvjpegdec`` on
Jetson decodes it on the dedicated JPEG engine with zero CPU usage (matches the
goal of avoiding CPU burden for LiDAR point-cloud processing).

Each ZMQ packet is one self-contained JPEG image (no Annex-B framing), so the
session is stateless per-call — unlike the stateful H.264/H.265 sessions in
``JetsonDecoder``.  We still keep a small per-frame-id cache so the GStreamer
pipeline is reused across frames of the same camera (creating + tearing down a
GStreamer pipeline per JPEG frame would add noticeable latency).
"""

from __future__ import annotations

import threading
from typing import Dict, Optional

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


def _nvjpegdec_available() -> bool:
    if not _GST_AVAILABLE:
        return False
    _ensure_gst()
    return Gst.ElementFactory.find('nvjpegdec') is not None


class JetsonJpegSession:
    """Per-camera JPEG hardware decode session.

    Stateless across calls (each JPEG is self-contained), but reuses one
    GStreamer pipeline per camera frame id to avoid per-frame setup cost.
    """

    def __init__(self, width: Optional[int] = None, height: Optional[int] = None):
        _ensure_gst()
        if not _nvjpegdec_available():
            raise UnsupportedCodecError(
                'nvjpegdec element is unavailable on this host; '
                'cannot hardware-decode JPEG')
        self.width = width
        self.height = height
        self._lock = threading.Lock()
        self._latest_rgb: Optional[np.ndarray] = None
        self._output_format = 'RGB'
        self._pipeline = self._build_pipeline()

    def _build_pipeline(self):
        # JPEG goes: appsrc (raw JPEG bytes) -> jpegparse -> nvjpegdec (hw) ->
        # nvvidconv (GPU) -> RGBA -> appsink.  ``nvjpegdec`` outputs frames in
        # NVIDIA NVMM (GPU) memory, which the CPU ``videoconvert`` element
        # mangles into a dark image.  The H.264/H.265 path already uses the
        # GPU ``nvvidconv`` converter (RGBA output) and produces correct
        # brightness, so mirror that here.
        if Gst.ElementFactory.find('nvvidconv') is not None:
            convert = 'nvvidconv'
            self._output_format = 'RGBA'
        else:
            convert = 'videoconvert'
            self._output_format = 'RGB'
        # ``mjpegdecode=true`` selects the continuous-MJPEG stream path
        # (NvMM block type 277).  The default still-image path (type 256)
        # freezes on the first frame and repeats it forever, which looked
        # like a dark/stale picture on the dashboard.  This property can
        # only be set while the element is in NULL or READY state, so it is
        # baked into the launch description here.
        # ``emit-signals=false``: we poll ``try-pull-sample`` from the caller
        # thread instead of relying on the ``new-sample`` signal.  The signal
        # requires a running GLib main loop, which the dashboard's Qt worker
        # thread does not own — iterating it there only burns a full CPU core
        # in a busy-wait.  Polling the appsink is synchronous, main-loop-free,
        # and blocks efficiently inside GStreamer.
        description = (
            'appsrc name=src is-live=true do-timestamp=true format=time '
            'caps=image/jpeg ! jpegparse ! nvjpegdec mjpegdecode=true ! {} ! '
            'video/x-raw,format={} ! appsink name=sink sync=false '
            'max-buffers=2 drop=true emit-signals=false'.format(
                convert, self._output_format))
        pipeline = Gst.parse_launch(description)
        self._appsrc = pipeline.get_by_name('src')
        self._appsink = pipeline.get_by_name('sink')
        pipeline.set_state(Gst.State.PLAYING)
        return pipeline

    def _sample_to_rgb(self, sample):
        """Convert an appsink sample to an RGB ndarray, or None if unusable.

        Returns a freshly-allocated, contiguous HxWx3 array that owns its own
        memory (``ascontiguousarray`` on the RGBA->RGB view always copies), so
        the caller may retain it after the GStreamer buffer is unmapped.
        """
        if sample is None:
            return None
        buf = sample.get_buffer()
        caps = sample.get_caps()
        if buf is None or caps is None:
            return None
        s = caps.get_structure(0)
        w = s.get_value('width')
        h = s.get_value('height')
        ok, mapinfo = buf.map(Gst.MapFlags.READ)
        if not ok:
            return None
        try:
            raw = np.frombuffer(mapinfo.data, dtype=np.uint8)
            channels = 4 if self._output_format == 'RGBA' else 3
            frame = raw.reshape((h, w, channels))
            if channels == 4:
                # nvvidconv honours the requested ``video/x-raw,format=RGBA``:
                # the bytes are genuinely R,G,B,A.  Just drop the alpha byte.
                # (Verified: a solid-red JPEG comes out [255,0,0,255].)
                frame = frame[:, :, :3]
            return np.ascontiguousarray(frame)
        finally:
            buf.unmap(mapinfo)

    def _accept_sample(self, sample):
        """Store the sample if it is not the NVDEC green concealment frame."""
        rgb = self._sample_to_rgb(sample)
        if rgb is None:
            return
        # NVDEC MJPEG concealment bug: ``nvjpegdec mjpegdecode=true``
        # periodically emits a uniform pure-green frame (R=0, G=147, B=0)
        # instead of the decoded image — roughly one frame in four.  The input
        # JPEG itself is fine (PIL decodes it correctly).  Detect this
        # concealment frame by its signature (green-dominant, red/blue ~zero,
        # near-zero variance) and keep the previous good frame instead.
        #
        # ``rgb`` is already a fresh contiguous copy (see ``_sample_to_rgb``),
        # so we store it directly — no second ``.copy()`` — to avoid doubling
        # the per-frame allocation churn that fragments the process heap.
        if not self._is_green_concealment(rgb):
            self._latest_rgb = rgb

    @staticmethod
    def _is_green_concealment(rgb: np.ndarray) -> bool:
        """True when ``rgb`` looks like the NVDEC green concealment frame.

        The NVDEC MJPEG bug emits a green frame whose exact value is not
        stable — observed both as a clean uniform [0,147,0] and as a noisy
        near-saturated green (R~0, G~251, B~14 with small variance).  The
        invariant in every observed case is that the red channel is
        essentially black while green is strongly dominant; a real camera
        scene (even a green object) always has a non-trivial red response.
        """
        # Downsample for a cheap statistical check.
        small = rgb[::16, ::16].astype(np.float32)
        r_mean = small[:, :, 0].mean()
        g_mean = small[:, :, 1].mean()
        b_mean = small[:, :, 2].mean()
        # Red channel essentially black, green strongly dominant.  Use a wide
        # margin: a real scene's red mean is comparable to its green mean.
        if g_mean < 100.0:
            return False
        if r_mean > 0.25 * g_mean:
            return False
        if b_mean > 0.40 * g_mean:
            return False
        return True

    def decode(self, header, payload: bytes) -> Optional[np.ndarray]:
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
            # Poll the appsink directly for the freshly-decoded frame.  This
            # blocks (efficiently) inside GStreamer until a frame is ready or
            # the timeout elapses — no GLib main-loop iteration, no busy-wait,
            # so the Qt worker thread does not spin a CPU core.
            #
            # Hardware JPEG decode here takes ~65-125 ms end-to-end (NVMM
            # conversion + GPU nvvidconv + memory copy); the timeout only
            # bounds the worst case (pipeline hiccup / dropped frame).
            sample = self._appsink.emit('try-pull-sample', 300 * 1000 * 1000)
            if sample is not None:
                self._accept_sample(sample)
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


class JetsonJpegDecoder:
    """Stateful decoder that caches one ``JetsonJpegSession`` per frame id.

    Mirrors the ``H264Decoder`` / ``H265Decoder`` interface so the zmq_source
    dispatcher can call it identically.
    """

    def __init__(self):
        self._sessions: Dict[str, JetsonJpegSession] = {}

    def decode(self, header, payload: bytes):
        frame_id = (header or {}).get('frame_id', '')
        session = self._sessions.get(frame_id)
        if session is None:
            session = JetsonJpegSession(
                width=int((header or {}).get('width', 0)) or None,
                height=int((header or {}).get('height', 0)) or None)
            self._sessions[frame_id] = session
        try:
            return session.decode(header, payload)
        except Exception as error:
            raise UnsupportedCodecError(
                'JPEG hardware decode failed ({} bytes): {}'.format(
                    len(payload), error))

    def close(self):
        for session in self._sessions.values():
            session.close()
        self._sessions.clear()