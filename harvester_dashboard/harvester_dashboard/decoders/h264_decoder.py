"""H.264 decoder: Jetson hardware decode via ``nvv4l2decoder``.

The decode session is stateful (one per camera frame id), so this decoder is
instantiated once per stream and reused.  On hosts without the Jetson decoder
(or without GStreamer), it raises :class:`UnsupportedCodecError` so the stream
model surfaces a visible error instead of crashing.
"""

from __future__ import annotations

from .errors import UnsupportedCodecError
from .jetson_decode import JetsonDecoder


class H264Decoder:

    def __init__(self):
        self._session = None
        self._frame_id = None

    def decode(self, header, payload: bytes):
        frame_id = (header or {}).get('frame_id', '')
        if self._session is None or frame_id != self._frame_id:
            if self._session is not None:
                self._session.close()
            self._frame_id = frame_id
            try:
                self._session = JetsonDecoder(
                    'h264',
                    width=int((header or {}).get('width', 0)) or None,
                    height=int((header or {}).get('height', 0)) or None)
            except UnsupportedCodecError:
                raise
            except Exception as error:  # GStreamer init failure etc.
                raise UnsupportedCodecError(
                    'H.264 hardware decode unavailable: {}'.format(error))
        try:
            return self._session.decode(header, payload)
        except Exception as error:
            raise UnsupportedCodecError(
                'H.264 decode failed ({} bytes): {}'.format(len(payload), error))

    def close(self):
        if self._session is not None:
            self._session.close()
            self._session = None
