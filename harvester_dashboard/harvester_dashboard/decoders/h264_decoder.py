"""Stub H.264 decoder: hardware decode is unavailable on this host.

The dashboard must not crash when a hardware source publishes
``codec: h264`` packets.  Decoding raises a clear error that the stream
model converts into a visible stream-error entry.  The real Jetson
decoder module drops in on Orin with the same interface.
"""

from __future__ import annotations

from .errors import UnsupportedCodecError


class H264Decoder:

    def decode(self, header, payload: bytes):
        raise UnsupportedCodecError(
            'H.264 hardware decode is unavailable on this host '
            '(codec h264, {} bytes); waiting for the Jetson decoder '
            'module'.format(len(payload)))
