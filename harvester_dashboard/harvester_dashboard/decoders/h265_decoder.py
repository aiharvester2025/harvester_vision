"""Stub H.265 decoder: hardware decode is unavailable on this host."""

from __future__ import annotations

from .errors import UnsupportedCodecError


class H265Decoder:

    def decode(self, header, payload: bytes):
        raise UnsupportedCodecError(
            'H.265 hardware decode is unavailable on this host '
            '(codec h265, {} bytes); waiting for the Jetson decoder '
            'module'.format(len(payload)))
