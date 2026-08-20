"""Shared decoder error type (kept import-cycle free)."""

from __future__ import annotations


class UnsupportedCodecError(NotImplementedError):
    """Raised when this host cannot decode the declared codec."""
