"""Utilities for producing capture-time UTC timestamps from DepthAI frames."""

import shutil
import subprocess
import time
from datetime import timedelta

PLC_NTP_SERVER = "192.168.50.40"
TIME_AUTHORITY = "plc_rtc_ntp"


def depthai_timestamp_to_monotonic_us(timestamp: timedelta) -> int:
    """Convert a DepthAI host-aligned ``timedelta`` to integer microseconds."""
    return int(round(timestamp.total_seconds() * 1_000_000))


def capture_timestamp_us(timestamp: timedelta, now_monotonic_ns=None, now_utc_ns=None) -> int:
    """Map a host-monotonic DepthAI timestamp to Unix-epoch microseconds.

    DepthAI v3 continuously aligns ``ImgFrame.getTimestamp()`` to the host's
    monotonic clock. Sampling the UTC-to-monotonic offset at receipt preserves
    the frame capture time rather than substituting the later publish time.
    """
    if now_monotonic_ns is None:
        now_monotonic_ns = time.monotonic_ns()
    if now_utc_ns is None:
        now_utc_ns = time.time_ns()
    frame_monotonic_us = depthai_timestamp_to_monotonic_us(timestamp)
    utc_offset_us = (now_utc_ns - now_monotonic_ns) // 1_000
    return frame_monotonic_us + utc_offset_us


class ChronyStatus:
    """Best-effort, cached chrony status; capture must never depend on it."""

    def __init__(self, refresh_seconds=30.0, expected_reference_ip=PLC_NTP_SERVER):
        self.refresh_seconds = refresh_seconds
        self.expected_reference_ip = expected_reference_ip
        self._next_refresh = 0.0
        self._quality = "unknown"
        self._offset_us = None

    def get(self):
        now = time.monotonic()
        if now >= self._next_refresh:
            self._next_refresh = now + self.refresh_seconds
            self._refresh()
        return self._quality, self._offset_us

    def _refresh(self):
        if not shutil.which("chronyc"):
            self._quality, self._offset_us = "unknown", None
            return
        try:
            result = subprocess.run(
                ["chronyc", "tracking", "-n"],
                check=True,
                capture_output=True,
                text=True,
                timeout=2,
            )
        except (OSError, subprocess.SubprocessError):
            self._quality, self._offset_us = "unknown", None
            return

        fields = {}
        for line in result.stdout.splitlines():
            if ":" in line:
                key, value = line.split(":", 1)
                fields[key.strip()] = value.strip()
        leap = fields.get("Leap status", "")
        reference = fields.get("Reference ID", "")
        if leap == "Normal" and self.expected_reference_ip in reference:
            self._quality = "synchronized"
        elif leap == "Normal":
            self._quality = "unexpected_source"
        else:
            self._quality = "holdover"
        try:
            # chronyc reports seconds for the current system-time correction.
            self._offset_us = int(round(float(fields["System time"].split()[0]) * 1_000_000))
        except (KeyError, ValueError, IndexError):
            self._offset_us = None
