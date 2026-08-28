"""Per-stream state model for canonical telemetry (Qt-free core).

The :class:`TelemetryModel` holds one :class:`StreamState` per channel,
tracking the last header/payload, local receipt monotonic time, sequence
gaps, and drop counts.  Freshness is computed from **local receipt
monotonic time** so live and replay sessions behave identically; header
``acquisition_timestamp_ns`` is display-only metadata tied to its
``clock_domain`` and is never compared across domains or hosts.
"""

from __future__ import annotations

import json
import time
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional

from ..protocol_shim import ProtocolError, unpack_message


JSON_CHANNELS = frozenset({
    'v1/range/docking',
    'v1/range/cutter',
    'v1/boom/state',
    'v1/docking/trunk_estimate',
    'v1/calibration/status',
    'v1/system/status',
    'v1/camera/cutter/camera_info',
    'v1/camera/docking/camera_info',
    'v1/camera/cutter/imu',
    'v1/camera/docking/imu',
    'v1/operator/target_selection',
})


@dataclass
class StreamState:
    channel: str
    last_header: Optional[Dict[str, Any]] = None
    last_payload: Optional[bytes] = None
    last_json: Optional[Any] = None
    last_recv_monotonic_s: Optional[float] = None
    expected_sequence: Optional[int] = None
    sequence_gaps: int = 0
    drops: int = 0
    decode_errors: int = 0
    last_error: str = ''
    ever_seen: bool = False

    def record_packet(self, header: Dict[str, Any], payload: bytes,
                      recv_monotonic_s: float) -> None:
        sequence = header.get('sequence')
        if (self.expected_sequence is not None and isinstance(sequence, int)
                and sequence > self.expected_sequence):
            self.sequence_gaps += sequence - self.expected_sequence
        if isinstance(sequence, int):
            self.expected_sequence = sequence + 1
        self.last_header = header
        self.last_payload = payload
        self.last_recv_monotonic_s = recv_monotonic_s
        self.ever_seen = True

    def record_drop(self, count: int = 1) -> None:
        self.drops += int(count)

    def record_decode_error(self, message: str) -> None:
        self.decode_errors += 1
        self.last_error = str(message)

    def age_s(self, now_monotonic_s: float) -> Optional[float]:
        if self.last_recv_monotonic_s is None:
            return None
        return max(0.0, now_monotonic_s - self.last_recv_monotonic_s)

    def is_stale(self, now_monotonic_s: float, stale_after_s: float) -> bool:
        age = self.age_s(now_monotonic_s)
        return age is None or age > stale_after_s


class TelemetryModel:
    """Aggregates stream states and source badges; pure Python."""

    def __init__(self, channels=None, clock=time.monotonic):
        self._clock = clock
        self._channels = list(channels) if channels is not None else []
        self._states: Dict[str, StreamState] = {}
        self.last_system_status: Optional[Dict[str, Any]] = None
        self.on_packet = None      # optional callback(channel, header, payload)
        self.on_json = None        # optional callback(channel, header, decoded)

    # ------------------------------------------------------------------ state
    def ensure_channel(self, channel: str) -> StreamState:
        state = self._states.get(channel)
        if state is None:
            state = StreamState(channel=channel)
            self._states[channel] = state
        return state

    def state(self, channel: str) -> StreamState:
        state = self._states.get(channel)
        if state is None:
            state = StreamState(channel=channel)
        return state

    @property
    def channels(self) -> List[str]:
        return sorted(self._states)

    def states(self) -> List[StreamState]:
        return [self._states[channel] for channel in sorted(self._states)]

    # ----------------------------------------------------------------- ingest
    def ingest_packet(self, channel: str, header: Dict[str, Any], payload: bytes,
                      recv_monotonic_s: Optional[float] = None):
        """Store one already-validated canonical packet."""
        recv_s = self._clock() if recv_monotonic_s is None else recv_monotonic_s
        state = self.ensure_channel(channel)
        state.record_packet(header, payload, recv_s)
        if channel in JSON_CHANNELS:
            try:
                decoded = json.loads(payload.decode('utf-8'))
                state.last_json = decoded
            except (UnicodeDecodeError, json.JSONDecodeError) as error:
                state.record_decode_error('json: {}'.format(error))
                state.last_json = None
            else:
                if channel == 'v1/system/status' and isinstance(decoded, dict):
                    self.last_system_status = decoded
                if self.on_json is not None:
                    self.on_json(channel, header, decoded)
        if self.on_packet is not None:
            self.on_packet(channel, header, payload)
        return state

    def ingest_frames(self, frames, recv_monotonic_s: Optional[float] = None):
        """Validate and store one canonical three-frame packet.

        Returns ``(channel, header, payload)`` on success or ``None``
        when the packet violates the contract (counted as a decode error
        on the raw channel when it can be identified).
        """
        try:
            channel, header, payload = unpack_message(frames)
        except ProtocolError as error:
            raw_channel = ''
            try:
                raw_channel = bytes(frames[0]).decode('utf-8', 'replace')
            except Exception:
                pass
            if raw_channel:
                self.ensure_channel(raw_channel).record_decode_error(
                    'protocol: {}'.format(error))
            return None
        self.ingest_packet(channel, header, payload, recv_monotonic_s)
        return channel, header, payload

    # ---------------------------------------------------------------- reports
    def source_mode(self) -> str:
        """Aggregate ``SIMULATION`` / ``HARDWARE`` / ``MIXED`` / ``NO DATA``."""
        modes = set()
        for state in self._states.values():
            if state.last_header is not None:
                modes.add(state.last_header.get('source_mode'))
        modes.discard(None)
        if not modes:
            return 'NO DATA'
        if len(modes) == 1:
            mode = modes.pop()
            return 'SIMULATION' if mode == 'simulation' else 'HARDWARE'
        return 'MIXED'

    def source_ids(self) -> str:
        ids = sorted({
            state.last_header.get('source_id')
            for state in self._states.values()
            if state.last_header is not None and state.last_header.get('source_id')
        })
        return ','.join(ids)

    def is_mixed(self) -> bool:
        return self.source_mode() == 'MIXED'

    def latest_capabilities(self) -> Dict[str, bool]:
        for channel in ('v1/system/status', 'v1/calibration/status'):
            state = self._states.get(channel)
            if state is not None and state.last_header is not None:
                caps = state.last_header.get('capabilities')
                if isinstance(caps, dict) and caps:
                    return dict(caps)
        for channel in sorted(self._states):
            state = self._states[channel]
            if state.last_header is not None:
                caps = state.last_header.get('capabilities')
                if isinstance(caps, dict) and caps:
                    return dict(caps)
        return {}

    def summary_rows(self, stale_after_s: float,
                     now_monotonic_s: Optional[float] = None) -> List[Dict[str, Any]]:
        """Build one display row per known stream for the errors panel."""
        now = self._clock() if now_monotonic_s is None else now_monotonic_s
        rows = []
        for state in self.states():
            age = state.age_s(now)
            rows.append({
                'channel': state.channel,
                'age_s': age,
                'stale': state.is_stale(now, stale_after_s),
                'ever_seen': state.ever_seen,
                'sequence_gaps': state.sequence_gaps,
                'drops': state.drops,
                'decode_errors': state.decode_errors,
                'last_error': state.last_error,
                'source_mode': (state.last_header or {}).get('source_mode', ''),
            })
        return rows

    def snapshot_ranges(self):
        """Return decoded docking-range list and cutter-range record."""
        docking = self._json_of('v1/range/docking')
        cutter = self._json_of('v1/range/cutter')
        docking_records = list(docking) if isinstance(docking, list) else []
        cutter_record = cutter if isinstance(cutter, dict) else None
        return docking_records, cutter_record

    def snapshot_trunk(self):
        return self._json_of('v1/docking/trunk_estimate')

    def snapshot_boom(self):
        """Return the decoded ``v1/boom/state`` payload (or None)."""
        return self._json_of('v1/boom/state')

    def snapshot_calibration(self):
        payload = self._json_of('v1/calibration/status')
        header = (self._states.get('v1/calibration/status') or StreamState('')).last_header
        valid = header.get('transform_valid') if header else None
        return payload, valid

    def snapshot_camera_info(self, camera: str):
        """Camera info JSON for 'cutter' or 'docking'."""
        return self._json_of('v1/camera/{}/camera_info'.format(camera))

    def _json_of(self, channel: str):
        state = self._states.get(channel)
        if state is None:
            return None
        return state.last_json
