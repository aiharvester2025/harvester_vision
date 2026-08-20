"""Exact canonical-packet recording and replay helpers.

Recordings retain the original three ZeroMQ frames as MessagePack binary
values.  They are ROS-independent so the future Orin adapter and dashboard
can use the same audit fixtures.
"""

from __future__ import annotations

import os
import tempfile
import time
from pathlib import Path
from typing import Dict, Iterator, Sequence, Tuple

import msgpack

from harvester_telemetry_contract import ProtocolError, unpack_message


RECORD_FORMAT_VERSION = 1


def _channel_directory(root: Path, channel: str) -> Path:
    return root / channel.replace('/', '_')


class PacketRecorder:
    """Persist canonical packets without modifying their wire representation."""

    def __init__(self, directory: str):
        self.root = Path(directory).expanduser() if directory else None
        self.recorded_packets = 0
        self.write_errors = 0
        self.last_error = ''
        if self.root is not None:
            self.root.mkdir(parents=True, exist_ok=True)

    @property
    def enabled(self) -> bool:
        return self.root is not None

    def write(self, frames: Sequence[bytes]) -> None:
        """Atomically write one complete packet, or raise on a storage error."""
        if not self.enabled:
            return
        channel, header, _ = unpack_message(frames)
        recorded_monotonic_ns = time.monotonic_ns()
        target_dir = _channel_directory(self.root, channel)
        target_dir.mkdir(parents=True, exist_ok=True)
        target = target_dir / '{:020d}_{:020d}.msgpack'.format(
            int(header['sequence']), recorded_monotonic_ns)
        record = {
            'record_format_version': RECORD_FORMAT_VERSION,
            'recorded_monotonic_ns': recorded_monotonic_ns,
            'frames': [bytes(frame) for frame in frames],
        }

        temporary_name = None
        try:
            with tempfile.NamedTemporaryFile(
                    mode='wb', dir=str(target_dir), prefix='.partial_', delete=False) as stream:
                temporary_name = stream.name
                stream.write(msgpack.packb(record, use_bin_type=True))
                stream.flush()
                os.fsync(stream.fileno())
            os.replace(temporary_name, str(target))
            self.recorded_packets += 1
        except Exception:
            self.write_errors += 1
            self.last_error = 'record write failed'
            if temporary_name:
                try:
                    os.unlink(temporary_name)
                except OSError:
                    pass
            raise

    def status(self) -> Dict[str, object]:
        return {
            'enabled': self.enabled,
            'directory': str(self.root) if self.root else '',
            'recorded_packets': self.recorded_packets,
            'write_errors': self.write_errors,
            'last_error': self.last_error,
        }


def load_recording(path: Path) -> Tuple[int, Tuple[bytes, bytes, bytes]]:
    """Load and validate one recorded canonical packet."""
    try:
        record = msgpack.unpackb(path.read_bytes(), raw=False, strict_map_key=False)
    except (OSError, msgpack.UnpackException, msgpack.FormatError, msgpack.StackError) as error:
        raise ValueError('cannot read recording {}: {}'.format(path, error)) from error
    if not isinstance(record, dict) or record.get('record_format_version') != RECORD_FORMAT_VERSION:
        raise ValueError('unsupported recording format in {}'.format(path))
    recorded_monotonic_ns = record.get('recorded_monotonic_ns')
    frames = record.get('frames')
    if isinstance(recorded_monotonic_ns, bool) or not isinstance(recorded_monotonic_ns, int):
        raise ValueError('recording timestamp is invalid in {}'.format(path))
    if not isinstance(frames, list) or len(frames) != 3:
        raise ValueError('recording frames are invalid in {}'.format(path))
    canonical_frames = tuple(bytes(frame) for frame in frames)
    try:
        unpack_message(canonical_frames)
    except ProtocolError as error:
        raise ValueError('recording violates canonical protocol in {}: {}'.format(path, error)) from error
    return recorded_monotonic_ns, canonical_frames


def iter_recordings(directory: str) -> Iterator[Tuple[int, Tuple[bytes, bytes, bytes]]]:
    """Yield a recording directory in original gateway-monotonic order."""
    root = Path(directory).expanduser()
    records = [load_recording(path) for path in root.glob('*/*.msgpack')]
    for record in sorted(records, key=lambda item: item[0]):
        yield record
