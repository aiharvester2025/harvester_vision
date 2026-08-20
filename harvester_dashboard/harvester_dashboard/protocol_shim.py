"""Import shim for the pure-Python canonical telemetry contract.

The contract package lives in the same colcon workspace source tree
(``src/harvester_telemetry_contract``) and has no ROS or ZeroMQ
dependency, so importing it from the system-Python dashboard is safe.
This shim inserts the source path (unless the package is already
importable) so the dashboard can run from a plain checkout without any
installation step.
"""

from __future__ import annotations

import sys
from pathlib import Path
from typing import List

# Candidate locations of the contract package relative to this file:
#   <repo>/canonical_zmq/harvester_telemetry_contract   (Orin repo layout)
#   <pkg>/harvester_dashboard/protocol_shim.py          (Xavier colcon layout)
#   <pkg>/.                                             (copied tree)
_CANDIDATES: List[Path] = [
    Path(__file__).resolve().parent.parent.parent /
    'canonical_zmq',
    Path(__file__).resolve().parent.parent.parent /
    'harvester_telemetry_contract',
    Path(__file__).resolve().parent.parent /
    'harvester_telemetry_contract',
]


def contract_path() -> Path:
    """Return the first existing contract source directory."""
    for candidate in _CANDIDATES:
        if (candidate / 'harvester_telemetry_contract' / 'protocol.py').is_file():
            return candidate
    raise ImportError(
        'harvester_telemetry_contract not found; expected it beside '
        'src/harvester_dashboard in the workspace source tree or inside '
        'the copied dashboard package directory')


def ensure_contract_importable() -> None:
    """Make ``harvester_telemetry_contract`` importable if it is not yet."""
    try:
        __import__('harvester_telemetry_contract')
        return
    except ImportError:
        pass
    sys.path.insert(0, str(contract_path()))


ensure_contract_importable()

from harvester_telemetry_contract import (  # noqa: E402  (after path setup)
    CANONICAL_CHANNELS,
    SCHEMA_VERSION,
    ProtocolError,
    pack_message,
    unpack_message,
    validate_header,
)

__all__ = [
    'CANONICAL_CHANNELS',
    'SCHEMA_VERSION',
    'ProtocolError',
    'pack_message',
    'unpack_message',
    'validate_header',
    'contract_path',
    'ensure_contract_importable',
]
