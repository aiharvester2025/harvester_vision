"""Runtime configuration for the operator dashboard (pure dataclass)."""

from __future__ import annotations

import argparse
from dataclasses import dataclass, field
from typing import Optional, Tuple


@dataclass
class DashboardConfig:
    """All endpoint/tuning values.  No Qt or ZeroMQ objects live here."""

    pub_endpoint: str = 'tcp://127.0.0.1:5590'
    # Empty string disables the status client entirely (replay sessions).
    status_endpoint: str = 'tcp://127.0.0.1:5600'
    # Empty string disables the optional annotation forwarder (default off).
    annotation_endpoint: str = ''
    stale_after_s: float = 2.0
    status_interval_s: float = 5.0
    queue_depth: int = 4
    socket_hwm: int = 8
    lidar_max_points: int = 2000
    annotation_depth_window_px: int = 3
    qml_directory: Optional[str] = None

    @property
    def status_enabled(self) -> bool:
        return bool(self.status_endpoint)

    @property
    def annotation_enabled(self) -> bool:
        return bool(self.annotation_endpoint)

    @classmethod
    def from_args(cls, args=None) -> 'DashboardConfig':
        parser = argparse.ArgumentParser(
            prog='harvester_dashboard',
            description='Canonical telemetry v1 operator dashboard (view-only)')
        parser.add_argument('--pub', default='tcp://127.0.0.1:5590',
                            help='canonical telemetry PUB endpoint to subscribe to')
        parser.add_argument('--status', default='tcp://127.0.0.1:5600',
                            help='read-only status REP endpoint; empty string disables')
        parser.add_argument('--annotation-pub', default='',
                            help='optional annotation forward PUB endpoint (default disabled)')
        parser.add_argument('--stale-after-s', type=float, default=2.0,
                            help='mark a stream stale after this many silent seconds')
        parser.add_argument('--status-interval-s', type=float, default=5.0,
                            help='period for polling the read-only status endpoint')
        parser.add_argument('--queue-depth', type=int, default=4,
                            help='per-channel bounded queue depth (max 4)')
        parser.add_argument('--socket-hwm', type=int, default=8,
                            help='subscriber RCVHWM in packets')
        parser.add_argument('--lidar-max-points', type=int, default=2000,
                            help='maximum LiDAR points kept for the inset scatter')
        known, _unknown = parser.parse_known_args(args)
        return cls(
            pub_endpoint=known.pub,
            status_endpoint=known.status,
            annotation_endpoint=known.annotation_pub,
            stale_after_s=known.stale_after_s,
            status_interval_s=known.status_interval_s,
            queue_depth=max(1, min(4, known.queue_depth)),
            socket_hwm=max(1, known.socket_hwm),
            lidar_max_points=max(1, known.lidar_max_points),
            qml_directory=known.qml_directory if hasattr(known, 'qml_directory') else None,
        )
