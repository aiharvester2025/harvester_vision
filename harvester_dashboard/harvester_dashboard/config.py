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
    pointcloud_max_points: int = 2000
    qml_directory: Optional[str] = None
    # Optional path to a JSON file overriding HUD panel layout/captions/sizes.
    # None falls back to the built-in defaults (or the shipped hud_config.json
    # when the launcher passes it explicitly).
    hud_config_path: Optional[str] = None
    # Optional path to the docking safety-guidance tuning file.  None (the
    # default) falls back to the module's best-effort path, then to the built-in
    # thresholds; a missing/malformed file never raises.
    safety_config_path: Optional[str] = None
    # Optional path to the cutter safety-guidance tuning file (mirrors
    # ``safety_config_path``; None falls back to the module path, then defaults).
    cutter_config_path: Optional[str] = None
    # Docking point offset below the trunk top (m), used by the LiDAR scan
    # estimate.  Deployment decision: 2.0 m below the trunk top.
    docking_offset_below_top_m: float = 2.0
    # Harvester base world X, used to derive the boom-pivot-relative horizontal
    # distance from a scanned trunk axis.  0 in the reference geometry (the boom
    # pivot is then at x = -0.87, so the trunk at x = 8.5 gives d_horiz = 9.37 m).
    base_x_m: float = 0.0
    # LiDAR standby/normal control endpoint for the operator scan (the producer's
    # PULL socket).  Empty disables it: the dashboard stays fully render-only.
    # This is the one opt-in exception to the render-only rule; see
    # ``lidar_control.py``.
    lidar_control_endpoint: str = ''
    # Guided scan window (s) before the estimate auto-completes.  None means
    # "use hud_config.json lidar_scan.scan_seconds" (default 15 s); an explicit
    # --lidar-scan-seconds always overrides it.
    lidar_scan_seconds: Optional[float] = None

    @property
    def status_enabled(self) -> bool:
        return bool(self.status_endpoint)

    @property
    def annotation_enabled(self) -> bool:
        return bool(self.annotation_endpoint)

    @property
    def lidar_control_enabled(self) -> bool:
        return bool(self.lidar_control_endpoint)

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
        parser.add_argument('--pointcloud-max-points', type=int, default=2000,
                            help='maximum depth-unprojected points kept for the camera point-cloud panel')
        parser.add_argument('--hud-config', default=None,
                            help='path to a JSON file overriding HUD panel layout/captions/sizes')
        parser.add_argument('--safety-config', default=None,
                            help='path to the docking safety-guidance tuning JSON file')
        parser.add_argument('--cutter-config', default=None,
                            help='path to the cutter safety-guidance tuning JSON file')
        parser.add_argument('--docking-offset-below-top-m', type=float, default=2.0,
                            help='docking point offset below the trunk top (m) for '
                                 'the LiDAR scan estimate (default 2.0)')
        parser.add_argument('--base-x', type=float, default=0.0,
                            help='harvester base world X (m), used to derive the '
                                 'boom-pivot-relative d_horiz from the scanned '
                                 'trunk axis (default 0.0)')
        parser.add_argument('--lidar-control', default='',
                            help='optional LiDAR standby/normal control endpoint for '
                                 'the scan HUD (the producer PULL socket, e.g. '
                                 'tcp://127.0.0.1:5571); empty disables it (the '
                                 'dashboard stays render-only)')
        parser.add_argument('--lidar-scan-seconds', type=float, default=None,
                            help='guided scan window (s) before the estimate '
                                 'auto-completes; unset uses hud_config '
                                 'lidar_scan.scan_seconds (default 15)')
        known, _unknown = parser.parse_known_args(args)
        scan_seconds = (None if known.lidar_scan_seconds is None
                        else max(1.0, known.lidar_scan_seconds))
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
            pointcloud_max_points=max(1, known.pointcloud_max_points),
            hud_config_path=(known.hud_config or None),
            safety_config_path=(known.safety_config or None),
            cutter_config_path=(known.cutter_config or None),
            docking_offset_below_top_m=max(0.0, known.docking_offset_below_top_m),
            base_x_m=float(getattr(known, 'base_x', 0.0)),
            lidar_control_endpoint=(known.lidar_control or ''),
            lidar_scan_seconds=scan_seconds,
        )
