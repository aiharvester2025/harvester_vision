#!/usr/bin/env python3
"""MID-360 LiDAR capture adapter for the canonical telemetry bus.

Reads points (+ IMU) from the Livox MID-360 through ``lidar.livox_source``,
levels them to gravity (rotation-only), filters to a bounded forward sector and
range, downsamples, and PUSHes canonical ``v1/lidar/raw`` packets into the
canonical aggregator's PULL socket.  It never binds the canonical ``5590`` PUB
endpoint — the aggregator is the sole owner of that endpoint.

Data flow::

    MID-360 (UDP) -> Livox-SDK2 callback -> drain -> level -> sector+range clip
        -> downsample -> canonical frames -> PUSH -> aggregator PULL
                                                    -> PUB tcp://*:5590
                                                    -> dashboard SUB

On-demand operation (the reason this adapter exists rather than a 24/7 feed):
the LiDAR is idle until a control command enables it.  A PULL socket accepts
``{"enabled": true|false}`` (the same one-way contract the OAK publishers use),
so a remote operator starts and stops the sensor without restarting any
process.  While disabled the SDK callback queue is drained and discarded so it
cannot grow without bound, and no packets are published.

Modes:

  * ``synthetic`` (default): emits a synthetic forward-sector cloud so the
    canonical path can be validated without hardware.
  * ``livox-sdk``: reads the real LiDAR through ``lidar.livox_source``.

Run::

    PYTHONPATH=canonical_zmq:. python3 -m canonical_zmq_publisher.lidar_capture \\
        --sdk-mode livox-sdk --sdk-config ./mid360_config.json \\
        --ingest-endpoint tcp://127.0.0.1:5570

Requires the depthai-env python (``zmq``, ``msgpack``) and, for
``livox-sdk`` mode, the built ``liblivox_lidar_sdk_shared.so``.
"""

from __future__ import annotations

import argparse
import math
import os
import signal
import struct
import time
from typing import List, Sequence

import zmq

import numpy as np

from time_sync import TIME_AUTHORITY, ChronyStatus
from harvester_telemetry_contract import pack_message

# Canonical channel for the LiDAR cloud (frozen by the contract).
LIDAR_CHANNEL = 'v1/lidar/raw'

# The canonical point layout: packed little-endian float32 x/y/z (12 bytes).
POINT_FIELDS = [
    {'name': 'x', 'type': 'float32'},
    {'name': 'y', 'type': 'float32'},
    {'name': 'z', 'type': 'float32'},
]
POINT_STRIDE_BYTES = 12

DEFAULT_FRAME_ID = 'mid360_link'


def build_lidar_header(frame_id, point_count, acquisition_timestamp_ns,
                       capabilities=None, source_mode='hardware',
                       clock_domain='plc_rtc_utc', timestamp_source=None):
    """Return a canonical header for a ``v1/lidar/raw`` packet.

    ``source_mode`` must reflect the real provenance: ``simulation`` for the
    synthetic generator, ``hardware`` for the real MID-360. Mislabelling
    synthetic data as hardware would make a remote operator believe the sensor
    is live.

    ``clock_domain`` and ``timestamp_source`` describe WHICH clock
    ``acquisition_timestamp_ns`` is expressed in and how it was obtained. They
    default to the frozen contract values (``plc_rtc_utc`` / unset) so existing
    callers are unaffected, but the LiDAR path passes the real values because a
    MID-360 that is not PTP/GPS-synchronized has no UTC clock at all: labelling
    its boot-relative counter ``plc_rtc_utc`` would be a lie that looks correct.
    """
    if source_mode not in ('hardware', 'simulation'):
        raise ValueError('unsupported source_mode {!r}'.format(source_mode))
    header = {
        'schema_version': 1,
        'source_mode': source_mode,
        'source_id': 'orin',
        'sequence': 0,  # owned by the aggregator
        'frame_id': frame_id,
        'acquisition_timestamp_ns': acquisition_timestamp_ns,
        'clock_domain': clock_domain,
        'gateway_monotonic_ns': 0,  # owned by the aggregator
        'calibration_id': 'mid360_v0',
        'capabilities': capabilities or {
            'lidar.raw_xyz': True,
            'lidar.intensity': False,
            'lidar.point_time': False,
            'target.world_fixed': False,
        },
        'codec': 'lidar_xyz_f32',
        'point_count': int(point_count),
        'point_stride_bytes': POINT_STRIDE_BYTES,
        'point_fields': [dict(field) for field in POINT_FIELDS],
    }
    if timestamp_source is not None:
        header['timestamp_source'] = timestamp_source
    return header



def pack_points(points: Sequence[Sequence[float]]) -> bytes:
    """Pack an ``(N, 3)`` float sequence into the canonical little-endian blob.

    Built with one numpy conversion rather than a per-point Python list: at the
    MID-360's ~200k points/s the list-then-varargs form allocates a Python float
    object per coordinate, which is measurable on this CPU-constrained host.

    Accepts any ``(N, 3)`` row sequence, including a NumPy array. The emptiness
    check uses ``.size`` rather than truthiness because ``if not points`` raises
    ``ValueError`` for an array with more than one row.
    """
    array = np.asarray(points, dtype='<f4')
    if array.size == 0:
        return b''
    if array.ndim != 2 or array.shape[1] != 3:
        array = array.reshape(-1, 3)
    return array.astype('<f4', copy=False).tobytes()


def sector_filter(points, sector_deg):
    """Drop points whose horizontal azimuth falls outside the forward sector.

    The sector is centred on ``+X`` (project forward) and spans ``sector_deg``
    total, i.e. ``±sector_deg/2``.  A full 360° sector (or larger) is a no-op.
    Azimuth is ``atan2(y, x)`` on the leveled cloud; leveling is rotation-only
    about the vertical so it preserves azimuth.
    """
    if sector_deg >= 360.0:
        return list(points)
    half = math.radians(sector_deg) / 2.0
    kept = []
    for point in points:
        x, y = point[0], point[1]
        # atan2 returns (-pi, pi]; the forward sector straddles 0.
        if abs(math.atan2(y, x)) <= half:
            kept.append(tuple(point))
    return kept


def range_clip(points, min_range_m, max_range_m):
    """Drop points inside the blind zone or beyond the configured range."""
    kept = []
    for point in points:
        x, y, z = point[0], point[1], point[2]
        distance = math.sqrt(x * x + y * y + z * z)
        if distance < min_range_m or distance > max_range_m:
            continue
        kept.append(tuple(point))
    return kept


def downsample(points, maximum):
    """Uniformly downsample a point list to at most ``maximum`` points."""
    if maximum <= 0 or len(points) <= maximum:
        return list(points)
    if maximum == 1:
        return [points[0]]
    step = (len(points) - 1) / float(maximum - 1)
    return [points[int(round(i * step))] for i in range(maximum)]


def run_synthetic(max_points, sector_deg, min_range_m, max_range_m):
    """Return a synthetic forward-sector ring cloud for plumbing tests."""
    points = []
    count = max(64, max_points * 3)
    for i in range(count):
        angle = 2.0 * math.pi * i / count
        radius = 2.0 + 6.0 * (i % 5) / 5.0
        points.append((radius * math.cos(angle), radius * math.sin(angle),
                       (i % 11) * 0.1))
    points = sector_filter(points, sector_deg)
    points = range_clip(points, min_range_m, max_range_m)
    return downsample(points, max_points)


class LidarCapture:
    """Publishes canonical ``v1/lidar/raw`` packets to an aggregator."""

    def __init__(self, sdk_mode='synthetic',
                 ingest_endpoint='tcp://127.0.0.1:5570',
                 control_endpoint='tcp://127.0.0.1:5571',
                 hz=10.0, max_points=2000, frame_id=DEFAULT_FRAME_ID,
                 level_source='imu', sector_deg=120.0,
                 min_range_m=0.15, max_range_m=40.0,
                 sdk_config_path=None, host_ip=None, sdk_library_path=None,
                 lidar_ip=None, disabled=True):
        if sdk_mode not in ('synthetic', 'livox-sdk'):
            raise ValueError('unsupported sdk_mode {!r}'.format(sdk_mode))
        if hz <= 0:
            raise ValueError('hz must be positive')
        if max_points <= 0:
            raise ValueError('max_points must be positive')
        self.sdk_mode = sdk_mode
        self.hz = float(hz)
        self.max_points = int(max_points)
        self.frame_id = frame_id
        self.level_source = level_source
        self.sector_deg = float(sector_deg)
        self.min_range_m = float(min_range_m)
        self.max_range_m = float(max_range_m)
        # The LiDAR is idle until enabled: this is the on-demand contract.
        self.enabled = not disabled

        self._source = None
        if sdk_mode == 'livox-sdk':
            from lidar.livox_source import (
                DEFAULT_HOST_IP, DEFAULT_LIDAR_IP, LivoxMid360Source)
            # auto_start=False: the SDK is initialised only when the operator
            # enables the sensor, so the MID-360 is genuinely idle (no UDP
            # stream, no power draw beyond standby) until then.
            self._source = LivoxMid360Source(
                level_source=level_source,
                sdk_config_path=sdk_config_path,
                host_ip=host_ip or DEFAULT_HOST_IP,
                lidar_ip=lidar_ip or DEFAULT_LIDAR_IP,
                sdk_library_path=sdk_library_path,
                auto_start=False,
            )

        context = zmq.Context.instance()
        self.push_socket = context.socket(zmq.PUSH)
        self.push_socket.setsockopt(zmq.LINGER, 0)
        self.push_socket.setsockopt(zmq.SNDHWM, 8)
        self.push_socket.connect(ingest_endpoint)
        self.control_socket = context.socket(zmq.PULL)
        self.control_socket.bind(control_endpoint)
        self.channel = LIDAR_CHANNEL
        self.chrony = ChronyStatus()

    # ------------------------------------------------------------------ control
    def poll_control(self):
        """Apply any pending enable/disable command; return True if changed."""
        changed = False
        while True:
            try:
                message = self.control_socket.recv_json(flags=zmq.NOBLOCK)
            except zmq.Again:
                return changed
            except (zmq.ZMQError, ValueError) as error:
                # Make an unparseable control frame visible: otherwise a dropped
                # "disable" is indistinguishable from "nothing was sent", and the
                # operator believes a physical sensor was stopped when it was not.
                print('[{}] ignoring malformed control message: {!r}'.format(
                    self.channel, error))
                continue
            if not isinstance(message, dict) or 'enabled' not in message:
                print('[{}] ignoring control message without "enabled": {!r}'.format(
                    self.channel, message))
                continue
            new_state = message['enabled']
            if not isinstance(new_state, bool):
                print('[{}] ignoring non-boolean enabled value {!r}'.format(
                    self.channel, new_state))
                continue
            started, stopped = self._apply_enabled(new_state)
            # The enabled flag always follows the command; started/stopped only
            # annotate whether the SDK also transitioned (synthetic mode has no
            # SDK, so the flag alone gates output).
            if self.enabled != new_state:
                self.enabled = new_state
                changed = True
            # Always report the applied state so the control channel is
            # acknowledged: a repeated identical command still confirms the
            # sensor's actual state rather than being silently swallowed.
            print('[{}] State: {}{}'.format(
                self.channel, 'ENABLED' if self.enabled else 'DISABLED',
                ' (SDK started)' if started else (
                    ' (SDK stopped)' if stopped else '')))
        return changed

    def _apply_enabled(self, enabled):
        """Start/stop the SDK to match the requested state.

        Returns ``(started, stopped)`` so the caller can log the transition.
        Synthetic mode has no SDK to start; the enabled flag alone gates output.
        """
        if self._source is None:
            return False, False
        if enabled and not self._source.is_running:
            self._source.start()
            return True, False
        if not enabled and self._source.is_running:
            self._source.close()
            self._source.drop_backlog()
            return False, True
        return False, False

    # ------------------------------------------------------------------ acquire
    def acquire(self):
        """Acquire, level, and bound one scan; return the point list."""
        if self.sdk_mode == 'synthetic':
            return run_synthetic(self.max_points, self.sector_deg,
                                 self.min_range_m, self.max_range_m)

        raw_points, quaternion, orientation_valid, _ = self._source.sample(
            max_points=self.max_points)
        if orientation_valid:
            from geometry.transforms import (
                level_points_rotation_only, rotation_from_quaternion)
            rotation = rotation_from_quaternion(*quaternion)
            points = level_points_rotation_only(raw_points, rotation)
        else:
            points = [tuple(point) for point in raw_points]
        points = sector_filter(points, self.sector_deg)
        points = range_clip(points, self.min_range_m, self.max_range_m)
        return downsample(points, self.max_points)

    # ------------------------------------------------------------------ publish
    def timing_snapshot(self):
        """Return one consistent timing snapshot for a scan.

        Synthetic mode has no device, so it is host time by construction. The
        LiDAR path delegates to the source's ``timing_snapshot``, which returns
        the chosen timestamp and the status describing it in a single read. That
        matters: taking the timestamp and the status from two separate calls
        could label a stale/host-clock value as ``lidar_ptp_utc``.
        """
        if self.sdk_mode != 'livox-sdk' or self._source is None:
            return {
                'timestamp_ns': time.time_ns(),
                'source': 'host_arrival',
                'clock_domain': 'orin_realtime',
                'synchronized': False,
                'time_type': None,
                'lidar_source': 'simulation',
                'arrival_age_s': None,
            }
        return self._source.timing_snapshot()

    def emit(self, points):
        quality, _offset = self.chrony.get()
        source_mode = 'hardware' if self.sdk_mode == 'livox-sdk' else 'simulation'
        # One snapshot drives BOTH the timestamp and its provenance, so the
        # header can never say "PTP" over a host-clock value.
        timing = self.timing_snapshot()
        header = build_lidar_header(
            self.frame_id, len(points), timing['timestamp_ns'],
            source_mode=source_mode, clock_domain=timing['clock_domain'],
            timestamp_source=timing['source'])
        # ``time_quality`` reflects the Orin's chrony state (is the HOST clock
        # disciplined by the PLC RTC?), while ``lidar_time_sync`` states whether
        # the SENSOR clock is absolute. Both matter and they are independent: a
        # host can be synced while the LiDAR is not, and vice versa.
        header['time_authority'] = TIME_AUTHORITY
        header['time_quality'] = quality
        header['level_source'] = self.level_source
        header['lidar_time_sync'] = bool(timing['synchronized'])
        header['lidar_time_type'] = timing['time_type']
        header['lidar_time_source'] = timing['lidar_source']
        try:
            frames = pack_message(self.channel, header, pack_points(points))
        except Exception as error:  # ProtocolError -> drop, do not crash
            print('[{}] rejected lidar packet: {}'.format(self.channel, error))
            return
        self.push_socket.send_multipart(frames)


    def run(self):
        period_s = 1.0 / self.hz
        print('[{}] MID-360 -> PUSH {} (mode={}, sector={}°, max {} pts)'.format(
            self.channel,
            self.push_socket.getsockopt(zmq.LAST_ENDPOINT),
            self.sdk_mode, self.sector_deg, self.max_points))
        print('[{}] State: {}'.format(
            self.channel, 'ENABLED' if self.enabled else 'DISABLED'))
        print('[{}] timing: acquisition time is the LiDAR PTP/GPS timestamp when '
              'the sensor is synchronized, otherwise the Orin receive clock '
              '({})'.format(self.channel, 'chrony ' + self.chrony.get()[0]))
        # Turn SIGTERM into KeyboardInterrupt so stop_all.sh (plain `kill`) still
        # runs the finally block below. Without this the motor-stop in
        # close() would only ever run on Ctrl-C, leaving the LiDAR spinning
        # after an ordinary shutdown.
        previous_term = signal.signal(signal.SIGTERM, self._on_sigterm)
        try:
            while True:
                self.poll_control()
                if self.enabled:
                    self.emit(self.acquire())
                elif self._source is not None:
                    # Idle: discard any backlog without per-point conversion.
                    # (In livox-sdk mode the SDK is also stopped, so this is
                    # normally empty; it covers a stop racing the last callback.)
                    self._source.drop_backlog()
                time.sleep(period_s)
        except KeyboardInterrupt:
            pass
        finally:
            signal.signal(signal.SIGTERM, previous_term)
            if self._source is not None:
                self._source.close()
            self.push_socket.close(0)
            self.control_socket.close(0)

    @staticmethod
    def _on_sigterm(signum, frame):
        raise KeyboardInterrupt


def _arguments(argv=None):
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--sdk-mode', choices=('synthetic', 'livox-sdk'),
                        default='synthetic')
    parser.add_argument('--ingest-endpoint', default='tcp://127.0.0.1:5570',
                        help='aggregator PULL endpoint to PUSH canonical frames into')
    parser.add_argument('--control-endpoint',
                        default=os.environ.get('LIDAR_CONTROL_ENDPOINT',
                                               'tcp://127.0.0.1:5571'),
                        help='PULL endpoint accepting {"enabled": bool} commands '
                             '(default loopback; set LIDAR_CONTROL_ENDPOINT or '
                             'pass e.g. tcp://*:5571 to allow remote control. '
                             'The control channel is unauthenticated, so only '
                             'expose it on a trusted sensor LAN)')
    parser.add_argument('--hz', type=float, default=10.0,
                        help='scan publish rate (default 10)')
    parser.add_argument('--max-points', type=int, default=2000,
                        help='cap on emitted points per scan (default 2000)')
    parser.add_argument('--frame-id', default=DEFAULT_FRAME_ID)
    parser.add_argument('--level-source', choices=('imu', 'tilt', 'boom', 'none'),
                        default='imu')
    parser.add_argument('--sector-deg', type=float, default=120.0,
                        help='forward sector width in degrees (default 120)')
    parser.add_argument('--min-range-m', type=float, default=0.15)
    parser.add_argument('--max-range-m', type=float, default=40.0)
    parser.add_argument('--sdk-config', default=None,
                        help='Livox-SDK2 JSON for livox-sdk mode')
    parser.add_argument('--host-ip', default=None,
                        help='host IP on the LiDAR link (default 192.168.50.10)')
    parser.add_argument('--lidar-ip', default=None,
                        help='LiDAR address on the sensor LAN (default 192.168.50.30)')
    parser.add_argument('--sdk-library', default=None,
                        help='Libox SDK shared library path')
    parser.add_argument('--disabled', action='store_true',
                        help='start idle; wait for an enable command (default)')
    parser.add_argument('--enabled', dest='disabled', action='store_false',
                        help='start streaming immediately')
    parser.set_defaults(disabled=True)
    return parser.parse_args(argv)


def main(argv=None):
    args = _arguments(argv)
    try:
        capture = LidarCapture(
            sdk_mode=args.sdk_mode,
            ingest_endpoint=args.ingest_endpoint,
            control_endpoint=args.control_endpoint,
            hz=args.hz,
            max_points=args.max_points,
            frame_id=args.frame_id,
            level_source=args.level_source,
            sector_deg=args.sector_deg,
            min_range_m=args.min_range_m,
            max_range_m=args.max_range_m,
            sdk_config_path=args.sdk_config,
            host_ip=args.host_ip,
            lidar_ip=args.lidar_ip,
            sdk_library_path=args.sdk_library,
            disabled=args.disabled,
        )
    except (ValueError, FileNotFoundError, RuntimeError, ImportError) as error:
        raise SystemExit('[{}] init failed: {}'.format(LIDAR_CHANNEL, error))
    capture.run()


if __name__ == '__main__':
    main()
