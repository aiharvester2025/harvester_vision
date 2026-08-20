"""Canonical ZeroMQ v1 aggregator — the Orin-side canonical publisher.

This is the Orin equivalent of the Xavier ``harvester_telemetry_gateway``, but
with **no ROS dependency**.  The Orin has no Gazebo/ROS, so instead of ROS
subscriptions it accepts canonical three-frame packets from local ingest
adapters (OAK, MID-360, range sensors — added later) over ZeroMQ PUSH/PULL, and
can optionally forward (relay) a remote source such as the Xavier simulation
gateway.

It owns a single canonical PUB endpoint (default ``tcp://*:5590``) and a
read-only REP status endpoint (default ``tcp://*:5600``), identical in shape to
the Xavier gateway.  It owns per-channel ``sequence`` counters, bounded
newest-wins queues (``ZMQ_CONFLATE`` never used), drop counting, opt-in
exact-packet recording, and a periodic ``v1/system/status`` stream.

Safety boundary: this is observation-only.  It never emits a joint, velocity,
PLC, solenoid, or motion command, and its status endpoint is read-only.
"""

from __future__ import annotations

import json
import threading
import time
from collections import defaultdict, deque

import zmq

from harvester_telemetry_contract import ProtocolError, pack_message, unpack_message

from .recording import PacketRecorder


class CanonicalAggregator:
    """Binds the canonical PUB/REP endpoints and drains ingest sources.

    Incoming packets are produced by :meth:`publish` (called by ingest threads)
    and held in bounded per-channel queues; a single drain loop sends the
    newest complete packet per channel with ``send_multipart(NOBLOCK)``,
    discarding stale complete packets.  This mirrors the Xavier gateway's
    ``flush_one_packet`` + bounded-queue policy.
    """

    def __init__(self, pub_endpoint='tcp://*:5590',
                 status_endpoint='tcp://*:5600',
                 source_id='orin',
                 source_mode='hardware',
                 queue_depth=2,
                 socket_hwm=8,
                 record_dir='',
                 relay_endpoint='',
                 ingest_endpoint=''):
        self.source_id = source_id
        self.source_mode = source_mode
        self.queue_depth = max(1, int(queue_depth))
        self.socket_hwm = max(1, int(socket_hwm))
        # Shared mutable state is guarded by ``_lock``: the drain loop, relay
        # thread, and ingest thread all mutate the queues/sequences/drop counts.
        self._lock = threading.Lock()
        self.sequence = defaultdict(int)
        self.queues = defaultdict(lambda: deque(maxlen=self.queue_depth))
        self.last_stream_status = {}
        self.drop_counts = defaultdict(int)
        self.started_monotonic_ns = time.monotonic_ns()
        self.recorder = PacketRecorder(record_dir)
        # Source capabilities, not live-stream health (latter lives in
        # v1/system/status so a missing sensor is never masked).
        self.capabilities = {
            'camera.cutter.rgb': False,
            'camera.cutter.depth': False,
            'camera.cutter.camera_info': False,
            'camera.docking.rgb': False,
            'camera.docking.depth': False,
            'camera.docking.camera_info': False,
            'lidar.raw_xyz': False,
            'lidar.intensity': False,
            'lidar.point_time': False,
            'range.docking': False,
            'range.cutter': False,
            'docking.trunk_estimate': False,
            'calibration.status': True,
            'packet.recording': True,
            'packet.replay': True,
            'target.world_fixed': False,
        }

        context = zmq.Context.instance()
        self.pub_socket = context.socket(zmq.PUB)
        self.pub_socket.setsockopt(zmq.LINGER, 0)
        self.pub_socket.setsockopt(zmq.SNDHWM, self.socket_hwm)
        self.pub_socket.bind(pub_endpoint)

        self.status_socket = context.socket(zmq.REP)
        self.status_socket.setsockopt(zmq.LINGER, 0)
        self.status_socket.bind(status_endpoint)

        self._ingest_socket = None
        self._relay_socket = None
        self._relay_thread = None
        self._running = threading.Event()
        self._running.set()

        if ingest_endpoint:
            self._ingest_socket = context.socket(zmq.PULL)
            self._ingest_socket.setsockopt(zmq.LINGER, 0)
            self._ingest_socket.setsockopt(zmq.RCVHWM, self.socket_hwm)
            self._ingest_socket.bind(ingest_endpoint)

        if relay_endpoint:
            self._relay_socket = context.socket(zmq.SUB)
            self._relay_socket.setsockopt(zmq.LINGER, 0)
            self._relay_socket.setsockopt(zmq.RCVHWM, self.socket_hwm)
            self._relay_socket.connect(relay_endpoint)
            self._relay_socket.setsockopt(zmq.SUBSCRIBE, b'v1/')
            self._relay_thread = threading.Thread(
                target=self._relay_loop, name='canonical-relay', daemon=True)

        self._ingest_thread = threading.Thread(
            target=self._ingest_loop, name='canonical-ingest', daemon=True)

    # ------------------------------------------------------------------ public
    def start(self):
        """Start the ingest/relay worker threads."""
        if self._ingest_socket is not None:
            self._ingest_thread.start()
        if self._relay_thread is not None:
            self._relay_thread.start()

    def publish(self, channel, header, payload):
        """Publish one canonical packet (the ingest entry point).

        ``header`` may omit ``sequence``/``source_id``/``gateway_monotonic_ns``;
        they are filled/overwritten here so the aggregator stays the sole owner
        of per-channel sequence numbers and local freshness.  Packets are
        validated against the contract before entering the bounded queue.
        """
        header = dict(header)
        with self._lock:
            self.sequence[channel] += 1
            header['sequence'] = self.sequence[channel]
            header['source_id'] = self.source_id
            header['gateway_monotonic_ns'] = time.monotonic_ns()
        try:
            frames = pack_message(channel, header, payload)
        except ProtocolError as error:
            with self._lock:
                self.last_stream_status[channel] = {'enabled': False, 'error': str(error)}
            return
        if self.recorder.enabled:
            try:
                self.recorder.write(frames)
            except OSError:
                # Audit storage errors must never stop the live telemetry path.
                pass
        with self._lock:
            queue = self.queues[channel]
            if len(queue) == queue.maxlen:
                self.drop_counts[channel] += 1
            queue.append(frames)
            self.last_stream_status[channel] = {
                'enabled': True,
                'last_sequence': header['sequence'],
                'last_acquisition_timestamp_ns': header['acquisition_timestamp_ns'],
                'frame_id': header['frame_id'],
            }

    def publish_json(self, channel, header, payload_obj):
        """Convenience wrapper for JSON channels."""
        self.publish(
            channel, header,
            json.dumps(payload_obj, separators=(',', ':')).encode('utf-8'))

    # ------------------------------------------------------------------ ingest
    def _relay_loop(self):
        """Forward a remote canonical source (e.g. Xavier) onto the local PUB.

        Relay preserves the remote packet's original ``source_id``/``source_mode``
        so the dashboard badge stays correct; the aggregator does not re-sequence
        or re-own forwarded packets (their origin is the remote source).
        """
        while self._running.is_set():
            try:
                frames = self._relay_socket.recv_multipart(flags=zmq.NOBLOCK)
            except zmq.Again:
                time.sleep(0.001)
                continue
            except zmq.ZMQError:
                break
            if len(frames) != 3:
                continue
            try:
                channel, header, _payload = unpack_message(frames)
            except ProtocolError:
                continue
            # Track relayed streams in status without re-owning the sequence.
            with self._lock:
                self.last_stream_status[channel] = {
                    'enabled': True,
                    'last_sequence': header['sequence'],
                    'last_acquisition_timestamp_ns': header['acquisition_timestamp_ns'],
                    'frame_id': header['frame_id'],
                    'source_mode': header.get('source_mode'),
                    'relayed': True,
                }
            try:
                self.pub_socket.send_multipart(frames, flags=zmq.NOBLOCK)
            except zmq.Again:
                with self._lock:
                    self.drop_counts[channel] += 1

    def _ingest_loop(self):
        """Drain adapter PUSH packets; re-own sequence and republish."""
        while self._running.is_set():
            try:
                frames = self._ingest_socket.recv_multipart(flags=zmq.NOBLOCK)
            except zmq.Again:
                time.sleep(0.001)
                continue
            except zmq.ZMQError:
                break
            if len(frames) != 3:
                continue
            try:
                channel, header, payload = unpack_message(frames)
            except ProtocolError:
                continue
            # Re-own the packet on the canonical bus: aggregator sequence +
            # source identity, preserving acquisition time and frame info.
            header['source_id'] = self.source_id
            self.publish(channel, header, payload)

    # ------------------------------------------------------------------ drain
    def flush_one_packet(self):
        """Send the newest complete packet for the first non-empty channel."""
        with self._lock:
            for channel in sorted(self.queues):
                queue = self.queues[channel]
                if not queue:
                    continue
                frames = queue.pop()  # newest wins; discard stale complete packets.
                dropped = len(queue)
                if dropped:
                    self.drop_counts[channel] += dropped
                    queue.clear()
                break
            else:
                return False
        try:
            self.pub_socket.send_multipart(frames, flags=zmq.NOBLOCK)
        except zmq.Again:
            with self._lock:
                self.drop_counts[channel] += 1
        return True

    # ------------------------------------------------------------------ status
    def _effective_source_mode(self):
        """Report the mode the dashboard will actually observe.

        In relay-only mode the forwarded packets keep the remote source's
        ``source_mode`` (e.g. ``simulation`` from Xavier), so reporting the
        configured ``hardware`` default would make the dashboard badge show
        ``MIXED``.  Prefer the mode of any relayed stream when present.
        """
        relayed_modes = {
            entry.get('source_mode')
            for entry in self.last_stream_status.values()
            if isinstance(entry, dict) and entry.get('relayed')
        }
        relayed_modes.discard(None)
        if relayed_modes:
            return sorted(relayed_modes)[0]
        return self.source_mode

    def publish_system_status(self):
        effective_mode = self._effective_source_mode()
        header = {
            'schema_version': 1,
            'source_mode': effective_mode,
            'source_id': self.source_id,
            'sequence': 0,
            'frame_id': '',
            'acquisition_timestamp_ns': time.time_ns(),
            'clock_domain': 'utc_host',
            'gateway_monotonic_ns': time.monotonic_ns(),
            'calibration_id': 'none',
            'codec': 'json',
            'capabilities': dict(self.capabilities),
        }
        payload = {
            'source_id': self.source_id,
            'source_mode': effective_mode,
            'uptime_s': (time.monotonic_ns() - self.started_monotonic_ns) / 1e9,
            'streams': dict(self.last_stream_status),
            'dropped_packets': dict(self.drop_counts),
            'errors': [],
            'recording': self.recorder.status(),
            'capabilities': self.capabilities,
        }
        self.publish_json('v1/system/status', header, payload)

    def handle_status_request(self):
        try:
            self.status_socket.recv_multipart(flags=zmq.NOBLOCK)
        except zmq.Again:
            return
        response = {
            'schema_version': 1,
            'active_profile': self.source_mode,
            'calibration_revision': {'status': 'orin_hardware_v0'},
            'streams': self.last_stream_status,
            'dropped_packets': dict(self.drop_counts),
            'recording': self.recorder.status(),
            'capabilities': self.capabilities,
            'latest_status': 'OK',
        }
        self.status_socket.send(json.dumps(response, separators=(',', ':')).encode('utf-8'))

    def close(self):
        self._running.clear()
        self.pub_socket.close(0)
        self.status_socket.close(0)
        if self._ingest_socket is not None:
            self._ingest_socket.close(0)
        if self._relay_socket is not None:
            self._relay_socket.close(0)
