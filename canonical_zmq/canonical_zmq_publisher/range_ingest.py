#!/usr/bin/env python3
"""Raspberry Pi range/boom ingest adapter for the canonical telemetry bus.

Subscribes to the Pi PLC's single-part JSON sensor stream
(``tcp://192.168.50.40:5555``, topic ``harvester.sensors.v1``) and republishes it
as canonical three-frame packets into the Orin aggregator's PULL socket
(``tcp://127.0.0.1:5570``).  It never binds the canonical ``5590`` PUB endpoint.

Canonical channels produced:
  * ``v1/range/docking``      — the five docking range readings (JSON list)
  * ``v1/boom/state``         — boom angle / extension / leveling / phase (JSON)
  * ``v1/docking/trunk_estimate`` — the simulated trunk pose (JSON)

The ``telemetry_key`` -> ``frame_id`` mapping comes from
``calibration/frames.*.json`` (``sensor_telemetry_bindings``).

Safety boundary: observation-only.  This adapter only forwards measurements; it
never emits an actuation or motion command.

Run under the depthai-env python::

    PYTHONPATH=canonical_zmq python3 -m canonical_zmq_publisher.range_ingest \
        --sensor-sub tcp://192.168.50.40:5555 \
        --ingest-endpoint tcp://127.0.0.1:5570
"""

from __future__ import annotations

import argparse
import json
import time

import zmq

from harvester_telemetry_contract import pack_message


# The five docking sensors, in a stable display order, with their canonical
# ``telemetry_key`` (wire key on the Pi stream) and ``frame_id`` (calibration).
SENSOR_BINDINGS = [
    ("center_line", "sensor_center_line_frame"),
    ("diagonal_left_45deg", "sensor_diagonal_left_frame"),
    ("diagonal_right_45deg", "sensor_diagonal_right_frame"),
    ("c_channel_left", "sensor_c_channel_left_frame"),
    ("c_channel_right", "sensor_c_channel_right_frame"),
]

CALIBRATION_ID = "pi_plc_provisional_v0"
SOURCE_ID = "pi_plc"
CLOCK_DOMAIN = "plc_rtc_utc"


def build_capabilities():
    return {
        'range.docking': True,
        'boom.state': True,
        'docking.trunk_estimate': True,
        'target.world_fixed': False,
    }


def _base_header(channel, acquisition_timestamp_ns, frame_id, capabilities):
    return {
        'schema_version': 1,
        'source_mode': 'hardware',
        'source_id': SOURCE_ID,
        'sequence': 0,  # owned by the aggregator
        'frame_id': frame_id,
        'acquisition_timestamp_ns': acquisition_timestamp_ns,
        'clock_domain': CLOCK_DOMAIN,
        'gateway_monotonic_ns': 0,  # owned by the aggregator
        'calibration_id': CALIBRATION_ID,
        'capabilities': capabilities,
        'codec': 'json',
    }


def map_docking_records(sensors):
    """Map the Pi ``sensors`` dict to canonical ``v1/range/docking`` records."""
    records = []
    for telemetry_key, frame_id in SENSOR_BINDINGS:
        reading = sensors.get(telemetry_key) if isinstance(sensors, dict) else None
        if not isinstance(reading, dict):
            records.append({
                'telemetry_key': telemetry_key,
                'distance_m': None,
                'valid': False,
                'frame_id': frame_id,
                'acquisition_timestamp_ns': time.time_ns(),
                'calibration_id': CALIBRATION_ID,
                'min_range_m': 0.1,
                'max_range_m': 9.999,
            })
            continue
        distance = reading.get('distance_m')
        valid = bool(reading.get('valid', False))
        records.append({
            'telemetry_key': telemetry_key,
            'distance_m': (None if distance is None else float(distance)),
            'valid': valid,
            'frame_id': frame_id,
            'acquisition_timestamp_ns': time.time_ns(),
            'calibration_id': CALIBRATION_ID,
            'min_range_m': 0.1,
            'max_range_m': 9.999,
        })
    return records


def map_boom_state(simulation):
    """Map the Pi ``simulation`` dict to the ``v1/boom/state`` payload."""
    sim = simulation if isinstance(simulation, dict) else {}
    return {
        'phase': sim.get('phase', 'UNKNOWN'),
        'boom_angle_deg': sim.get('boom_angle_deg'),
        'boom_extension_m': sim.get('boom_extension_m'),
        'platform_roll_deg': sim.get('platform_roll_deg'),
        'platform_pitch_deg': sim.get('platform_pitch_deg'),
        'docked': bool(sim.get('docked', False)),
        'target_tree_height_m': sim.get('target_tree_height_m'),
    }


def map_trunk_estimate(simulation):
    """Derive a trunk pose estimate from the simulated approach state.

    The trunk centre sits one radius beyond the bark seen by S_C, so its +X
    position is ``center_bark_distance + trunk_radius``.  The Pi payload now
    publishes ``simulation.center_bark_distance_m``; when it is absent (older
    payload or non-docking phase) we fall back to the docked position.
    """
    sim = simulation if isinstance(simulation, dict) else {}
    trunk_radius = 0.300  # matches the Pi script TRUNK_RADIUS_M
    final_clearance = 0.120  # matches the Pi script FINAL_CLEARANCE_M
    bark = sim.get('center_bark_distance_m')
    try:
        bark = float(bark)
    except (TypeError, ValueError):
        bark = None
    if bark is None:
        bark = final_clearance
    return {
        'pose': {
            'position': {'x': bark + trunk_radius, 'y': 0.0, 'z': 0.0},
            'orientation': {'x': 0.0, 'y': 0.0, 'z': 0.0, 'w': 1.0},
        },
        'covariance': [0.0] * 36,
        'phase': sim.get('phase', 'UNKNOWN'),
    }


class RangeIngest:
    """SUB to the Pi stream and PUSH canonical packets to the aggregator."""

    def __init__(self, sensor_sub='tcp://192.168.50.40:5555',
                 topic='harvester.sensors.v1',
                 ingest_endpoint='tcp://127.0.0.1:5570'):
        self.topic = topic.encode('utf-8')
        self.capabilities = build_capabilities()

        context = zmq.Context.instance()
        self.sub_socket = context.socket(zmq.SUB)
        self.sub_socket.setsockopt(zmq.LINGER, 0)
        self.sub_socket.setsockopt(zmq.RCVHWM, 8)
        self.sub_socket.setsockopt(zmq.SUBSCRIBE, self.topic)
        self.sub_socket.connect(sensor_sub)

        self.push_socket = context.socket(zmq.PUSH)
        self.push_socket.setsockopt(zmq.LINGER, 0)
        self.push_socket.setsockopt(zmq.SNDHWM, 8)
        self.push_socket.connect(ingest_endpoint)

    def _publish_json(self, channel, header, payload_obj):
        header['frame_id'] = header.get('frame_id', '')
        try:
            frames = pack_message(
                channel, header,
                json.dumps(payload_obj, separators=(',', ':')).encode('utf-8'))
        except Exception as error:
            print('[range_ingest] rejected {} packet: {}'.format(channel, error))
            return
        self.push_socket.send_multipart(frames)

    def handle(self, payload):
        if not isinstance(payload, dict):
            return
        acquisition_ns = time.time_ns()
        sensors = payload.get('sensors', {})
        simulation = payload.get('simulation', {})

        # v1/range/docking
        records = map_docking_records(sensors)
        header = _base_header('v1/range/docking', acquisition_ns,
                              'docking_sensor_array_link', self.capabilities)
        self._publish_json('v1/range/docking', header, records)

        # v1/boom/state
        boom = map_boom_state(simulation)
        header = _base_header('v1/boom/state', acquisition_ns,
                              'boom_link', self.capabilities)
        self._publish_json('v1/boom/state', header, boom)

        # v1/docking/trunk_estimate
        trunk = map_trunk_estimate(simulation)
        header = _base_header('v1/docking/trunk_estimate', acquisition_ns,
                              'docking_reference', self.capabilities)
        self._publish_json('v1/docking/trunk_estimate', header, trunk)

    def run(self):
        print('[range_ingest] SUB {} topic {!r} -> PUSH {}'.format(
            'sensor', self.topic.decode(), self.push_socket.getsockopt(zmq.LAST_ENDPOINT)))
        while True:
            try:
                frames = self.sub_socket.recv_multipart()
            except KeyboardInterrupt:
                break
            # Single-part JSON: [topic, payload].
            if len(frames) < 2:
                continue
            try:
                payload = json.loads(frames[-1].decode('utf-8'))
            except (UnicodeDecodeError, json.JSONDecodeError) as error:
                print('[range_ingest] bad JSON: {}'.format(error))
                continue
            self.handle(payload)

    def close(self):
        self.sub_socket.close(0)
        self.push_socket.close(0)


def _arguments(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--sensor-sub', default='tcp://192.168.50.40:5555',
                        help='Pi PLC sensor PUB endpoint to subscribe to')
    parser.add_argument('--topic', default='harvester.sensors.v1',
                        help='subscription topic on the Pi stream')
    parser.add_argument('--ingest-endpoint', default='tcp://127.0.0.1:5570',
                        help='aggregator PULL endpoint to PUSH canonical frames into')
    return parser.parse_args(argv)


def main(argv=None):
    args = _arguments(argv)
    adapter = RangeIngest(
        sensor_sub=args.sensor_sub,
        topic=args.topic,
        ingest_endpoint=args.ingest_endpoint,
    )
    try:
        adapter.run()
    except KeyboardInterrupt:
        pass
    finally:
        adapter.close()


if __name__ == '__main__':
    main()
