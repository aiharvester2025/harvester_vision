#!/usr/bin/env python3
"""MQTT ingest adapter for the canonical telemetry bus.

Subscribes to a Mosquitto MQTT broker (``mqtt://192.168.50.100:1883``, topic
``harvester/sensors/v1``) that carries the harvester PLC sensor stream published
by Node-RED, and republishes it as canonical three-frame packets into the Orin
aggregator's PULL socket (``tcp://127.0.0.1:5570``).  It never binds the
canonical ``5590`` PUB endpoint.

Canonical channels produced:
  * ``v1/boom/state``         — boom angle / extension / platform tilt / slew (JSON)
  * ``v1/range/docking``      — docking range readings (JSON list)

The source payload is a flat JSON object with these wire keys (from the
Node-RED ``mqtt out`` node on ``harvester/sensors/v1``):

  ``RUN``, ``Platform Tilt X1``, ``Platform Tilt Y1``, ``PrimeMover Tilt X2``,
  ``PrimeMover Tilt Y2``, ``Boom_Length``, ``Boom_Angle``, ``Slew Angle``,
  ``Ultrasonic Left``.

Safety boundary: observation-only.  This adapter only forwards measurements; it
never emits an actuation or motion command.

Run under the depthai-env python::

    PYTHONPATH=canonical_zmq python3 -m canonical_zmq_publisher.mqtt_ingest \
        --mqtt-host 192.168.50.100 --mqtt-port 1883 \
        --topic harvester/sensors/v1 \
        --ingest-endpoint tcp://127.0.0.1:5570
"""

from __future__ import annotations

import argparse
import json
import time

import paho.mqtt.client as mqtt
import zmq

from harvester_telemetry_contract import pack_message


# The docking/range sensors this adapter emits.  The live payload only carries a
# single "Ultrasonic Left" reading today, but the canonical ``v1/range/docking``
# channel expects a list of telemetry-keyed records; we emit the ultrasonic
# reading as the sole record with a stable frame_id.  Additional sensors can be
# appended here as the Node-RED stream grows without changing the contract.
SENSOR_BINDINGS = [
    ("ultrasonic_left", "sensor_ultrasonic_left_frame"),
]

CALIBRATION_ID = "mqtt_plc_provisional_v0"
SOURCE_ID = "mqtt_plc"
CLOCK_DOMAIN = "plc_rtc_utc"


def build_capabilities():
    return {
        'boom.state': True,
        'range.docking': True,
        'docking.trunk_estimate': False,
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


def _as_float(value):
    """Coerce a wire value to float, or None when it is not a number."""
    if value is None:
        return None
    try:
        return float(value)
    except (TypeError, ValueError):
        return None


def map_boom_state(payload):
    """Map the flat MQTT sensor payload to the ``v1/boom/state`` payload."""
    if not isinstance(payload, dict):
        payload = {}
    return {
        'phase': 'RUN' if bool(payload.get('RUN', False)) else 'IDLE',
        'boom_angle_deg': _as_float(payload.get('Boom_Angle')),
        'boom_extension_m': _as_float(payload.get('Boom_Length')),
        'slew_angle_deg': _as_float(payload.get('Slew Angle')),
        'platform_tilt_x1_deg': _as_float(payload.get('Platform Tilt X1')),
        'platform_tilt_y1_deg': _as_float(payload.get('Platform Tilt Y1')),
        'primemover_tilt_x2_deg': _as_float(payload.get('PrimeMover Tilt X2')),
        'primemover_tilt_y2_deg': _as_float(payload.get('PrimeMover Tilt Y2')),
        'platform_roll_deg': _as_float(payload.get('Platform Tilt X1')),
        'platform_pitch_deg': _as_float(payload.get('Platform Tilt Y1')),
        'docked': False,
    }


def map_docking_records(payload):
    """Map the flat MQTT sensor payload to ``v1/range/docking`` records."""
    if not isinstance(payload, dict):
        payload = {}
    records = []
    for telemetry_key, frame_id in SENSOR_BINDINGS:
        wire_key = {'ultrasonic_left': 'Ultrasonic Left'}.get(telemetry_key)
        distance = _as_float(payload.get(wire_key)) if wire_key else None
        valid = distance is not None
        records.append({
            'telemetry_key': telemetry_key,
            'distance_m': distance,
            'valid': valid,
            'frame_id': frame_id,
            'acquisition_timestamp_ns': time.time_ns(),
            'calibration_id': CALIBRATION_ID,
            'min_range_m': 0.02,
            'max_range_m': 4.0,
        })
    return records


class MqttIngest:
    """MQTT subscriber that PUSHes canonical packets to the aggregator."""

    def __init__(self, mqtt_host='192.168.50.100', mqtt_port=1883,
                 topic='harvester/sensors/v1',
                 ingest_endpoint='tcp://127.0.0.1:5570',
                 username=None, password=None):
        self.topic = topic
        self.capabilities = build_capabilities()

        context = zmq.Context.instance()
        self.push_socket = context.socket(zmq.PUSH)
        self.push_socket.setsockopt(zmq.LINGER, 0)
        self.push_socket.setsockopt(zmq.SNDHWM, 8)
        self.push_socket.connect(ingest_endpoint)

        self._client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
        self._client.on_connect = self._on_connect
        self._client.on_message = self._on_message
        self._client.on_disconnect = self._on_disconnect
        if username is not None:
            self._client.username_pw_set(username, password)
        self._mqtt_host = mqtt_host
        self._mqtt_port = int(mqtt_port)

    def _on_connect(self, client, userdata, flags, reason_code, properties):
        if reason_code == 0:
            print('[mqtt_ingest] connected to {}:{} ; subscribing {!r}'.format(
                self._mqtt_host, self._mqtt_port, self.topic))
            client.subscribe(self.topic, qos=1)
        else:
            print('[mqtt_ingest] connect failed: {}'.format(reason_code))

    def _on_disconnect(self, client, userdata, flags, reason_code, properties):
        print('[mqtt_ingest] disconnected ({})'.format(reason_code))

    def _on_message(self, client, userdata, message):
        try:
            payload = json.loads(message.payload.decode('utf-8'))
        except (UnicodeDecodeError, json.JSONDecodeError) as error:
            print('[mqtt_ingest] bad JSON on {}: {}'.format(message.topic, error))
            return
        self.handle(payload)

    def _publish_json(self, channel, header, payload_obj):
        header['frame_id'] = header.get('frame_id', '')
        try:
            frames = pack_message(
                channel, header,
                json.dumps(payload_obj, separators=(',', ':')).encode('utf-8'))
        except Exception as error:
            print('[mqtt_ingest] rejected {} packet: {}'.format(channel, error))
            return
        self.push_socket.send_multipart(frames)

    def handle(self, payload):
        if not isinstance(payload, dict):
            return
        acquisition_ns = time.time_ns()

        # v1/boom/state
        boom = map_boom_state(payload)
        header = _base_header('v1/boom/state', acquisition_ns,
                              'boom_link', self.capabilities)
        self._publish_json('v1/boom/state', header, boom)

        # v1/range/docking
        records = map_docking_records(payload)
        header = _base_header('v1/range/docking', acquisition_ns,
                              'docking_sensor_array_link', self.capabilities)
        self._publish_json('v1/range/docking', header, records)

    def run(self):
        print('[mqtt_ingest] MQTT {}:{} topic {!r} -> PUSH {}'.format(
            self._mqtt_host, self._mqtt_port, self.topic,
            self.push_socket.getsockopt(zmq.LAST_ENDPOINT)))
        self._client.connect(self._mqtt_host, self._mqtt_port, keepalive=60)
        self._client.loop_forever()

    def close(self):
        try:
            self._client.disconnect()
        except Exception:
            pass
        self.push_socket.close(0)


def _arguments(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--mqtt-host', default='192.168.50.100',
                        help='Mosquitto broker host')
    parser.add_argument('--mqtt-port', type=int, default=1883,
                        help='Mosquitto broker port')
    parser.add_argument('--topic', default='harvester/sensors/v1',
                        help='MQTT topic to subscribe to')
    parser.add_argument('--username', default=None, help='optional MQTT username')
    parser.add_argument('--password', default=None, help='optional MQTT password')
    parser.add_argument('--ingest-endpoint', default='tcp://127.0.0.1:5570',
                        help='aggregator PULL endpoint to PUSH canonical frames into')
    return parser.parse_args(argv)


def main(argv=None):
    args = _arguments(argv)
    adapter = MqttIngest(
        mqtt_host=args.mqtt_host,
        mqtt_port=args.mqtt_port,
        topic=args.topic,
        ingest_endpoint=args.ingest_endpoint,
        username=args.username,
        password=args.password,
    )
    try:
        adapter.run()
    except KeyboardInterrupt:
        pass
    finally:
        adapter.close()


if __name__ == '__main__':
    main()
