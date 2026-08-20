import unittest

import zmq

from harvester_dashboard.status_client import StatusClient


class StatusModeTest(unittest.TestCase):
    def test_unreachable_returns_unknown(self):
        self.assertEqual(StatusClient.source_mode_of(None), 'unknown')

    def test_simulation_via_active_profile(self):
        response = {'schema_version': 1, 'active_profile': 'simulation'}
        self.assertEqual(StatusClient.source_mode_of(response), 'simulation')

    def test_hardware_via_source_mode(self):
        response = {'source_mode': 'hardware'}
        self.assertEqual(StatusClient.source_mode_of(response), 'hardware')

    def test_hardware_via_active_profile_substring(self):
        response = {'active_profile': 'orin_hardware_v2'}
        self.assertEqual(StatusClient.source_mode_of(response), 'hardware')

    def test_stream_headers_fallback(self):
        response = {'streams': {
            'v1/camera/cutter/rgb': {'source_mode': 'hardware'},
            'v1/lidar/raw': {'source_mode': 'hardware'},
        }}
        self.assertEqual(StatusClient.source_mode_of(response), 'hardware')

    def test_mixed_streams_stay_unknown_for_maintenance(self):
        response = {'streams': {
            'v1/camera/cutter/rgb': {'source_mode': 'hardware'},
            'v1/lidar/raw': {'source_mode': 'simulation'},
        }}
        self.assertEqual(StatusClient.source_mode_of(response), 'unknown')


class StatusRoundTripTest(unittest.TestCase):
    def test_query_round_trip_against_fake_rep(self):
        import json
        import threading
        context = zmq.Context()
        rep = context.socket(zmq.REP)
        rep.setsockopt(zmq.LINGER, 0)
        endpoint = 'inproc://status-{}'.format(id(self))
        rep.bind(endpoint)
        client = StatusClient(endpoint, timeout_ms=1000, context=context)
        response = {'schema_version': 1, 'active_profile': 'simulation',
                    'calibration_revision': {'cutter': 'c1'},
                    'streams': {}, 'dropped_packets': {},
                    'recording': {'enabled': False}, 'capabilities': {}}

        def serve():
            rep.recv()
            rep.send(json.dumps(response).encode('utf-8'))

        thread = threading.Thread(target=serve)
        thread.start()
        result = client.query()
        thread.join(timeout=2)
        self.assertEqual(result, response)
        client.close()
        rep.close(0)
        context.term()

    def test_query_returns_none_when_no_reply(self):
        context = zmq.Context()
        client = StatusClient(
            'inproc://status-dead-{}'.format(id(self)), timeout_ms=150,
            context=context)
        self.assertIsNone(client.query())
        self.assertIsNone(client.query())   # REQ socket rebuilt cleanly
        client.close()
        context.term()


import zmq


if __name__ == '__main__':
    unittest.main()
