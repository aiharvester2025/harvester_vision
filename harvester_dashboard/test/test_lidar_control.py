"""Tests for the opt-in LiDAR standby/normal control publisher.

This is the one place the dashboard writes a sensor command, so its scope is
asserted directly: disabled by default, only ever a boolean ``enabled``, and
de-duplicated.
"""

import unittest

from harvester_dashboard.lidar_control import LidarControlPublisher


class DisabledByDefaultTest(unittest.TestCase):
    def test_empty_endpoint_creates_no_socket(self):
        publisher = LidarControlPublisher('')
        self.assertFalse(publisher.enabled)
        self.assertIsNone(publisher.socket)
        # A call on a disabled publisher is a safe no-op.
        self.assertFalse(publisher.set_enabled(True))

    def test_config_flag_reflects_endpoint(self):
        from harvester_dashboard.config import DashboardConfig
        self.assertFalse(DashboardConfig().lidar_control_enabled)
        self.assertTrue(DashboardConfig(
            lidar_control_endpoint='tcp://127.0.0.1:5571').lidar_control_enabled)


class PayloadTest(unittest.TestCase):
    def _publisher(self):
        publisher = LidarControlPublisher('')
        sent = []

        class _Socket:
            def send_json(self, payload, flags=0):
                sent.append((payload, flags))

            def close(self, linger=0):
                pass

        publisher.socket = _Socket()
        return publisher, sent

    def test_sends_only_boolean_enabled(self):
        publisher, sent = self._publisher()
        self.assertTrue(publisher.set_enabled(True))
        self.assertTrue(publisher.set_enabled(False))
        self.assertEqual([p for p, _ in sent],
                         [{'enabled': True}, {'enabled': False}])
        for payload, _ in sent:
            self.assertEqual(set(payload), {'enabled'})
            self.assertIsInstance(payload['enabled'], bool)

    def test_consecutive_identical_values_are_deduplicated(self):
        publisher, sent = self._publisher()
        publisher.set_enabled(True)
        self.assertFalse(publisher.set_enabled(True))   # no second write
        publisher.set_enabled(False)
        self.assertEqual(len(sent), 2)


if __name__ == '__main__':
    unittest.main()
