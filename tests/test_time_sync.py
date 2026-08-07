from datetime import timedelta
import unittest

from time_sync import capture_timestamp_us, depthai_timestamp_to_monotonic_us


class TimestampTests(unittest.TestCase):
    def test_depthai_timestamp_converts_to_integer_microseconds(self):
        self.assertEqual(
            depthai_timestamp_to_monotonic_us(timedelta(seconds=12, microseconds=345)),
            12_000_345,
        )

    def test_capture_timestamp_uses_sampled_utc_monotonic_offset(self):
        timestamp = timedelta(seconds=10, microseconds=250)
        self.assertEqual(
            capture_timestamp_us(timestamp, now_monotonic_ns=30_000_000_000, now_utc_ns=1_700_000_030_000_000_000),
            1_700_000_010_000_250,
        )
