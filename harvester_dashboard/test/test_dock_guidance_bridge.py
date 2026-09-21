"""Bridge-level docking safety-guidance tests (speed, staleness, hysteresis).

These exercise the render-only guidance path in ``DashboardBridge`` using a
monkeypatched ``time.monotonic`` so the least-squares window, the staleness
threshold and the debounce are all deterministic.
"""

import math
import unittest

try:
    from PySide2.QtGui import QGuiApplication
    _QT_OK = True
except ImportError:  # pragma: no cover - headless without Qt
    _QT_OK = False

from harvester_dashboard.config import DashboardConfig
from harvester_dashboard.model.telemetry_model import TelemetryModel
from harvester_dashboard.model.target_model import AnnotationState
from harvester_dashboard.protocol_shim import ensure_contract_importable
from harvester_dashboard.safety_guidance import SafetyConfig

ensure_contract_importable()


class _FakeClock:
    def __init__(self, start=1000.0):
        self.value = start

    def __call__(self):
        return self.value

    def advance(self, seconds):
        self.value += seconds


@unittest.skipUnless(_QT_OK, 'PySide2 unavailable')
class DockGuidanceBridgeTest(unittest.TestCase):
    def setUp(self):
        import harvester_dashboard.bridge as bridge_module

        self.app = QGuiApplication.instance() or QGuiApplication(
            ['dock-guidance-bridge'])
        self.clock = _FakeClock()
        # The model stamps receipt times with its own clock; share the fake one
        # so the bridge's staleness/slope see a single consistent domain.
        self.model = TelemetryModel(clock=self.clock)
        self.bridge_module = bridge_module
        self._real_monotonic = bridge_module.time.monotonic
        # A fixed, permissive config so the physics is easy to reason about.
        self.cfg = SafetyConfig(
            a_max_m_s2=0.1, latency_s=0.3, warn_margin=0.7,
            warn_ttc_s=4.0, danger_ttc_s=1.5,
            warn_distance_m=1.0, danger_distance_m=0.3,
            stationary_epsilon_cm_s=0.5, stale_s=2.0,
            debounce_s=0.4, speed_ema_alpha=1.0)
        self.bridge = bridge_module.DashboardBridge(
            DashboardConfig(status_endpoint='', annotation_endpoint=''),
            self.model, AnnotationState(), safety_config=self.cfg)
        bridge_module.time.monotonic = self.clock
        self.bridge._guidance_state_since = self.clock()

    def tearDown(self):
        self.bridge_module.time.monotonic = self._real_monotonic

    def _feed(self, distance_m, valid=True, advance=0.0):
        if advance:
            self.clock.advance(advance)
        from helpers import json_packet
        self.model.ingest_frames(json_packet('v1/range/docking', [
            {'telemetry_key': 'center_line', 'distance_m': distance_m,
             'valid': valid}], sequence=(int(self.clock()) % 100) + 1))
        self.bridge.refresh()

    def _feed_approach(self, start_m, step_m, steps, dt=0.1):
        distance = start_m
        for index in range(steps):
            self._feed(distance, advance=(0.0 if index == 0 else dt))
            distance -= step_m

    def test_boots_no_data(self):
        self.assertEqual(self.bridge.dockSafetyState, 'no_data')
        self.assertTrue(math.isnan(self.bridge.dockCenterDistanceM))

    def test_approaching_series_reports_closing_speed(self):
        # 50 cm/s closing: drop 5 cm every 0.1 s, keeping enough samples.
        self._feed_approach(3.0, 0.05, 6)
        speed = self.bridge.dockSpeedCmS
        self.assertTrue(math.isfinite(speed))
        # A shrinking gap is a positive closing speed (~50 cm/s).
        self.assertGreater(speed, 40.0)
        self.assertLess(speed, 60.0)

    def test_slow_far_approach_is_safe(self):
        self._feed_approach(5.0, 0.01, 8)
        self.assertEqual(self.bridge.dockSafetyState, 'safe')
        self.assertTrue(math.isfinite(self.bridge.dockMaxSpeedCmS))

    def test_fast_close_approach_reaches_danger(self):
        # Very fast closing, already close: stopping distance cannot be met.
        self._feed_approach(0.5, 0.05, 8)
        self.assertEqual(self.bridge.dockSafetyState, 'danger')
        self.assertTrue(math.isfinite(self.bridge.dockStopDistanceM))

    def test_inside_danger_floor_is_danger(self):
        self._feed(0.2)
        self.assertEqual(self.bridge.dockSafetyState, 'danger')

    def test_danger_does_not_become_safe_when_stream_pauses(self):
        # A fast approach at a gap where only the closing speed drives DANGER
        # (well outside the absolute danger floor).  If the range stream pauses
        # long enough to empty the 1.5 s fit window but not to hit stale_s, the
        # HUD must NOT invent a 0 cm/s speed and flip to SAFE (a false green).
        self._feed_approach(4.0, 0.1, 8)
        self.assertEqual(self.bridge.dockSafetyState, 'danger')
        for _ in range(int((self.cfg.stale_s - 0.1) / 0.15)):
            self.clock.advance(0.15)
            self.bridge.refresh()
            self.assertNotEqual(self.bridge.dockSafetyState, 'safe',
                                'DANGER flipped to SAFE on a stalled stream')
            self.assertNotIn('approach clear', self.bridge.dockGuidanceText)

    def test_stale_range_returns_to_no_data_immediately(self):
        # Two samples + a debounce-safe hold so the state leaves NO_DATA first.
        self._feed(2.0)
        self._feed(2.0, advance=0.5)
        self.assertNotEqual(self.bridge.dockSafetyState, 'no_data')
        # Advance past stale_s without new data: no_data bypasses debounce.
        self.clock.advance(self.cfg.stale_s + 0.1)
        self.bridge.refresh()
        self.assertEqual(self.bridge.dockSafetyState, 'no_data')
        self.assertIn('awaiting', self.bridge.dockGuidanceText)

    def test_invalid_range_returns_no_data(self):
        self._feed(2.0)
        self._feed(2.0, valid=False, advance=0.1)
        self.assertEqual(self.bridge.dockSafetyState, 'no_data')

    def test_guidance_message_matches_shown_state(self):
        # Whatever state is shown, the message must not contradict it (never a
        # STOP message while the state is warn/safe/no_data, etc.).
        distance = 5.0
        for index in range(12):
            self._feed(distance, advance=(0.0 if index == 0 else 0.1))
            distance -= 0.1
            state = self.bridge.dockSafetyState
            text = self.bridge.dockGuidanceText
            if state != 'danger':
                self.assertNotIn('STOP', text)
            if state == 'no_data':
                self.assertIn('awaiting', text)


if __name__ == '__main__':
    unittest.main()
