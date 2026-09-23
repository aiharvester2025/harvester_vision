"""Bridge-level cutter safety-guidance tests (speed, staleness, phase, confirm).

Exercises the render-only cutter guidance path in ``DashboardBridge`` with a
monkeypatched ``time.monotonic`` so the least-squares window, the staleness
threshold, the debounce and the cut-sequence phase are all deterministic.
Mirrors ``test_dock_guidance_bridge.py``.
"""

import math
import unittest

try:
    from PySide2.QtGui import QGuiApplication
    _QT_OK = True
except ImportError:  # pragma: no cover - headless without Qt
    _QT_OK = False

from harvester_dashboard.config import DashboardConfig
from harvester_dashboard.cutter_safety_guidance import (
    PHASE_APPROACH, PHASE_ALIGN, PHASE_OPEN, CutterConfig)
from harvester_dashboard.model.telemetry_model import TelemetryModel
from harvester_dashboard.model.target_model import AnnotationState
from harvester_dashboard.protocol_shim import ensure_contract_importable

ensure_contract_importable()


class _FakeClock:
    def __init__(self, start=1000.0):
        self.value = start

    def __call__(self):
        return self.value

    def advance(self, seconds):
        self.value += seconds


@unittest.skipUnless(_QT_OK, 'PySide2 unavailable')
class CutterGuidanceBridgeTest(unittest.TestCase):
    def setUp(self):
        import harvester_dashboard.bridge as bridge_module

        self.app = QGuiApplication.instance() or QGuiApplication(
            ['cutter-guidance-bridge'])
        self.clock = _FakeClock()
        self.model = TelemetryModel(clock=self.clock)
        self.bridge_module = bridge_module
        self._real_monotonic = bridge_module.time.monotonic
        # A fixed, permissive config so the physics/offset is easy to reason
        # about.  speed_ema_alpha=1.0 makes the smoothed speed equal the raw.
        self.cfg = CutterConfig(
            sensor_to_tip_offset_m=0.19,
            a_max_m_s2=0.1, latency_s=0.3, warn_margin=0.7,
            warn_clearance_m=0.30, danger_clearance_m=0.10,
            ready_standoff_m=0.20, advance_distance_m=0.05,
            align_tolerance_m=0.05,
            stationary_epsilon_cm_s=0.5, stale_s=2.0,
            debounce_s=0.4, speed_ema_alpha=1.0)
        self.bridge = bridge_module.DashboardBridge(
            DashboardConfig(status_endpoint='', annotation_endpoint=''),
            self.model, AnnotationState(), cutter_config=self.cfg)
        bridge_module.time.monotonic = self.clock
        self.bridge._cutter_state_since = self.clock()

    def tearDown(self):
        self.bridge_module.time.monotonic = self._real_monotonic

    def _feed(self, distance_m, valid=True, advance=0.0):
        """Feed a raw cutter range reading (m) and refresh the bridge."""
        if advance:
            self.clock.advance(advance)
        from helpers import json_packet
        self.model.ingest_frames(json_packet('v1/range/cutter', {
            'telemetry_key': 'cutter_forward', 'distance_m': distance_m,
            'valid': valid}, sequence=(int(self.clock()) % 100) + 1))
        self.bridge.refresh()

    def _feed_approach(self, start_m, step_m, steps, dt=0.1):
        distance = start_m
        for index in range(steps):
            self._feed(distance, advance=(0.0 if index == 0 else dt))
            distance -= step_m

    def test_boots_no_data_and_idle(self):
        # Initial state must be NO_DATA / IDLE (never a false green).
        self.assertEqual(self.bridge.cutterSafetyState, 'no_data')
        self.assertEqual(self.bridge.cutterPhase, 'idle')
        self.assertTrue(math.isnan(self.bridge.cutterClearanceM))
        self.assertTrue(math.isnan(self.bridge.cutterRawRangeM))

    def test_clearance_applies_sensor_to_tip_offset(self):
        # Raw 0.50 m with a 0.19 m offset -> 0.31 m tip clearance.
        self._feed(0.50)
        self.assertTrue(math.isfinite(self.bridge.cutterClearanceM))
        self.assertAlmostEqual(self.bridge.cutterClearanceM, 0.31, places=6)
        self.assertAlmostEqual(self.bridge.cutterRawRangeM, 0.50, places=6)

    def test_approaching_series_reports_closing_speed(self):
        # 50 cm/s closing: drop 5 cm every 0.1 s.
        self._feed_approach(0.80, 0.05, 6)
        speed = self.bridge.cutterSpeedSmoothedCmS
        self.assertTrue(math.isfinite(speed))
        self.assertGreater(speed, 40.0)
        self.assertLess(speed, 60.0)

    def test_fast_close_approach_reaches_danger(self):
        # Raw range 0.30 -> clearance 0.11, closing 50 cm/s: the stopping
        # distance cannot be met.
        self._feed_approach(0.40, 0.02, 8)
        self.assertEqual(self.bridge.cutterSafetyState, 'danger')

    def test_inside_danger_floor_is_danger(self):
        # Raw 0.25 -> clearance 0.06 < danger_clearance 0.10.
        self._feed(0.25)
        self.assertEqual(self.bridge.cutterSafetyState, 'danger')

    def test_warn_inside_warn_clearance(self):
        # Raw 0.45 -> clearance 0.26, inside warn_clearance 0.30 (stationary).
        self._feed(0.45)
        self._feed(0.45, advance=0.5)
        self.assertEqual(self.bridge.cutterSafetyState, 'warn')

    def test_invalid_range_returns_no_data(self):
        self._feed(0.50)
        self._feed(0.50, valid=False, advance=0.1)
        self.assertEqual(self.bridge.cutterSafetyState, 'no_data')

    def test_stale_range_returns_to_no_data_immediately(self):
        self._feed(0.60)
        self._feed(0.60, advance=0.5)
        self.assertNotEqual(self.bridge.cutterSafetyState, 'no_data')
        # no_data bypasses the debounce.
        self.clock.advance(self.cfg.stale_s + 0.1)
        self.bridge.refresh()
        self.assertEqual(self.bridge.cutterSafetyState, 'no_data')

    def test_guidance_message_matches_shown_state(self):
        distance = 1.20
        for index in range(12):
            self._feed(distance, advance=(0.0 if index == 0 else 0.1))
            distance -= 0.05
            state = self.bridge.cutterSafetyState
            text = self.bridge.cutterGuidanceText
            if state != 'danger':
                self.assertNotIn('STOP', text)
            if state == 'no_data':
                self.assertIn('awaiting', text)

    def test_phase_advances_to_align_when_settled_in_band(self):
        # Raw 0.39 -> clearance 0.20 (ready standoff), stationary.  Two fresh
        # samples clear the debounce out of the initial NO_DATA state.
        self._feed(0.39)
        self._feed(0.39, advance=0.5)
        self.assertEqual(self.bridge.cutterPhase, PHASE_ALIGN)
        self.assertIn('STOP', self.bridge.cutterPhaseText)

    def test_phase_stays_approach_while_closing_fast(self):
        # Inside the ready band but still closing fast: not ready to cut.  The
        # phase must be APPROACH and the closing speed must exceed the settle
        # bound, which is the exact condition that withholds ALIGN.
        self._feed_approach(0.44, 0.02, 5)
        self.assertEqual(self.bridge.cutterPhase, PHASE_APPROACH)
        self.assertGreater(
            self.bridge.cutterSpeedSmoothedCmS,
            self.cfg.warn_margin * self.bridge.cutterMaxSpeedCmS)

    def test_phase_never_aligns_inside_danger_floor(self):
        # The tip inside the danger floor is DANGER; the cut sequence must not
        # simultaneously advertise "STOP — ready to cut".
        self._feed(0.25)   # raw 0.25 -> clearance 0.06 < danger 0.10
        self._feed(0.25, advance=0.5)
        self.assertEqual(self.bridge.cutterSafetyState, 'danger')
        self.assertEqual(self.bridge.cutterPhase, PHASE_APPROACH)

    def test_confirm_phase_advances_and_is_blocked_when_danger(self):
        self._feed(0.39)
        self._feed(0.39, advance=0.5)
        self.assertEqual(self.bridge.cutterPhase, PHASE_ALIGN)
        self.assertTrue(self.bridge.cutterCanConfirm)
        self.bridge.cutter_confirm_phase()
        self.assertEqual(self.bridge.cutterPhase, PHASE_OPEN)
        # Now drive the tip into DANGER: confirm must be a no-op.
        self._feed(0.25)
        self.assertEqual(self.bridge.cutterSafetyState, 'danger')
        self.assertFalse(self.bridge.cutterCanConfirm)
        self.bridge.cutter_confirm_phase()
        self.assertEqual(self.bridge.cutterPhase, PHASE_OPEN)

    def test_confirm_phase_noop_on_no_data(self):
        self.assertEqual(self.bridge.cutterPhase, 'idle')
        self.bridge.cutter_confirm_phase()
        # From idle the confirm would start at align, but NO_DATA blocks it.
        self.assertEqual(self.bridge.cutterPhase, 'idle')
        self.assertFalse(self.bridge.cutterCanConfirm)

    def test_operator_phase_holds_through_dropout(self):
        self._feed(0.39)
        self._feed(0.39, advance=0.5)
        self.bridge.cutter_confirm_phase()  # align -> open
        self.assertEqual(self.bridge.cutterPhase, PHASE_OPEN)
        # A stale range must not reset the operator's in-progress step.
        self.clock.advance(self.cfg.stale_s + 0.1)
        self.bridge.refresh()
        self.assertEqual(self.bridge.cutterPhase, PHASE_OPEN)

    def test_phase_text_tracks_phase_immediately(self):
        # The banner prompt is derived from the live phase, so it must never
        # lag the phase the CONFIRM STEP button acts on (even while the
        # clearance *state* is still debouncing out of NO_DATA).
        self._feed(0.39)
        self.assertEqual(self.bridge.cutterPhase, PHASE_APPROACH)
        self.assertIn('approach', self.bridge.cutterPhaseText)
        self._feed(0.39, advance=0.5)
        self.assertEqual(self.bridge.cutterPhase, PHASE_ALIGN)
        self.assertIn('STOP', self.bridge.cutterPhaseText)

    def test_clearance_rows_present_and_shaped(self):
        rows = self.bridge.cutterSafetyRow
        keys = [row['key'] for row in rows]
        self.assertEqual(keys, ['state', 'phase', 'clearance', 'speed',
                                'ttc', 'max_speed'])
        by_key = {r['key']: r for r in rows}
        self.assertEqual(by_key['state']['value'], 'NO DATA')
        self.assertFalse(by_key['state']['valid'])
        for row in rows:
            self.assertIn('label', row)
            self.assertIn('value', row)

    def test_float_properties_are_nan_without_data(self):
        for value in (self.bridge.cutterClearanceM,
                      self.bridge.cutterRawRangeM,
                      self.bridge.cutterSpeedSmoothedCmS,
                      self.bridge.cutterMaxSpeedCmS,
                      self.bridge.cutterStopDistanceM,
                      self.bridge.cutterTtcS):
            self.assertTrue(math.isnan(value), value)

    def test_cutter_path_never_writes_a_socket(self):
        # Render-only proof: driving the cutter guidance path (ranges + refresh
        # + operator confirm) must perform NO socket I/O.  Patch zmq.Socket
        # send methods at the module the bridge would use if it ever tried.
        import zmq

        calls = []
        original_send = zmq.Socket.send_multipart

        def spy(self, *args, **kwargs):
            calls.append(args)
            return original_send(self, *args, **kwargs)

        zmq.Socket.send_multipart = spy
        try:
            for i in range(6):
                self._feed(0.60 - 0.02 * i, advance=(0.0 if i == 0 else 0.1))
            self.bridge.refresh()
            self.bridge.cutter_confirm_phase()
            self.bridge.refresh()
        finally:
            zmq.Socket.send_multipart = original_send
        self.assertEqual(calls, [], 'cutter guidance path wrote to a socket')


if __name__ == '__main__':
    unittest.main()
