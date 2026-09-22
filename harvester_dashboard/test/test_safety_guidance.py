"""Unit tests for the docking approach safety-guidance model (pure Python).

No Qt, no sockets, no simulation: the model is a stateless pure-function suite,
so these tests run anywhere the dashboard package imports.
"""

import json
import math
import os
import tempfile
import unittest

from harvester_dashboard.safety_guidance import (
    DANGER, NO_DATA, SAFE, WARN, Guidance, SafetyConfig, evaluate,
    state_message, _max_safe_speed_cm_s, _stop_distance_m)


class SafetyConfigTest(unittest.TestCase):
    def test_defaults(self):
        cfg = SafetyConfig()
        self.assertAlmostEqual(cfg.a_max_m_s2, 0.10)
        self.assertAlmostEqual(cfg.latency_s, 0.30)
        self.assertAlmostEqual(cfg.warn_margin, 0.7)

    def test_missing_file_returns_defaults(self):
        cfg = SafetyConfig.load('/nonexistent/safety.json')
        self.assertEqual(cfg, SafetyConfig())

    def test_none_path_returns_defaults(self):
        self.assertEqual(SafetyConfig.load(None), SafetyConfig())

    def test_malformed_json_returns_defaults(self):
        handle = tempfile.NamedTemporaryFile(
            mode='w', suffix='.json', delete=False)
        handle.write('{not json')
        handle.close()
        self.addCleanup(os.unlink, handle.name)
        with self.assertLogs('harvester_dashboard.safety_guidance',
                             level='WARNING'):
            cfg = SafetyConfig.load(handle.name)
        self.assertEqual(cfg, SafetyConfig())

    def test_missing_file_logs_no_warning(self):
        # An absent tuning file is a valid configuration (defaults); it must not
        # spam a warning on every startup.  (assertNoLogs is 3.10+; this repo
        # runs 3.8, so capture with a handler.)
        import logging
        records = []

        class _Capture(logging.Handler):
            def emit(self, record):
                records.append(record)

        logger = logging.getLogger('harvester_dashboard.safety_guidance')
        handler = _Capture()
        logger.addHandler(handler)
        try:
            cfg = SafetyConfig.load('/nonexistent/safety.json')
        finally:
            logger.removeHandler(handler)
        self.assertEqual(cfg, SafetyConfig())
        self.assertEqual([r for r in records if r.levelno >= logging.WARNING],
                         [])

    def test_malformed_file_logs_warning(self):
        handle = tempfile.NamedTemporaryFile(
            mode='w', suffix='.json', delete=False)
        handle.write('{not json')
        handle.close()
        self.addCleanup(os.unlink, handle.name)
        with self.assertLogs('harvester_dashboard.safety_guidance',
                             level='WARNING') as captured:
            SafetyConfig.load(handle.name)
        self.assertIn('malformed', captured.output[0])

    def test_non_object_file_logs_warning(self):
        handle = tempfile.NamedTemporaryFile(
            mode='w', suffix='.json', delete=False)
        json.dump([1, 2, 3], handle)
        handle.close()
        self.addCleanup(os.unlink, handle.name)
        with self.assertLogs('harvester_dashboard.safety_guidance',
                             level='WARNING') as captured:
            SafetyConfig.load(handle.name)
        self.assertIn('JSON object', captured.output[0])

    def test_non_object_json_returns_defaults(self):
        handle = tempfile.NamedTemporaryFile(
            mode='w', suffix='.json', delete=False)
        json.dump([1, 2, 3], handle)
        handle.close()
        self.addCleanup(os.unlink, handle.name)
        self.assertEqual(SafetyConfig.load(handle.name), SafetyConfig())

    def test_partial_override(self):
        handle = tempfile.NamedTemporaryFile(
            mode='w', suffix='.json', delete=False)
        json.dump({'warn_distance_m': 2.5}, handle)
        handle.close()
        self.addCleanup(os.unlink, handle.name)
        cfg = SafetyConfig.load(handle.name)
        self.assertAlmostEqual(cfg.warn_distance_m, 2.5)
        # Unspecified keys keep their defaults.
        self.assertAlmostEqual(cfg.a_max_m_s2, 0.10)

    def test_sanitizes_out_of_range_values(self):
        # a_max <= 0 would divide by zero; margins are clamped to [0, 1].
        handle = tempfile.NamedTemporaryFile(
            mode='w', suffix='.json', delete=False)
        json.dump({'a_max_m_s2': 0.0, 'latency_s': -1.0,
                   'warn_margin': 5.0, 'speed_ema_alpha': -2.0,
                   'debounce_s': -3.0, 'stale_s': -4.0}, handle)
        handle.close()
        self.addCleanup(os.unlink, handle.name)
        cfg = SafetyConfig.load(handle.name)
        self.assertGreaterEqual(cfg.a_max_m_s2, 1e-3)
        self.assertGreaterEqual(cfg.latency_s, 0.0)
        self.assertLessEqual(cfg.warn_margin, 1.0)
        self.assertGreaterEqual(cfg.warn_margin, 0.0)
        self.assertLessEqual(cfg.speed_ema_alpha, 1.0)
        self.assertGreaterEqual(cfg.speed_ema_alpha, 0.0)
        self.assertGreaterEqual(cfg.debounce_s, 0.0)
        self.assertGreaterEqual(cfg.stale_s, 0.0)

    def test_warn_margin_zero_is_floored_positive(self):
        # warn_margin = 0 would otherwise make the WARN speed bound and the
        # recommendation 0 cm/s ("SLOW to 0 cm/s").
        handle = tempfile.NamedTemporaryFile(
            mode='w', suffix='.json', delete=False)
        json.dump({'warn_margin': 0.0}, handle)
        handle.close()
        self.addCleanup(os.unlink, handle.name)
        cfg = SafetyConfig.load(handle.name)
        self.assertGreater(cfg.warn_margin, 0.0)
        g = evaluate(5.0, 5.0, cfg)
        self.assertGreater(g.recommended_speed_cm_s, 0.0)

    def test_danger_bands_clamped_inside_warn_bands(self):
        # An inverted file (danger > warn) must not remove the TTC/distance
        # escalations: the danger band is pulled inside the warn band.
        handle = tempfile.NamedTemporaryFile(
            mode='w', suffix='.json', delete=False)
        json.dump({'warn_ttc_s': 1.0, 'danger_ttc_s': 10.0,
                   'warn_distance_m': 2.0, 'danger_distance_m': 0.5}, handle)
        handle.close()
        self.addCleanup(os.unlink, handle.name)
        cfg = SafetyConfig.load(handle.name)
        self.assertLessEqual(cfg.danger_ttc_s, cfg.warn_ttc_s)
        self.assertLessEqual(cfg.danger_distance_m, cfg.warn_distance_m)

    def test_warn_message_always_leads_with_slow(self):
        # A WARN banner must never carry a bare "…s to impact" or a STOP word.
        cfg = SafetyConfig(warn_margin=1e-3)
        for speed, distance in ((20.0, 1.0), (50.0, 0.6), (5.0, 0.8)):
            g = evaluate(speed, distance, cfg)
            if g.state == WARN:
                self.assertTrue(g.message.startswith('SLOW'), g.message)
                self.assertNotIn('STOP', g.message)

    def test_non_numeric_value_falls_back_to_defaults(self):
        handle = tempfile.NamedTemporaryFile(
            mode='w', suffix='.json', delete=False)
        json.dump({'a_max_m_s2': 'fast'}, handle)
        handle.close()
        self.addCleanup(os.unlink, handle.name)
        self.assertEqual(SafetyConfig.load(handle.name), SafetyConfig())


class PhysicsTest(unittest.TestCase):
    def test_stop_distance_formula(self):
        cfg = SafetyConfig()
        # 30 cm/s = 0.3 m/s: 0.3*0.3 + 0.09/(2*0.1) = 0.09 + 0.45 = 0.54 m.
        self.assertAlmostEqual(_stop_distance_m(30.0, cfg), 0.54, places=6)

    def test_stop_distance_zero_speed(self):
        self.assertAlmostEqual(_stop_distance_m(0.0, SafetyConfig()), 0.0)

    def test_max_safe_speed_is_inverse_of_stop_distance(self):
        cfg = SafetyConfig()
        for distance in (0.3, 0.5, 1.0, 2.0, 5.0):
            v_max = _max_safe_speed_cm_s(distance, cfg) / 100.0
            # Stopping from exactly v_max must need exactly the gap.
            self.assertAlmostEqual(
                _stop_distance_m(v_max * 100.0, cfg), distance, places=6)

    def test_max_safe_speed_monotonic_increasing(self):
        cfg = SafetyConfig()
        previous = -1.0
        for distance in (0.0, 0.1, 0.5, 1.0, 2.0, 10.0):
            value = _max_safe_speed_cm_s(distance, cfg)
            self.assertGreater(value, previous)
            previous = value


class EvaluateStateTest(unittest.TestCase):
    def setUp(self):
        self.cfg = SafetyConfig()

    def test_missing_inputs_are_no_data(self):
        self.assertEqual(evaluate(None, 1.0, self.cfg).state, NO_DATA)
        self.assertEqual(evaluate(10.0, None, self.cfg).state, NO_DATA)

    def test_stale_is_no_data(self):
        g = evaluate(10.0, 1.0, self.cfg, stale=True)
        self.assertEqual(g.state, NO_DATA)
        self.assertIn('awaiting', g.message)

    def test_zero_or_negative_distance_is_no_data(self):
        self.assertEqual(evaluate(10.0, 0.0, self.cfg).state, NO_DATA)
        self.assertEqual(evaluate(10.0, -1.0, self.cfg).state, NO_DATA)

    def test_non_finite_distance_is_no_data_never_safe(self):
        # NaN compares false against every bound, so an unguarded NaN gap would
        # fall through every DANGER/WARN test and be reported as a false-green
        # SAFE.  A non-finite reading must be NO_DATA.
        for distance in (float('nan'), float('inf'), -float('inf')):
            with self.subTest(distance=distance):
                g = evaluate(0.0, distance, self.cfg)
                self.assertEqual(g.state, NO_DATA)
                self.assertNotEqual(g.state, SAFE)

    def test_non_finite_speed_is_no_data(self):
        for speed in (float('nan'), float('inf'), -float('inf')):
            with self.subTest(speed=speed):
                self.assertEqual(evaluate(speed, 1.0, self.cfg).state, NO_DATA)

    def test_non_finite_never_reports_safe(self):
        # Exhaustive: no combination of non-finite inputs may be SAFE.
        values = [float('nan'), float('inf'), -float('inf'), 0.0, 1.0, None]
        for speed in values:
            for distance in values:
                state = evaluate(speed, distance, self.cfg).state
                if (speed is not None and distance is not None
                        and not math.isfinite(speed)
                        or (distance is not None and not math.isfinite(distance))):
                    self.assertNotEqual(state, SAFE,
                                        'speed={} distance={}'.format(
                                            speed, distance))

    def test_no_data_never_reports_safe(self):
        # A false green on absent telemetry is the most dangerous failure.
        for speed, distance, stale in ((None, None, False), (10.0, 1.0, True),
                                       (5.0, None, False)):
            self.assertNotEqual(
                evaluate(speed, distance, self.cfg, stale=stale).state, SAFE)

    def test_stationary_far_is_safe(self):
        self.assertEqual(evaluate(0.0, 3.0, self.cfg).state, SAFE)

    def test_stationary_inside_danger_distance_is_danger(self):
        # Too close to linger: danger by the absolute floor, no motion needed.
        self.assertEqual(evaluate(0.0, 0.2, self.cfg).state, DANGER)

    def test_moving_away_is_safe(self):
        # Beyond the warn standoff, moving away is always safe.
        self.assertEqual(evaluate(-40.0, 3.0, self.cfg).state, SAFE)

    def test_far_slow_is_safe(self):
        self.assertEqual(evaluate(5.0, 5.0, self.cfg).state, SAFE)

    def test_warn_by_distance(self):
        # Inside warn_distance (1.0) but not danger, slowly approaching.
        g = evaluate(1.0, 0.8, self.cfg)
        self.assertEqual(g.state, WARN)

    def test_warn_by_ttc(self):
        # fast-but-far: TTC 3 s < warn_ttc 4 s but above danger_ttc 1.5 s.
        g = evaluate(20.0, 0.6, self.cfg)
        self.assertEqual(g.state, WARN)

    def test_danger_by_ttc(self):
        # TTC 1 s < danger_ttc 1.5 s.
        g = evaluate(50.0, 0.5, self.cfg)
        self.assertEqual(g.state, DANGER)

    def test_danger_by_stopping_distance(self):
        # 50 cm/s at 0.5 m needs ~0.9 m to stop: danger.
        g = evaluate(50.0, 0.5, self.cfg)
        self.assertEqual(g.state, DANGER)
        self.assertIn('STOP', g.message)

    def test_danger_absolute_distance_floor(self):
        self.assertEqual(evaluate(0.0, 0.25, self.cfg).state, DANGER)

    def test_recommended_speed_decreases_with_distance(self):
        # Both inside the warn standoff so both are WARN and carry a
        # recommendation; the closer gap must recommend the slower speed.
        near = evaluate(20.0, 0.5, self.cfg)
        far = evaluate(20.0, 0.95, self.cfg)
        self.assertEqual(near.state, WARN)
        self.assertEqual(far.state, WARN)
        self.assertLess(near.recommended_speed_cm_s,
                        far.recommended_speed_cm_s)

    def test_warn_margin_scales_recommendation(self):
        cfg_low = SafetyConfig(warn_margin=0.5)
        cfg_high = SafetyConfig(warn_margin=0.9)
        low = evaluate(20.0, 0.6, cfg_low)
        high = evaluate(20.0, 0.6, cfg_high)
        if (low.recommended_speed_cm_s is not None
                and high.recommended_speed_cm_s is not None):
            self.assertLess(low.recommended_speed_cm_s,
                            high.recommended_speed_cm_s)

    def test_safe_has_no_recommended_speed(self):
        self.assertIsNone(evaluate(5.0, 5.0, self.cfg).recommended_speed_cm_s)

    def test_danger_imminent_message(self):
        g = evaluate(5.0, 0.1, self.cfg)
        self.assertEqual(g.state, DANGER)
        self.assertIn('imminent', g.message)


class StateMessageTest(unittest.TestCase):
    def test_state_message_matches_held_state(self):
        # During debounce the displayed message must match the shown colour,
        # never the candidate's (no "STOP" text on an orange banner).
        candidate = evaluate(50.0, 0.5, SafetyConfig())  # candidate DANGER
        self.assertEqual(candidate.state, DANGER)
        held_warn = state_message(WARN, candidate)
        self.assertIn('SLOW', held_warn)
        self.assertNotIn('STOP', held_warn)

    def test_state_message_danger(self):
        g = evaluate(5.0, 5.0, SafetyConfig())
        self.assertEqual(state_message(DANGER, g), 'STOP — collision risk')

    def test_state_message_no_data(self):
        g = Guidance(NO_DATA, None, 0.0, None, None, None, None, '')
        self.assertIn('awaiting', state_message(NO_DATA, g))


if __name__ == '__main__':
    unittest.main()
