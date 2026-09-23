"""Unit tests for the cutter safety-guidance model (pure Python).

No Qt, no sockets, no simulation: the model is a stateless pure-function suite
plus the cut-sequence phase machine, so these tests run anywhere the dashboard
package imports.  Mirrors ``test_safety_guidance.py`` for the docking sibling.
"""

import json
import math
import os
import tempfile
import unittest

from harvester_dashboard.cutter_safety_guidance import (
    DANGER, NO_DATA, SAFE, WARN,
    PHASE_ADVANCE, PHASE_ALIGN, PHASE_APPROACH, PHASE_CUT, PHASE_IDLE,
    PHASE_OPEN, PHASES,
    CutterConfig, CutterGuidance, advance_phase, evaluate, next_phase,
    phase_message, state_message, tip_clearance_m,
    _max_safe_speed_cm_s, _stop_distance_m)


class CutterConfigTest(unittest.TestCase):
    def test_defaults(self):
        cfg = CutterConfig()
        self.assertAlmostEqual(cfg.sensor_to_tip_offset_m, 0.19)
        self.assertAlmostEqual(cfg.a_max_m_s2, 0.10)
        self.assertAlmostEqual(cfg.latency_s, 0.30)
        self.assertAlmostEqual(cfg.warn_margin, 0.7)
        self.assertAlmostEqual(cfg.warn_clearance_m, 0.30)
        self.assertAlmostEqual(cfg.danger_clearance_m, 0.10)
        self.assertAlmostEqual(cfg.ready_standoff_m, 0.20)
        self.assertAlmostEqual(cfg.advance_distance_m, 0.05)
        self.assertAlmostEqual(cfg.align_tolerance_m, 0.05)

    def test_missing_file_returns_defaults(self):
        self.assertEqual(CutterConfig.load('/nonexistent/cutter.json'),
                         CutterConfig())

    def test_none_path_returns_defaults(self):
        self.assertEqual(CutterConfig.load(None), CutterConfig())

    def test_malformed_json_returns_defaults(self):
        handle = tempfile.NamedTemporaryFile(
            mode='w', suffix='.json', delete=False)
        handle.write('{not json')
        handle.close()
        self.addCleanup(os.unlink, handle.name)
        self.assertEqual(CutterConfig.load(handle.name), CutterConfig())

    def test_non_object_json_returns_defaults(self):
        handle = tempfile.NamedTemporaryFile(
            mode='w', suffix='.json', delete=False)
        json.dump([1, 2, 3], handle)
        handle.close()
        self.addCleanup(os.unlink, handle.name)
        self.assertEqual(CutterConfig.load(handle.name), CutterConfig())

    def test_partial_override_keeps_other_defaults(self):
        handle = tempfile.NamedTemporaryFile(
            mode='w', suffix='.json', delete=False)
        json.dump({'sensor_to_tip_offset_m': 0.25, 'ready_standoff_m': 0.35},
                  handle)
        handle.close()
        self.addCleanup(os.unlink, handle.name)
        cfg = CutterConfig.load(handle.name)
        self.assertAlmostEqual(cfg.sensor_to_tip_offset_m, 0.25)
        self.assertAlmostEqual(cfg.ready_standoff_m, 0.35)
        self.assertAlmostEqual(cfg.a_max_m_s2, 0.10)

    def test_shipped_config_matches_defaults(self):
        from harvester_dashboard.cutter_safety_guidance import (
            default_config_path)
        path = default_config_path()
        self.assertTrue(path.exists(), path)
        self.assertEqual(CutterConfig.load(path), CutterConfig())

    def test_sanitizes_zero_a_max(self):
        # A zero a_max would divide-by-zero in the stopping-distance math.
        handle = tempfile.NamedTemporaryFile(
            mode='w', suffix='.json', delete=False)
        json.dump({'a_max_m_s2': 0.0}, handle)
        handle.close()
        self.addCleanup(os.unlink, handle.name)
        cfg = CutterConfig.load(handle.name)
        self.assertGreaterEqual(cfg.a_max_m_s2, 1e-3)

    def test_sanitizes_negative_and_out_of_range(self):
        handle = tempfile.NamedTemporaryFile(
            mode='w', suffix='.json', delete=False)
        json.dump({'latency_s': -1.0, 'warn_margin': 5.0,
                   'speed_ema_alpha': -2.0, 'stale_s': -3.0}, handle)
        handle.close()
        self.addCleanup(os.unlink, handle.name)
        cfg = CutterConfig.load(handle.name)
        self.assertGreaterEqual(cfg.latency_s, 0.0)
        self.assertLessEqual(cfg.warn_margin, 1.0)
        self.assertGreaterEqual(cfg.speed_ema_alpha, 0.0)
        self.assertGreaterEqual(cfg.stale_s, 0.0)

    def test_danger_clearance_is_clamped_inside_warn(self):
        # An inverted config (danger > warn) would make the danger floor
        # unreachable; the loader clamps danger <= warn.
        handle = tempfile.NamedTemporaryFile(
            mode='w', suffix='.json', delete=False)
        json.dump({'warn_clearance_m': 0.10, 'danger_clearance_m': 0.90},
                  handle)
        handle.close()
        self.addCleanup(os.unlink, handle.name)
        cfg = CutterConfig.load(handle.name)
        self.assertLessEqual(cfg.danger_clearance_m, cfg.warn_clearance_m)

    def test_bad_strings_return_defaults(self):
        handle = tempfile.NamedTemporaryFile(
            mode='w', suffix='.json', delete=False)
        json.dump({'a_max_m_s2': 'fast', 'latency_s': 'soon'}, handle)
        handle.close()
        self.addCleanup(os.unlink, handle.name)
        self.assertEqual(CutterConfig.load(handle.name), CutterConfig())


class TipOffsetTest(unittest.TestCase):
    def test_offset_is_subtracted(self):
        # The sensor sits BEHIND the tip, so clearance = range - offset.
        cfg = CutterConfig(sensor_to_tip_offset_m=0.19)
        self.assertAlmostEqual(tip_clearance_m(0.50, cfg), 0.31)
        self.assertAlmostEqual(tip_clearance_m(0.19, cfg), 0.0)

    def test_negative_clearance_allowed(self):
        # Pressing on the object yields a small negative clearance.
        cfg = CutterConfig(sensor_to_tip_offset_m=0.19)
        self.assertLess(tip_clearance_m(0.15, cfg), 0.0)


class CutterEvaluateTest(unittest.TestCase):
    cfg = CutterConfig(
        a_max_m_s2=0.1, latency_s=0.3, warn_margin=0.7,
        warn_clearance_m=0.30, danger_clearance_m=0.10,
        stationary_epsilon_cm_s=0.5, speed_ema_alpha=1.0)

    def test_missing_inputs_are_no_data(self):
        for speed, clear in ((None, 0.5), (10.0, None), (None, None)):
            with self.subTest(speed=speed, clear=clear):
                g = evaluate(speed, clear, self.cfg)
                self.assertEqual(g.state, NO_DATA)

    def test_stale_is_no_data(self):
        g = evaluate(10.0, 0.5, self.cfg, stale=True)
        self.assertEqual(g.state, NO_DATA)

    def test_non_finite_readings_are_no_data(self):
        # NaN/Inf must never be reported as SAFE (every comparison is false).
        for speed, clear in ((float('nan'), 0.5), (10.0, float('nan')),
                             (float('inf'), 0.5), (10.0, float('inf'))):
            with self.subTest(speed=speed, clear=clear):
                g = evaluate(speed, clear, self.cfg)
                self.assertEqual(g.state, NO_DATA)

    def test_inside_danger_floor_is_danger(self):
        g = evaluate(0.0, 0.05, self.cfg)
        self.assertEqual(g.state, DANGER)

    def test_stationary_far_is_safe(self):
        g = evaluate(0.0, 0.60, self.cfg)
        self.assertEqual(g.state, SAFE)

    def test_moving_away_is_safe(self):
        g = evaluate(-20.0, 0.50, self.cfg)
        self.assertEqual(g.state, SAFE)

    def test_stopping_distance_danger(self):
        # Fast closing near the tip: d_stop >= clearance -> DANGER.
        g = evaluate(80.0, 0.20, self.cfg)
        self.assertEqual(g.state, DANGER)

    def test_warn_by_clearance(self):
        # Inside the warn clearance but not dangerous, and stationary.
        g = evaluate(0.0, 0.25, self.cfg)
        self.assertEqual(g.state, WARN)

    def test_warn_by_speed(self):
        # Exceeding warn_margin * v_max(clearance) triggers WARN.
        v_max = _max_safe_speed_cm_s(0.5, self.cfg)
        g = evaluate(self.cfg.warn_margin * v_max + 5.0, 0.5, self.cfg)
        self.assertEqual(g.state, WARN)

    def test_recommended_speed_none_when_safe_or_no_data(self):
        self.assertIsNone(evaluate(0.0, 0.6, self.cfg).recommended_speed_cm_s)
        self.assertIsNone(evaluate(None, None, self.cfg).recommended_speed_cm_s)

    def test_recommended_speed_decreases_as_clearance_shrinks(self):
        far = evaluate(0.0, 0.30, self.cfg).recommended_speed_cm_s
        near = evaluate(0.0, 0.12, self.cfg).recommended_speed_cm_s
        self.assertIsNotNone(far)
        self.assertIsNotNone(near)
        self.assertGreater(far, near)

    def test_max_safe_speed_is_monotonic(self):
        values = [_max_safe_speed_cm_s(d, self.cfg)
                  for d in (0.05, 0.1, 0.2, 0.4, 0.8, 1.6)]
        self.assertEqual(values, sorted(values))

    def test_stop_distance_increases_with_speed(self):
        values = [_stop_distance_m(v, self.cfg) for v in (0, 10, 20, 40, 80)]
        self.assertEqual(values, sorted(values))

    def test_phase_is_echoed(self):
        g = evaluate(0.0, 0.6, self.cfg, phase=PHASE_OPEN)
        self.assertEqual(g.phase, PHASE_OPEN)
        self.assertEqual(g.phase_message, phase_message(PHASE_OPEN, self.cfg))

    def test_ttc_only_when_approaching(self):
        self.assertIsNone(evaluate(0.0, 0.5, self.cfg).ttc_s)
        self.assertIsNone(evaluate(-10.0, 0.5, self.cfg).ttc_s)
        self.assertIsNotNone(evaluate(10.0, 0.5, self.cfg).ttc_s)

    def test_speed_none_when_no_data(self):
        # A NO_DATA result must not carry a fabricated speed.
        self.assertIsNone(evaluate(None, None, self.cfg).speed_cm_s)

    def test_no_data_message_distinguishes_missing_from_unavailable(self):
        # No clearance at all -> "awaiting range"; a non-finite (or zero)
        # clearance -> "range unavailable".  Both are grey NO_DATA, but the text
        # differs (mirrors the docking sibling).
        self.assertIn('awaiting', evaluate(None, None, self.cfg).message)
        self.assertIn(
            'unavailable',
            evaluate(10.0, float('nan'), self.cfg).message)
        from harvester_dashboard.cutter_safety_guidance import _no_data_message
        self.assertIn('awaiting', _no_data_message(None))
        self.assertIn('unavailable', _no_data_message(0.0))
        self.assertIn('unavailable', _no_data_message(float('inf')))


class StateMessageTest(unittest.TestCase):
    cfg = CutterConfig()

    def test_danger_message_is_stop(self):
        g = evaluate(0.0, 0.05, self.cfg)
        self.assertIn('STOP', state_message(DANGER, g))

    def test_message_never_contradicts_held_state(self):
        # The held state's colour and its message must always agree.
        danger = evaluate(80.0, 0.2, self.cfg)
        self.assertNotIn('STOP', state_message(WARN, danger))
        self.assertNotIn('STOP', state_message(SAFE, danger))
        self.assertIn('awaiting', state_message(NO_DATA, danger))

    def test_warn_message_leads_with_slow(self):
        g = evaluate(0.0, 0.25, self.cfg)
        self.assertTrue(state_message(WARN, g).startswith('SLOW'))


class PhaseMachineTest(unittest.TestCase):
    cfg = CutterConfig(
        a_max_m_s2=0.1, latency_s=0.3, warn_margin=0.7,
        warn_clearance_m=0.30, danger_clearance_m=0.10,
        ready_standoff_m=0.20, align_tolerance_m=0.05)

    def test_idle_stays_idle_without_range(self):
        self.assertEqual(next_phase(PHASE_IDLE, None, 0.0, self.cfg),
                         PHASE_IDLE)

    def test_approach_without_range(self):
        self.assertEqual(next_phase(PHASE_APPROACH, None, 0.0, self.cfg),
                         PHASE_APPROACH)

    def test_approach_to_align_when_settled_in_band(self):
        # Inside the ready band and stationary -> ready to cut.
        self.assertEqual(next_phase(PHASE_APPROACH, 0.20, 0.0, self.cfg),
                         PHASE_ALIGN)

    def test_align_not_reached_while_warn_speed(self):
        # Inside the ready band but still closing fast: must NOT claim ready,
        # or the "ready to cut" prompt would contradict the SLOW warning.
        fast = self.cfg.warn_margin * _max_safe_speed_cm_s(0.20, self.cfg) + 20.0
        self.assertEqual(next_phase(PHASE_APPROACH, 0.20, fast, self.cfg),
                         PHASE_APPROACH)

    def test_align_not_reached_outside_band(self):
        self.assertEqual(next_phase(PHASE_APPROACH, 0.50, 0.0, self.cfg),
                         PHASE_APPROACH)

    def test_align_not_reached_inside_danger_floor(self):
        # Inside the danger floor the phase must not advertise "ready to cut",
        # even when the (clamped) tip is slow enough to look settled.
        self.assertEqual(
            next_phase(PHASE_APPROACH, self.cfg.danger_clearance_m, 0.0,
                       self.cfg),
            PHASE_APPROACH)
        self.assertEqual(
            next_phase(PHASE_APPROACH, 0.02, 0.0, self.cfg), PHASE_APPROACH)

    def test_align_allowed_with_warn_clearance(self):
        # The ready band sits inside the WARN band by design: the tip settles in
        # a stable WARN "ready-to-cut" state, so ALIGN must be reachable there.
        self.assertEqual(next_phase(PHASE_APPROACH, 0.20, 0.0, self.cfg),
                         PHASE_ALIGN)
        self.assertEqual(evaluate(0.0, 0.20, self.cfg).state, WARN)

    def test_non_finite_speed_is_not_settled(self):
        self.assertEqual(
            next_phase(PHASE_APPROACH, 0.20, float('inf'), self.cfg),
            PHASE_APPROACH)

    def test_operator_phases_hold_through_dropout(self):
        # A brief range dropout must not wipe in-progress operator steps.
        for phase in (PHASE_ALIGN, PHASE_OPEN, PHASE_ADVANCE, PHASE_CUT):
            with self.subTest(phase=phase):
                self.assertEqual(
                    next_phase(phase, None, 0.0, self.cfg, stale=True), phase)
                self.assertEqual(
                    next_phase(phase, None, 0.0, self.cfg), phase)

    def test_advance_phase_walks_the_sequence(self):
        self.assertEqual(advance_phase(PHASE_ALIGN), PHASE_OPEN)
        self.assertEqual(advance_phase(PHASE_OPEN), PHASE_ADVANCE)
        self.assertEqual(advance_phase(PHASE_ADVANCE), PHASE_CUT)
        # Terminal: cut stays cut.
        self.assertEqual(advance_phase(PHASE_CUT), PHASE_CUT)

    def test_advance_phase_from_unconfirmed_starts_at_align(self):
        self.assertEqual(advance_phase(PHASE_IDLE), PHASE_ALIGN)
        self.assertEqual(advance_phase(PHASE_APPROACH), PHASE_ALIGN)

    def test_phase_messages_are_operator_prompts(self):
        cfg = CutterConfig(advance_distance_m=0.05)
        self.assertIn('STOP', phase_message(PHASE_ALIGN, cfg))
        self.assertIn('OPEN', phase_message(PHASE_OPEN, cfg))
        self.assertIn('5 cm', phase_message(PHASE_ADVANCE, cfg))
        self.assertIn('CUT', phase_message(PHASE_CUT, cfg))

    def test_phases_tuple_is_complete(self):
        self.assertEqual(set(PHASES), {
            PHASE_IDLE, PHASE_APPROACH, PHASE_ALIGN, PHASE_OPEN,
            PHASE_ADVANCE, PHASE_CUT})


if __name__ == '__main__':
    unittest.main()
