"""Guards for the full-screen LiDAR scan overlay's QML projection.

The overlay implements its own projection in QML (the Canvas cannot call the
Python helper per point), and it must mirror ``harvester_dashboard/projection``.
That duplication is the one real correctness risk in the overlay: an early
version handled only the camera view and returned the centre for every vehicle
view, which silently drew the whole cloud as a single dot.

These tests assert, at the source level, that every view in ``projection.VIEWS``
has a distinct screen mapping in the QML, so a missing branch fails loudly.
"""

import os
import unittest

_QML_PATH = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
    'qml', 'LidarScanOverlay.qml')


def _overlay_source():
    with open(_QML_PATH, 'r', encoding='utf-8') as handle:
        return handle.read()


class OverlayProjectionTest(unittest.TestCase):
    def test_every_view_has_a_branch(self):
        source = _overlay_source()
        # `top` is the documented fall-through (also the safe default for an
        # unknown view), so it has no explicit branch; the other five must.
        for view in ('camera', 'front', 'left', 'right', 'iso'):
            with self.subTest(view=view):
                self.assertIn('view === "%s"' % view, source,
                              'overlay projection is missing the %s view' % view)
        # The fall-through returns the top mapping, not the centre.
        self.assertIn('return [cx + x * s, cy - y * s];', source)

    def test_no_stale_guiding_phase(self):
        # `guiding` was removed from the state machine; the overlay must not
        # reference it (its crosshair gated on it silently never drew).
        self.assertNotIn('"guiding"', _overlay_source())

    def test_vehicle_views_are_not_collapsed_to_the_centre(self):
        # The specific regression: vehicle views must map a point to something
        # other than [cx, cy].  Assert the branches use the coordinates.
        source = _overlay_source()
        fwd = source[source.index('function projectPoint'):]
        for token in ('cx + y * s', 'cx + x * s', 'cx - x * s',
                      'cx + (x - y) * iso * s'):
            with self.subTest(token=token):
                self.assertIn(token, fwd)

    def test_top_view_maps_forward_right_and_left_up(self):
        source = _overlay_source()
        body = source[source.index('function projectPoint'):]
        # top is the fall-through branch.
        self.assertIn('return [cx + x * s, cy - y * s];', body)


if __name__ == '__main__':
    unittest.main()
