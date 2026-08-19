import importlib.util
import math
import unittest
from pathlib import Path

# Import the example script as a module so its pure functions are testable.
ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location(
    "estimate_tree_height", ROOT / "examples" / "estimate_tree_height.py"
)
example = importlib.util.module_from_spec(spec)
spec.loader.exec_module(example)


class TreeHeightTests(unittest.TestCase):
    def test_classify_separates_trunk_from_canopy(self):
        # A narrow vertical trunk plus a wide ring at the same height.
        points = []
        for z in range(6):
            points.append((0.0, 0.0, float(z)))  # trunk on the axis
        for i in range(20):
            angle = i * 2 * math.pi / 20
            points.append((1.0 * math.cos(angle), 1.0 * math.sin(angle), 4.0))  # canopy
        trunk, canopy = example.classify_trunk_vs_canopy(points, trunk_radius_m=0.3)
        self.assertEqual(len(trunk), 6)
        self.assertEqual(len(canopy), 20)

    def test_empty_cloud_returns_empty(self):
        trunk, canopy = example.classify_trunk_vs_canopy([], 0.3)
        self.assertEqual(trunk, [])
        self.assertEqual(canopy, [])

    def test_synthetic_cloud_has_trunk_and_canopy(self):
        points = example.synthetic_tree_cloud()
        self.assertGreater(len(points), 0)
        trunk, canopy = example.classify_trunk_vs_canopy(points, 0.3)
        self.assertGreater(len(trunk), 0)
        self.assertGreater(len(canopy), 0)


if __name__ == "__main__":
    unittest.main()
