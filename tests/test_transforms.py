import unittest
import json
from pathlib import Path
from xml.etree import ElementTree

from geometry.transforms import FrameGraph, TransformConfigurationError, validate_configuration


ROOT = Path(__file__).resolve().parents[1]
NOMINAL = ROOT / "calibration" / "frames.nominal.json"
DEPLOYMENT_TEMPLATE = ROOT / "calibration" / "frames.deployment.template.json"
NOMINAL_URDF = ROOT / "urdf" / "harvester_vision_nominal.urdf"


class FrameSetupTests(unittest.TestCase):
    def test_nominal_configuration_is_valid_for_simulation_only(self):
        self.assertEqual(validate_configuration(NOMINAL, "simulation"), [])

    def test_nominal_bindings_match_existing_camera_and_sensor_contracts(self):
        with NOMINAL.open() as config_file:
            config = json.load(config_file)
        self.assertEqual(set(config["camera_bindings"]), {"docking_camera", "cutting_camera"})
        self.assertEqual(
            {item["telemetry_key"] for item in config["sensor_telemetry_bindings"]},
            {"diagonal_left_45deg", "diagonal_right_45deg", "center_line", "c_channel_left", "c_channel_right"},
        )

    def test_nominal_configuration_is_rejected_for_deployment(self):
        issues = validate_configuration(NOMINAL, "deployment")
        self.assertTrue(any("simulation-only" in issue for issue in issues))
        self.assertTrue(any("not allowed in deployment" in issue for issue in issues))

    def test_deployment_template_is_valid_for_planning_but_not_runtime(self):
        self.assertEqual(validate_configuration(DEPLOYMENT_TEMPLATE, "planning"), [])
        issues = validate_configuration(DEPLOYMENT_TEMPLATE, "deployment")
        self.assertTrue(any("missing measured transform values" in issue for issue in issues))

    def test_optical_frame_forward_axis_maps_to_mechanical_forward_axis(self):
        graph = FrameGraph.from_configuration(NOMINAL)
        point = graph.transform_point((0.0, 0.0, 1.0), "docking_camera_optical_frame", "docking_camera_link")
        self.assertAlmostEqual(point[0], 1.0, places=7)
        self.assertAlmostEqual(point[1], 0.0, places=7)
        self.assertAlmostEqual(point[2], 0.0, places=7)

    def test_static_lookup_does_not_invent_dynamic_base_pose(self):
        graph = FrameGraph.from_configuration(NOMINAL)
        with self.assertRaises(TransformConfigurationError):
            graph.lookup("rail_frame", "docking_camera_link")

    def test_nominal_urdf_fixed_joints_match_frame_registry(self):
        with NOMINAL.open() as config_file:
            expected = {item["id"]: item for item in json.load(config_file)["static_transforms"]}
        root = ElementTree.parse(NOMINAL_URDF).getroot()
        actual = {joint.attrib["name"]: joint for joint in root.findall("joint") if joint.attrib["type"] == "fixed"}
        self.assertEqual(set(actual), set(expected))
        for name, definition in expected.items():
            joint = actual[name]
            self.assertEqual(joint.find("parent").attrib["link"], definition["parent"])
            self.assertEqual(joint.find("child").attrib["link"], definition["child"])
            origin = joint.find("origin")
            self.assertEqual(
                tuple(float(value) for value in origin.attrib["xyz"].split()),
                tuple(definition["translation_m"]),
            )
            self.assertEqual(
                tuple(float(value) for value in origin.attrib["rpy"].split()),
                tuple(definition["rotation_rpy_rad"]),
            )


if __name__ == "__main__":
    unittest.main()
