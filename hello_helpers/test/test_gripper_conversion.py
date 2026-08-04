import math
import re
import unittest
from pathlib import Path

from hello_helpers.gripper_conversion import GripperConversion


class TestGripperConversion(unittest.TestCase):
    def setUp(self):
        self.conversion = GripperConversion()

    def test_default_geometry_matches_exact_xacro_value(self):
        self.assertAlmostEqual(self.conversion.finger_length_m, 0.171099)

    def test_robotis_and_aperture_round_trip_without_clamping(self):
        for robotis_value in (-20.0, 0.0, 35.0, 70.0, 95.0):
            with self.subTest(robotis_value=robotis_value):
                aperture = self.conversion.robotis_to_aperture(robotis_value)
                self.assertAlmostEqual(
                    self.conversion.aperture_to_robotis(aperture), robotis_value
                )

    def test_aperture_and_finger_angle_round_trip(self):
        for aperture_m in (-0.01, 0.0, 0.045, 0.09, 0.12):
            with self.subTest(aperture_m=aperture_m):
                finger_rad = self.conversion.aperture_to_finger_rad(aperture_m)
                self.assertAlmostEqual(
                    self.conversion.finger_rad_to_aperture(finger_rad), aperture_m
                )

    def test_custom_geometry_is_supported(self):
        conversion = GripperConversion(
            finger_length_m=0.2,
            open_aperture_m=0.1,
            closed_aperture_m=0.01,
            open_robotis=80.0,
            closed_robotis=5.0,
        )
        self.assertAlmostEqual(conversion.robotis_to_aperture(80.0), 0.1)
        self.assertAlmostEqual(conversion.aperture_to_finger_rad(0.1), 0.25)

    def test_invalid_geometry_is_rejected(self):
        invalid_arguments = (
            {"finger_length_m": 0.0},
            {"finger_length_m": math.nan},
            {"open_aperture_m": 0.0},
            {"open_robotis": 0.0},
        )
        for arguments in invalid_arguments:
            with self.subTest(arguments=arguments):
                with self.assertRaises(ValueError):
                    GripperConversion(**arguments)

    def test_default_geometry_matches_every_gripper_xacro(self):
        repository_root = Path(__file__).resolve().parents[2]
        xacro_files = sorted(
            (repository_root / "stretch_description" / "batch").glob(
                "*/urdf/stretch_gripper*.xacro"
            )
        )
        self.assertGreater(len(xacro_files), 0)

        scale_pattern = re.compile(
            r'<xacro:property name="scale_finger_length" value="([0-9.]+)"'
        )
        length_pattern = re.compile(
            r'xyz="\$\{scale_finger_length \* (-?[0-9.]+)\}[^" ]*'
        )

        for xacro_file in xacro_files:
            contents = xacro_file.read_text(encoding="utf-8")
            scale_match = scale_pattern.search(contents)
            length_matches = length_pattern.findall(contents)
            with self.subTest(xacro_file=xacro_file):
                self.assertIsNotNone(scale_match)
                self.assertGreater(len(length_matches), 0)
                scale = float(scale_match.group(1))
                for unscaled_length in length_matches:
                    expected_length = scale * abs(float(unscaled_length))
                    self.assertAlmostEqual(
                        self.conversion.finger_length_m, expected_length
                    )


if __name__ == "__main__":
    unittest.main()
