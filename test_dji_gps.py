import math
import unittest

from dji_gps import decode, pose


class PoseTests(unittest.TestCase):
    def sample(self):
        return {"Doc1:Protocol": "dvtm_Mini5Pro.proto",
                "Doc1:Dvtm_Mini5Pro_1-1-2": "02.01.07",
                "Doc1:SampleTime": 0, "Doc1:GPSLatitude": 0,
                "Doc1:GPSLongitude": 0, "Doc1:AbsoluteAltitude": 0,
                "Doc1:Dvtm_Mini5Pro_3-4-4-1": "float 1"}

    def test_camera_axes_and_origin(self):
        matrix = decode(self.sample())["frames"][0]["world_to_camera"]
        self.assertEqual(matrix, [[1, 0, 0, 0], [0, 0, -1, 0],
                                  [0, 1, 0, 0], [0, 0, 0, 1]])

    def test_heading_changes_optical_axis(self):
        q = [math.sqrt(.5), 0, 0, math.sqrt(.5)]
        matrix = pose([0, 0, 0], q, [0, 0, 0])
        # 90 degrees clockwise from north: camera looks east.
        for value, expected in zip(matrix[2][:3], [1, 0, 0]):
            self.assertAlmostEqual(value, expected)

    def test_translation_is_in_metres(self):
        matrix = pose([0, 0, 10], [1, 0, 0, 0], [0, 0, 0])
        self.assertAlmostEqual(matrix[1][3], 10)

    def test_no_guessed_orientation(self):
        for replacement in [None, "float 0", "float nan", "malformed"]:
            sample = self.sample()
            key = "Doc1:Dvtm_Mini5Pro_3-4-4-1"
            if replacement is None:
                del sample[key]
            else:
                sample[key] = replacement
            with self.assertRaises(ValueError):
                decode(sample)

    def test_unknown_schema_and_missing_gps_rejected(self):
        for key in ["Doc1:Protocol", "Doc1:GPSLatitude"]:
            sample = self.sample()
            del sample[key]
            with self.assertRaises(ValueError):
                decode(sample)


if __name__ == "__main__":
    unittest.main()
