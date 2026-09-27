import math
import unittest
from sensor_client import obstacle_stop


class ObstacleStopTests(unittest.TestCase):
    def frame(self):
        return {'time_s': 1.0, 'front_scan': {'valid': True, 'stamp_s': .95,
                'angle_min_rad': 0, 'angle_increment_rad': math.pi / 180,
                'ranges_m': [None] * 360}}

    def test_front_and_rear_are_distinguished(self):
        f = self.frame()
        f['front_scan']['ranges_m'][180] = .3
        self.assertGreater(obstacle_stop(f)[0], 0)
        f['front_scan']['ranges_m'][0] = .3
        self.assertEqual(obstacle_stop(f)[2], 'obstacle_ahead')

    def test_body_corridor_not_just_center_ray(self):
        f = self.frame()
        f['front_scan']['ranges_m'][20] = .4
        self.assertEqual(obstacle_stop(f)[0], 0)

    def test_stale_future_and_invalid_stop(self):
        for stamp in (0.1, 1.2):
            f = self.frame()
            f['front_scan']['stamp_s'] = stamp
            self.assertEqual(obstacle_stop(f)[0], 0)
        f = self.frame()
        f['front_scan']['ranges_m'][0] = float('nan')
        self.assertEqual(obstacle_stop(f)[0], 0)


if __name__ == '__main__':
    unittest.main()
