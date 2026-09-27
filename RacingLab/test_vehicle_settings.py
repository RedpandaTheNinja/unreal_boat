import unittest
from vehicle_settings import validate_vehicle_settings,apply_environment,MAX_SPEED_MPS
class SettingsTests(unittest.TestCase):
    def test_units_and_upper_bound(self):
        self.assertAlmostEqual(MAX_SPEED_MPS,138.5824)
        for speed in [.89408,1.2,138.5824]:
            self.assertEqual(validate_vehicle_settings(dict(speed_mps=speed))['max_speed_mps'],MAX_SPEED_MPS)
    def test_grip_both_axles(self):
        c=validate_vehicle_settings(dict(speed_mps=1.2,tire_grip_multiplier=.4))
        self.assertEqual(c['front_tire_grip_multiplier'],.4)
        self.assertEqual(c['rear_tire_grip_multiplier'],.4)
    def test_invalid(self):
        for cfg in [dict(speed_mps=139),dict(speed_mps=float('nan')),dict(speed_mps=1,max_speed_mps=.5),dict(speed_mps=1,tire_grip_multiplier=-.1),dict(speed_mps=1,tire_grip_multiplier=4)]:
            with self.assertRaises(ValueError): validate_vehicle_settings(cfg)
    def test_ack_required(self):
        class Client:
            def request(self,*args): return {'schema':'f1_sensors_v1'}
        with self.assertRaisesRegex(RuntimeError,'older'): apply_environment(Client(),dict(speed_mps=1.2))
if __name__=='__main__': unittest.main()
