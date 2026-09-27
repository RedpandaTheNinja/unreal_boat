import unittest,math
from rl_demo import Track,ROOT,state,reward,wrap,terminal
class GeometryTests(unittest.TestCase):
    def test_unreal_origin_and_left_error(self):
        t=Track(ROOT/'track/generated/track.json'); a,b=t.points[:2]
        h=math.atan2(b[1]-a[1],b[0]-a[0])
        p=dict(x_m=a[0]+50-.1*math.sin(h),y_m=a[1]+.1*math.cos(h),yaw_rad=h)
        o=t.observe(p,.8)
        self.assertAlmostEqual(o['cross'],.1,places=2)
        self.assertLess(o['baseline'],0)
    def test_reward_finish_wrap(self):
        a=dict(station=359.9,cross=0,heading_error=0)
        b=dict(station=.1,cross=0,heading_error=0)
        self.assertAlmostEqual(reward(a,b,360,.1,0),.2)
    def test_angle_wrap(self): self.assertAlmostEqual(wrap(2*math.pi+.1),.1)
    def test_safety_contact_stale_and_edge(self):
        import json
        p=json.loads((ROOT/'car_model/bump_substep.json').read_text())[20]
        o=Track(ROOT/'track/generated/track.json').observe(p,.8)
        self.assertIsNone(terminal(p,o))
        self.assertEqual(terminal(p,dict(o,margin=.01)),'track_margin')
        p['wheels'][0]['in_contact']=False
        self.assertEqual(terminal(p,o),'lost_wheel_contact')
        p['wheels'][0]['in_contact']=True
        p['imu']['stamp_s']=p['time_s']-1
        self.assertEqual(terminal(p,o),'invalid_imu')
if __name__=='__main__': unittest.main()
