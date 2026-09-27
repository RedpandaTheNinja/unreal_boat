import math
import unittest
from server import track_projection, lidar_points, Feed


class DashboardGeometryTests(unittest.TestCase):
    def test_track_bridge_offset_and_edge_margin(self):
        track=dict(ue_origin_cm=[15000,0,0],width_m=1.2,centerline_m=[[0,0,0],[10,0,0],[10,10,0],[0,10,0]])
        p=track_projection(dict(x_m=55,y_m=.2,yaw_rad=0),track)
        self.assertAlmostEqual(p['x_m'],5)
        self.assertAlmostEqual(p['progress_m'],5)
        self.assertAlmostEqual(p['cross_track_m'],.2)
        self.assertAlmostEqual(p['estimated_edge_margin_m'],.275)

    def test_lidar_mount_rotation_null_and_stale(self):
        scan=dict(valid=True,stamp_s=2,range_min_m=.12,range_max_m=3.5,
                  angle_min_rad=0,angle_increment_rad=math.pi/2,ranges_m=[1,None,4],
                  sensor_pose_ground_truth=dict(x_m=50,y_m=.12,z_m=1,qx=0,qy=0,qz=math.sqrt(.5),qw=math.sqrt(.5)))
        p=dict(x_m=50,y_m=0,yaw_rad=math.pi/2,time_s=2,front_scan=scan)
        pts=lidar_points(p)
        self.assertEqual(len(pts),1)
        self.assertAlmostEqual(pts[0][0],1.12)
        self.assertAlmostEqual(pts[0][1],0)
        p['time_s']=3
        self.assertEqual(lidar_points(p),[])

    def test_no_connection_is_explicit(self):
        state=Feed(7449,{}).snapshot()
        self.assertEqual(state['status'],'offline')
        self.assertIsNone(state['telemetry'])


if __name__=='__main__': unittest.main()
