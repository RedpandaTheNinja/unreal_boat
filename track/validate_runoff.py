"""Bounded low-speed curb crossing and reverse recovery test in Unreal PIE."""
from pathlib import Path
import json
import math
import sys
import time
import argparse
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'simulation_ml/integrations/rapyuta'))
from rgb_imu_client import SensorClient
sys.path.insert(0,str(ROOT/'simulation_ml/integrations/rapyuta/dashboard'))
from server import track_projection

parser=argparse.ArgumentParser();parser.add_argument('--side',choices=['left','right'],default='left');args=parser.parse_args()
steer=1 if args.side=='left' else -1
track=json.loads((ROOT/'track/generated_smooth/track.json').read_text())
client=SensorClient(7449); rows=[]
def phase(name,speed,steer,seconds,until=None):
    end=time.monotonic()+seconds;last=None
    while time.monotonic()<end:
        start=time.monotonic()
        last=client.request(dict(speed_mps=speed,steering_rad=steer,sensors=True),lambda r:r.get('schema')=='f1_sensors_v1')
        last['phase']=name;last['track']=track_projection(last,track);rows.append(last)
        q=last['imu']['orientation_ground_truth_xyzw']
        assert 1-2*(q['x']**2+q['y']**2)>.7,'Vehicle tilt exceeded test limit'
        if until and until(last):break
        time.sleep(max(0,.05-(time.monotonic()-start)))
    return last
try:
    phase('settle',0,0,1)
    outside=phase('leave_road',.5,steer,12,lambda p:abs(p['track']['cross_track_m'])>1.0)
    assert abs(outside['track']['cross_track_m'])>1,'Did not cross curb onto runoff'
    phase('stop_offroad',0,0,1)
    inside=phase('reverse_to_road',-.5,steer,12,lambda p:abs(p['track']['cross_track_m'])<.10)
    assert abs(inside['track']['cross_track_m'])<.10,'Did not return to road center'
    stop=phase('stopped_on_road',0,0,1.5)
    result=dict(passed=True,maximum_cross_track_m=max(abs(r['track']['cross_track_m']) for r in rows),
                final_cross_track_m=stop['track']['cross_track_m'],final_speed_mps=stop['forward_speed_mps'],
                final_wheel_contacts=[w['in_contact'] for w in stop['wheels']],
                minimum_z_m=min(r['z_m'] for r in rows),maximum_z_m=max(r['z_m'] for r in rows))
    assert all(result['final_wheel_contacts']) and abs(result['final_speed_mps'])<.05,result
    (ROOT/f'track/generated_smooth/runoff_{args.side}_result.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result,indent=2))
finally:
    client.sock.send(b'{"speed_mps":0,"steering_rad":0}');client.close()
    (ROOT/f'track/generated_smooth/runoff_{args.side}_packets.json').write_text(json.dumps(rows,indent=2)+'\n')
