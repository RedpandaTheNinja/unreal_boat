"""Control the Chaos miniature F1 using speed and steering angle, SI units.
Positive steering means left. The tandem's speed/yaw commands are not compatible.
"""
import argparse
import json
import math
import time
from pathlib import Path
from rgb_imu_client import SensorClient

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--speed',type=float,default=.3)
    p.add_argument('--steering',type=float,default=0,help='virtual steering angle, radians; positive left')
    p.add_argument('--seconds',type=float,default=3)
    p.add_argument('--port',type=int,default=7449)
    p.add_argument('--output',type=Path,default=Path('simulation_ml/integrations/rapyuta/validation/f1_drive.json'))
    a=p.parse_args()
    if not all(math.isfinite(v) for v in (a.speed,a.steering,a.seconds)) or a.seconds<=0: p.error('Use finite values and positive duration')
    c=SensorClient(a.port); records=[]
    try:
        end=time.monotonic()+a.seconds
        while time.monotonic()<end:
            t=time.monotonic()
            records.append(c.request({'speed_mps':a.speed,'steering_rad':a.steering,'sensors':True},lambda r:r.get('schema')=='f1_sensors_v1'))
            time.sleep(max(0,.05-(time.monotonic()-t)))
    finally:
        c.sock.send(b'{"speed_mps":0,"steering_rad":0}')
        c.close()
    a.output.parent.mkdir(parents=True,exist_ok=True)
    a.output.write_text(json.dumps(records,indent=2)+'\n')
    print(json.dumps({'samples':len(records),'first':{k:v for k,v in records[0].items() if k not in ('front_scan','rear_scan','imu','rgb')},'last':{k:v for k,v in records[-1].items() if k not in ('front_scan','rear_scan','imu','rgb')}},indent=2))
if __name__=='__main__': main()
