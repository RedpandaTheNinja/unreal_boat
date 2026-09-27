"""Exercise both steering directions and both UDP command modes in PIE."""
import json
import math
from pathlib import Path
import time
from rgb_imu_client import SensorClient

client=SensorClient(7449); samples=[]; results=[]
try:
    for mode in ('angle','direct'):
        for sign in (1,-1):
            command=({'speed_mps':0,'steering_rad':sign*10} if mode=='angle' else
                     {'throttle':0,'steer':sign,'brake':1})
            end=time.monotonic()+1.3
            while time.monotonic()<end:
                p=client.request(command,lambda r:r.get('schema')=='f1_sensors_v1')
                samples.append(p); time.sleep(.04)
            angles=[w['steering_rad'] for w in p['wheels']]
            inside=angles[0 if sign>0 else 1]; outside=angles[1 if sign>0 else 0]
            assert sign*inside>sign*outside>0, (mode,angles)
            assert abs(abs(inside)-math.pi/6)<.001, angles
            assert max(abs(a) for a in angles[2:])<1e-6
            # Difference in cotangents must equal track / wheelbase.
            assert abs((1/math.tan(abs(outside))-1/math.tan(abs(inside)))-20/37)<.002
            results.append(dict(mode=mode,direction='left' if sign>0 else 'right',angles_deg=[math.degrees(a) for a in angles]))
    assert all(abs(w['steering_rad'])<=math.pi/6+1e-5 for p in samples for w in p['wheels'])
finally:
    client.sock.send(b'{"speed_mps":0,"steering_rad":0}');client.close()
output=Path(__file__).parent/'validation'/'f1_steering.json'
output.write_text(json.dumps(dict(passed=True,results=results,samples=len(samples)),indent=2)+'\n')
print(output.read_text())
