"""Low-speed F1 acceptance test in PIE. Sends bounded driving commands."""
import json
import math
import time
from pathlib import Path
from rgb_imu_client import SensorClient

out=Path(__file__).parent/'validation'/'f1_acceptance';out.mkdir(parents=True,exist_ok=True)
c=SensorClient(7449);records=[]
def phase(name,seconds,speed,steering):
    rows=[];end=time.monotonic()+seconds
    while time.monotonic()<end:
        tick=time.monotonic()
        command={'sensors':True}
        if speed is not None: command.update(speed_mps=speed,steering_rad=steering)
        r=c.request(command,lambda x:x.get('schema')=='f1_sensors_v1')
        r['phase']=name;rows.append(r);records.append(r)
        time.sleep(max(0,.05-(time.monotonic()-tick)))
    return rows
try:
    still=phase('stationary',1.5,0,0)
    left=phase('steer_left',1,0,.25)
    right=phase('steer_right',1,0,-.25)
    forward=phase('forward',3,.4,0)
    braking=phase('braking',1.5,0,0)
    turning=phase('left_turn',1.5,.3,.20)
    stop=phase('stop',1.5,0,0)
    idle=phase('idle_before_reverse',4,0,0)
    reverse=phase('reverse_after_idle',2,-.3,0)
    timeout=phase('command_timeout',2,None,0)
    for i in range(8):
        jpeg,meta=c.camera();time.sleep(.11)
    (out/'camera.jpg').write_bytes(jpeg)
    (out/'camera.json').write_text(json.dumps(meta,indent=2)+'\n')
finally:
    c.sock.send(b'{"speed_mps":0,"steering_rad":0}');c.close()
    (out/'packets.json').write_text(json.dumps(records,indent=2)+'\n')

def angles(r):return [w['steering_rad'] for w in r['wheels']]
l=angles(left[-1]);r=angles(right[-1])
travel=math.hypot(forward[-1]['x_m']-forward[0]['x_m'],forward[-1]['y_m']-forward[0]['y_m'])
turn=math.atan2(math.sin(turning[-1]['yaw_rad']-turning[0]['yaw_rad']),math.cos(turning[-1]['yaw_rad']-turning[0]['yaw_rad']))
result={
    'physics_valid':all(x['physics_valid'] for x in records),
    'stationary_wheel_contacts':[w['in_contact'] for w in still[-1]['wheels']],
    'stationary_suspension_offsets_m':[w['suspension_offset_m'] for w in still[-1]['wheels']],
    'stationary_z_range_m':max(x['z_m'] for x in still)-min(x['z_m'] for x in still),
    'left_wheel_angles_rad':l,'right_wheel_angles_rad':r,
    'forward_travel_m':travel,'final_forward_speed_mps':forward[-1]['forward_speed_mps'],
    'after_braking_speed_mps':braking[-1]['forward_speed_mps'],
    'left_turn_yaw_change_rad':turn,
    'max_front_drive_torque':max(abs(w['drive_torque_nm']) for x in forward for w in x['wheels'][:2]),
    'max_rear_drive_torque':max(abs(w['drive_torque_nm']) for x in forward for w in x['wheels'][2:]),
    'rgb_dimensions':[meta['rgb']['width'],meta['rgb']['height']],
    'lidar_samples':[len(stop[-1]['front_scan']['ranges_m']),len(stop[-1]['rear_scan']['ranges_m'])],
    'imu_valid':stop[-1]['imu']['valid'],
    'reverse_after_idle_speed_mps':reverse[-1]['forward_speed_mps'],
    'reverse_after_idle_travel_m':math.hypot(reverse[-1]['x_m']-reverse[0]['x_m'],reverse[-1]['y_m']-reverse[0]['y_m']),
    'after_timeout_speed_mps':timeout[-1]['forward_speed_mps'],
}
result['passed']=(result['physics_valid'] and all(result['stationary_wheel_contacts'])
                  and result['stationary_z_range_m']<.015
                  and l[0]>l[1]>0 and r[1]<r[0]<0 and max(abs(x) for x in l[2:]+r[2:])<1e-6
                  and travel>.3 and abs(result['after_braking_speed_mps'])<.05 and turn>.01
                  and result['max_front_drive_torque']<1e-6 and result['max_rear_drive_torque']>0
                  and result['rgb_dimensions']==[640,480] and result['lidar_samples']==[360,360] and result['imu_valid']
                  and result['reverse_after_idle_speed_mps']<-.05 and result['reverse_after_idle_travel_m']>.1
                  and abs(result['after_timeout_speed_mps'])<.05)
(out/'summary.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2))
if not result['passed']:raise SystemExit('F1 acceptance failed; inspect packets.')
