"""Live smoke/acceptance tests. These establish simulation behavior, not real-world calibration."""
import json,math,statistics,time
from pathlib import Path
from boat_client import BoatClient
out=Path(__file__).parent/'validation';out.mkdir(exist_ok=True)
c=BoatClient();checks={};logs={}
def record(name,rows):
 logs[name]=rows;(out/(name+'.jsonl')).write_text('\n'.join(json.dumps(r) for r in rows)+'\n');return rows[-1]
def reset():
 c.environment();c.request(reset=True);time.sleep(.15);return record('settle_latest',c.sample(10))
def speed(r):return math.sqrt(sum(v*v for v in r['velocity_ue_mps'][:2]))
try:
 rest=reset();print('REST',json.dumps(rest),flush=True)
 checks['loaded_mass']=abs(rest['mass_kg']-172.3651006)<.001
 checks['float_displacement']=abs(rest['displacement_m3']-.1723651006)<.01
 checks['rest_stable']=speed(rest)<.05 and abs(rest['velocity_ue_mps'][2])<.05 and max(abs(x) for x in rest['roll_pitch_yaw_deg'][:2])<10
 checks['physics_fixed_step']=abs(rest['physics_dt_s']-1/120)<.00001
 start=rest;forward=record('forward',c.sample(6,.6,.6));print('FORWARD',speed(forward),flush=True)
 checks['forward_motion']=forward['position_ue_m'][0]<start['position_ue_m'][0]-.5 and speed(forward)>.25
 coast=record('coast',c.sample(6));checks['coast_drag']=speed(coast)<speed(forward)
 reset();spin=record('opposite_thrust',c.sample(5,.6,-.6));checks['differential_turn']=abs(spin['angular_velocity_body_radps'][2])>.1
 start=reset();reverse=record('reverse',c.sample(5,-.6,-.6));checks['reverse_motion']=reverse['position_ue_m'][0]>start['position_ue_m'][0]+.3
 c.sample(1,.5,.5);expired=record('timeout',c.sample(3,command=False));checks['timeout']=expired['command_expired'] and abs(expired['left_thrust_n'])<.2 and abs(expired['right_thrust_n'])<.2
 start=reset();c.environment(cy=.3);drift=record('current_drift',c.sample(10));checks['current_drift']=drift['position_ue_m'][1]>start['position_ue_m'][1]+.2
 reset();c.environment(amplitude=.03,period=3,length=8,direction=.7);wave=record('waves',c.sample(9));rows=logs['waves'];zs=[r['position_ue_m'][2] for r in rows];checks['wave_response']=max(zs)-min(zs)>.005 and all(not r['deck_awash'] for r in rows)
 c.environment();reset()
finally:
 try:c.request(left=0,right=0)
 finally:c.close()
 result={'checks':checks,'passed':all(checks.values()),'calibration_status':'not_calibrated_against_real_boat','logs':list(logs)}
 (out/'acceptance.json').write_text(json.dumps(result,indent=2));print(json.dumps(result,indent=2),flush=True)
if not result['passed']:raise SystemExit(1)
