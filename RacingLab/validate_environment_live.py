"""Stationary native bridge acceptance: speed ceiling and actual wheel-setting API."""
import json,sys
from pathlib import Path
HERE=Path(__file__).resolve().parent
sys.path.insert(0,str(HERE.parent/'simulation_ml/integrations/rapyuta'))
from rgb_imu_client import SensorClient
from vehicle_settings import apply_environment
cfg=json.loads((HERE/'settings.json').read_text());client=SensorClient(cfg['udp_port']);evidence=[]
try:
    for grip in [1.0,.35,0,1.5,1.0]:
        packet=apply_environment(client,dict(cfg,tire_grip_multiplier=grip))
        evidence.append({k:packet[k] for k in ['time_s','configuration_accepted','max_speed_mps','front_tire_grip_multiplier','rear_tire_grip_multiplier','speed_target_mps']})
        assert packet['speed_target_mps']==0
    invalid=client.request(dict(configure_vehicle=dict(max_speed_mps=139,front_tire_grip_multiplier=1,rear_tire_grip_multiplier=1),speed_mps=0,steering_rad=0),lambda r:r.get('schema')=='f1_sensors_v1')
    assert invalid['configuration_accepted'] is False
    assert abs(invalid['max_speed_mps']-138.5824)<.0001
    result=dict(passed=True,configuration_roundtrips=evidence,rejected_above_limit=True,note='Acknowledges native SetWheelFrictionMultiplier calls; does not validate high-speed dynamics or measure tire force.')
    (HERE/'runs/environment_acceptance.json').write_text(json.dumps(result,indent=2));print(json.dumps(result,indent=2))
finally:
    try: apply_environment(client,cfg)
    finally: client.sock.send(b'{"speed_mps":0,"steering_rad":0}');client.close()
