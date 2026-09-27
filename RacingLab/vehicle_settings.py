"""SI-unit experiment settings and acknowledged native Chaos configuration."""
import math

MAX_SPEED_MPS=310*.44704

def validate_vehicle_settings(cfg):
    speed=cfg['speed_mps']; limit=cfg.get('max_speed_mps',MAX_SPEED_MPS)
    grip=cfg.get('tire_grip_multiplier',1.0)
    for key,value in [('speed_mps',speed),('max_speed_mps',limit),('tire_grip_multiplier',grip)]:
        if isinstance(value,bool) or not isinstance(value,(int,float)) or not math.isfinite(value):
            raise ValueError(f'{key} must be a finite number')
    if not 0<limit<=MAX_SPEED_MPS: raise ValueError(f'max_speed_mps must be in (0,{MAX_SPEED_MPS}] (310 mph ceiling)')
    if not 0<speed<=limit: raise ValueError('speed_mps must be positive and no greater than max_speed_mps; units are m/s')
    if not 0<=grip<=3: raise ValueError('tire_grip_multiplier must be in [0,3]; 1 preserves original grip')
    return dict(max_speed_mps=limit,front_tire_grip_multiplier=grip,rear_tire_grip_multiplier=grip)

def apply_environment(client,cfg):
    expected=validate_vehicle_settings(cfg)
    reply=client.request(dict(configure_vehicle=expected,speed_mps=0,steering_rad=0,sensors=True),lambda r:r.get('schema')=='f1_sensors_v1')
    if reply.get('vehicle_config_version',0)<1:
        raise RuntimeError('The running Unreal plugin is older than the speed/grip update. Install the new build and restart Unreal.')
    if not reply.get('configuration_accepted'):
        raise RuntimeError('Unreal rejected vehicle settings: '+reply.get('configuration_error','unknown reason'))
    for key,value in expected.items():
        if not math.isclose(reply.get(key,float('nan')),value,rel_tol=1e-6,abs_tol=1e-6):
            raise RuntimeError(f'Unreal did not acknowledge {key}={value}')
    return reply
