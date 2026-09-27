"""Bounded live sensor acceptance check. Moves the car slowly during PIE."""
import hashlib
import json
import math
import statistics
import time
from pathlib import Path
from rgb_imu_client import SensorClient

OUT = Path(__file__).parent / 'validation' / 'rgb_imu_acceptance'
OUT.mkdir(parents=True, exist_ok=True)
client = SensorClient()
records = []


def phase(name, seconds, speed, yaw):
    end = time.monotonic() + seconds
    result = []
    while time.monotonic() < end:
        start = time.monotonic()
        data = client.request({'sensors': True, 'v_mps': speed, 'yaw_radps': yaw},
                              lambda r: r.get('schema') == 'tandem_sensors_v1')
        data['test_phase'] = name
        result.append(data)
        records.append(data)
        time.sleep(max(0, 0.025 - (time.monotonic() - start)))
    return result


try:
    stationary = phase('stationary', 1.5, 0, 0)
    first, first_meta = client.camera()
    moving = phase('forward', 2, .15, 0)
    turning = phase('left_turn', 2, 0, .5)
    stopped = phase('stopped', 1, 0, 0)
    last, last_meta = client.camera()
finally:
    client.sock.send(b'{"v_mps":0,"yaw_radps":0}')
    client.close()
    (OUT / 'packets.json').write_text(json.dumps(records, indent=2) + '\n')

for name, image, meta in [('before', first, first_meta), ('after', last, last_meta)]:
    (OUT / (name + '.jpg')).write_bytes(image)
    (OUT / (name + '.json')).write_text(json.dumps(meta, indent=2) + '\n')

forces = [math.sqrt(sum(v*v for v in r['imu']['linear_acceleration_mps2'].values())) for r in stationary]
dt = turning[-1]['time_s'] - turning[0]['time_s']
yaw_change = math.atan2(math.sin(turning[-1]['yaw_rad'] - turning[0]['yaw_rad']),
                        math.cos(turning[-1]['yaw_rad'] - turning[0]['yaw_rad']))
gyro_mean = statistics.mean(r['imu']['angular_velocity_radps']['z'] for r in turning)
travel = math.hypot(moving[-1]['x_m']-moving[0]['x_m'], moving[-1]['y_m']-moving[0]['y_m'])
age = [r['time_s']-r['imu']['stamp_s'] for r in records]
summary = {
    'stationary_specific_force_mean_mps2': statistics.mean(forces),
    'imu_median_sample_dt_s': statistics.median(r['imu']['sample_dt_s'] for r in records),
    'imu_max_age_s': max(age), 'forward_travel_m': travel,
    'left_turn_yaw_change_rad': yaw_change, 'mean_gyro_z_radps': gyro_mean,
    'pose_mean_yaw_rate_radps': yaw_change/dt,
    'image_changed': hashlib.sha256(first).digest()!=hashlib.sha256(last).digest(),
    'rgb_dimensions': [last_meta['rgb']['width'],last_meta['rgb']['height']],
    'lidar_sample_counts': [len(records[-1]['front_scan']['ranges_m']),len(records[-1]['rear_scan']['ranges_m'])],
    'max_tandem_separation_error_m': max(abs(r['separation_m']-.30) for r in records),
}
summary['passed'] = (8.5 < summary['stationary_specific_force_mean_mps2'] < 11
                     and all(r['imu']['valid'] for r in records)
                     and all(math.isfinite(v) for r in records for key in ['linear_acceleration_mps2','angular_velocity_radps'] for v in r['imu'][key].values())
                     and min(age)>=0 and max(age)<.2 and travel>.1
                     and yaw_change>.005 and gyro_mean>0
                     and abs(gyro_mean-yaw_change/dt)<.05
                     and summary['image_changed'] and summary['rgb_dimensions']==[640,480]
                     and summary['lidar_sample_counts']==[360,360]
                     and summary['max_tandem_separation_error_m']<.01)
(OUT / 'summary.json').write_text(json.dumps(summary, indent=2)+'\n')
print(json.dumps(summary, indent=2))
if not summary['passed']:
    raise SystemExit('Sensor acceptance failed; see recorded packets.')
