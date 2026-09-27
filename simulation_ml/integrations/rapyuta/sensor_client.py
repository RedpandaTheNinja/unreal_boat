"""Record lidar frames or try a slow, lidar-based obstacle-stop baseline.

This baseline does not follow the track or avoid low obstacles below the scan.
Run with PIE active. Recording is read-only; --drive sends velocity commands.
"""
import argparse
import json
import math
import socket
import time
from pathlib import Path


def obstacle_stop(frame, speed=0.10):
    """Return (v, yaw, reason), using only front scan for forward obstacle stop.

    Conservative 0.36 m wide corridor and 0.65 m stop distance from lidar.
    This is an example controller, not a collision-free navigation guarantee.
    """
    scan = frame.get('front_scan', {})
    age = frame.get('time_s', 0) - scan.get('stamp_s', -100)
    if not scan.get('valid') or not 0 <= age <= 0.2:
        return 0.0, 0.0, 'missing_or_stale_scan'
    ranges = scan.get('ranges_m', [])
    increment = scan.get('angle_increment_rad', 0)
    if len(ranges) < 4 or increment <= 0:
        return 0.0, 0.0, 'invalid_scan'
    for i, distance in enumerate(ranges):
        if distance is None:  # no return within range; not a measured free-space guarantee
            continue
        if not isinstance(distance, (int, float)) or not math.isfinite(distance) or distance < 0:
            return 0.0, 0.0, 'invalid_range'
        angle = scan['angle_min_rad'] + i * increment
        x, y = distance * math.cos(angle), distance * math.sin(angle)
        if 0 <= x <= 0.65 and abs(y) <= 0.18:
            return 0.0, 0.0, 'obstacle_ahead'
    return speed, 0.0, 'forward_clear_at_lidar_height'


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port', type=int, default=7447)
    parser.add_argument('--seconds', type=float, default=10)
    parser.add_argument('--drive', action='store_true', help='Enable the slow obstacle-stop baseline')
    parser.add_argument('--output', type=Path, default=Path('simulation_ml/integrations/rapyuta/validation/sensors.jsonl'))
    args = parser.parse_args()
    if not math.isfinite(args.seconds) or args.seconds <= 0:
        parser.error('--seconds must be finite and positive')
    args.output.parent.mkdir(parents=True, exist_ok=True)
    samples = 0
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock, args.output.open('w') as log:
        sock.settimeout(0.5)
        destination = ('127.0.0.1', args.port)
        end = time.monotonic() + args.seconds
        command = None
        try:
            while time.monotonic() < end:
                start = time.monotonic()
                request = {'sensors': True}
                if args.drive:
                    request.update(v_mps=command[0] if command else 0, yaw_radps=command[1] if command else 0)
                sock.sendto(json.dumps(request).encode(), destination)
                try:
                    frame = json.loads(sock.recv(65535))
                except socket.timeout:
                    command = (0, 0, 'transport_timeout')
                    if args.drive:
                        sock.sendto(b'{"v_mps":0,"yaw_radps":0}', destination)
                    continue
                if frame.get('schema') != 'tandem_sensors_v1':
                    raise RuntimeError('Sensor-enabled plugin is not loaded. Rebuild/restart Unreal.')
                command = obstacle_stop(frame)
                frame['example_decision'] = dict(v_mps=command[0], yaw_radps=command[1], reason=command[2])
                log.write(json.dumps(frame, allow_nan=False) + '\n')
                samples += 1
                time.sleep(max(0, .05 - (time.monotonic() - start)))
        finally:
            if args.drive:
                sock.sendto(b'{"v_mps":0,"yaw_radps":0}', destination)
    print(json.dumps({'frames': samples, 'output': str(args.output), 'driving': args.drive}))
    if samples == 0:
        raise RuntimeError('No sensor frames received. Start Play and check the UDP port.')


if __name__ == '__main__':
    main()
