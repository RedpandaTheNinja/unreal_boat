"""Validate a recorded capture against a box whose near face is x=-9.30 m.

Validation fixture: UE cube center (9075,700,25) cm, dimensions (10,40,50) cm,
tandem at saved spawn (9000,700,3), stationary. Remove fixture after testing.
"""
import argparse
import json
import math
from pathlib import Path


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('capture', type=Path)
    args = p.parse_args()
    frames = [json.loads(line) for line in args.capture.read_text().splitlines() if line]
    assert frames, 'No frames'
    report = {}
    for name in ('front_scan', 'rear_scan'):
        errors, sequences = [], set()
        for frame in frames:
            scan = frame[name]
            assert scan['valid'], name
            assert len(scan['ranges_m']) == 360
            assert 0 <= frame['time_s'] - scan['stamp_s'] < .5
            assert all(v is None or (math.isfinite(v) and .12 <= v <= 3.51) for v in scan['ranges_m'])
            pose = scan['sensor_pose_ground_truth']
            forward_x = 1 - 2 * (pose['qy'] ** 2 + pose['qz'] ** 2)
            expected = (-9.30 - pose['x_m']) / forward_x
            measured = scan['ranges_m'][0]
            assert measured is not None, 'Box missing from forward ray'
            errors.append(abs(expected - measured))
            sequences.add(scan['sequence'])
        assert max(errors) < .015, (name, max(errors))
        assert len(sequences) > 1, 'No sensor updates'
        report[name] = {'unique_scans': len(sequences), 'max_range_error_m': max(errors)}
    out = args.capture.with_suffix('.validation.json')
    out.write_text(json.dumps(report, indent=2) + '\n')
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()
