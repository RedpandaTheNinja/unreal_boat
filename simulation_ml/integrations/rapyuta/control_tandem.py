"""Send SI velocity commands to the Windows tandem; report measured Chaos pose.

Run while PIE/Simulate is active. No third-party Python dependencies required.
Positive yaw means counterclockwise in the Fable XY coordinates.
"""
import argparse
import json
import math
import socket
import time
from pathlib import Path


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--speed', type=float, default=0.15)
    parser.add_argument('--yaw-rate', type=float, default=0.0)
    parser.add_argument('--seconds', type=float, default=3.0)
    parser.add_argument('--port', type=int, default=7447)
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    if not all(math.isfinite(x) for x in (args.speed, args.yaw_rate, args.seconds)) or args.seconds <= 0:
        parser.error('speed, yaw-rate and seconds must be finite; seconds must be positive')
    records = []
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
        sock.settimeout(1.0)
        endpoint = ('127.0.0.1', args.port)
        start = time.monotonic()
        try:
            while time.monotonic() - start < args.seconds:
                tick = time.monotonic()
                sock.sendto(json.dumps({'v_mps': args.speed, 'yaw_radps': args.yaw_rate}).encode(), endpoint)
                packet, _ = sock.recvfrom(65535)
                records.append(json.loads(packet))
                time.sleep(max(0, 0.05 - (time.monotonic() - tick)))
        finally:
            sock.sendto(b'{"v_mps":0,"yaw_radps":0}', endpoint)
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(records, indent=2) + '\n')
    print(json.dumps({'samples': len(records), 'first': records[0], 'last': records[-1]}, indent=2))


if __name__ == '__main__':
    main()
