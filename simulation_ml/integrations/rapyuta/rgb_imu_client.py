"""Fetch timestamped JPEG + IMU samples from the Windows tandem during Play.

Read-only: this client never sends motor commands. JPEG decoding yields RGB
with Pillow, or BGR with OpenCV's default imread. One image reader per car.
"""
import argparse
import base64
import json
import socket
import time
from pathlib import Path


class SensorClient:
    def __init__(self, port=7447):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.connect(('127.0.0.1', port))
        self.sock.settimeout(1.0)

    def close(self):
        self.sock.close()

    def request(self, data, accept):
        for _ in range(3):
            self.sock.send(json.dumps(data).encode())
            deadline = time.monotonic() + 1.0
            while time.monotonic() < deadline:
                try:
                    reply = json.loads(self.sock.recv(65535))
                except socket.timeout:
                    break
                if accept(reply):
                    return reply
        raise TimeoutError('No matching sensor reply. Start Play and check the updated plugin/port.')

    def sensors(self):
        return self.request({'sensors': True}, lambda r: r.get('schema') in ('tandem_sensors_v1', 'f1_sensors_v1'))

    def camera(self):
        # A different image reader can replace the server's one cached JPEG.
        # Explicit sequence checks prevent mixing chunks from different images.
        for _ in range(3):
            frame = self.request({'rgb': True}, lambda r: 'rgb' in r and 'valid' in r)
            if not frame['valid'] or not frame['rgb']['valid']:
                raise RuntimeError('RGB capture failed; inspect the Unreal output log.')
            meta = frame['rgb']
            chunks = []
            for index in range(meta['chunk_count']):
                reply = self.request({'rgb_sequence': meta['sequence'], 'rgb_chunk': index},
                                     lambda r: r.get('rgb_chunk') == index)
                if not reply.get('valid') or reply['rgb_sequence'] != meta['sequence']:
                    break
                chunks.append(base64.b64decode(reply['data_base64'], validate=True))
            else:
                jpeg = b''.join(chunks)
                if len(jpeg) != meta['byte_count'] or not jpeg.startswith(b'\xff\xd8') or not jpeg.endswith(b'\xff\xd9'):
                    raise RuntimeError('Incomplete or invalid JPEG payload')
                return jpeg, frame
        raise RuntimeError('Camera frame repeatedly replaced; use one RGB reader per car.')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port', type=int, default=7447)
    parser.add_argument('--frames', type=int, default=1)
    parser.add_argument('--interval', type=float, default=0.2)
    parser.add_argument('--output', type=Path, default=Path('simulation_ml/integrations/rapyuta/validation/rgb_imu'))
    args = parser.parse_args()
    if args.frames < 1 or not 0 <= args.interval <= 60:
        parser.error('frames must be positive; interval must be between 0 and 60 seconds')
    args.output.mkdir(parents=True, exist_ok=True)
    client = SensorClient(args.port)
    try:
        for i in range(args.frames):
            jpeg, meta = client.camera()
            meta['sensor_packet_after_capture'] = client.sensors()
            stem = f'frame_{i:04d}'
            (args.output / (stem + '.jpg')).write_bytes(jpeg)
            (args.output / (stem + '.json')).write_text(json.dumps(meta, indent=2) + '\n')
            print(json.dumps({'file': str(args.output / (stem + '.jpg')), 'rgb_sequence': meta['rgb']['sequence'],
                              'image_stamp_s': meta['rgb']['stamp_s'], 'imu': meta['imu']}))
            if i + 1 < args.frames:
                time.sleep(args.interval)
    finally:
        client.close()


if __name__ == '__main__':
    main()
