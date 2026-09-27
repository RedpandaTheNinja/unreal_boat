"""Read-only localhost F1 telemetry bridge; Python standard library only."""
import argparse
import json
import math
from pathlib import Path
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlsplit

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[3]
sys.path.insert(0, str(HERE.parent))
from rgb_imu_client import SensorClient


def track_projection(packet, track):
    # Telemetry bridge: UE_X=10000+100*x. Track has its own UE origin.
    origin = track['ue_origin_cm']
    x = packet['x_m'] + (10000-origin[0])/100
    y = packet['y_m'] + origin[1]/100
    points = track['centerline_m']
    best = None
    cumulative = 0.0
    for i, a in enumerate(points):
        b = points[(i+1) % len(points)]
        dx, dy, dz = (b[j]-a[j] for j in range(3))
        planar = dx*dx+dy*dy
        length = math.sqrt(planar+dz*dz)
        if planar > 1e-12:
            t = max(0., min(1., ((x-a[0])*dx+(y-a[1])*dy)/planar))
            dist2 = (x-a[0]-t*dx)**2+(y-a[1]-t*dy)**2
            if best is None or dist2 < best[0]:
                cross = (dx*(y-a[1])-dy*(x-a[0]))/math.sqrt(planar)
                heading = math.atan2(dy, dx)
                delta = packet['yaw_rad']-heading
                envelope = .30*abs(math.sin(delta))+.125*abs(math.cos(delta))
                best = (dist2, cumulative+t*length, cross, track['width_m']/2-abs(cross)-envelope)
        cumulative += length
    return dict(x_m=x, y_m=y, progress_m=best[1], progress_pct=100*best[1]/cumulative,
                cross_track_m=best[2], estimated_edge_margin_m=best[3], lap_length_m=cumulative)


def lidar_points(packet):
    """Transform timestamped sensor-frame returns to current body yaw frame."""
    points = []
    yaw = packet['yaw_rad']; c, s = math.cos(yaw), math.sin(yaw)
    for key in ('front_scan', 'rear_scan'):
        scan = packet.get(key, {})
        if not scan.get('valid') or packet['time_s']-scan.get('stamp_s', -100) > .5:
            continue
        pose = scan['sensor_pose_ground_truth']
        qx, qy, qz, qw = (pose[k] for k in ('qx','qy','qz','qw'))
        for i, r in enumerate(scan['ranges_m']):
            if r is None or not math.isfinite(r) or not scan['range_min_m'] <= r <= scan['range_max_m']:
                continue
            a = scan['angle_min_rad']+i*scan['angle_increment_rad']
            vx, vy = r*math.cos(a), r*math.sin(a)
            # Quaternion rotation, with local ray z=0.
            wx = (1-2*(qy*qy+qz*qz))*vx+2*(qx*qy-qz*qw)*vy+pose['x_m']-packet['x_m']
            wy = 2*(qx*qy+qz*qw)*vx+(1-2*(qx*qx+qz*qz))*vy+pose['y_m']-packet['y_m']
            points.append([c*wx+s*wy, -s*wx+c*wy, 0 if key=='front_scan' else 1])
    return points


class Feed:
    def __init__(self, port, track):
        self.port, self.track = port, track
        self.lock = threading.Lock()
        self.packet = None; self.jpeg = None; self.camera_meta = None
        self.received = self.advanced = self.camera_received = 0.
        self.error = 'Start Play in Unreal to connect.'
        self.camera_error = ''
        self.stop = threading.Event()

    def run(self):
        client = SensorClient(self.port)
        next_camera = 0.
        try:
            while not self.stop.is_set():
                start = time.monotonic()
                try:
                    packet = client.sensors()  # No speed, steer, throttle or brake fields.
                    if packet.get('schema') != 'f1_sensors_v1':
                        raise RuntimeError('Expected F1 telemetry on this port.')
                    packet['track'] = track_projection(packet, self.track)
                    packet['lidar_points_body_m'] = lidar_points(packet)
                    with self.lock:
                        if self.packet is None or packet['time_s'] != self.packet['time_s']:
                            self.advanced = time.monotonic()
                        self.packet = packet; self.received = time.monotonic(); self.error = ''
                    if start >= next_camera:
                        next_camera = start+.2
                        try:
                            jpeg, meta = client.camera()
                            with self.lock:
                                self.jpeg = jpeg; self.camera_meta = meta['rgb']
                                self.camera_received = time.monotonic(); self.camera_error = ''
                        except (OSError, ValueError, RuntimeError, KeyError) as exc:
                            with self.lock: self.camera_error = str(exc)
                except (OSError, ValueError, RuntimeError, KeyError) as exc:
                    with self.lock: self.error = str(exc)
                    self.stop.wait(.25)
                self.stop.wait(max(0, .1-(time.monotonic()-start)))
        finally:
            client.close()

    def snapshot(self):
        with self.lock:
            now = time.monotonic()
            age = now-self.received if self.packet else None
            live = age is not None and age < 1.5 and not self.error
            paused = live and now-self.advanced > 1.5
            return dict(bridge='mini_f1_dashboard_v1', status='paused' if paused else 'live' if live else 'offline',
                        age_s=age, error=self.error, telemetry=self.packet,
                        camera=self.camera_meta, camera_age_s=now-self.camera_received if self.jpeg else None,
                        camera_error=self.camera_error)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port', type=int, default=8765)
    parser.add_argument('--udp-port', type=int, default=7449)
    args = parser.parse_args()
    track = json.loads((ROOT/'track/generated/track.json').read_text())
    feed = Feed(args.udp_port, track)
    class Handler(BaseHTTPRequestHandler):
        def do_GET(self):
            path = urlsplit(self.path).path
            if path == '/api/state':
                body = json.dumps(feed.snapshot(), allow_nan=False).encode(); mime='application/json'
            elif path == '/api/track':
                body=json.dumps(track).encode(); mime='application/json'
            elif path == '/camera.jpg':
                with feed.lock: body=feed.jpeg
                if body is None: self.send_error(503, 'Camera not ready'); return
                mime='image/jpeg'
            elif path == '/': body=(HERE/'index.html').read_bytes(); mime='text/html; charset=utf-8'
            elif path == '/dashboard.js': body=(HERE/'dashboard.js').read_bytes(); mime='text/javascript; charset=utf-8'
            else: self.send_error(404); return
            self.send_response(200)
            self.send_header('Content-Type',mime); self.send_header('Content-Length',str(len(body)))
            self.send_header('Cache-Control','no-store'); self.send_header('X-Content-Type-Options','nosniff')
            self.end_headers()
            try: self.wfile.write(body)
            except (BrokenPipeError, ConnectionResetError): pass
        def log_message(self, *_): pass
    server = ThreadingHTTPServer(('127.0.0.1',args.port),Handler)
    threading.Thread(target=feed.run,daemon=True).start()
    print(f'F1 dashboard http://127.0.0.1:{args.port} — read-only UDP {args.udp_port}',flush=True)
    try: server.serve_forever()
    except KeyboardInterrupt: pass
    finally: feed.stop.set(); server.server_close()


if __name__ == '__main__': main()
