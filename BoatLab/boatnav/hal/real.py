"""Real-boat backend for the Jetson Orin Nano (UNTESTED ON HARDWARE - bench test first).

Current hardware path (from simulation_ml/boat/boat_hardware.json):
    Jetson --serial--> Arduino --> continuous motor controller --> 2 fixed reversible stern motors
Planned: Cube Orange+ with Here4 RTK (MAVLink), ZED2i depth camera + webcam.

Serial protocol to the Arduino (see arduino/boat_motor_bridge/boat_motor_bridge.ino):
    "M,<left>,<right>\\n"   left/right integers in [-1000, 1000]; Arduino neutralises after 500 ms silence
    "A,<NAME>\\n"           actuator: DEPLOY | LAUNCH | RECOVER ; replies "ACK,<NAME>"
    "S\\n"                  stop (neutral)

Navigation source (boat.json "real.nav_source"):
    "mavlink": pymavlink GLOBAL_POSITION_INT + ATTITUDE from the Cube (RTK when Here4 has a fix)
    "nmea":    plain NMEA GPS on a serial port (GGA/RMC, HDT if available)
Camera: OpenCV webcam, or ZED2i via pyzed (depth) when "use_zed": true.

Everything above the HAL (vision, map, planner, pure pursuit, mission) is unchanged from simulation.
"""
from __future__ import annotations

import math
import threading
import time

import numpy as np

from .. import geometry as G
from ..geo import LocalFrame, heading_from_compass
from ..types import BoatState, CameraFrame
from .base import BoatInterface


class RealBoat(BoatInterface):
    name = "real"
    is_sim = False
    realtime = True

    def __init__(self, boat_cfg: dict, course: dict, dry_run: bool = False, **_):
        self.cfg = boat_cfg
        self.rc = boat_cfg.get("real", {})
        self.course = course
        self.frame = LocalFrame.from_course(course)
        self.dry_run = dry_run
        self.lock = threading.Lock()
        self.nav = {"t": 0.0, "lat": None, "lon": None, "heading": None, "speed": 0.0, "course": None, "yaw_rate": 0.0,
                    "fix": 0}
        self.t0 = time.monotonic()
        self._stop = threading.Event()
        self.ser = None
        if not dry_run:
            import serial  # pyserial
            self.ser = serial.Serial(self.rc.get("motor_serial", "/dev/ttyUSB0"), self.rc.get("motor_baud", 115200),
                                     timeout=0.05)
            time.sleep(2.0)            # Arduino resets when the port opens
        src = self.rc.get("nav_source", "mavlink")
        target = self._mavlink_loop if src == "mavlink" else self._nmea_loop
        self.nav_error = ""

        def guarded():
            try:
                target()
            except Exception as e:  # noqa: BLE001 - report instead of dying silently in the thread
                self.nav_error = f"{type(e).__name__}: {e}"
                print(f"[real] navigation source '{src}' failed: {self.nav_error}", flush=True)
        threading.Thread(target=guarded, daemon=True).start()
        self.cap = None
        self.zed = None
        self._prev_heading = None

    # ------------------------------------------------------------------ navigation threads
    def _mavlink_loop(self):
        from pymavlink import mavutil
        url = self.rc.get("mavlink_url", "/dev/ttyACM0")
        m = mavutil.mavlink_connection(url, baud=self.rc.get("mavlink_baud", 115200))
        m.wait_heartbeat(timeout=30)
        m.mav.request_data_stream_send(m.target_system, m.target_component, mavutil.mavlink.MAV_DATA_STREAM_ALL, 20, 1)
        while not self._stop.is_set():
            msg = m.recv_match(type=["GLOBAL_POSITION_INT", "ATTITUDE", "GPS_RAW_INT"], blocking=True, timeout=1)
            if msg is None:
                continue
            with self.lock:
                if msg.get_type() == "GLOBAL_POSITION_INT":
                    self.nav.update(lat=msg.lat / 1e7, lon=msg.lon / 1e7, t=time.monotonic() - self.t0,
                                    vn=msg.vx / 100.0, ve=msg.vy / 100.0)
                elif msg.get_type() == "ATTITUDE":
                    # ArduPilot yaw: rad, clockwise from north (NED) -> ENU CCW from east
                    self.nav.update(heading=G.wrap(math.pi / 2 - msg.yaw), yaw_rate=-msg.yawspeed)
                elif msg.get_type() == "GPS_RAW_INT":
                    self.nav["fix"] = msg.fix_type        # 5 = RTK float, 6 = RTK fixed

    def _nmea_loop(self):
        import serial
        port = serial.Serial(self.rc.get("nmea_serial", "/dev/ttyUSB1"), self.rc.get("nmea_baud", 9600), timeout=1)

        def dm(v, hemi):
            if not v:
                return None
            d = int(float(v) / 100)
            x = d + (float(v) - d * 100) / 60
            return -x if hemi in ("S", "W") else x

        while not self._stop.is_set():
            line = port.readline().decode("ascii", "ignore").strip()
            f = line.split(",")
            with self.lock:
                if line[3:6] == "GGA" and len(f) > 6 and f[2]:
                    self.nav.update(lat=dm(f[2], f[3]), lon=dm(f[4], f[5]), fix=int(f[6] or 0), t=time.monotonic() - self.t0)
                elif line[3:6] == "RMC" and len(f) > 8 and f[7]:
                    self.nav["speed"] = float(f[7]) * 0.514444
                    if f[8]:
                        self.nav["course"] = heading_from_compass(float(f[8]))
                elif line[3:6] == "HDT" and len(f) > 1 and f[1]:
                    self.nav["heading"] = heading_from_compass(float(f[1]))

    # ------------------------------------------------------------------ interface
    def state(self) -> BoatState:
        with self.lock:
            n = dict(self.nav)
        t = time.monotonic() - self.t0
        if n["lat"] is None:
            return BoatState(t, 0.0, 0.0, 0.0, valid=False, source=self.nav_error or "no GPS yet")
        x, y = self.frame.to_local(n["lat"], n["lon"])
        h = n["heading"] if n["heading"] is not None else (n["course"] if n["course"] is not None else 0.0)
        if "ve" in n:
            ve, vn = n["ve"], n["vn"]
            u = math.cos(h) * ve + math.sin(h) * vn
            v = -math.sin(h) * ve + math.cos(h) * vn
        else:
            u, v = n["speed"], 0.0
        valid = (t - n["t"]) < 1.0 and (n["heading"] is not None or n["course"] is not None)
        src = {6: "rtk_fixed", 5: "rtk_float"}.get(n.get("fix", 0), "gps")
        return BoatState(t, x, y, h, u, v, n.get("yaw_rate", 0.0), n["lat"], n["lon"], valid=valid, source=src)

    def _send(self, line: str):
        if self.dry_run or self.ser is None:
            return
        self.ser.write((line + "\n").encode("ascii"))

    def command(self, left: float, right: float):
        k = int(self.rc.get("motor_scale", 1000))
        inv = self.rc.get("invert", [False, False])
        l = float(np.clip(left, -1, 1)) * (-1 if inv[0] else 1)
        r = float(np.clip(right, -1, 1)) * (-1 if inv[1] else 1)
        self._send(f"M,{int(round(l * k))},{int(round(r * k))}")

    def stop(self):
        self._send("S")

    def actuate(self, name: str, **kw) -> dict:
        if self.dry_run or self.ser is None:
            return {"action": name, "ok": True, "dry_run": True}
        self.ser.reset_input_buffer()
        self._send(f"A,{name.upper()}")
        t0 = time.monotonic()
        while time.monotonic() - t0 < 2.0:
            line = self.ser.readline().decode("ascii", "ignore").strip()
            if line == f"ACK,{name.upper()}":
                return {"action": name, "ok": True}
        return {"action": name, "ok": False, "reason": "no ACK from Arduino"}

    def camera(self) -> CameraFrame | None:
        cam = self.cfg["camera"]
        pitch = math.radians(cam.get("pitch_down_deg", 0.0))
        st = self.state()
        if self.rc.get("use_zed"):
            return self._zed_frame(st, pitch)
        import cv2
        if self.cap is None:
            self.cap = cv2.VideoCapture(self.rc.get("camera_index", 0))
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, cam["width"])
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, cam["height"])
        ok, bgr = self.cap.read()
        if not ok:
            return None
        rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
        H, W = rgb.shape[:2]
        f = W / (2 * math.tan(math.radians(cam["hfov_deg"]) / 2))
        return CameraFrame(st.t, rgb, None, f, f, W / 2, H / 2, tuple(cam["mount_flu_m"]), pitch,
                           (st.x, st.y, st.heading), 0, "webcam")

    def _zed_frame(self, st, pitch):
        import pyzed.sl as sl
        if self.zed is None:
            self.zed = sl.Camera()
            init = sl.InitParameters(depth_mode=sl.DEPTH_MODE.PERFORMANCE, coordinate_units=sl.UNIT.METER)
            if self.zed.open(init) != sl.ERROR_CODE.SUCCESS:
                raise RuntimeError("ZED2i open failed")
            self._zimg, self._zdep = sl.Mat(), sl.Mat()
            calib = self.zed.get_camera_information().camera_configuration.calibration_parameters.left_cam
            self._zk = (calib.fx, calib.fy, calib.cx, calib.cy)
        if self.zed.grab() != sl.ERROR_CODE.SUCCESS:
            return None
        self.zed.retrieve_image(self._zimg, sl.VIEW.LEFT)
        self.zed.retrieve_measure(self._zdep, sl.MEASURE.DEPTH)
        rgb = self._zimg.get_data()[:, :, :3][:, :, ::-1].copy()     # BGRA -> RGB
        depth = np.nan_to_num(self._zdep.get_data(), nan=0.0, posinf=0.0, neginf=0.0).astype(np.float32)
        fx, fy, cx, cy = self._zk
        return CameraFrame(st.t, rgb, depth, fx, fy, cx, cy, tuple(self.cfg["camera"]["mount_flu_m"]), pitch,
                           (st.x, st.y, st.heading), 0, "zed2i")

    def close(self):
        self._stop.set()
        self.stop()
        if self.ser is not None:
            self.ser.close()
        if self.cap is not None:
            self.cap.release()
        if self.zed is not None:
            self.zed.close()
