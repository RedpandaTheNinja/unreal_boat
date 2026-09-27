"""Mission runner: the single control loop used for the twin, Unreal and the real boat.

    python -m boatnav.runner --backend twin   --mission aimm_full
    python -m boatnav.runner --backend unreal --mission aimm_full --realtime
    python -m boatnav.runner --backend real   --mission waypoints_demo --dry-run

Loop (20 Hz): state -> perception (camera_hz) -> buoy map -> executive -> motor command
-> scorer (sims) -> telemetry/logs.
"""
from __future__ import annotations

import argparse
import collections
import json
import math
import sys
import time
from pathlib import Path

import cv2

from .config import ROOT, load_boat, load_course, load_mission, load_vision
from .evaluation import Scorer
from .hal import make_backend
from .mission.context import Ctx
from .mission.executive import Executive
from .perception.detector import ColorBlobDetector, annotate
from .perception.mapping import BuoyMap
from .telemetry import TelemetryHub, serve


def parse_pair(s):
    if not s:
        return (0.0, 0.0)
    a, b = (float(v) for v in s.split(","))
    return (a, b)


class Runner:
    def __init__(self, args):
        self.a = args
        self.boat = load_boat(args.boat)
        self.course = load_course(args.course)
        self.vision = load_vision(args.vision)
        self.mission = load_mission(args.mission)
        if args.color:
            for t in self.mission["tasks"]:
                if t["type"] == "identify":
                    t["color"] = args.color
        if args.start_paused:
            self.mission["start_paused"] = True
        kw = dict(seed=args.seed)
        if args.backend == "twin":
            kw.update(realtime=args.realtime, camera=not args.no_camera)
            if args.gps_noise is not None:
                kw["gps_noise_m"] = args.gps_noise
        if args.backend == "unreal":
            kw.update(port=args.ue_port, camera=not args.no_camera, gps_noise_m=args.gps_noise, connect_wait_s=args.ue_wait)
        if args.backend == "real":
            kw.update(dry_run=args.dry_run)
        self.hal = make_backend(args.backend, self.boat, self.course, **kw)
        nav = args.nav or self.mission.get("nav_mode", "fused")
        self.bmap = BuoyMap(self.course, nav, prior_sigma_m=self.mission.get("prior_sigma_m", 1.0))
        ident = next((t.get("color") for t in self.mission["tasks"] if t["type"] == "identify"), None)
        self.ctx = Ctx(self.boat, self.course, self.bmap, self.hal, identify_color=ident)
        self.exe = Executive(self.ctx, self.mission)
        self.scorer = Scorer(self.course, self.boat, self.ctx.identify_color) if self.hal.is_sim else None
        self.ctx.on_action = (lambda res: self.scorer.record_action(res)) if self.scorer else None
        self.detector = None if args.no_camera else ColorBlobDetector(self.vision, self.boat["camera"])
        cur, wind = parse_pair(args.current), parse_pair(args.wind)
        if any(cur) or any(wind) or args.fog:
            self.hal.set_environment(current=cur, wind=wind, fog_visibility_m=args.fog)
            mag = math.hypot(*cur)
            if mag > 0.02:
                self.ctx.env_upstream = (math.atan2(-cur[1], -cur[0]), mag)
        self.hub = TelemetryHub()
        self.srv = serve(self.hub, args.api_port) if args.api_port else None
        stamp = time.strftime("%Y%m%d_%H%M%S")
        self.logdir = Path(args.log_dir)
        self.logdir.mkdir(parents=True, exist_ok=True)
        self.log_path = self.logdir / f"{stamp}_{args.backend}_{self.mission.get('name', 'mission')}.jsonl"
        self.summary_path = self.log_path.with_name(self.log_path.stem + "_summary.json")
        self.logf = open(self.log_path, "w", encoding="utf-8")
        self.cmd = (0.0, 0.0)
        self.last_dets = []
        self.cam_meta = {}
        self.jpeg = None
        self.quit = False
        self.trail = []
        self.nav_hist = collections.deque(maxlen=200)

    def nav_pose_at(self, t, st):
        """Navigation pose closest in time to t (camera capture), from the last ~10 s of states."""
        if not self.nav_hist:
            return (st.x, st.y, st.heading)
        best = min(self.nav_hist, key=lambda p: abs(p[0] - t))
        return best[1:]

    # ------------------------------------------------------------------ commands
    def handle_commands(self):
        while not self.hub.commands.empty():
            msg = self.hub.commands.get()
            cmd = msg.pop("cmd")
            if cmd == "quit":
                self.quit = True
            elif cmd == "reset":
                self.hal.reset(msg.get("pose"))
                self.bmap.__init__(self.course, self.bmap.mode)
                self.exe.__init__(self.ctx, self.mission)
                if self.scorer:
                    self.scorer = Scorer(self.course, self.boat, self.ctx.identify_color)
                    self.ctx.on_action = self.scorer.record_action
                self.trail.clear()
                self.ctx.log("Simulation reset")
            elif cmd == "env":
                res = self.hal.set_environment(current=tuple(msg.get("current", (0, 0))), wind=tuple(msg.get("wind", (0, 0))),
                                               fog_visibility_m=msg.get("fog"))
                self.ctx.log(f"Environment: {msg} -> {res}")
            else:
                if cmd == "estop":
                    self.hal.stop()
                if cmd == "set_color" and self.scorer:
                    self.scorer.identify_color = msg.get("color", self.scorer.identify_color)
                self.exe.command(cmd, **msg)

    # ------------------------------------------------------------------ loop
    def run(self):
        a = self.a
        dt = 1.0 / self.boat["control"]["rate_hz"]
        cam_period = 1.0 / max(0.1, a.camera_hz)
        next_cam = -1.0
        next_pub = 0.0
        next_log = 0.0
        wall0 = time.monotonic()
        t_start = None
        last_t = None
        ticks = 0
        try:
            while not self.quit:
                loop_start = time.monotonic()
                st = self.hal.state()
                if last_t is not None and st.t < last_t:
                    # Sim clock restarted (twin "Reset sim" sets t = 0, Unreal Play restarted): the schedules
                    # below are in sim time, so without re-arming the camera and log would stall until the new
                    # clock caught up with the old one.
                    next_cam, next_log, t_start = -1.0, 0.0, st.t
                    self.nav_hist.clear()
                last_t = st.t
                self.nav_hist.append((st.t, st.x, st.y, st.heading))
                if t_start is None:
                    t_start = st.t
                if self.detector is not None and st.t >= next_cam:
                    next_cam = st.t + cam_period
                    frame = self.hal.camera()
                    if frame is not None:
                        if self.hal.is_sim:
                            # a real boat only knows its GPS/compass pose: place detections with the navigation
                            # pose at the frame's capture time (not sim truth, not "now" - a turning boat would
                            # smear detections sideways by range * yaw_rate * latency)
                            frame.pose = self.nav_pose_at(frame.t, st)
                        dets = self.detector.detect(frame)
                        self.bmap.update(dets, st.t)
                        self.last_dets = dets
                        self.cam_meta = {"sequence": frame.sequence, "stamp_s": frame.t, "source": frame.source,
                                         "width": frame.rgb.shape[1], "height": frame.rgb.shape[0]}
                        if self.srv is not None:
                            img = annotate(frame.rgb, dets, self.detector.horizon_row(frame))
                            ok, buf = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, 80])
                            self.jpeg = buf.tobytes() if ok else None
                self.handle_commands()
                if not st.valid:
                    # never drive on stale/absent navigation (GPS lost, Unreal paused)
                    l, r = 0.0, 0.0
                    self.ctx.phase = f"waiting for valid navigation ({st.source})"
                else:
                    l, r = self.exe.step(st)
                self.cmd = (float(l), float(r))
                self.hal.command(*self.cmd)
                truth = self.hal.truth() if self.hal.is_sim else None
                if self.scorer and truth:
                    self.scorer.update(truth)
                if st.t >= next_log:
                    next_log = st.t + 0.2
                    self._log_row(st, truth)
                    self.trail.append([round(st.x, 2), round(st.y, 2)])
                    if len(self.trail) > 4000:
                        del self.trail[:1000]
                if self.srv is not None and time.monotonic() >= next_pub:
                    next_pub = time.monotonic() + 0.1
                    self.hub.publish(self.snapshot(st, truth), self.jpeg)
                ticks += 1
                if self.exe.finished or self.exe.aborted:
                    if not a.stay:
                        break
                if a.max_time and st.t - t_start > a.max_time:
                    self.ctx.log(f"Max time {a.max_time}s reached", "warn")
                    break
                self.hal.tick(dt)                     # twin integrates physics; live backends ignore
                if self.hal.realtime:
                    time.sleep(max(0.0, dt - (time.monotonic() - loop_start)))
        except KeyboardInterrupt:
            self.ctx.log("Interrupted by user (Ctrl+C) - motors neutral", "warn")
        finally:
            self.hal.stop()
            summary = self.finish(time.monotonic() - wall0, ticks)
            self.hal.close()
        return summary

    def _log_row(self, st, truth):
        row = {"t": round(st.t, 3), "x": round(st.x, 3), "y": round(st.y, 3), "heading": round(st.heading, 4),
               "u": round(st.u, 3), "r": round(st.r, 4), "cmd": [round(self.cmd[0], 3), round(self.cmd[1], 3)],
               "phase": self.ctx.phase, "task": self.exe.index, "pp": {"mode": self.ctx.pp.info.mode,
               "xte": round(self.ctx.pp.info.cross_track, 3)}}
        if truth:
            row["truth"] = {k: (round(v, 4) if isinstance(v, float) else v) for k, v in truth.items() if k in ("x", "y", "heading", "u", "v", "r")}
        path = self.ctx.plan_path
        if path is not None and id(path) != getattr(self, "_logged_path", None):
            self._logged_path = id(path)
            row["path"] = {"name": path.name, "points": path.to_list(0.5)}
        if self.ctx.approach is not None and self.ctx.approach is not getattr(self, "_logged_appr", None):
            self._logged_appr = self.ctx.approach
            row["approach"] = self.ctx.approach
        self.logf.write(json.dumps(row) + "\n")

    def snapshot(self, st, truth):
        pp = self.ctx.pp.info
        path = self.ctx.plan_path
        return {
            "schema": "boatlab_state_v1", "status": "live", "backend": self.hal.name, "nav_mode": self.bmap.mode,
            "t": st.t, "wall": time.time(), "state": st.as_dict(), "truth": truth, "cmd": list(self.cmd),
            "phase": self.ctx.phase, "identify_color": self.ctx.identify_color,
            "pp": {"mode": pp.mode, "alpha_deg": math.degrees(pp.alpha), "lookahead": list(pp.lookahead),
                   "lookahead_m": pp.lookahead_m, "cross_track_m": pp.cross_track, "u_des": pp.u_des,
                   "r_des": pp.r_des, "remaining_m": pp.remaining, "s": pp.s},
            "path": path.to_list(0.6) if path is not None else [], "path_name": getattr(path, "name", ""),
            "approach": self.ctx.approach, "mission": self.exe.snapshot(),
            "map": self.bmap.as_list(),
            "detections": [{"label": d.label, "x": round(d.x, 2), "y": round(d.y, 2), "range_m": round(d.range_m, 2),
                            "bearing_deg": round(math.degrees(d.bearing), 1), "src": d.range_source}
                           for d in self.last_dets],
            "camera": self.cam_meta, "events": self.ctx.events[-40:], "actions": self.ctx.actions,
            "score": self.scorer.report() if self.scorer else None, "trail": self.trail[-1500:],
            "log_file": str(self.log_path),
        }

    def finish(self, wall, ticks):
        self.logf.close()
        summary = {"mission": self.exe.snapshot(), "backend": self.hal.name, "nav_mode": self.bmap.mode,
                   "wall_s": round(wall, 1), "ticks": ticks, "sim_time_s": round(self.ctx.t, 1),
                   "score": self.scorer.report() if self.scorer else None, "actions": self.ctx.actions,
                   "log": str(self.log_path)}
        self.summary_path.write_text(json.dumps(summary, indent=2, default=float))
        if self.srv is not None:
            snap, _ = self.hub.get()
            snap = dict(snap, status="finished")
            self.hub.publish(snap)
        print(json.dumps({"summary": str(self.summary_path),
                          "tasks": [(t["type"], t["status"]) for t in summary["mission"]["tasks"]],
                          "passed": summary["score"]["passed_count"] if summary["score"] else None}, indent=1), flush=True)
        return summary


def build_parser():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--backend", default="twin", choices=["twin", "unreal", "real"])
    p.add_argument("--mission", default="aimm_full")
    p.add_argument("--boat", default="boat.json")
    p.add_argument("--course", default="course_aimm_2025.json")
    p.add_argument("--vision", default="vision_colors.json")
    p.add_argument("--nav", choices=["prior", "fused", "vision"], help="buoy map mode (default from mission)")
    p.add_argument("--color", help="identify colour (blue/orange/purple/yellow); dashboard can change it live")
    p.add_argument("--realtime", action="store_true", help="twin: run at wall-clock speed (for the dashboard)")
    p.add_argument("--no-camera", action="store_true", help="disable perception (prior positions only)")
    p.add_argument("--camera-hz", type=float, default=4.0)
    p.add_argument("--gps-noise", type=float, default=None, help="sim GPS noise sigma (m); default from boat.json")
    p.add_argument("--current", help="water current 'x,y' m/s (sim)")
    p.add_argument("--wind", help="wind 'x,y' m/s (sim)")
    p.add_argument("--fog", type=float, help="fog visibility (m) for the twin camera")
    p.add_argument("--seed", type=int, default=0)
    p.add_argument("--api-port", type=int, default=8772, help="0 disables the telemetry API")
    p.add_argument("--ue-port", type=int, default=7450)
    p.add_argument("--ue-wait", type=float, default=120.0, help="unreal: seconds to wait for Play to start")
    p.add_argument("--dry-run", action="store_true", help="real backend: never open the motor serial port")
    p.add_argument("--stay", action="store_true", help="keep running (neutral) after the mission ends")
    p.add_argument("--start-paused", action="store_true")
    p.add_argument("--max-time", type=float, default=1500.0)
    p.add_argument("--log-dir", default=str(ROOT / "runs"))
    return p


def main(argv=None):
    args = build_parser().parse_args(argv)
    runner = Runner(args)
    summary = runner.run()
    return 0 if summary["mission"]["finished"] else 1


if __name__ == "__main__":
    sys.exit(main())
