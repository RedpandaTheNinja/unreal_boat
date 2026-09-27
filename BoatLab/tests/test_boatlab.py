"""BoatLab regression tests.  Run:  python -m pytest tests -q   (about 2 minutes)."""
from __future__ import annotations

import json
import math
import socket
import subprocess
import sys
import threading
import time
import urllib.request
from pathlib import Path

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from boatnav import geometry as G  # noqa: E402
from boatnav.config import load_boat, load_course, load_vision  # noqa: E402
from boatnav.control.pure_pursuit import PurePursuit, mix  # noqa: E402
from boatnav.geo import LocalFrame, compass_from_heading, heading_from_compass  # noqa: E402
from boatnav.hal.twin import TwinBoat  # noqa: E402
from boatnav.model import BoatModel  # noqa: E402
from boatnav.perception.detector import ColorBlobDetector  # noqa: E402
from boatnav.perception.mapping import BuoyMap  # noqa: E402
from boatnav.planning import course as C  # noqa: E402
from boatnav.planning.approach import desired_offset, plan_standoff  # noqa: E402
from boatnav.planning.grid import GridPlanner  # noqa: E402

BOAT, COURSE = load_boat(), load_course()


def _free_port():
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.bind(("127.0.0.1", 0))
    p = s.getsockname()[1]
    s.close()
    return p


# ------------------------------------------------------------------ model fidelity
@pytest.mark.parametrize("log,left,right", [("ue_forward.jsonl", 0.6, 0.6), ("ue_opposite_thrust.jsonl", 0.6, -0.6)])
def test_twin_matches_unreal_step_response(log, left, right):
    """The Python twin must reproduce the Unreal BoatPhysics step responses (same equations)."""
    rows = [json.loads(l) for l in (ROOT / "tests/data" / log).read_text().splitlines()]
    moving = next(i for i, r in enumerate(rows) if abs(r["left_thrust_n"]) > 1.0)
    t0 = rows[max(0, moving - 1)]["time_s"]
    tw = TwinBoat(BOAT, COURSE, camera=False, gps_noise_m=0.0)
    ue_speed, tw_speed, ue_r, tw_r = [], [], [], []
    t = 0.0
    for r in rows[max(0, moving - 1):]:
        while t < r["time_s"] - t0 - 1e-9:
            tw.command(left, right)
            tw.tick(0.025)
            t += 0.025
        v = r["velocity_ue_mps"]
        ue_speed.append(math.hypot(v[0], v[1]))
        tw_speed.append(math.hypot(tw.u, tw.v))
        ue_r.append(-r["angular_velocity_body_radps"][2])
        tw_r.append(tw.r)
    assert np.max(np.abs(np.array(ue_speed) - tw_speed)) < 0.12
    assert np.max(np.abs(np.array(ue_r) - tw_r)) < 0.06


def test_motor_curve_and_inverse():
    m = BoatModel(BOAT)
    assert m.motor_curve(0.03) == 0.0
    assert m.motor_curve(1.0) == pytest.approx(BOAT["motors"]["max_forward_n"])
    assert m.motor_curve(-1.0) == pytest.approx(-BOAT["motors"]["max_reverse_n"])
    for f in (-60.0, -5.0, 10.0, 100.0):
        assert m.motor_curve(m.cmd_for_thrust(f)) == pytest.approx(f, rel=1e-6)
    assert 1.35 < m.steady_speed(0.6) < 1.5          # Unreal log: 1.41 m/s at 6 s


# ------------------------------------------------------------------ geometry / frames
def test_geo_roundtrip_and_compass():
    fr = LocalFrame.from_course(COURSE)
    lat, lon = fr.to_geo(25.0, -12.0)
    x, y = fr.to_local(lat, lon)
    assert (x, y) == pytest.approx((25.0, -12.0), abs=1e-6)
    assert compass_from_heading(math.pi / 2) == pytest.approx(0.0)       # north
    assert heading_from_compass(90.0) == pytest.approx(0.0, abs=1e-9)    # east


def test_mixer_keeps_turn_priority():
    l, r = mix(0.9, 0.5)
    assert r - l == pytest.approx(1.0)
    assert max(abs(l), abs(r)) <= 1.0 + 1e-9
    l, r = mix(0.0, 0.3)
    assert l < 0 < r                                   # counter-clockwise = starboard motor forward, port reverse


def test_gate_direction_keeps_red_to_starboard():
    bmap = BuoyMap(COURSE)
    g = next(g for g in COURSE["gates"] if g["id"] == "gateA")
    center, d, width = C.gate_frame(bmap, g["red"], g["green"])
    red = np.array(bmap.position(g["red"]))
    right = np.array([d[1], -d[0]])
    assert np.dot(red - center, right) > 0              # red on the starboard side of travel
    assert width == pytest.approx(10 * 0.3048, abs=1e-6)


def test_slalom_points_pass_on_rule_side():
    bmap = BuoyMap(COURSE)
    gA = next(g for g in COURSE["gates"] if g["id"] == "gateA")
    gB = next(g for g in COURSE["gates"] if g["id"] == "gateB")
    buoys = COURSE["challenges"]["2_dodge"]["buoys"]
    pts = C.slalom_points(bmap, gA, gB, buoys, 1.6, 5.0, 3.0)
    axis = np.array(bmap.position(buoys[-1])) - np.array(bmap.position(buoys[0]))
    axis /= np.linalg.norm(axis)
    left = np.array([-axis[1], axis[0]])
    for b, p in zip(sorted(buoys, key=lambda b: float(np.dot(bmap.position(b), axis))), pts[2:-2]):
        side = np.dot(np.array(p) - np.array(bmap.position(b)), left)
        assert (side > 0) == (bmap.get(b).label == "red")   # boat left of red, right of green


def test_standoff_puts_target_abeam_starboard():
    bmap = BuoyMap(COURSE)
    planner = GridPlanner(COURSE, BOAT)
    objs = {k: (t.x, t.y, 0.135) for k, t in bmap.tracks.items()}
    z = bmap.position("zebra")
    appr = plan_standoff("deploy", z, tuple(COURSE["start_pose"]), objs, planner, BOAT)
    assert appr is not None
    bx, by = G.world_to_body(*appr.pose, *z)
    want = desired_offset(BOAT, "deploy")
    assert (bx, by) == pytest.approx(want, abs=1e-6)
    assert by < -1.0                                      # starboard side, clear of the hull


# ------------------------------------------------------------------ control
def test_pure_pursuit_converges_on_straight_line():
    tw = TwinBoat(BOAT, COURSE, camera=False, gps_noise_m=0.02, start_pose=(-30, 2, 0.0))
    pp = PurePursuit(BoatModel(BOAT), BOAT["control"])
    path = G.Path([(-30, 0), (30, 0)], 1.2).resampled(0.4)
    pp.set_path(path, stop_at_end=False)
    xte = []
    for _ in range(int(40 / 0.05)):
        st = tw.state()
        tw.command(*pp.update(st, 0.05))
        tw.tick(0.05)
        xte.append(path.project((tw.x, tw.y)).lateral)
    late = np.array(xte[len(xte) // 2:])
    assert np.sqrt(np.mean(late ** 2)) < 0.15


def test_pivot_turn_uses_opposite_thrust():
    tw = TwinBoat(BOAT, COURSE, camera=False, gps_noise_m=0.0, start_pose=(0, 0, 0.0))
    pp = PurePursuit(BoatModel(BOAT), BOAT["control"])
    pp.set_path(G.Path([(0, 0), (-10, 0)], 1.0).resampled(0.4))      # target directly behind
    l, r = pp.update(tw.state(), 0.05)
    assert pp.info.mode == "pivot" and l * r < 0


# ------------------------------------------------------------------ perception
def test_detector_localises_gate_buoys():
    tw = TwinBoat(BOAT, COURSE, gps_noise_m=0.0, start_pose=(28.0, 9.75, math.pi))
    det = ColorBlobDetector(load_vision(), BOAT["camera"])
    dets = det.detect(tw.camera())
    objs = {o["id"]: o for o in COURSE["objects"]}
    for oid in ("gateA_red", "gateA_green"):
        o = objs[oid]
        near = [d for d in dets if d.label == o["color"] and math.hypot(d.x - o["x"], d.y - o["y"]) < 0.3]
        assert near, f"{oid} not detected within 0.3 m"


# ------------------------------------------------------------------ end to end
def test_full_mission_on_twin_prior_positions():
    from boatnav.runner import build_parser, Runner
    args = build_parser().parse_args(["--backend", "twin", "--mission", "aimm_full", "--no-camera", "--api-port", "0",
                                      "--log-dir", str(ROOT / "runs" / "pytest")])
    summary = Runner(args).run()
    sc = summary["score"]
    assert summary["mission"]["finished"]
    assert sc["passed_count"] == 8, {k: v for k, v in sc.items() if isinstance(v, dict)}
    unintended = set(sc["contacts"]) - {"identify_blue", "return_blue"}
    assert not unintended, sc["contacts"]


def test_camera_keeps_running_after_reset_sim():
    """Dashboard "Reset sim" sets the twin clock back to 0; the runner's camera schedule must follow."""
    from boatnav.runner import build_parser, Runner
    args = build_parser().parse_args(["--backend", "twin", "--mission", "aimm_full", "--start-paused", "--api-port", "0",
                                      "--max-time", "15", "--log-dir", str(ROOT / "runs" / "pytest")])
    runner = Runner(args)
    hal = runner.hal
    renders, reset_sent = [], []
    real_state, real_camera = hal.state, hal.camera

    def state():
        st = real_state()
        if not reset_sent and st.t >= 10.0:
            runner.hub.commands.put({"cmd": "reset"})
            reset_sent.append(st.t)
        return st

    def camera():
        renders.append((len(reset_sent), hal.t))
        return real_camera()

    hal.state, hal.camera = state, camera
    runner.run()
    after = [t for n, t in renders if n and t < 5.0]
    assert reset_sent and after and after[0] < 1.0, renders[-10:]


def test_unreal_backend_against_fake_plugin():
    """UnrealBoat speaks the real plugin protocol; the fake server replies in UE axes."""
    from tools.fake_unreal import FakeUnreal
    from boatnav.hal.unreal import UnrealBoat
    port = _free_port()
    fake = FakeUnreal(port)
    threading.Thread(target=fake.serve, daemon=True).start()
    time.sleep(0.3)
    ue = UnrealBoat(BOAT, COURSE, port=port, gps_noise_m=0.0)
    st = ue.state()
    sx, sy, sh = COURSE["start_pose"]
    assert (st.x, st.y) == pytest.approx((sx, sy), abs=0.05)
    assert abs(G.wrap(st.heading - sh)) < 0.02
    t0 = time.monotonic()
    while time.monotonic() - t0 < 2.0:                   # forward thrust -> boat moves west (heading pi)
        ue.command(0.6, 0.6)
        time.sleep(0.05)
    st = ue.state()
    assert st.x < sx - 0.3 and st.u > 0.3
    frame = None
    t0 = time.monotonic()
    while frame is None and time.monotonic() - t0 < 5:
        frame = ue.camera()
        time.sleep(0.1)
    assert frame is not None and frame.rgb.shape == (480, 640, 3) and frame.depth is not None
    ue.close()


def test_dashboard_server_serves_course_and_state():
    port = _free_port()
    proc = subprocess.Popen([sys.executable, str(ROOT / "dashboard" / "server.py"), "--port", str(port),
                             "--runner", str(_free_port()), "--ue-port", str(_free_port())], cwd=str(ROOT))
    try:
        for _ in range(150):                    # startup can take several seconds on a busy PC / Google Drive
            try:
                course = json.loads(urllib.request.urlopen(f"http://127.0.0.1:{port}/api/course", timeout=1).read())
                break
            except OSError:
                time.sleep(0.1)
        assert course["course"]["name"] == "aimm_icc_2025"
        state = json.loads(urllib.request.urlopen(f"http://127.0.0.1:{port}/api/state", timeout=2).read())
        assert state["status"] == "offline"
        html = urllib.request.urlopen(f"http://127.0.0.1:{port}/", timeout=1).read()
        assert b"Autonomy dashboard" in html
    finally:
        proc.terminate()
