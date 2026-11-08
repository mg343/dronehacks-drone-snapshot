"""Fusion + the real step engine against a crude drone, with faults injected.

The drone model is simplest_square_drone's test model (pose tracking at 0.6 m/s, the kit's 0.3 m
floor, yaw at 60 deg/s, twists followed, auto-disarm 5 s after landing), plus the kit's rule that Offboard only
starts once our node streams a setpoint after the pilot arms. Every sensor reads the truth unless a
fault says otherwise.
"""
import math

from simplest_fusion_drone.fusion import Config, Fusion, Readings
from simplest_square_drone.engine import DONE, Mission
from simplest_square_drone.steps import STEPS

DT = 0.05
START_YAW = 0.4


def fly(fault=None, cfg=Config(settle_s=5.0), max_t=260.0):
    """fault(t, readings, truth) may spoil readings. Returns (mission, log of (t, state, belief, cmd, xyz))."""
    m, f = Mission(STEPS), Fusion(cfg)
    x = y = z = 0.0
    yaw = START_YAW
    armed, offboard, landed_since = True, False, None      # the pilot armed on the pad at t=0
    log, t = [], 0.0
    while t < max_t and m.state != DONE:
        t += DT
        landed = z <= 1e-6
        if landed and offboard:
            landed_since = t if landed_since is None else landed_since
            if t - landed_since >= 5.0:
                armed = offboard = False
        else:
            landed_since = None
        r = Readings(camera=(x, y, z, yaw), camera_t=t, vo_state=1, vo_t=t,
                     ekf=(x + 0.02, y, z), ekf_xy_valid=True, dist_bottom=z + 0.07, ekf_t=t,
                     px4_yaw_enu=yaw, att_t=t, ev_pos=True, ev_yaw=True, rng_hgt=True, flags_t=t,
                     texture=0.15, brightness=100.0, image_t=t)
        if fault:
            fault(t, r)
        b = f.update(t, r, m.state != 'WAIT')
        cmd = m.step(t, b.pose, r.camera_t if b.pose else None, offboard, landed, armed, b.yaw,
                     (x + 0.02, y, z))
        if armed and not offboard and cmd.kind == 'pose':
            offboard = True                                  # the kit enters Offboard on our stream
        if offboard and cmd.kind == 'pose':
            d = [cmd.x - 0.02 - x, cmd.y - y, max(cmd.z, 0.3) - z]
            n = math.sqrt(sum(v * v for v in d))
            k = min(1.0, 0.6 * DT / n) if n else 0.0
            x, y, z = x + d[0] * k, y + d[1] * k, z + d[2] * k
            if cmd.yaw is not None:                          # PX4 turns toward it at up to 60 deg/s
                e = (cmd.yaw - yaw + math.pi) % (2 * math.pi) - math.pi
                yaw += max(-math.radians(60) * DT, min(math.radians(60) * DT, e))
        elif offboard and cmd.kind == 'twist':
            x, y, z = x + cmd.x * DT, y + cmd.y * DT, max(0.0, z + cmd.z * DT)
        log.append((t, m.state, b, cmd, (x, y, z), m.reason))
    return m, log


def first(log, pred):
    return next((e for e in log if pred(e)), None)


def test_clean_run_settles_then_flies_the_whole_square_and_circle():
    m, log = fly()
    assert m.state == DONE and m.reason == 'disarmed on the ground', m.reason
    takeoff = first(log, lambda e: e[1] == 'FLY')
    assert 5.0 <= takeoff[0] < 5.2                        # nothing streamed during the 5 s settle
    assert all(e[3].kind == 'none' for e in log if e[0] < 4.9)
    assert max(e[4][2] for e in log) > 0.7
    assert max(e[4][0] for e in log) > 0.95 and max(e[4][1] for e in log) > 0.95   # the 1 m box
    reasons = [e[5] for e in log]
    assert all('step {}/20'.format(i) in ' '.join(reasons) for i in range(1, 21))   # every step, in turn
    assert not any('took over' in r or 'pose' in r for r in reasons)               # no timeout or hold


def test_heading_mismatch_never_takes_off():
    def turned(t, r):
        r.px4_yaw_enu = START_YAW + math.radians(5)
    m, log = fly(turned, max_t=40.0)
    assert m.state == 'WAIT' and all(e[3].kind == 'none' for e in log)
    assert 'heading: PX4 and camera 5.0 deg apart' in log[-1][2].words()


def _holds_then_lands(start, log):
    """From start: holds still (zero twist), then straight down within ~3 s, then disarms."""
    during = [e for e in log if start + 0.1 <= e[0] < start + 2.8]
    assert during and all(e[3].kind == 'twist' and (e[3].x, e[3].y, e[3].z) == (0, 0, 0) for e in during)
    late = [e for e in log if start + 3.2 <= e[0] < start + 4.0]
    assert late and all(e[3].kind == 'twist' and e[3].z < 0 for e in late)


def _coasts(start, end, log):
    """Between start and end the drone keeps flying its steps on EKF2's position."""
    span = [e for e in log if start + 0.1 <= e[0] < end - 0.1]
    assert span and all(e[2].coasting and e[3].kind == 'pose' for e in span)


def test_camera_lost_for_good_coasts_6_s_then_holds_then_lands():
    def lost(t, r):
        if t >= 20.0:
            r.vo_state = 2
    m, log = fly(lost)
    _coasts(20.0, 26.0, log)
    _holds_then_lands(26.0, log)
    assert m.state == DONE and log[-1][4][2] == 0.0
    assert 'coast: over 6 s' in first(log, lambda e: e[0] >= 26.1)[2].words()


def test_blank_view_for_good_coasts_then_lands():
    """The case cuVSLAM itself never reports: it keeps 'tracking' on a blank wall."""
    def blank(t, r):
        if t >= 25.0:
            r.texture = 0.001
    m, log = fly(blank)
    _coasts(25.0, 31.0, log)
    _holds_then_lands(31.0, log)
    assert 'image: blank' in first(log, lambda e: e[0] >= 25.1)[2].words()


def test_ekf2_drift_holds_then_lands_without_coasting():
    """Camera and EKF2 disagree while both look healthy: no way to tell which is wrong."""
    def drifting(t, r):
        if t >= 30.0:
            r.ekf = (r.ekf[0] + 0.5, r.ekf[1], r.ekf[2])
    m, log = fly(drifting)
    _holds_then_lands(30.0, log)
    assert not any(e[2].coasting for e in log)


def test_a_dark_stretch_like_the_tunnel_is_flown_through():
    """5 s of darkness with tracking lost (the tube at 0.3 m/s): coast, then carry on and finish."""
    def tunnel(t, r):
        if 30.0 <= t < 35.0:
            r.brightness, r.vo_state = 3.0, 2
    m, log = fly(tunnel)
    _coasts(30.0, 35.0, log)
    assert m.state == DONE and m.reason == 'disarmed on the ground', m.reason
    assert not any('took over' in e[5] for e in log)


def test_vision_back_from_a_new_origin_is_re_anchored():
    """cuVSLAM restarts 0.6 m off after losing track: no jump, no drift alarm, the flight finishes."""
    def restart(t, r):
        if 30.0 <= t < 32.0:
            r.vo_state = 2
        if t >= 32.0:
            r.camera = (r.camera[0] + 0.6, r.camera[1] - 0.2, r.camera[2], r.camera[3])
    m, log = fly(restart)
    assert m.state == DONE and m.reason == 'disarmed on the ground', m.reason
    assert any('re-anchored the camera by 0.6' in e[2].words() for e in log)
    assert not any('drift' in e[2].words() for e in log)
    path = [e[4] for e in log if 31.5 <= e[0] < 33.0]
    assert max(math.dist(a, b) for a, b in zip(path, path[1:])) < 0.05     # no jump in the flight


def test_a_short_blip_only_pauses():
    """Under 3 s untrusted: the drone holds, then carries on and finishes."""
    def blip(t, r):
        if 22.0 <= t < 23.0:
            r.vo_state = 2
    m, log = fly(blip)
    assert m.state == DONE and m.reason == 'disarmed on the ground', m.reason
