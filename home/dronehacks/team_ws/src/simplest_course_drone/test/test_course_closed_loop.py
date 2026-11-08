"""The whole course, closed loop: planner steps -> fusion -> the real engine -> a crude drone.

The map is the default, the taped course corrected by the team's flights
(maps/course_arena_2026-11-08_flown.yaml), flown at its 1.23 m (odom 1.13 m).

The drone model is the one test_engine.py and test_fusion_closed_loop.py use (pose tracking
at 0.6 m/s, the kit's 0.3 m floor, yaw at 60 deg/s, twists followed, auto-disarm 5 s after landing,
Offboard only once our node streams), plus skybeta-11's measured airframe push: after every yaw of
dpsi the drone slides to the left of its new nose by up to 0.18 m * sin(dpsi/2) / sin(45 deg),
peaking 2 s after the turn and under 5 cm by 8 s (docs/flight-findings.md, flight 20261108-013018).
PX4 does not see that slide as an error (it is its integrator relearning), so it is added on top of
PX4's tracking. Sensors read the truth unless a test spoils them. The referee is course.py's
Progress, fed the true position, and the nearest wall along the way.

What this can show: the steps, the trust gate and the engine work together on this map's geometry,
and flying at a fixed heading takes the post-yaw drift out of play (turns=True: the turn rule keeps
the drift measured on a good day off the walls).
What it cannot: the real drone's full dynamics, latency, the real tunnel, how far the real course
sits from the map (two real strikes happened where the taped map was empty).
"""
import math
import pathlib

import simplest_course_drone.route_steps as route_steps
from simplest_course_drone.planner.course_map import load_map
from simplest_course_drone.planner.route import plan
from simplest_course_drone.route_steps import course_steps
from simplest_course_drone.fusion import Config, Fusion, Readings
from simplest_course_drone.course import DRONE_EXTENT, CourseFrame, Progress, _along_across, nearest_wall
from simplest_course_drone.engine import DONE, Mission

MAP = load_map(pathlib.Path(__file__).parents[1] / 'maps' / 'course_arena_2026-11-08_flown.yaml')
FRAME = CourseFrame.from_map(MAP)
ROUTE = plan(MAP)
TUNNEL = next(g for g in MAP.goals if g.name == 'tunnel')
ALL = ['tunnel', 'gate_b', 'pad']
DT = 0.05
PUSH_90_M = 0.18          # worst drift after a 90 deg yaw on skybeta-11


def in_tunnel(e, n):
    along, across = _along_across(TUNNEL, e, n)
    return abs(along) <= TUNNEL.depth / 2 and abs(across) <= float(TUNNEL.params['width']) / 2


def push_shape(s):
    """0 at the end of the turn, 1 at 2 s, 0.2 at 8 s."""
    return 0.0 if s <= 0 else (s / 2.0) * math.exp(1.0 - s / 2.0)


def fly(until=None, fault=None, push=True, max_t=500.0, cfg=Config(settle_s=5.0),
        cam_turn_deg=0.0, cam_shift=(0.0, 0.0, 0.0), px4_shift=(0.0, 0.0, 0.0), turns=False, push_90=PUSH_90_M):
    """One flight from the launch mark. The truth frame is the launch mark facing the launch heading.
    The camera's frame may be anywhere: turned by cam_turn_deg and shifted by cam_shift (where cuVSLAM
    started, and what it drifted by since). PX4's frame follows the camera's axes (EKF2 fuses vision
    yaw) but may be shifted by px4_shift (its own origin, its rangefinder height)."""
    th = math.radians(cam_turn_deg)
    ct, st = math.cos(th), math.sin(th)
    to_cam = lambda px, py, pz: (ct * px - st * py + cam_shift[0], st * px + ct * py + cam_shift[1], pz + cam_shift[2])
    to_px4 = lambda c: tuple(a + b for a, b in zip(c, px4_shift))
    def px4_to_truth(px, py, pz):
        cx, cy, cz = px - px4_shift[0] - cam_shift[0], py - px4_shift[1] - cam_shift[1], pz - px4_shift[2] - cam_shift[2]
        return ct * cx + st * cy, -st * cx + ct * cy, cz
    steps = course_steps(ROUTE, FRAME, MAP, until, turns)
    m, f, ref = Mission(steps), Fusion(cfg), Progress(MAP)
    x = y = z = yaw = 0.0                  # where PX4's own tracking puts the drone
    turns, turning_from, last_yaw = [], None, 0.0   # (end time, size m, unit left of new nose)
    armed, offboard, landed_since = True, False, None
    t = 0.0
    out = {'clearance': math.inf, 'where': None, 'coasted': 0.0, 'start': None, 'path': [], 'top': 0.0, 'yawed': 0.0}
    while t < max_t and m.state != DONE:
        t += DT
        dx = dy = 0.0                      # the airframe push on top of PX4's tracking
        if push:
            for t_end, size, (ux, uy) in turns:
                g = size * push_shape(t - t_end)
                dx, dy = dx + g * ux, dy + g * uy
        tx, ty = x + dx, y + dy            # the truth
        landed = z <= 1e-6
        if landed and offboard:
            landed_since = t if landed_since is None else landed_since
            if t - landed_since >= 5.0:
                armed = offboard = False
        else:
            landed_since = None
        cam = to_cam(tx, ty, z)
        cyaw = yaw + th
        r = Readings(camera=cam + (cyaw,), camera_t=t, vo_state=1, vo_t=t, ekf=to_px4(cam), ekf_xy_valid=True,
                     dist_bottom=z + 0.10, ekf_t=t, px4_yaw_enu=cyaw, att_t=t, ev_pos=True, ev_yaw=True,
                     rng_hgt=True, flags_t=t, texture=0.15, brightness=100.0, image_t=t)
        e, n = FRAME.to_course(tx, ty)
        if fault:
            fault(t, r, (e, n, z), m)
        b = f.update(t, r, m.state != 'WAIT')
        out['coasted'] += DT if b.coasting else 0.0
        cmd = m.step(t, b.pose, t if b.pose else None, offboard, landed, armed, b.yaw, to_px4(cam))
        if armed and not offboard and cmd.kind == 'pose':
            offboard, out['start'] = True, t
        if offboard and cmd.kind == 'pose':
            gx, gy, gz = px4_to_truth(cmd.x, cmd.y, cmd.z)                # PX4 flies it in its frame
            d = [gx - x, gy - y, max(gz, 0.3) - z]
            k = min(1.0, 0.6 * DT / math.sqrt(sum(v * v for v in d))) if any(d) else 0.0
            x, y, z = x + d[0] * k, y + d[1] * k, z + d[2] * k
            if cmd.yaw is not None:
                err = (cmd.yaw - th - yaw + math.pi) % (2 * math.pi) - math.pi
                yaw += max(-math.radians(60) * DT, min(math.radians(60) * DT, err))
        elif offboard and cmd.kind == 'twist':                            # a twist is in the camera's axes
            vx, vy = ct * cmd.x + st * cmd.y, -st * cmd.x + ct * cmd.y
            x, y, z = x + vx * DT, y + vy * DT, max(0.0, z + cmd.z * DT)
        # a turn ends when the nose stops moving: register its push
        moving = abs(yaw - last_yaw) > math.radians(0.2)
        if moving and turning_from is None:
            turning_from = last_yaw
        elif not moving and turning_from is not None:
            dpsi = yaw - turning_from
            if abs(dpsi) > math.radians(5):
                size = push_90 * abs(math.sin(dpsi / 2)) / math.sin(math.radians(45))
                turns.append((t, size, (-math.sin(yaw), math.cos(yaw))))   # left of the new nose (odom)
            turning_from = None
        out['yawed'] += abs(yaw - last_yaw)
        last_yaw = yaw
        e, n = FRAME.to_course(x + dx, y + dy)
        ref.update(e, n, landed=z <= 1e-6, armed=armed)
        out['top'] = max(out['top'], z)
        if z > 0.05:
            c = nearest_wall(MAP, e, n)[0]
            if c < out['clearance']:
                out['clearance'], out['where'] = c, (round(e, 2), round(n, 2), round(t, 1))
        out['path'].append((t, e, n, z, m.reason))
    out['time'] = t - (out['start'] or 0.0)
    return m, ref, out


def test_the_whole_course_every_checkpoint_in_order_no_prop_touches():
    m, ref, out = fly()
    assert m.state == DONE and m.reason == 'disarmed on the ground', m.reason
    assert ref.passed == ALL, ref.passed
    assert out['clearance'] > DRONE_EXTENT + 0.02, out['clearance']   # props 2 cm+ off, with yaw drift
    assert out['time'] < 180.0, out['time']                            # inside the 3-minute limit
    assert not any('took over' in p[4] or 'pose' in p[4] for p in out['path'])


def test_the_nose_never_yaws_so_a_stronger_push_changes_nothing():
    """003042's push (0.25 m/s^2) predicts 25-35 cm of slide per 90 deg yaw, twice 013018's. With
    the nose held at the launch heading there is no yaw, so the flight is the same at any push."""
    runs = [fly(push_90=p) for p in (PUSH_90_M, 0.35, 0.6)]
    for m, ref, out in runs:
        assert ref.passed == ALL and out['yawed'] < math.radians(1), out['yawed']
        assert out['clearance'] > DRONE_EXTENT + 0.02 and out['time'] < 180.0
    assert len({round(out['clearance'], 6) for _, _, out in runs}) == 1


def test_the_turn_rule_clears_the_good_day_drift():
    """turns=True, the option for a better camera view: fine at 013018's 18 cm."""
    m, ref, out = fly(turns=True)
    assert m.state == DONE and ref.passed == ALL, (m.reason, ref.passed)
    assert out['clearance'] > DRONE_EXTENT + 0.02, out['clearance']


def test_turning_at_every_corner_would_put_a_prop_on_a_wall(monkeypatch):
    """The old rule (face every leg) against the same measured drift: why the nose stays put."""
    monkeypatch.setattr(route_steps, 'TURN_ROOM_M', 0.0)
    m, ref, out = fly(turns=True)
    assert out['clearance'] < DRONE_EXTENT, (out['clearance'], out['where'])


def test_the_course_with_a_blind_tunnel():
    """Camera blind, tracking lost and EKF2 not fusing vision for the whole tube: coast through."""
    def dark(t, r, where, m):
        if in_tunnel(where[0], where[1]):
            r.brightness, r.vo_state, r.ev_pos = 2.0, 2, False
    m, ref, out = fly(fault=dark)
    assert out['coasted'] > 3.0, out['coasted']           # it really flew blind
    assert ref.passed == ALL, ref.passed
    assert out['clearance'] > DRONE_EXTENT


def test_a_camera_frozen_at_take_off_never_flies_away():
    """Flight 20261108-113426: odometry stuck at take-off, still 'tracking'; PX4 climbed to 3.7 m."""
    stuck = {}
    def frozen(t, r, where, m):
        if m.state == 'FLY':
            stuck.setdefault('pose', r.camera)
            r.camera = stuck['pose']
    m, ref, out = fly(fault=frozen, max_t=60.0)
    assert m.state == DONE, m.reason
    assert out['top'] < 0.6, out['top']                   # caught low, then down
    assert ref.passed == []


def test_flown_one_piece_at_a_time():
    """until: each piece passes its own checkpoints, then lands clear of every wall."""
    for goal, passed in {'tunnel': ['tunnel'], 'gate_b': ['tunnel', 'gate_b']}.items():
        m, ref, out = fly(until=goal)
        assert m.state == DONE and m.reason == 'disarmed on the ground', (goal, m.reason)
        assert ref.passed == passed, (goal, ref.passed)
        _, e, n, z, _ = out['path'][-1]
        assert z == 0.0 and nearest_wall(MAP, e, n)[0] > DRONE_EXTENT, (goal, e, n)


def test_the_camera_frame_can_be_anywhere_and_flights_follow_each_other_without_a_restart():
    """No global localization: the steps are measured from each take-off pose. Flight 1 with the
    camera's origin where 20261108-114136 found it (-79.7, 1.9) and its axes turned 37 deg, PX4's
    frame 5 m off the camera's; flight 2 straight after (the node starts a fresh Fusion and Mission
    once PX4 disarms), the drone carried back to the mark while the tracker drifted again."""
    first = fly(cam_turn_deg=37.0, cam_shift=(-79.7, 1.9, 0.0), px4_shift=(5.0, -3.0, 0.08))
    second = fly(cam_turn_deg=41.5, cam_shift=(-78.4, 3.2, 0.05), px4_shift=(5.0, -3.0, 0.08))
    for m, ref, out in (first, second):
        assert m.state == DONE and m.reason == 'disarmed on the ground', m.reason
        assert ref.passed == ALL, ref.passed
        assert out['clearance'] > DRONE_EXTENT + 0.02, out['clearance']


def test_a_tracker_that_drifted_in_height_on_the_ground_is_refused():
    """20261108-114136: after a tracker error the camera read 9.02 m with the drone on the floor."""
    m, ref, out = fly(cam_shift=(-79.66, 1.86, 9.02), px4_shift=(0.0, 0.0, -8.96), max_t=40.0)
    assert m.state == 'WAIT' and out['top'] == 0.0
    assert 'heights disagree by 8.96 m' in m.reason, m.reason
