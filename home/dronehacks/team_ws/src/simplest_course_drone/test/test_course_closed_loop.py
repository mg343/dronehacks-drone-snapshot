"""The whole course, closed loop: planner steps -> fusion -> the real engine -> a crude drone.

The map is the default, the taped course corrected by the team's flights
(simplest_map_drone/maps/course_arena_2026-11-08_flown.yaml), flown at its 1.23 m (odom 1.13 m).

The drone model is the one simplest_square_drone and simplest_fusion_drone test with (pose tracking
at 0.6 m/s, the kit's 0.3 m floor, yaw at 60 deg/s, twists followed, auto-disarm 5 s after landing,
Offboard only once our node streams), plus skybeta-11's measured airframe push: after every yaw of
dpsi the drone slides to the left of its new nose by up to 0.18 m * sin(dpsi/2) / sin(45 deg),
peaking 2 s after the turn and under 5 cm by 8 s (docs/flight-findings.md, flight 20261108-013018).
PX4 does not see that slide as an error (it is its integrator relearning), so it is added on top of
PX4's tracking. Sensors read the truth unless a test spoils them. The referee is simplest_map_drone's
Progress, fed the true position, and the nearest wall along the way.

What this can show: the steps, the trust gate and the engine work together on this map's geometry,
and the turn rule keeps the measured post-yaw drift off the walls.
What it cannot: the real drone's full dynamics, latency, the real tunnel, how far the real course
sits from the map (two real strikes happened where the taped map was empty).
"""
import math
import pathlib

import simplest_course_drone.route_steps as route_steps
from dronehacks_planner.course_map import load_map
from dronehacks_planner.route import plan
from simplest_course_drone.route_steps import course_steps
from simplest_fusion_drone.fusion import Config, Fusion, Readings
from simplest_map_drone.course import DRONE_EXTENT, CourseFrame, Progress, _along_across, nearest_wall
from simplest_square_drone.engine import DONE, Mission

MAP = load_map(pathlib.Path(__file__).parents[2] / 'simplest_map_drone' / 'maps' / 'course_arena_2026-11-08_flown.yaml')
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


def fly(until=None, fault=None, push=True, max_t=500.0, cfg=Config(settle_s=5.0)):
    steps = course_steps(ROUTE, FRAME, MAP, until)
    m, f, ref = Mission(steps), Fusion(cfg), Progress(MAP)
    x = y = z = yaw = 0.0                  # where PX4's own tracking puts the drone
    turns, turning_from, last_yaw = [], None, 0.0   # (end time, size m, unit left of new nose)
    armed, offboard, landed_since = True, False, None
    t = 0.0
    out = {'clearance': math.inf, 'where': None, 'coasted': 0.0, 'start': None, 'path': [], 'top': 0.0}
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
        r = Readings(camera=(tx, ty, z, yaw), camera_t=t, vo_state=1, vo_t=t, ekf=(tx, ty, z), ekf_xy_valid=True,
                     dist_bottom=z + 0.10, ekf_t=t, px4_yaw_enu=yaw, att_t=t, ev_pos=True, ev_yaw=True,
                     rng_hgt=True, flags_t=t, texture=0.15, brightness=100.0, image_t=t)
        e, n = FRAME.to_course(tx, ty)
        if fault:
            fault(t, r, (e, n, z), m)
        b = f.update(t, r, m.state != 'WAIT')
        out['coasted'] += DT if b.coasting else 0.0
        cmd = m.step(t, b.pose, t if b.pose else None, offboard, landed, armed, b.yaw, (tx, ty, z))
        if armed and not offboard and cmd.kind == 'pose':
            offboard, out['start'] = True, t
        if offboard and cmd.kind == 'pose':
            d = [cmd.x - x, cmd.y - y, max(cmd.z, 0.3) - z]                # PX4 chases the setpoint
            k = min(1.0, 0.6 * DT / math.sqrt(sum(v * v for v in d))) if any(d) else 0.0
            x, y, z = x + d[0] * k, y + d[1] * k, z + d[2] * k
            if cmd.yaw is not None:
                err = (cmd.yaw - yaw + math.pi) % (2 * math.pi) - math.pi
                yaw += max(-math.radians(60) * DT, min(math.radians(60) * DT, err))
        elif offboard and cmd.kind == 'twist':
            x, y, z = x + cmd.x * DT, y + cmd.y * DT, max(0.0, z + cmd.z * DT)
        # a turn ends when the nose stops moving: register its push
        moving = abs(yaw - last_yaw) > math.radians(0.2)
        if moving and turning_from is None:
            turning_from = last_yaw
        elif not moving and turning_from is not None:
            dpsi = yaw - turning_from
            if abs(dpsi) > math.radians(5):
                size = PUSH_90_M * abs(math.sin(dpsi / 2)) / math.sin(math.radians(45))
                turns.append((t, size, (-math.sin(yaw), math.cos(yaw))))   # left of the new nose (odom)
            turning_from = None
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


def test_turning_at_every_corner_would_put_a_prop_on_a_wall(monkeypatch):
    """The old rule (face every leg) against the same measured drift: why the turn rule exists."""
    monkeypatch.setattr(route_steps, 'TURN_ROOM_M', 0.0)
    m, ref, out = fly()
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
