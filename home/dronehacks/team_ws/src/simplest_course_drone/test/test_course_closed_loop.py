"""The whole course, closed loop: planner steps -> fusion -> the real engine -> a crude drone.

The drone model is the one simplest_square_drone and simplest_fusion_drone test with (pose tracking
at 0.6 m/s, the kit's 0.3 m floor, yaw at 60 deg/s, twists followed, auto-disarm 5 s after landing,
Offboard only once our node streams). Its sensors read the truth unless a test darkens them. The
referee is simplest_map_drone's Progress, fed the true position, plus the nearest wall along the way.

What this can show: the steps, the trust gate and the engine work together on this map's geometry.
What it cannot: the real drone's dynamics, latency, the real tunnel, the real course's placement.
"""
import math
import pathlib

from dronehacks_planner.course_map import load_map
from dronehacks_planner.route import plan
from simplest_course_drone.route_steps import course_steps
from simplest_fusion_drone.fusion import Config, Fusion, Readings
from simplest_map_drone.course import DRONE_EXTENT, CourseFrame, Progress, _along_across, nearest_wall
from simplest_square_drone.engine import DONE, Mission

MAP = load_map(pathlib.Path(__file__).parents[2] / 'simplest_map_drone' / 'maps' / 'course_v3_step.yaml')
FRAME = CourseFrame.from_map(MAP)
ROUTE = plan(MAP)
TUNNEL = next(g for g in MAP.goals if g.name == 'tunnel')
DT = 0.05


def in_tunnel(e, n):
    along, across = _along_across(TUNNEL, e, n)
    return abs(along) <= TUNNEL.depth / 2 and abs(across) <= float(TUNNEL.params['width']) / 2


def fly(until=None, dark_tunnel=False, max_t=400.0):
    steps = course_steps(ROUTE, FRAME, until)
    m, f, ref = Mission(steps), Fusion(Config(settle_s=5.0)), Progress(MAP)
    x = y = z = yaw = 0.0
    armed, offboard, landed_since = True, False, None
    t, out = 0.0, {'clearance': math.inf, 'coasted': 0.0, 'start': None, 'path': []}
    while t < max_t and m.state != DONE:
        t += DT
        landed = z <= 1e-6
        if landed and offboard:
            landed_since = t if landed_since is None else landed_since
            if t - landed_since >= 5.0:
                armed = offboard = False
        else:
            landed_since = None
        e, n = FRAME.to_course(x, y)
        r = Readings(camera=(x, y, z, yaw), camera_t=t, vo_state=1, vo_t=t, ekf=(x, y, z), ekf_xy_valid=True,
                     dist_bottom=z + 0.07, ekf_t=t, px4_yaw_enu=yaw, att_t=t, ev_pos=True, ev_yaw=True,
                     rng_hgt=True, flags_t=t, texture=0.15, brightness=100.0, image_t=t)
        if dark_tunnel and in_tunnel(e, n):
            r.brightness, r.vo_state, r.ev_pos = 2.0, 2, False      # blind, lost, EKF2 not fusing vision
        b = f.update(t, r, m.state != 'WAIT')
        out['coasted'] += DT if b.coasting else 0.0
        cmd = m.step(t, b.pose, t if b.pose else None, offboard, landed, armed, b.yaw, (x, y, z))
        if armed and not offboard and cmd.kind == 'pose':
            offboard, out['start'] = True, t
        if offboard and cmd.kind == 'pose':
            d = [cmd.x - x, cmd.y - y, max(cmd.z, 0.3) - z]
            k = min(1.0, 0.6 * DT / math.sqrt(sum(v * v for v in d))) if any(d) else 0.0
            x, y, z = x + d[0] * k, y + d[1] * k, z + d[2] * k
            if cmd.yaw is not None:
                err = (cmd.yaw - yaw + math.pi) % (2 * math.pi) - math.pi
                yaw += max(-math.radians(60) * DT, min(math.radians(60) * DT, err))
        elif offboard and cmd.kind == 'twist':
            x, y, z = x + cmd.x * DT, y + cmd.y * DT, max(0.0, z + cmd.z * DT)
        e, n = FRAME.to_course(x, y)
        ref.update(e, n, landed=z <= 1e-6, armed=armed)
        if z > 0.05:
            out['clearance'] = min(out['clearance'], nearest_wall(MAP, e, n)[0])
        out['path'].append((t, e, n, z, m.reason))
    out['time'] = t - (out['start'] or 0.0)
    return m, ref, out


def test_the_whole_course_every_checkpoint_in_order_no_wall_touched():
    m, ref, out = fly()
    assert m.state == DONE and m.reason == 'disarmed on the ground', m.reason
    assert ref.passed == ['cp2_gate_a', 'tunnel', 'gate_b', 'pad'], ref.passed
    assert out['clearance'] > DRONE_EXTENT, out['clearance']        # props never reach a wall
    assert out['time'] < 180.0, out['time']                          # inside the 3-minute limit
    assert not any('took over' in p[4] or 'pose' in p[4] for p in out['path'])


def test_the_course_with_a_blind_tunnel():
    """Camera blind, tracking lost and EKF2 not fusing vision for the whole tube: coast through."""
    m, ref, out = fly(dark_tunnel=True)
    assert out['coasted'] > 3.0, out['coasted']                      # it really flew blind
    assert ref.passed == ['cp2_gate_a', 'tunnel', 'gate_b', 'pad'], ref.passed
    assert out['clearance'] > DRONE_EXTENT


def test_flown_one_piece_at_a_time():
    """until: each piece passes its own checkpoints, then lands clear of every wall."""
    expect = {'cp2_gate_a': ['cp2_gate_a'], 'tunnel': ['cp2_gate_a', 'tunnel'],
              'gate_b': ['cp2_gate_a', 'tunnel', 'gate_b']}
    for goal, passed in expect.items():
        m, ref, out = fly(until=goal)
        assert m.state == DONE and m.reason == 'disarmed on the ground', (goal, m.reason)
        assert ref.passed == passed, (goal, ref.passed)
        _, e, n, z, _ = out['path'][-1]
        assert z == 0.0 and nearest_wall(MAP, e, n)[0] > DRONE_EXTENT, (goal, e, n)
