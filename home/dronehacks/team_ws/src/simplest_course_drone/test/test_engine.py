"""Fly steps.STEPS against a crude drone model, including PX4's auto-disarm after landing.

square_circle's test_mission.py, run on the step list instead of plan(): 1 m sides, 2 s hovers."""
import math

import dataclasses

from simplest_course_drone.engine import DONE, STOPPED, Config, Mission, _wrap
from simplest_course_drone.steps import STEPS

S, H, HOLD = 1.0, 0.73, 2.0     # what steps.py flies

DT = 0.05
START = (2.0, -1.0, 0.0)
COM_DISARM_LAND = 5.0            # skybeta-11, raised from 2 s with the organisers' permission
START_YAW = 0.4                  # rad: not zero, so the test catches a missing offset


def fly(mission, drop_pose=None, takeover_at=None, max_t=250.0, heading=True, off=None):
    """Moves toward a pose setpoint at <= 0.6 m/s (z floored 0.3 m, as the kit does), follows
    twists, sits on the ground at z 0. Disarms after COM_DISARM_LAND landed, which ends Offboard."""
    x, y, z = START
    yaw = START_YAW
    armed = True
    landed_since = None
    track, cmds = [], []
    t = 0.0
    while t < max_t:
        t += DT
        offboard = armed and t > 1.0 and (takeover_at is None or t < takeover_at)
        landed = z <= 1e-6
        if landed and t > 1.0:
            landed_since = t if landed_since is None else landed_since
            if t - landed_since >= COM_DISARM_LAND:
                armed = offboard = False
        else:
            landed_since = None
        stale = drop_pose and drop_pose[0] <= t < drop_pose[1]
        px4 = None if off is None else (x + off[0], y + off[1], z + off[2])
        cmd = mission.step(t, None if stale else (x, y, z), None if stale else t, offboard, landed, armed,
                           yaw if heading and not stale else None, px4)
        cmds.append((t, cmd))
        if not offboard:
            pass                              # PX4 ignores setpoints outside Offboard
        elif cmd.kind == 'pose':
            o = off or (0.0, 0.0, 0.0)          # PX4 flies the setpoint in its own frame
            d = [cmd.x - o[0] - x, cmd.y - o[1] - y, max(cmd.z, 0.3) - o[2] - z]
            n = math.sqrt(sum(v * v for v in d))
            f = min(1.0, 0.6 * DT / n) if n else 0.0
            x, y, z = x + d[0] * f, y + d[1] * f, z + d[2] * f
            if cmd.yaw is not None:           # PX4 turns toward it at up to 60 deg/s
                e = _wrap(cmd.yaw - yaw)
                yaw = _wrap(yaw + max(-math.radians(60) * DT, min(math.radians(60) * DT, e)))
        elif cmd.kind == 'twist':
            x, y, z = x + cmd.x * DT, y + cmd.y * DT, max(0.0, z + cmd.z * DT)
        track.append((t, x, y, z, armed, yaw))
        if mission.state in (DONE, STOPPED):
            break
    return track, cmds


def at(dx, dy, h=0.0):
    """A step's spot, (forward, left) from the take-off spot along the take-off heading, in the
    camera's frame: how engine.py places every step."""
    c, s = math.cos(START_YAW), math.sin(START_YAW)
    return (START[0] + c * dx - s * dy, START[1] + s * dx + c * dy, h)


def in_takeoff_frame(x, y):
    """The inverse: a camera-frame point as (forward, left) from the take-off spot."""
    c, s = math.cos(START_YAW), math.sin(START_YAW)
    dx, dy = x - START[0], y - START[1]
    return c * dx + s * dy, -s * dx + c * dy


def held(track, p, tol=0.12):
    """Longest stretch spent within tol of p, in seconds."""
    best = run = 0.0
    for _, x, y, z, _, _ in track:
        run = run + DT if math.dist((x, y, z), p) <= tol else 0.0
        best = max(best, run)
    return best


def test_full_sequence():
    m = Mission(STEPS)
    track, _ = fly(m)
    assert m.state == DONE and m.reason == 'disarmed on the ground', m.reason
    s, h = S, H
    x0, y0, _ = START
    for dx, dy in [(0, 0), (s, 0), (s, s), (0, s)]:          # forward, left of the take-off heading
        assert held(track, at(dx, dy, h)) >= HOLD - 0.1, (dx, dy)
    touchdowns = sum(1 for a, b in zip(track, track[1:]) if a[3] > 1e-6 and b[3] <= 1e-6)
    assert touchdowns == 2
    first_down = next(i for i, p in enumerate(track) if i and p[3] <= 1e-6 and track[i - 1][3] > 1e-6)
    assert all(p[4] for p in track[:first_down + 40])        # still armed through the 2 s ground rest
    assert max(p[3] for p in track[first_down:]) > 0.7       # climbed back to height
    _, x, y, z, armed, yaw = track[-1]
    assert not armed and z == 0.0 and math.hypot(x - x0, y - y0) < 0.05
    assert abs(_wrap(yaw - START_YAW)) < math.radians(5)         # faces its take-off heading again
    r = next(st.diameter for st in STEPS if st.kind == 'circle') / 2
    box = [in_takeoff_frame(p[1], p[2]) for p in track]
    assert all(-0.1 <= f <= max(s, 2 * r) + 0.1 and -0.1 <= l <= max(s, 2 * r) + 0.1 for f, l in box)
    second = track[first_down:]                              # the circle inside the box: its four sides
    for dx, dy in [(r, 0), (2 * r, r), (r, 2 * r), (0, r)]:
        assert min(math.dist(p[1:4], at(dx, dy, h)) for p in second) < 0.05, (dx, dy)


def test_ground_rest_longer_than_auto_disarm_ends_after_first_landing():
    m = Mission([dataclasses.replace(s, seconds=COM_DISARM_LAND + 0.5) if s.kind == 'ground' else s for s in STEPS])
    track, _ = fly(m)
    assert m.state == DONE and 'COM_DISARM_LAND' in m.reason
    assert max(p[3] for p in track[-30:]) < 0.05


def test_pilot_takeover_stops_for_good():
    m = Mission(STEPS)
    _, cmds = fly(m, takeover_at=8.0, max_t=12.0)
    assert m.state == STOPPED
    assert all(c.kind == 'none' for t, c in cmds if t >= 8.0)


def test_stale_pose_holds_then_descends():
    m = Mission(STEPS)
    _, cmds = fly(m, drop_pose=(10.0, 30.0), max_t=14.5)
    during = [c for t, c in cmds if 10.0 <= t < 12.9]
    assert during and all(c.kind == 'twist' and (c.x, c.y, c.z) == (0.0, 0.0, 0.0) for c in during)
    late = [c for t, c in cmds if 13.1 <= t < 14.5]
    assert late and all(c.kind == 'twist' and c.z < 0 and c.x == c.y == 0.0 for c in late)


def test_no_pose_no_stream():
    m = Mission(STEPS)
    assert m.step(0.0, None, None, False, True, False).kind == 'none'
    assert m.step(1.0, (0, 0, 0), 1.0, False, True, False).kind == 'pose'


def test_turns_slowly_to_face_each_leg():
    m = Mission(STEPS)
    track, cmds = fly(m)
    s, h = S, H
    # at each corner, the heading it settles on before moving on: left turns of 90 deg
    for (dx, dy), turn in [((s, 0), 90), ((s, s), 180), ((0, s), 270)]:
        box = track[:next(i for i, p in enumerate(track) if i > _up(track) and p[3] < 0.12)]    # before the circle
        here = [p for p in box if math.dist(p[1:4], at(dx, dy, h)) < 0.03]
        assert abs(_wrap(here[-1][5] - START_YAW - math.radians(turn))) < math.radians(5), (dx, dy)
    yaws = [(t, c.yaw) for t, c in cmds if c.kind == 'pose' and c.yaw is not None and t > 1.1]
    for (t0, a), (t1, b) in zip(yaws, yaws[1:]):
        assert abs(_wrap(b - a)) <= math.radians(20) * (t1 - t0) + 1e-9   # never faster than 20 deg/s
    assert max(abs(_wrap(b - a)) for (_, a), (_, b) in zip(yaws, yaws[1:])) > 0   # it did turn


def test_without_heading_it_flies_without_commanding_yaw():
    m = Mission(STEPS)
    _, cmds = fly(m, heading=False)
    assert m.state == DONE
    assert all(c.yaw is None for _, c in cmds)


def test_px4_frame_offset_is_corrected():
    for off in [(0.02, -0.03, 0.15), (0.0, 0.0, -0.25)]:
        m = Mission(STEPS)
        track, _ = fly(m, off=off)
        assert m.state == DONE and m.reason == 'disarmed on the ground', (off, m.reason)
        for dx, dy in [(0, 0), (S, 0), (S, S), (0, S)]:
            assert held(track, at(dx, dy, H)) >= HOLD - 0.1, (off, dx, dy)


def test_large_px4_offset_refuses_to_fly():
    m = Mission(STEPS)
    _, cmds = fly(m, off=(0.0, 0.0, 0.73), max_t=5.0)
    assert m.state == 'WAIT' and 'disagree' in m.reason
    assert all(c.kind == 'none' for _, c in cmds)


def test_offset_that_changes_after_takeoff_is_followed():
    """PX4's rangefinder height reads 0.14 m on the ground and right in the air."""
    m = Mission(STEPS)
    x, y, z = START
    t, track = 0.0, []
    while t < 250.0 and m.state not in (DONE, STOPPED):
        t += DT
        off = (0.0, 0.0, 0.14 if z < 0.1 else 0.0)
        offboard = t > 1.0
        cmd = m.step(t, (x, y, z), t, offboard, z <= 1e-6, True, None, (x, y, z + off[2]))
        if offboard and cmd.kind == 'pose':
            d = [cmd.x - x, cmd.y - y, max(cmd.z, 0.3) - off[2] - z]
            n = math.sqrt(sum(v * v for v in d))
            f = min(1.0, 0.6 * DT / n) if n else 0.0
            x, y, z = x + d[0] * f, y + d[1] * f, z + d[2] * f
        elif offboard and cmd.kind == 'twist':
            x, y, z = x + cmd.x * DT, y + cmd.y * DT, max(0.0, z + cmd.z * DT)
        track.append((t, x, y, z, True, START_YAW))
        if m.step_name == 'land':
            break
    x0, y0, _ = START
    for dx, dy in [(0, 0), (S, 0), (S, S), (0, S)]:
        assert held(track, (x0 + dx, y0 + dy, H), tol=0.05) >= HOLD - 0.1, (dx, dy)


def test_a_stale_offboard_flag_while_disarmed_does_not_start_a_flight():
    """Just after a disarm the kit can still report Offboard active; the next flight's fresh Mission
    keeps waiting (and streaming) until PX4 is armed again."""
    m = Mission(STEPS)
    assert m.step(0.0, (0, 0, 0), 0.0, True, True, False).kind == 'pose'
    assert m.state == 'WAIT' and m.start is None
    m.step(0.05, (0, 0, 0), 0.05, True, True, True)
    assert m.state == 'FLY'


def test_disarm_mid_flight_ends_the_run_at_once():
    m = Mission(STEPS)
    fly(m, max_t=10.0)
    assert m.step(10.05, (0, 0, 0.7), 10.05, True, False, False).kind == 'none'
    assert m.state == DONE and 'disarmed during step' in m.reason


def _up(track):
    """Index where the drone first reached the box height."""
    return next(i for i, p in enumerate(track) if p[3] > 0.7)


def test_landing_pulls_back_over_the_last_fly_to_not_the_take_off_spot():
    """A route that ends 1 m forward must land there (on the course: the pad, not the launch mark)."""
    from simplest_course_drone.steps import fly_to, hover, land, land_and_disarm
    m = Mission([fly_to(0, 0, H), fly_to(1.0, 0, H), hover(seconds=1, facing=0), land(), land_and_disarm()])
    track, _ = fly(m)
    assert m.state == DONE and m.reason == 'disarmed on the ground', m.reason
    _, x, y, z, armed, _ = track[-1]
    assert math.dist((x, y), at(1.0, 0)[:2]) < 0.05 and z == 0.0      # 1 m along the take-off heading


def test_the_followed_offset_never_moves_more_than_0_3_m():
    """Camera frozen at the take-off spot while PX4's own position keeps climbing: the offset the
    engine adds to its setpoints is held at 0.3 m from its value at Offboard."""
    m = Mission(STEPS)
    pose, px4 = (0.0, 0.0, 0.0), (0.0, 0.0, 0.1)
    m.step(0.0, pose, 0.0, False, True, True, 0.0, px4)                  # streaming, waiting
    cmd = m.step(0.05, pose, 0.05, True, True, True, 0.0, px4)           # Offboard: offset (0, 0, 0.1)
    for k in range(2, 400):
        t = k * 0.05
        px4 = (0.0, 0.0, 0.1 + 0.02 * k)                                 # PX4 climbs, camera does not
        cmd = m.step(t, pose, t, True, False, True, 0.0, px4)
    assert abs(m.offset[2] - 0.1) <= 0.3 + 1e-9, m.offset
    assert cmd.z <= 0.3 + 0.1 + 0.3 + 1e-6, cmd                          # target (<= lead) + offset0 + limit


def test_steps_are_measured_from_the_take_off_pose_and_heading():
    """The camera's origin and axes do not matter: facing 90 deg left at (5, 5), 'forward 1 m' is +y."""
    m = Mission(STEPS)
    m.step(0.0, (5.0, 5.0, 0.0), 0.0, False, True, True, math.pi / 2, (5.0, 5.0, 0.1))
    m.step(0.05, (5.0, 5.0, 0.0), 0.05, True, True, True, math.pi / 2, (5.0, 5.0, 0.1))
    x, y, z = m._at((1.0, 0.0, 0.5))
    assert abs(x - 5.0) < 1e-9 and abs(y - 6.0) < 1e-9 and abs(z - 0.5) < 1e-9


def test_a_sideways_offset_is_carried_but_a_height_offset_is_refused():
    m = Mission(STEPS)                                       # PX4's frame 80 m away sideways: fine
    cmd = m.step(0.0, (0.0, 0.0, 0.0), 0.0, False, True, True, 0.0, (-79.7, 1.9, 0.1))
    assert cmd.kind == 'pose' and abs(cmd.x + 79.7) < 1e-9   # streamed, in PX4's frame
    m = Mission(STEPS)                                       # the camera 9 m up on the floor: refused
    cmd = m.step(0.0, (0.0, 0.0, 9.02), 0.0, False, True, True, 0.0, (0.0, 0.0, 0.06))
    assert cmd.kind == 'none' and 'heights disagree by 8.96 m' in m.reason
