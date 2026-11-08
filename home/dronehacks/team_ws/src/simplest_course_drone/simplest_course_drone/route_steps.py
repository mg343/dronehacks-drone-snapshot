"""The course as a plain step list, written by the planner instead of by hand. Pure Python, testable.

    course map --planner/-->           route (corners in course metres)
               --CourseFrame-->        the same corners in odom (forward, left)
               --this file-->          fly_to / hover / land lines, as in steps.py

The rules for writing the lines, all of them about the drone, none about this course:

    1. rise straight up to flight height, then hover 2 s facing the launch heading
    2. keep that heading the whole way (the default, turns=False): every leg is flown forwards,
       sideways or backwards, which PX4 does as easily, and the nose never yaws.
       Why: skybeta-11 carries a steady push fixed to its airframe, and PX4's integrator holds the
       correction in world axes, so every yaw leaves it pointing the wrong way until relearned
       (docs/flight-findings.md). The slide after a 90 deg yaw was 10-18 cm in flight
       20261108-013018 (push 0.10 m/s^2) but the push was 0.25 m/s^2 in 003042, which predicts
       25-35 cm, against a few cm of room past the props at the course's narrow spots. A hold without
       a yaw stayed within 7 cm. No yaw, no slide, whatever the push is that day.
       turns=True instead turns the nose to face the next leg where the turn is over TURN_DEG and the
       corner has TURN_ROOM_M of room, then hovers TURN_SETTLE_S: a better camera view down the
       corridors, sized for the 18 cm slide only.
    3. legs longer than MAX_LEG_M are cut into pieces (each step must finish inside the engine's 30 s)
    4. at the end, hover 2 s, land, and keep pressing down until PX4 disarms

`until` flies only part of the course: stop after a goal (e.g. 'cp2_gate_a' or 'tunnel') and land
there. That is how the course gets flown for real one piece at a time.
"""
import math

from simplest_course_drone.course import DRONE_EXTENT, nearest_wall
from simplest_course_drone.steps import fly_to, hover, land, land_and_disarm

TURN_DEG = 15.0          # smaller turns are flown as they come; the nose keeps its heading
POST_YAW_DRIFT_M = 0.18  # worst drift after a 90 deg yaw on skybeta-11 (flight 20261108-013018)
TURN_ROOM_M = DRONE_EXTENT + POST_YAW_DRIFT_M + 0.04   # 0.50 m: props stay 4 cm off at the worst drift
TURN_SETTLE_S = 3.0      # covers the drift's peak, 1-3 s after the turn (the hover's seconds start
                         # once the drone is within 20 cm and 10 deg: engine rule)
MAX_LEG_M = 3.0          # 3 m at 0.3 m/s is 10 s, well inside the engine's 30 s step timeout
START_HOVER_S = 2.0
END_HOVER_S = 2.0


def _unwrap_near(angle, ref):
    """angle (degrees) moved by whole turns to be within 180 of ref."""
    return angle + 360.0 * round((ref - angle) / 360.0)


def corners(route, until=None):
    """The route's points after the start, as course (east, north), cut short after `until`."""
    out = []
    for w in route.waypoints[1:]:
        p = tuple(w.position)
        if not out or math.dist(p, out[-1]) > 0.01:
            out.append(p)
        if until is not None and w.goal == until and w.kind in ('exit', 'goal'):
            return out
    if until is not None and until not in route.goals:
        raise ValueError('no goal "{}" on the route (goals: {})'.format(until, ', '.join(route.goals)))
    return out


def room(cmap, east, north):
    """Metres from the drone's centre to the nearest wall, net or obstacle."""
    return nearest_wall(cmap, east, north)[0]


def course_steps(route, frame, cmap, until=None, turns=False):
    """A step list for engine.py (odom metres, from the take-off spot). turns: rule 2."""
    z = round(frame.flight_z, 3)
    steps = [fly_to(0.0, 0.0, z), hover(seconds=START_HOVER_S, facing=0)]   # 1. up, settle
    here, facing = frame.launch, 0.0                                         # course point, odom deg
    for e, n in corners(route, until):
        x0, y0 = frame.to_odom(*here)
        x, y = frame.to_odom(e, n)
        heading = _unwrap_near(math.degrees(math.atan2(y - y0, x - x0)), facing)
        if turns and abs(heading - facing) > TURN_DEG and room(cmap, *here) >= TURN_ROOM_M:   # 2.
            facing = round(heading, 1)
            steps.append(hover(seconds=TURN_SETTLE_S, facing=facing))
        pieces = max(1, math.ceil(math.dist((x0, y0), (x, y)) / MAX_LEG_M))   # 3. short pieces
        for k in range(1, pieces + 1):
            steps.append(fly_to(round(x0 + (x - x0) * k / pieces, 3), round(y0 + (y - y0) * k / pieces, 3), z))
        here = (e, n)
    steps += [hover(seconds=END_HOVER_S, facing=facing), land(), land_and_disarm()]   # 4. down
    return steps


def describe(steps):
    """The step list as readable lines, like steps.py."""
    out = []
    for i, s in enumerate(steps, 1):
        if s.kind == 'move':
            what = 'fly_to({:+.2f}, {:+.2f}, {:.2f})'.format(*s.at)
        elif s.kind == 'hold':
            what = 'hover(seconds={:g}, facing={:g})'.format(s.seconds, s.facing)
        else:
            what = {'land': 'land()', 'disarm': 'land_and_disarm()'}.get(s.kind, s.kind)
        out.append('{:>3}. {}'.format(i, what))
    return '\n'.join(out)
