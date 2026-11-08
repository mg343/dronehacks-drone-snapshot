"""The course as a plain step list, written by the planner instead of by hand. Pure Python, testable.

    course map --dronehacks_planner--> route (corners in course metres)
               --CourseFrame-->        the same corners in odom (forward, left)
               --this file-->          fly_to / hover / land lines, as in simplest_square_drone/steps.py

The rules for writing the lines, all of them about the drone, none about this course:

    1. rise straight up to flight height, then hover 2 s facing the launch heading
    2. at each corner: if the next leg turns more than TURN_DEG, hover first and turn to face it,
       so the camera looks down the next corridor, not at the wall ahead (field guide: facing a wall
       up close starves the tracker). Then fly_to the corner after it.
    3. legs longer than MAX_LEG_M are cut into pieces (each step must finish inside the engine's 30 s)
    4. at the end, hover 2 s, land, and keep pressing down until PX4 disarms

`until` flies only part of the course: stop after a goal (e.g. 'cp2_gate_a' or 'tunnel') and land
there. That is how the course gets flown for real one piece at a time.
"""
import math

from simplest_square_drone.steps import fly_to, hover, land, land_and_disarm

TURN_DEG = 15.0        # smaller turns are flown as they come; the nose keeps its heading
TURN_HOVER_S = 0.5     # the hover's seconds start after the turn is done (engine rule)
MAX_LEG_M = 3.0        # 3 m at 0.3 m/s is 10 s, well inside the engine's 30 s step timeout
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


def course_steps(route, frame, until=None):
    """A step list for simplest_square_drone's engine (odom metres, from the take-off spot)."""
    z = round(frame.flight_z, 3)
    steps = [fly_to(0.0, 0.0, z), hover(seconds=START_HOVER_S, facing=0)]   # 1. up, settle
    here, facing = (0.0, 0.0), 0.0
    for e, n in corners(route, until):
        x, y = frame.to_odom(e, n)
        leg = math.dist(here, (x, y))
        heading = _unwrap_near(math.degrees(math.atan2(y - here[1], x - here[0])), facing)
        if abs(heading - facing) > TURN_DEG:                                 # 2. face the next leg
            facing = round(heading, 1)
            steps.append(hover(seconds=TURN_HOVER_S, facing=facing))
        pieces = max(1, math.ceil(leg / MAX_LEG_M))                          # 3. short pieces
        for k in range(1, pieces + 1):
            px = here[0] + (x - here[0]) * k / pieces
            py = here[1] + (y - here[1]) * k / pieces
            steps.append(fly_to(round(px, 3), round(py, 3), z))
        here = (x, y)
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
