"""course.py against the real map and the planner's real route."""
import dataclasses
import math
import pathlib

import pytest

from dronehacks_planner.course_map import load_map
from dronehacks_planner.route import plan
from simplest_map_drone.course import CourseFrame, Progress, nearest_wall

MAP = load_map(pathlib.Path(__file__).parents[1] / 'maps' / 'course_v3_step.yaml')
FRAME = CourseFrame.from_map(MAP)


def walk(points, step=0.02):
    """The route's corners joined by straight lines, sampled every `step` metres."""
    out = [points[0]]
    for a, b in zip(points, points[1:]):
        n = max(1, int(math.dist(a, b) / step))
        out += [(a[0] + (b[0] - a[0]) * k / n, a[1] + (b[1] - a[1]) * k / n) for k in range(1, n + 1)]
    return out


def test_launch_is_the_odom_origin_facing_forward():
    assert FRAME.to_odom(*MAP.launch.position) == (0.0, 0.0)
    assert abs(FRAME.flight_z - 0.73) < 1e-9                       # 0.8 m above the floor, 0.07 m rest
    assert FRAME.direction_to_odom_deg(0, 1) == 0.0                 # north = forward (launch faces north)
    assert round(FRAME.direction_to_odom_deg(-1, 0)) == 90          # west = left
    assert round(FRAME.direction_to_odom_deg(1, 0)) == -90          # east = right


def test_pad_in_odom_and_back():
    pad = next(g for g in MAP.goals if g.name == 'pad').centre
    x, y = FRAME.to_odom(*pad)
    assert (round(x, 3), round(y, 3)) == (-1.706, -0.472)           # 1.706 m behind, 0.472 m right
    e, n = FRAME.to_course(x, y)
    assert math.dist((e, n), pad) < 1e-9


def test_a_map_without_a_launch_is_refused():
    with pytest.raises(ValueError, match='no launch'):
        CourseFrame.from_map(dataclasses.replace(MAP, launch=None))


def test_nearest_wall():
    d, what = nearest_wall(MAP, *MAP.launch.position)
    assert 0.3 < d < 1.0
    seg = MAP.segments[0]
    mid = ((seg.start[0] + seg.end[0]) / 2, (seg.start[1] + seg.end[1]) / 2)
    assert abs(nearest_wall(MAP, *mid)[0] - (-seg.thickness / 2)) < 1e-9      # on the wall's line
    off = (mid[0], mid[1] - 0.10 - seg.thickness / 2)                # 10 cm off its face (it runs east-west)
    assert abs(nearest_wall(MAP, *off)[0] - 0.10) < 1e-6


def test_the_planned_route_passes_every_checkpoint_in_order():
    route = plan(MAP)
    p = Progress(MAP)
    for e, n in walk([tuple(w.position) for w in route.waypoints]):
        p.update(e, n)
    assert p.passed == ['cp2_gate_a', 'tunnel', 'gate_b']           # the pad needs a landing
    pad = next(g for g in MAP.goals if g.name == 'pad').centre
    assert p.update(*pad, landed=True, armed=True) is None           # still armed: not yet
    assert p.update(*pad, landed=True, armed=False) == 'pad' and p.done


def test_going_round_an_opening_does_not_count():
    """Past the tunnel's far face but outside its width: not through it."""
    p = Progress(MAP)
    tunnel = next(g for g in MAP.goals if g.name == 'tunnel')
    p.passed = ['cp2_gate_a']
    beside = (tunnel.centre[0] - 2.0, tunnel.centre[1])
    for k in range(60):                                              # move "south" beside the tube
        p.update(beside[0], beside[1] + 1.0 - k * 0.05)
    assert p.passed == ['cp2_gate_a']


def test_words():
    p = Progress(MAP)
    assert p.words(*MAP.launch.position).startswith('next: cp2_gate_a ')
