"""route_steps.py: the planner's route as a step list, on the default (flown) map."""
import math
import pathlib

import pytest

from dronehacks_planner.course_map import load_map
from dronehacks_planner.route import plan
from simplest_course_drone.route_steps import (MAX_LEG_M, TURN_ROOM_M, TURN_SETTLE_S, course_steps, describe,
                                               room)
from simplest_map_drone.course import CourseFrame
from simplest_square_drone.engine import check_steps

MAPS = pathlib.Path(__file__).parents[2] / 'simplest_map_drone' / 'maps'
MAP = load_map(MAPS / 'course_arena_2026-11-08_flown.yaml')
FRAME = CourseFrame.from_map(MAP)
ROUTE = plan(MAP)
STEPS = course_steps(ROUTE, FRAME, MAP)


def walk(steps):
    """[(step, odom spot before it)]"""
    here, out = (0.0, 0.0), []
    for s in steps:
        out.append((s, here))
        if s.kind == 'move':
            here = s.at[:2]
    return out


def test_the_whole_course_is_a_valid_step_list_at_the_window_height():
    check_steps(STEPS)                                       # the engine would accept it
    assert [s.kind for s in STEPS[:2]] == ['move', 'hold'] and STEPS[0].at == (0.0, 0.0, 1.13)
    assert [s.kind for s in STEPS[-3:]] == ['hold', 'land', 'disarm']


def test_it_ends_over_the_pad():
    pad = next(g for g in MAP.goals if g.name == 'pad').centre
    last = [s for s in STEPS if s.kind == 'move'][-1].at
    assert math.dist(last[:2], FRAME.to_odom(*pad)) < 0.01


def test_every_leg_is_short():
    for s, here in walk(STEPS)[1:]:
        if s.kind == 'move':
            assert math.dist(here, s.at[:2]) <= MAX_LEG_M + 1e-6


def test_the_nose_turns_only_where_there_is_room_and_then_settles():
    turns = [(s, here) for s, here in walk(STEPS)[2:-3] if s.kind == 'hold']
    assert turns                                             # it does turn where it can (camera view)
    for s, here in turns:
        assert s.seconds == TURN_SETTLE_S
        assert room(MAP, *FRAME.to_course(*here)) >= TURN_ROOM_M, (s, here)


def test_no_turn_at_a_narrow_corner():
    """Gate B's corner wants a 134 deg turn with 0.35 m of room: it is flown without turning."""
    narrow = {tuple(round(v, 3) for v in FRAME.to_odom(*w.position))
              for w in ROUTE.waypoints[1:-1] if room(MAP, *w.position) < TURN_ROOM_M}
    assert narrow
    for s, here in walk(STEPS)[2:-3]:
        assert not (s.kind == 'hold' and tuple(round(v, 3) for v in here) in narrow), here


def test_until_flies_only_part_of_the_course():
    for goal in ['tunnel', 'gate_b']:
        part = course_steps(ROUTE, FRAME, MAP, until=goal)
        check_steps(part)
        assert len(part) < len(STEPS)
    with pytest.raises(ValueError, match='no goal "window"'):
        course_steps(ROUTE, FRAME, MAP, until='window')


def test_describe_reads_like_steps_py():
    text = describe(course_steps(ROUTE, FRAME, MAP, until='tunnel'))
    assert text.splitlines()[0] == '  1. fly_to(+0.00, +0.00, 1.13)' and 'land_and_disarm()' in text


def test_other_maps_plan_too():
    """Generality: different map files, no code change."""
    for name in ['course_v3_step.yaml', 'course_arena_2026-11-08.yaml']:
        m = load_map(MAPS / name)
        check_steps(course_steps(plan(m), CourseFrame.from_map(m), m))


def test_the_route_keeps_off_where_two_real_flights_hit():
    """082135 and 113021 struck at about (2.14, -0.87) and (2.07, -0.77); the taped map's route
    passed within a few cm. The flown map's route keeps its centre away."""
    def closest(route):
        pts = [tuple(w.position) for w in route.waypoints]
        return min(math.dist((a[0] + (b[0] - a[0]) * k / 50, a[1] + (b[1] - a[1]) * k / 50), hit)
                   for a, b in zip(pts, pts[1:]) for k in range(51) for hit in [(2.14, -0.87), (2.07, -0.77)])
    assert closest(plan(load_map(MAPS / 'course_arena_2026-11-08.yaml'))) < 0.1
    assert closest(ROUTE) > 0.2
