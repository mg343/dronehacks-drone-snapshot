"""route_steps.py: the planner's route as a step list."""
import math
import pathlib

import pytest

from dronehacks_planner.course_map import load_map
from dronehacks_planner.route import plan
from simplest_course_drone.route_steps import MAX_LEG_M, TURN_DEG, course_steps, describe
from simplest_map_drone.course import CourseFrame
from simplest_square_drone.engine import check_steps

MAPS = pathlib.Path(__file__).parents[2] / 'simplest_map_drone' / 'maps'
MAP = load_map(MAPS / 'course_v3_step.yaml')
FRAME = CourseFrame.from_map(MAP)
ROUTE = plan(MAP)


def test_the_whole_course_is_a_valid_step_list():
    steps = course_steps(ROUTE, FRAME)
    check_steps(steps)                                       # the engine would accept it
    assert [s.kind for s in steps[:2]] == ['move', 'hold'] and steps[0].at == (0.0, 0.0, 0.73)
    assert [s.kind for s in steps[-3:]] == ['hold', 'land', 'disarm']


def test_it_ends_over_the_pad():
    pad = next(g for g in MAP.goals if g.name == 'pad').centre
    last = [s for s in course_steps(ROUTE, FRAME) if s.kind == 'move'][-1].at
    assert math.dist(last[:2], FRAME.to_odom(*pad)) < 0.01


def test_every_leg_is_short_and_every_big_turn_hovers_first():
    steps = course_steps(ROUTE, FRAME)
    here, facing = (0.0, 0.0), 0.0
    for prev, s in zip(steps, steps[1:]):
        if s.kind == 'hold':
            facing = s.facing
        if s.kind == 'move' and prev.kind != 'land':
            assert math.dist(here, s.at[:2]) <= MAX_LEG_M + 1e-6
            if math.dist(here, s.at[:2]) > 0.01:
                leg = math.degrees(math.atan2(s.at[1] - here[1], s.at[0] - here[0]))
                off = (leg - facing + 180) % 360 - 180
                assert abs(off) <= TURN_DEG + 0.1, (s, facing)    # flown roughly nose-first
            here = s.at[:2]


def test_until_flies_only_part_of_the_course():
    whole = course_steps(ROUTE, FRAME)
    for goal in ['cp2_gate_a', 'tunnel', 'gate_b']:
        part = course_steps(ROUTE, FRAME, until=goal)
        check_steps(part)
        assert len(part) < len(whole)
    with pytest.raises(ValueError, match='no goal "window"'):
        course_steps(ROUTE, FRAME, until='window')


def test_describe_reads_like_steps_py():
    text = describe(course_steps(ROUTE, FRAME, until='cp2_gate_a'))
    assert text.splitlines()[0] == '  1. fly_to(+0.00, +0.00, 0.73)' and 'land_and_disarm()' in text


def test_the_taped_course_map_also_plans():
    """Generality: a different map file, no code change."""
    m = load_map(MAPS / 'course_arena_2026-11-08.yaml')
    steps = course_steps(plan(m), CourseFrame.from_map(m))
    check_steps(steps)
