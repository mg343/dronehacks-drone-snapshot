"""steps.py must be exactly the flight-tested sequence.

``flight_tested_plan`` is square_circle/square_circle/mission.py's ``plan()`` (dronehacks-drone-snapshot
a3bd679, from DroneHacks core/square-test 1238fea), copied verbatim, with the values its launch
file flew: side_m 1.0, height_m 0.73, hold_s 2.0, ground_s 2.0, circle_d_m 1.0 (the Config default).
"""
from simplest_square_drone.steps import STEPS


class _C:
    side_m, height_m, hold_s, ground_s, circle_d_m = 1.0, 0.73, 2.0, 2.0, 1.0


def flight_tested_plan(c):
    s, h, r = c.side_m, c.height_m, c.circle_d_m / 2.0
    box = [(0, 0, h), (s, 0, h), (s, s, h), (0, s, h), (0, 0, h)]
    steps = []
    for k, p in enumerate(box):
        steps += [('move', p, 0, None), ('hold', p, c.hold_s, 90.0 * k)]
    steps += [('land', None, 0, None), ('ground', None, c.ground_s, None),
              ('move', (0, 0, h), 0, None), ('hold', (0, 0, h), c.hold_s, 360.0),
              ('move', (r, 0, h), 0, None), ('circle', (r, 0, h), 0, None),
              ('move', (0, 0, h), 0, None), ('hold', (0, 0, h), c.hold_s, 360.0),
              ('land', None, 0, None), ('disarm', None, 0, None)]
    return steps


def test_steps_are_the_flight_tested_sequence():
    """square_circle gave each hold the spot of the move before it, and its engine held at that
    move's end (it never read the hold's spot); steps.py's hover() just takes no spot."""
    tested = flight_tested_plan(_C)
    for i, (kind, spot, _, _) in enumerate(tested):
        if kind == 'hold':
            assert tested[i - 1][0] == 'move' and tested[i - 1][1] == spot
    tested = [(k, None if k == 'hold' else p, sec, f) for k, p, sec, f in tested]
    ours = [(s.kind, s.at, s.seconds, s.facing) for s in STEPS]
    assert ours == tested


def test_the_circle_is_the_tested_size():
    assert [s.diameter for s in STEPS if s.kind == 'circle'] == [_C.circle_d_m]


def test_twenty_steps_ending_in_a_disarm():
    assert len(STEPS) == 20 and STEPS[-1].kind == 'disarm'


def test_steps_pass_the_engines_checks():
    from simplest_square_drone.engine import check_steps
    check_steps(STEPS)
