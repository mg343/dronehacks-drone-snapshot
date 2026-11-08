"""engine.check_steps refuses step lists the engine cannot fly as written."""
import math

import pytest

from simplest_course_drone.engine import Mission, check_steps
from simplest_course_drone.steps import (STEPS, circle, fly_to, hover, land, land_and_disarm, rest)

UP = 0.73


def refused(steps, words):
    with pytest.raises(ValueError, match=words):
        check_steps(steps)


def test_the_shipped_steps_pass():
    check_steps(STEPS)


def test_must_start_with_fly_to():
    refused([hover(seconds=2, facing=0), land(), land_and_disarm()], 'first step')


def test_must_end_with_land_and_disarm():
    refused([fly_to(0, 0, UP), land()], 'last step')
    refused([fly_to(0, 0, UP), land_and_disarm(), fly_to(0, 0, UP), land(), land_and_disarm()],
            'must be the last')


def test_hover_must_be_in_the_air():
    refused([fly_to(0, 0, UP), land(), hover(seconds=2, facing=0), land_and_disarm()], 'in the air')


def test_rest_must_follow_land():
    refused([fly_to(0, 0, UP), rest(seconds=2), land(), land_and_disarm()], 'right after land')


def test_circle_must_follow_a_fly_to_its_spot():
    refused([fly_to(0, 0, UP), circle(0.5, 0, UP, diameter=1.0), land(), land_and_disarm()], 'same spot')
    refused([fly_to(0, 0, UP), hover(seconds=2, facing=0), circle(0, 0, UP, diameter=1.0), land(),
             land_and_disarm()], 'same spot')


def test_spots_must_be_in_the_air():
    refused([fly_to(1, 0, 0.0), land(), land_and_disarm()], 'above the ground')


def test_a_circle_away_from_the_start_begins_on_its_own_spot():
    """No jump at the start of the circle: its first target is the circle's own spot."""
    spot = (0.2, -0.3, UP)
    m = Mission([fly_to(*spot), circle(*spot, diameter=0.6), land(), land_and_disarm()])
    start = (2.0, -1.0, 0.0)
    m.step(0.0, start, 0.0, True, True, True)                 # enters FLY at the start
    m.i = 1                                                  # jump to the circle step, drone on its spot
    here = tuple(s + o for s, o in zip(start, spot))
    m.target = here
    cmd = m.step(0.05, here, 0.05, True, False, True)
    assert math.dist((cmd.x, cmd.y, cmd.z), here) < 0.02
    # half way round it is a diameter to the left of the spot
    m.arc = math.pi - 1e-9
    m.target = (here[0], here[1] + 0.6, here[2])
    cmd = m.step(0.10, m.target, 0.10, True, False, True)
    assert math.dist((cmd.x, cmd.y, cmd.z), (here[0], here[1] + 0.6, here[2])) < 0.02
