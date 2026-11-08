"""The whole flight, one instruction per line. Read it top to bottom.

Where:   metres from the take-off spot.
         forward = the way the nose pointed when the drone powered on
         left    = 90 degrees to the left of that
         up      = height above the take-off spot
Facing:  degrees turned LEFT from the take-off heading (90 = a quarter turn left).

What each instruction means (engine.py carries them out):
    fly_to       slide to the spot at 0.3 m/s without turning
    hover        stay where the last fly_to ended: first turn (slowly, 20 deg/s) to face the given
                 way, then wait until the drone is within 20 cm of that spot and 10 deg of the
                 heading, then stay there this many seconds
    land         go down at 0.3 m/s, drifting back over the take-off spot, until PX4 says landed
    rest         sit on the ground with motors idling and pressing down gently (NOT a disarm:
                 only the pilot can re-arm, and PX4 disarms by itself 5 s after landing)
    circle       starting from this spot (fly_to it first), go once round a circle of this diameter
                 whose centre is half a diameter to the LEFT: counter-clockwise seen from above,
                 heading forward first, without turning the nose, ending back on this spot
    land_and_disarm   go down like land, then keep pressing down until PX4 disarms

engine.check_steps refuses a list that breaks these rules (first step a fly_to, last step
land_and_disarm, rest right after land, circle right after a fly_to to its spot, ...).

Safety that applies to every line (engine.py): a step over 30 s lands where it is; a stale camera
pose holds still, then goes straight down after 3 s; the pilot taking over stops it for good.
"""
from dataclasses import dataclass


@dataclass(frozen=True)
class Step:
    kind: str
    at: tuple = None          # (forward, left, up) in metres from the take-off spot
    seconds: float = 0.0
    facing: float = None      # degrees left of the take-off heading
    diameter: float = 0.0     # circle only


def fly_to(forward, left, up):
    return Step('move', (forward, left, up))


def hover(seconds, facing):
    return Step('hold', None, seconds, facing)


def land():
    return Step('land')


def rest(seconds):
    return Step('ground', seconds=seconds)


def circle(forward, left, up, diameter):
    return Step('circle', (forward, left, up), diameter=diameter)


def land_and_disarm():
    return Step('disarm')


STEPS = [
    # ---- the square: 1 m sides, 0.73 m up, a left turn at every corner ----
    fly_to(0.0, 0.0, 0.73),                          # 1.  rise straight up to 0.73 m
    hover(seconds=2, facing=0),                      # 2.  hover 2 s, facing the take-off way
    fly_to(1.0, 0.0, 0.73),                          # 3.  fly forward 1 m
    hover(seconds=2, facing=90),                     # 4.  turn left to 90, hover 2 s
    fly_to(1.0, 1.0, 0.73),                          # 5.  fly left 1 m
    hover(seconds=2, facing=180),                    # 6.  turn left to 180, hover 2 s
    fly_to(0.0, 1.0, 0.73),                          # 7.  fly back 1 m
    hover(seconds=2, facing=270),                    # 8.  turn left to 270, hover 2 s
    fly_to(0.0, 0.0, 0.73),                          # 9.  fly right 1 m, back over the start
    hover(seconds=2, facing=360),                    # 10. turn left to 360 (the take-off way), hover 2 s

    # ---- touch down and go again ----
    land(),                                          # 11. land
    rest(seconds=2),                                 # 12. sit on the ground 2 s, motors idling
    fly_to(0.0, 0.0, 0.73),                          # 13. rise straight up to 0.73 m
    hover(seconds=2, facing=360),                    # 14. hover 2 s

    # ---- the circle: 1 m across, inside the square ----
    fly_to(0.5, 0.0, 0.73),                          # 15. fly forward 0.5 m to the circle's edge
    circle(0.5, 0.0, 0.73, diameter=1.0),            # 16. go once round the circle, back to here
    fly_to(0.0, 0.0, 0.73),                          # 17. fly back 0.5 m, over the start
    hover(seconds=2, facing=360),                    # 18. hover 2 s

    # ---- done ----
    land(),                                          # 19. land
    land_and_disarm(),                               # 20. keep pressing down until PX4 disarms
]
