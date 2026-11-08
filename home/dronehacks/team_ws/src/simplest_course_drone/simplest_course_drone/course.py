"""Where are we on the course? Pure Python, testable. Every number comes from the map file.

The map is planner/'s course map (contracts/course.md format, plus its PROPOSED launch,
gate and group fields): walls and the net as thick line segments, boulders as discs, and the goals
in visiting order. This file adds three things on top of it:

1. CourseFrame  course (east, north) <-> the drone's odom frame (forward, left, up).
   Setup rule this relies on: the drone powers on ON the map's launch mark, FACING the map's
   launch heading, so cuVSLAM's origin and axes are the launch pose. contracts/frames.md says
   this transform should be measured, not assumed: `launch_offset` and `rest_height_m` are the
   knobs for a measured correction. Flight height comes from the map (slice_height, 0.8 m above
   the floor); odom z is measured from the odom origin, which sits rest_height_m above the floor when
   the stack starts on the mark: skybeta-11's Pixhawk rests 0.10 m up (team, 2026-11-08; the course
   follower's odom_origin_height_m). On the taped course, 1.23 m (the window's measured middle) - 0.10
   = 1.13 m.

2. nearest_wall  distance from a point to the closest wall, net or boulder surface.

3. Progress      the checkpoints in order. A pass-through goal (gate, tunnel, window) counts when
   the drone's centre gets DRONE_EXTENT past the goal's far face (the whole drone across), inside
   the opening's width. The pad counts when the drone is landed and disarmed with the whole drone
   inside the pad. Goals in the same group are alternatives (Gate B or the window).
"""
import math
from dataclasses import dataclass

DRONE_EXTENT = 0.28        # centre to prop tip, metres (contracts/course.md derived error budget)


# ---- 1. the frame ----
@dataclass
class CourseFrame:
    launch: tuple              # (east, north) of the launch mark
    heading: tuple             # unit (east, north) the drone faces at launch
    flight_height_m: float     # above the floor
    rest_height_m: float       # the body's height above the floor when it sits on the mark

    @classmethod
    def from_map(cls, cmap, rest_height_m=0.10, launch_offset=(0.0, 0.0)):
        if cmap.launch is None or cmap.launch.heading is None:
            raise ValueError('the map has no launch position and heading: cannot place the drone on it')
        e, n = cmap.launch.position
        return cls((e + launch_offset[0], n + launch_offset[1]), tuple(cmap.launch.heading),
                   cmap.slice_height, rest_height_m)

    @property
    def left(self):
        return (-self.heading[1], self.heading[0])           # heading turned 90 deg anticlockwise

    @property
    def flight_z(self):
        """Odom z for the map's flight height."""
        return self.flight_height_m - self.rest_height_m

    def to_odom(self, east, north):
        d = (east - self.launch[0], north - self.launch[1])
        return (d[0] * self.heading[0] + d[1] * self.heading[1], d[0] * self.left[0] + d[1] * self.left[1])

    def to_course(self, x, y):
        return (self.launch[0] + x * self.heading[0] + y * self.left[0],
                self.launch[1] + x * self.heading[1] + y * self.left[1])

    def direction_to_odom_deg(self, east, north):
        """A course direction as degrees left of the launch heading (the steps' `facing`)."""
        x, y = self.to_odom(self.launch[0] + east, self.launch[1] + north)
        return math.degrees(math.atan2(y, x))


# ---- 2. walls ----
def _to_segment(p, a, b):
    ab = (b[0] - a[0], b[1] - a[1])
    L2 = ab[0] ** 2 + ab[1] ** 2
    t = 0.0 if L2 == 0 else max(0.0, min(1.0, ((p[0] - a[0]) * ab[0] + (p[1] - a[1]) * ab[1]) / L2))
    return math.dist(p, (a[0] + t * ab[0], a[1] + t * ab[1]))


def nearest_wall(cmap, east, north):
    """(distance to the nearest surface in metres, what it is). A thick wall reads negative inside
    it; blocks drawn as zero-thickness outlines read 0 on their edge."""
    best = (math.inf, '')
    for i, s in enumerate(cmap.segments):
        d = _to_segment((east, north), s.start, s.end) - s.thickness / 2.0
        best = min(best, (d, 'wall {}'.format(i)))        # the net and block outlines are walls too
    for i, b in enumerate(cmap.boulders):
        best = min(best, (math.dist((east, north), b.centre) - b.radius, 'boulder {}'.format(i)))
    return best


# ---- 3. checkpoints ----
def _along_across(goal, east, north):
    """Position relative to a pass-through goal: (along its axis, across it), metres from its centre."""
    ax = goal.axis
    d = (east - goal.centre[0], north - goal.centre[1])
    return d[0] * ax[0] + d[1] * ax[1], -d[0] * ax[1] + d[1] * ax[0]


def through(goal, east, north):
    """True once the whole drone is past this opening's far face, inside its width."""
    along, across = _along_across(goal, east, north)
    return along >= goal.depth / 2.0 + DRONE_EXTENT and abs(across) <= float(goal.params['width']) / 2.0


def landed_on(goal, east, north):
    return math.dist((east, north), goal.centre) <= float(goal.params['radius']) - DRONE_EXTENT


class Progress:
    """Which checkpoint is next. Feed it the drone's course position (and landed/armed for the pad)."""

    def __init__(self, cmap):
        self.stages = cmap.stages()          # [(goal,), (gate_b, window), ...] in visiting order
        self.passed = []                     # names, in the order they were passed
        self._was_short = {}                 # goal name -> drone was short of its far face last time

    @property
    def done(self):
        return len(self.passed) == len(self.stages)

    @property
    def next(self):
        return None if self.done else self.stages[len(self.passed)]

    def label(self, stage):
        return stage[0].group or stage[0].name

    def update(self, east, north, landed=False, armed=True):
        """Returns the name of a goal passed on this call, else None."""
        if self.done:
            return None
        for g in self.next:
            if g.passes_through:
                along, _ = _along_across(g, east, north)
                was_short = self._was_short.get(g.name, True)
                self._was_short[g.name] = along < g.depth / 2.0 + DRONE_EXTENT
                hit = was_short and through(g, east, north)    # crossed going forward, this call
            else:
                hit = landed and armed is False and landed_on(g, east, north)
            if hit:
                self.passed.append(g.name)
                return g.name
        return None

    def distance_to_next(self, east, north):
        """Metres from the drone to the next goal's centre (the nearest alternative)."""
        if self.done:
            return 0.0
        return min(math.dist((east, north), g.centre) for g in self.next)

    def words(self, east, north):
        passed = ', '.join(self.passed) if self.passed else 'none'
        if self.done:
            return 'ALL CHECKPOINTS PASSED ({})'.format(passed)
        return 'next: {} {:.2f} m | passed: {}'.format(
            self.label(self.next), self.distance_to_next(east, north), passed)
