"""How each instruction in steps.py is carried out. Copied from square_circle/mission.py (the
flight-tested cage test); the only change is that it flies the list in steps.py instead of
building the plan itself. Pure logic, no ROS, so it can be unit-tested.

Frame: ENU ``odom`` (cuVSLAM's), whose origin and axes are the body's at power-on: x forward,
y left, z up. Every step's spot is measured from the take-off spot, so errors never add up from
one step to the next.

Drift handling that runs under every step (not written as steps):
* PX4 and the camera can disagree about where the drone is. The gap is measured when Offboard
  starts, followed (smoothed over 1 s) all flight, and added to every setpoint. More than 0.3 m
  apart before take-off: it refuses to fly.
* A hover's seconds only count once the drone is within 20 cm and 10 deg of its target.
* Landings pull back over the last fly_to's spot while descending (the take-off spot in the square).
* A moving target never runs more than 0.3 m ahead of the drone.

States: WAIT (stream the current position so the kit can enter Offboard on arm-then-Position),
FLY (working through the steps), DONE (disarmed), STOPPED (Offboard ended early: the pilot has
it; never tries again).
"""
import math
from dataclasses import dataclass

WAIT, FLY, DONE, STOPPED = 'WAIT', 'FLY', 'DONE', 'STOPPED'


@dataclass
class Config:
    speed_mps: float = 0.3
    lead_m: float = 0.3
    arrive_m: float = 0.20          # hold counts within this; 0.10 left 7-15 s holds in flight 20261108-013018
    step_timeout_s: float = 30.0    # a corner took 15.4 s in flight 20261108-013018
    descent_mps: float = 0.3        # the real follower's descent_mps
    land_xy_gain: float = 1.0       # the real follower's land_xy_gain
    land_xy_max_mps: float = 0.2    # the real follower's land_xy_max_mps
    pose_timeout_s: float = 0.5
    stale_land_s: float = 3.0
    yaw_rate_dps: float = 20.0      # slow turn: 90 deg in 4.5 s
    yaw_tol_deg: float = 10.0
    max_offset_m: float = 0.3       # PX4 vs camera position on the ground; more than this: refuse to fly
    offset_tau_s: float = 1.0       # smoothing of the PX4-vs-camera offset in flight


@dataclass
class Command:
    kind: str                        # 'none', 'pose' or 'twist'
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    yaw: float = None                # ENU rad for a pose; None: do not command yaw


NONE = Command('none')


def check_steps(steps):
    """Refuse a step list the engine cannot fly as written. Raises ValueError naming the step."""
    def bad(i, why):
        raise ValueError('step {} ({}): {}'.format(i + 1, steps[i].kind, why))
    if not steps:
        raise ValueError('no steps')
    if steps[0].kind != 'move':
        bad(0, 'the first step must be fly_to(): the drone starts on the ground')
    if steps[-1].kind != 'disarm':
        bad(len(steps) - 1, 'the last step must be land_and_disarm()')
    for i, st in enumerate(steps):
        prev = steps[i - 1] if i else None
        if st.kind not in ('move', 'hold', 'land', 'ground', 'circle', 'disarm'):
            bad(i, 'unknown kind')
        if st.kind in ('move', 'circle') and (st.at is None or len(st.at) != 3):
            bad(i, 'needs a spot (forward, left, up)')
        if st.kind in ('move', 'circle') and st.at[2] <= 0:
            bad(i, 'the spot must be above the ground (up > 0); use land() to come down')
        if st.kind in ('hold', 'land', 'ground', 'disarm') and st.at is not None:
            bad(i, 'takes no spot: it acts where the drone is')
        if st.kind == 'hold' and (st.facing is None or st.seconds <= 0):
            bad(i, 'hover needs seconds > 0 and a facing')
        if st.kind == 'hold' and (prev is None or prev.kind not in ('move', 'hold', 'circle')):
            bad(i, 'hover() must come in the air: after a fly_to(), hover() or circle()')
        if st.kind == 'ground' and (prev is None or prev.kind != 'land' or st.seconds <= 0):
            bad(i, 'rest() must come right after land(), with seconds > 0')
        if st.kind == 'circle' and (prev is None or prev.kind != 'move' or prev.at != st.at):
            bad(i, 'circle() must come right after a fly_to() to the same spot')
        if st.kind == 'circle' and st.diameter <= 0:
            bad(i, 'the diameter must be > 0')
        if st.kind == 'disarm' and i != len(steps) - 1:
            bad(i, 'land_and_disarm() must be the last step')


class Mission:
    def __init__(self, steps, cfg: Config = Config()):
        self.cfg = cfg
        self.steps = list(steps)
        check_steps(self.steps)
        self.state = WAIT
        self.reason = 'waiting for arm, then Position'
        self.i = 0
        self.start = None
        self.target = None           # the sliding pose target, ENU
        self.start_yaw = None        # ENU rad at Offboard; None if the pose had no heading
        self.heading = 0.0           # the sliding heading target, degrees from start_yaw, unwrapped
        self.since = None            # when the current step started
        self.dwell_since = None
        self.stale_since = None
        self.last_t = None
        self.offset = (0.0, 0.0, 0.0)  # PX4 local minus camera, smoothed
        self.arc = 0.0               # radians of the circle covered

    @property
    def step_name(self):
        return self.steps[self.i].kind if self.state == FLY else self.state

    def step(self, t, pose, pose_t, offboard, landed, armed, yaw=None, px4=None):
        """One tick. pose: (x, y, z) or None; pose_t: when it was captured (s); yaw: its ENU
        heading in radians, or None (then the drone holds whatever heading it has); px4: PX4's
        own local position, ENU (``/skybeta/state`` local_enu), or None.

        The kit passes pose setpoints to PX4 unanchored (``anchor_setpoints`` false), so PX4 flies
        them in its own frame. Its height can sit off the camera's (0.73 m in one flight, 0.15 m
        after a clean power cycle), so every setpoint carries the PX4-minus-camera offset taken at
        Offboard, then followed (smoothed) through the flight, and all checks stay in the camera frame."""
        c = self.cfg
        fresh = pose is not None and pose_t is not None and t - pose_t <= c.pose_timeout_s
        if self.state in (DONE, STOPPED):
            return NONE
        if self.state == WAIT:
            if not fresh:
                return NONE          # no stream: the kit will not enter Offboard
            self.offset = (0.0, 0.0, 0.0) if px4 is None else tuple(p - o for p, o in zip(px4, pose))
            if math.hypot(*self.offset) > c.max_offset_m:
                self.reason = 'PX4 and camera disagree by {:.2f} m: power-cycle on the spot'.format(
                    math.hypot(*self.offset))
                return NONE          # no stream: no Offboard, the pilot keeps Position
            self.reason = 'waiting for arm, then Position'
            if not offboard:
                return Command('pose', *self._px4(pose))
            self.start, self.target, self.start_yaw, self.heading = pose, pose, yaw, 0.0
            self.state, self.i, self.since, self.last_t = FLY, 0, t, t
        elif armed is False:
            # checked first: PX4 disarming also ends Offboard, and the kit can still report it active
            kind = self.steps[self.i].kind
            self.state = DONE
            self.reason = 'disarmed on the ground' if kind == 'disarm' else \
                'PX4 disarmed during step {} ({}): landed longer than COM_DISARM_LAND?'.format(self.i + 1, kind)
            return NONE
        elif not offboard:
            self.state, self.reason = STOPPED, 'Offboard ended: the pilot has it'
            return NONE

        dt, self.last_t = max(0.0, t - self.last_t), t
        if not fresh:
            if self.stale_since is None:
                self.stale_since = t
            if t - self.stale_since >= c.stale_land_s:
                self._abort('pose lost: straight down')
                return Command('twist', 0.0, 0.0, -c.descent_mps)
            self.reason = 'pose stale: holding still'
            return Command('twist')
        self.stale_since = None
        if px4 is not None:
            # follow PX4's frame as it changes (its rangefinder height is wrong on the ground, right in
            # the air); 1 s smoothing; both inputs measure the same drone, so this is not a control loop
            a = min(1.0, dt / c.offset_tau_s)
            self.offset = tuple(o + a * (p - q - o) for o, p, q in zip(self.offset, px4, pose))

        st = self.steps[self.i]
        kind, offset, secs, heading = st.kind, st.at, st.seconds, st.facing
        self.reason = 'step {}/{}: {}'.format(self.i + 1, len(self.steps), kind)
        if kind not in ('land', 'disarm') and t - self.since > c.step_timeout_s:
            self._abort('step {} ({}) took over {:.0f} s: landing here'.format(self.i + 1, kind, c.step_timeout_s))
            self.target = pose                      # land where it is
            kind = 'disarm'

        if kind == 'move':
            goal = self._at(offset)
            gap = math.dist(self.target, goal)
            if math.dist(pose, self.target) < c.lead_m and gap > 0:
                f = min(1.0, c.speed_mps * dt / gap)
                self.target = tuple(a + (b - a) * f for a, b in zip(self.target, goal))
            if math.dist(self.target, goal) < 1e-6:
                self._next(t)
            return self._pose()
        if kind == 'circle':
            r = st.diameter / 2.0
            if math.dist(pose, self.target) < c.lead_m:
                self.arc = min(2.0 * math.pi, self.arc + c.speed_mps * dt / r)
            # the circle starts at its own spot ``offset`` and goes round the centre r to its left;
            # the entry is at -pi/2, so it heads forward first, counter-clockwise seen from above.
            # (square_circle hard-coded the centre at (r, r) from the take-off spot, which is the
            # same place when the circle starts at (r, 0), as it does in steps.py)
            a = -math.pi / 2 + self.arc
            self.target = self._at((offset[0] + r * math.cos(a), offset[1] + r + r * math.sin(a), offset[2]))
            if self.arc >= 2.0 * math.pi:
                self.target = self._at(offset)
                self._next(t)
            return self._pose()
        if kind == 'hold':
            turn = heading - self.heading
            self.heading += max(-c.yaw_rate_dps * dt, min(c.yaw_rate_dps * dt, turn))
            facing = self.start_yaw is None or (
                abs(turn) < 1e-9 and (yaw is None or abs(_wrap(yaw - self._yaw_cmd())) <= math.radians(c.yaw_tol_deg)))
            if math.dist(pose, self.target) > c.arrive_m or not facing:
                self.dwell_since = None
            elif self.dwell_since is None:
                self.dwell_since = t
            elif t - self.dwell_since >= secs:
                self._next(t)
            return self._pose()
        if kind == 'ground':
            if self.dwell_since is None:
                self.dwell_since = t
            if t - self.dwell_since >= secs:
                self.target = (self.target[0], self.target[1], pose[2])   # take off again over the landing spot
                self._next(t)
            return Command('twist', 0.0, 0.0, -0.3 * c.descent_mps)
        # land / disarm: the real follower's descent, pulling back over the last target (the spot the
        # last fly_to reached; in the square that is always the take-off spot, as square_circle did)
        if landed:
            if kind == 'land':
                self._next(t)
            return Command('twist', 0.0, 0.0, -0.3 * c.descent_mps)
        vx = _clip(c.land_xy_gain * (self.target[0] - pose[0]), c.land_xy_max_mps)
        vy = _clip(c.land_xy_gain * (self.target[1] - pose[1]), c.land_xy_max_mps)
        return Command('twist', vx, vy, -c.descent_mps)

    # ------------------------------------------------------------------
    def _yaw_cmd(self):
        return _wrap(self.start_yaw + math.radians(self.heading))

    def _pose(self):
        return Command('pose', *self._px4(self.target), yaw=None if self.start_yaw is None else self._yaw_cmd())

    def _px4(self, p):
        return tuple(a + o for a, o in zip(p, self.offset))

    def _at(self, offset):
        return tuple(s + o for s, o in zip(self.start, offset))

    def _next(self, t):
        self.i, self.since, self.dwell_since, self.arc = self.i + 1, t, None, 0.0

    def _abort(self, why):
        self.i = len(self.steps) - 1            # the final 'disarm' landing
        self.reason = why


def _wrap(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def _clip(v, m):
    return max(-m, min(m, v))
