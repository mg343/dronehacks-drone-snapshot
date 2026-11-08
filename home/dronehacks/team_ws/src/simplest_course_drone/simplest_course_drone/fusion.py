"""Sensor fusion: one answer to "where am I, and can I trust it?". Pure Python, testable.

Inputs: the relevant subset from subscriptions.py (its reasoning thread says
why each one is there). Output: a Belief. The navigation rule that uses it is one line (node.py):

    a pose  -> give engine.py that pose (it flies as flight-tested)
    no pose -> give it nothing (its tested behaviour: hold still, after 3 s land straight down;
               before take-off: stream nothing, so the kit never enters Offboard)

Where the pose comes from, in order:
    1. the camera, when every check passes                                     (source "camera")
    2. EKF2, for up to coast_s, when only the camera side is failing (the dark tunnel, a blank
       wall, tracking lost) and EKF2 is still healthy: EKF2 keeps estimating from the Pixhawk's
       IMU and the rangefinder. Shifted by the last camera-to-EKF2 gap, so nothing jumps.  ("ekf2")
    3. nothing: both sides failing, the coast ran out, or camera and EKF2 disagree while both look
       healthy (then we cannot tell which one is wrong)

When the camera comes back after a coast, it is re-anchored: shifted so it continues from where
the coast left off (cuVSLAM may restart from a new origin after losing track; field guide: "when
vision returns, continue from PX4's own estimate so there is no jump").

Fusion here means comparing sources that fail differently:
    camera (cuVSLAM)    position + heading; confidently wrong on a blank view, blind in the dark
    tracking status     cuVSLAM's own "lost"; misses the blank-view case
    EKF2 (PX4)          fuses camera + Pixhawk IMU + rangefinder; says whether it accepts vision
    PX4 heading         from EKF2's attitude; the field guide's heading gate compares it to vision
    the image           texture and brightness: the honest signal for "the camera can't see"
    rangefinder         height above what is directly below (in vehicle_local_position)

Frames: camera and the kit are ENU (x forward at power-on, y left, z up). PX4 is NED:
    x_enu = y_ned, y_enu = x_ned, z_enu = -z_ned, yaw_enu = 90 deg - yaw_ned   (the kit's guide)
"""
import math
from dataclasses import dataclass, field


@dataclass
class Config:
    pose_timeout_s: float = 0.5        # older camera pose: stale
    other_timeout_s: float = 1.0       # older status / EKF2 / image reading: stale
    flags_timeout_s: float = 2.5       # PX4 publishes estimator_status_flags at 1 Hz (gaps up to 1.05 s
                                       # in every recorded flight); 1.0 s raised false alarms mid-flight
    heading_tol_deg: float = 2.0       # field guide: start only when PX4 and vision agree within 2 deg
    drift_tol_m: float = 0.2           # camera vs EKF2 horizontal, change since take-off. Real flights:
                                       # at most 0.06 m clean (082135, 081345); 0.32 m 2.6 s into the
                                       # frozen-camera fly-away (113426)
    height_tol_m: float = 0.25         # camera vs EKF2 height, change since take-off. Real flights: at
                                       # most 0.15 m clean; 0.31 m at 2.6 s in 113426, where the camera
                                       # froze at take-off, still "tracking", while PX4 climbed to 3.7 m
    min_texture: float = 0.01          # field guide's guard: under 1% textured pixels is blank
    min_brightness: float = 15.0       # mean of mono8, 0-255: darker than this the camera is blind
    settle_s: float = 30.0             # field guide: 30 s still and trusted before the route starts
    steady_tol_m: float = 0.10         # on the ground the PX4-minus-camera gap must hold within this
                                       # for the whole settle: any size of gap is fine, a moving one is
                                       # not (a tracker error jumped the camera 9 m in 20261108-114136)
    obstacle_below_m: float = 0.3      # camera height minus rangefinder: more than this, something is under us
    require_image: bool = False        # True: no image heard counts against trust (off: first form, report only)
    coast_s: float = 6.0               # longest EKF2-only stretch: the 1.5 m tunnel at 0.3 m/s is ~5 s.
                                       # PX4 ends it sooner if EKF2 marks its position invalid (xy_valid).


@dataclass
class Readings:
    """The latest value from each input and when it arrived (seconds, receipt clock). None = never."""
    camera: tuple = None               # (x, y, z, yaw_enu_rad)
    camera_t: float = None
    vo_state: int = None               # 1 tracking, 2 lost, 0 unknown
    vo_t: float = None
    ekf: tuple = None                  # (x_enu, y_enu, z_enu) from vehicle_local_position
    ekf_xy_valid: bool = None
    dist_bottom: float = None          # rangefinder, metres; None when invalid
    ekf_t: float = None
    px4_yaw_enu: float = None          # from vehicle_attitude, converted to ENU
    att_t: float = None
    ev_pos: bool = None                # estimator_status_flags.cs_ev_pos: EKF2 fusing vision position
    ev_yaw: bool = None
    rng_hgt: bool = None
    flags_t: float = None
    texture: float = None              # fraction of pixels with a strong gradient, 0-1
    brightness: float = None           # mean pixel, 0-255
    image_t: float = None


@dataclass
class Belief:
    pose: tuple = None                 # (x, y, z) to fly on, or None
    yaw: float = None
    source: str = None                 # 'camera', 'ekf2' (coasting) or None
    trusted: bool = False              # there is a pose to fly on
    ready: bool = False                # pre-flight: trusted for settle_s; the route may start
    problems: list = field(default_factory=list)
    notes: list = field(default_factory=list)   # sensed, not acted on (yet)

    @property
    def coasting(self):
        return self.source == 'ekf2'

    def words(self):
        head = 'TRUSTED' if self.trusted else 'NOT TRUSTED'
        if self.coasting:
            head = 'COASTING ON EKF2'
        elif self.trusted and not self.ready:
            head += ' (settling)'
        return ' | '.join([head] + self.problems + self.notes)


def _fresh(now, t, limit):
    return t is not None and now - t <= limit


def _wrap(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def _add(a, b):
    return tuple(x + y for x, y in zip(a, b))


def _sub(a, b):
    return tuple(x - y for x, y in zip(a, b))


def px4_yaw_to_enu(yaw_ned):
    return _wrap(math.pi / 2.0 - yaw_ned)


def ned_to_enu(x, y, z):
    return (y, x, -z)


def yaw_from_quaternion(w, x, y, z):
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


class Fusion:
    """Remembers what fusion needs between ticks: the settle timer, the take-off gap between camera
    and EKF2, the latest gap, the coast timer, and the re-anchor shift."""

    def __init__(self, cfg: Config = Config()):
        self.cfg = cfg
        self.trusted_since = None
        self.settle_gap = None         # EKF2 minus camera when the settle began
        self.offset0 = None            # EKF2 minus camera, 3-D, frozen when the route starts
        self.gap = None                # EKF2 minus camera, 3-D, the latest while both were good
        self.below0 = None             # camera z minus rangefinder at rest, frozen with offset0
        self.coast_since = None
        self.coast_pose = None         # the last pose flown on while coasting
        self.shift = (0.0, 0.0, 0.0)   # added to every camera position (re-anchoring)
        self.tripped = None            # the drift/height problem that ended trust for this flight

    # ---- the two sides ----
    def camera_problems(self, now, r):
        c, out = self.cfg, []
        if r.camera is None or not _fresh(now, r.camera_t, c.pose_timeout_s):
            out.append('camera: no fresh pose')
        if not _fresh(now, r.vo_t, c.other_timeout_s):
            out.append('tracking: no status')
        elif r.vo_state != 1:
            out.append('tracking: LOST' if r.vo_state == 2 else 'tracking: state {}'.format(r.vo_state))
        if _fresh(now, r.image_t, c.other_timeout_s):
            if r.brightness < c.min_brightness:
                out.append('image: too dark ({:.0f})'.format(r.brightness))
            elif r.texture < c.min_texture:
                out.append('image: blank ({:.1%} texture)'.format(r.texture))
        elif c.require_image:
            out.append('image: none')
        return out

    def ekf_problems(self, now, r, camera_ok=True):
        c, out = self.cfg, []
        if not _fresh(now, r.flags_t, c.flags_timeout_s):
            out.append('ekf2: no status flags')
        elif not r.ev_pos and camera_ok and self.coast_since is None:
            # EKF2 rejecting vision counts against it only while the camera looks fine (then the two
            # disagree). When the camera has failed, or we are coasting, EKF2 is expected to drop it.
            out.append('ekf2: NOT fusing vision')
        if r.ekf is None or not _fresh(now, r.ekf_t, c.other_timeout_s) or not r.ekf_xy_valid:
            out.append('ekf2: no valid position')
        return out

    # ---- one tick ----
    def update(self, now, r: Readings, flying: bool) -> Belief:
        """flying: the engine is past WAIT (Offboard has started). Before that, pre-flight rules."""
        b = Belief()
        cam_bad = self.camera_problems(now, r)
        ekf_bad = self.ekf_problems(now, r, camera_ok=not cam_bad)
        if not self.cfg.require_image and not _fresh(now, r.image_t, self.cfg.other_timeout_s):
            b.notes.append('image: not heard')
        if flying:
            self._in_flight(now, r, cam_bad, ekf_bad, b)
        else:
            self._pre_flight(now, r, cam_bad + ekf_bad, b)
        self._below(now, r, b)
        return b

    def _in_flight(self, now, r, cam_bad, ekf_bad, b):
        c = self.cfg
        if self.tripped:
            # latched: in flight 20261108-081345 the camera jumped 37 m at a wall strike and EKF2
            # followed it 1.8 s later, closing the gap again; a source that jumped is never trusted
            # again this flight (the engine is already holding, then landing straight down)
            b.problems += self.tripped
            return
        if not cam_bad and not ekf_bad:
            if self.coast_since is not None:                  # vision is back: re-anchor
                jump = _sub(self.coast_pose, _add(r.camera[:3], self.shift))
                self.shift = _add(self.shift, jump)
                b.notes.append('re-anchored the camera by {:.2f} m'.format(math.hypot(*jump)))
                self.coast_since = self.coast_pose = None
            cam = _add(r.camera[:3], self.shift)
            gap = _sub(r.ekf, cam)
            if self.offset0 is not None:
                sideways = math.dist(gap[:2], self.offset0[:2])
                updown = abs(gap[2] - self.offset0[2])
                if sideways > c.drift_tol_m:
                    b.problems.append('drift: camera and EKF2 moved {:.2f} m apart'.format(sideways))
                if updown > c.height_tol_m:
                    b.problems.append('height: camera and EKF2 moved {:.2f} m apart (frozen camera?)'.format(updown))
                if b.problems:
                    self.tripped = ['{} (latched)'.format(p) for p in b.problems]
                    return
            self.gap = gap
            b.pose, b.yaw, b.source = cam, r.camera[3], 'camera'
        elif cam_bad and not ekf_bad and self.gap is not None:
            self.coast_since = now if self.coast_since is None else self.coast_since
            if now - self.coast_since > c.coast_s:
                b.problems += cam_bad + ['coast: over {:.0f} s'.format(c.coast_s)]
                return
            b.pose = _sub(r.ekf, self.gap)
            b.yaw = r.px4_yaw_enu if _fresh(now, r.att_t, c.other_timeout_s) else None
            b.source = 'ekf2'
            self.coast_pose = b.pose
            b.notes.append('coast {:.1f}/{:.0f} s: {}'.format(now - self.coast_since, c.coast_s, ', '.join(cam_bad)))
        else:
            b.problems += cam_bad + ekf_bad
            return
        b.trusted = b.ready = True

    def _pre_flight(self, now, r, problems, b):
        c = self.cfg
        b.problems += problems
        if not problems:
            # heading gate: only before take-off (in flight the steps turn the drone on purpose,
            # and EKF2 follows vision yaw, so a mismatch would show up as drift instead)
            if _fresh(now, r.att_t, c.other_timeout_s):
                err = math.degrees(abs(_wrap(r.camera[3] - r.px4_yaw_enu)))
                if err > c.heading_tol_deg:
                    b.problems.append('heading: PX4 and camera {:.1f} deg apart'.format(err))
            else:
                b.problems.append('heading: no PX4 attitude')
        if not b.problems:
            gap = _sub(r.ekf, _add(r.camera[:3], self.shift))
            if self.trusted_since is None:
                self.settle_gap = gap
            elif math.dist(gap, self.settle_gap) > c.steady_tol_m:
                b.problems.append('ground: PX4 and camera moved {:.2f} m apart while still'.format(
                    math.dist(gap, self.settle_gap)))
        if b.problems:
            self.trusted_since = None
            return
        b.trusted = True
        self.trusted_since = now if self.trusted_since is None else self.trusted_since
        b.ready = now - self.trusted_since >= c.settle_s
        if not b.ready:
            b.notes.append('settling {:.0f}/{:.0f} s'.format(now - self.trusted_since, c.settle_s))
            return                         # no pose, no stream, no Offboard until it has settled
        cam = _add(r.camera[:3], self.shift)
        self.gap = _sub(r.ekf, cam)
        self.offset0 = self.gap
        if r.dist_bottom is not None:
            self.below0 = cam[2] - r.dist_bottom
        b.pose, b.yaw, b.source = cam, r.camera[3], 'camera'

    def _below(self, now, r, b):
        """Sensing only: something under us (a boulder reads as a floor step)."""
        if (b.pose is None or r.dist_bottom is None or self.below0 is None
                or not _fresh(now, r.ekf_t, self.cfg.other_timeout_s)):
            return
        rise = (b.pose[2] - r.dist_bottom) - self.below0
        if rise > self.cfg.obstacle_below_m:
            b.notes.append('below: something {:.2f} m tall under us'.format(rise))
