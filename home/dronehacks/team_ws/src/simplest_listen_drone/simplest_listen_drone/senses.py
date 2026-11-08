"""Turn the drone's two input messages into plain words. Pure Python, no ROS, so it can be tested.

The organisers' interface gives us two things to read (docs/competitor_guide.md, section 1):

    /visual_slam/tracking/odometry   the camera (cuVSLAM): where the drone is and which way it faces.
                                     Metres from where it powered on: x forward, y left, z up
                                     (forward/left = the way the nose pointed at power-on).
    /skybeta/state                   PX4, the flight controller, as JSON at 5 Hz: armed or not,
                                     flight mode, Offboard or not, landed or not, PX4's own
                                     position estimate, and whether the kit changed our setpoint.

Nothing here decides anything. It only reports.
"""
import json
import math

STALE_S = 1.0        # older than this and we say so instead of trusting it


def camera_from_odometry(x, y, z, qx, qy, qz, qw):
    """Position (m) and heading (degrees left of the power-on heading) from an Odometry pose."""
    yaw = math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))
    return {'x': x, 'y': y, 'z': z, 'heading_deg': math.degrees(yaw)}


def px4_from_state(text):
    """The /skybeta/state JSON as a dict, or None if it does not parse."""
    try:
        s = json.loads(text)
    except ValueError:
        return None
    return s if isinstance(s, dict) else None


def describe(now, camera, camera_t, px4, px4_t):
    """One line of plain words: what the camera says, what PX4 says, and whether they agree."""
    return ' | '.join([_camera_words(now, camera, camera_t), _px4_words(now, px4, px4_t),
                       _agree_words(now, camera, camera_t, px4, px4_t)])


def _fresh(now, t):
    return t is not None and now - t <= STALE_S


def _camera_words(now, c, t):
    if c is None:
        return 'camera: nothing yet'
    if not _fresh(now, t):
        return 'camera: no pose for {:.1f} s'.format(now - t)
    return 'camera: {:+.2f} fwd {:+.2f} left {:+.2f} up, facing {:+.0f} deg'.format(
        c['x'], c['y'], c['z'], c['heading_deg'])


def _px4_words(now, s, t):
    if s is None:
        return 'px4: nothing yet'
    if not _fresh(now, t):
        return 'px4: no state for {:.1f} s'.format(now - t)
    if not s.get('px4_link'):
        return 'px4: NO LINK to the flight controller'
    words = ['ARMED' if s.get('armed') else 'disarmed',
             str(s.get('nav_state_name', '?')),
             'offboard ON' if s.get('offboard_active') else 'offboard off',
             {True: 'on the ground', False: 'in the air'}.get(s.get('landed'), 'ground unknown')]
    if s.get('clamped'):
        words.append('kit clamped: ' + ','.join(s['clamped']))
    if s.get('frozen'):
        words.append('FROZEN (hold)')
    if s.get('envelope_ok') is False:
        words.append('ABOVE max altitude')
    return 'px4: ' + ', '.join(words)


def _agree_words(now, c, ct, s, st):
    """The camera's position against PX4's own estimate. The guide: if they diverge, the vision
    link is degrading. (square_circle refuses to take off when they are over 0.3 m apart.)"""
    if c is None or s is None or not _fresh(now, ct) or not _fresh(now, st):
        return 'agree: unknown'
    local = s.get('local_enu')
    if not (isinstance(local, list) and len(local) == 3):
        return 'agree: px4 has no position'
    gap = math.dist((c['x'], c['y'], c['z']), local)
    return 'agree: px4 and camera {:.2f} m apart{}'.format(gap, '  <-- over 0.3 m' if gap > 0.3 else '')
