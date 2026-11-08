"""fusion.py: each rule on its own."""
import math

from simplest_fusion_drone.fusion import Config, Fusion, Readings, ned_to_enu, px4_yaw_to_enu

NOW = 100.0


def good(**kw):
    r = Readings(camera=(1.0, 2.0, 0.73, 0.3), camera_t=NOW, vo_state=1, vo_t=NOW,
                 ekf=(1.05, 2.0, 0.75), ekf_xy_valid=True, dist_bottom=0.75, ekf_t=NOW,
                 px4_yaw_enu=0.3, att_t=NOW, ev_pos=True, ev_yaw=True, rng_hgt=True, flags_t=NOW,
                 texture=0.2, brightness=90.0, image_t=NOW)
    for k, v in kw.items():
        setattr(r, k, v)
    return r


def test_frames():
    assert ned_to_enu(1.0, 2.0, -3.0) == (2.0, 1.0, 3.0)
    assert abs(px4_yaw_to_enu(0.0) - math.pi / 2) < 1e-9          # NED north = ENU +y (90 deg)
    assert abs(px4_yaw_to_enu(math.pi / 2)) < 1e-9                 # NED east = ENU +x (0 deg)


def test_all_good_in_flight_is_trusted_and_flies_the_camera_pose():
    b = Fusion().update(NOW, good(), flying=True)
    assert b.trusted and b.pose == (1.0, 2.0, 0.73) and b.yaw == 0.3, b.words()


def test_each_failure_breaks_trust_in_flight():
    cases = {
        'camera: no fresh pose': dict(camera_t=NOW - 1.0),
        'tracking: LOST': dict(vo_state=2),
        'tracking: no status': dict(vo_t=None),
        'ekf2: NOT fusing vision': dict(ev_pos=False),
        'ekf2: no status flags': dict(flags_t=NOW - 5),
        'image: too dark': dict(brightness=5.0),
        'image: blank': dict(texture=0.002),
        'ekf2: no valid position': dict(ekf_xy_valid=False),
    }
    for words, kw in cases.items():
        b = Fusion().update(NOW, good(**kw), flying=True)
        assert not b.trusted and b.pose is None and any(p.startswith(words) for p in b.problems), (words, b.words())


def test_no_image_is_only_a_note_unless_required():
    b = Fusion().update(NOW, good(image_t=None), flying=True)
    assert b.trusted and 'image: not heard' in b.notes
    b = Fusion(Config(require_image=True)).update(NOW, good(image_t=None), flying=True)
    assert not b.trusted and 'image: none' in b.problems


def test_preflight_waits_for_settle_and_withholds_the_pose():
    f = Fusion(Config(settle_s=5.0))
    b = f.update(NOW, good(), flying=False)
    assert b.trusted and not b.ready and b.pose is None and 'settling 0/5 s' in b.notes
    b = f.update(NOW + 5.0, good(camera_t=NOW + 5, vo_t=NOW + 5, ekf_t=NOW + 5, att_t=NOW + 5,
                                  flags_t=NOW + 5, image_t=NOW + 5), flying=False)
    assert b.ready and b.pose == (1.0, 2.0, 0.73)


def test_a_lapse_restarts_the_settle_timer():
    f = Fusion(Config(settle_s=5.0))
    f.update(NOW, good(), flying=False)
    f.update(NOW + 3, good(camera_t=NOW + 3, vo_t=NOW + 3, ekf_t=NOW + 3, att_t=NOW + 3, flags_t=NOW + 3,
                           image_t=NOW + 3, vo_state=2), flying=False)
    t = NOW + 6
    b = f.update(t, good(camera_t=t, vo_t=t, ekf_t=t, att_t=t, flags_t=t, image_t=t), flying=False)
    assert not b.ready and 'settling 0/5 s' in b.notes


def test_heading_gate_only_before_take_off():
    r = good(px4_yaw_enu=0.3 + math.radians(3.0))
    b = Fusion().update(NOW, r, flying=False)
    assert not b.trusted and any(p.startswith('heading: PX4 and camera 3.0 deg apart') for p in b.problems)
    assert Fusion().update(NOW, r, flying=True).trusted


def test_drift_since_take_off():
    f = Fusion(Config(settle_s=0.0))
    assert f.update(NOW, good(), flying=False).ready            # baseline offset (0.05, 0)
    assert f.update(NOW, good(ekf=(1.30, 2.0, 0.75)), flying=True).trusted     # moved 0.25 m: ok
    b = f.update(NOW, good(ekf=(1.40, 2.0, 0.75)), flying=True)                # moved 0.35 m
    assert not b.trusted and any(p.startswith('drift: camera and EKF2 moved 0.35 m') for p in b.problems)


def test_something_under_us_is_noted_not_acted_on():
    f = Fusion(Config(settle_s=0.0))
    f.update(NOW, good(camera=(1.0, 2.0, 0.0, 0.3), dist_bottom=0.07), flying=False)   # at rest: below0 -0.07
    b = f.update(NOW, good(dist_bottom=0.20), flying=True)    # at 0.73 m the floor should read ~0.80
    assert b.trusted and any(n.startswith('below: something 0.60 m tall') for n in b.notes), b.words()


def test_coasts_only_while_ekf2_is_healthy():
    f = Fusion(Config(settle_s=0.0))
    f.update(NOW, good(), flying=False)                      # take-off: remembers the gap
    b = f.update(NOW, good(vo_state=2), flying=True)
    assert b.coasting and b.pose == (1.0, 2.0, 0.73), b.words()          # EKF2 minus the gap
    b = f.update(NOW, good(vo_state=2, ekf_xy_valid=False), flying=True)
    assert not b.trusted and b.pose is None and 'ekf2: no valid position' in b.problems


def test_coast_starts_when_camera_and_ekf2_vision_drop_together():
    """Entering the dark tunnel: the image goes dark AND EKF2 stops fusing vision on the same tick.
    EKF2 dropping vision is expected when the camera is the side that failed; coast, don't stop."""
    f = Fusion()
    assert f.update(NOW, good(), flying=True).trusted                 # one good tick sets the gap
    b = f.update(NOW + 0.05, good(brightness=2.0, vo_state=2, ev_pos=False), flying=True)
    assert b.coasting and b.trusted and b.pose is not None, b.words()
