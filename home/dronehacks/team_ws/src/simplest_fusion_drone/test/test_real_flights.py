"""Fusion replayed on skybeta-11's own recorded flights (team branch core/flight-iteration).

test/flights/<run>.csv holds, at 20 Hz, what the fusion node would have held at each instant: the
latest camera odometry, cuVSLAM status, PX4 local position (converted to ENU), attitude, estimator
flags and the kit's /skybeta/state, read from the run's bag (scratch script: every topic sampled as
its latest message, times in seconds from Offboard). No images were recorded, so the image checks
are idle, as they are when require_image is off.

    082135  the farthest real run: 55 s of Offboard, through the tunnel, then a wall strike just
            after Offboard ended (the pilot switched to Position first)
    081345  hit the north wall 23.5 s into Offboard, after which the camera pose jumped 37 m
    113426  the camera froze at take-off, still reporting "tracking"; PX4 climbed to 3.7 m
"""
import csv
import pathlib

from simplest_fusion_drone.fusion import Config, Fusion, Readings

HERE = pathlib.Path(__file__).parent / 'flights'


def replay(run, cfg=Config(settle_s=5.0)):
    """[(t, flying, belief)] over the recording; flying from the kit's first Offboard report."""
    rows = list(csv.DictReader(open(HERE / (run + '.csv'))))
    num = lambda v: float(v) if v != '' else None
    f, out, flying = Fusion(cfg), [], False
    for row in rows:
        v = {k: num(x) for k, x in row.items()}
        flying = flying or v['offboard'] == 1
        r = Readings(camera=(v['cam_x'], v['cam_y'], v['cam_z'], v['cam_yaw']), camera_t=v['cam_t'],
                     vo_state=int(v['vo_state']), vo_t=v['vo_t'], ekf=(v['ekf_x'], v['ekf_y'], v['ekf_z']),
                     ekf_xy_valid=bool(v['ekf_xy_valid']), dist_bottom=v['dist_bottom'], ekf_t=v['ekf_t'],
                     px4_yaw_enu=v['px4_yaw_enu'], att_t=v['att_t'], ev_pos=bool(v['ev_pos']),
                     ev_yaw=bool(v['ev_yaw']), rng_hgt=bool(v['rng_hgt']), flags_t=v['flags_t'])
        out.append((v['t'], flying, v['offboard'] == 1, f.update(v['t'], r, flying)))
    return out


def test_the_clean_run_is_trusted_from_settle_to_the_end_of_offboard():
    out = replay('20261108-082135')
    assert any(b.ready for t, flying, on, b in out if not flying)        # pre-flight would have streamed
    during = [b for t, flying, on, b in out if on]
    assert len(during) > 1000                                             # 55 s at 20 Hz
    bad = [b.words() for b in during if not b.trusted]
    assert not bad, bad[:3]                                               # no false alarm, tunnel included


def test_the_frozen_camera_is_caught_within_3_s_of_take_off():
    out = replay('20261108-113426')
    t_on = next(t for t, flying, on, b in out if on)
    caught = next(t for t, flying, on, b in out if flying and not b.trusted)
    assert caught - t_on < 3.0, caught - t_on
    later = [b for t, flying, on, b in out if flying and t > caught]
    assert all(not b.trusted for b in later)                              # and it stays caught


def test_the_wall_strike_jump_is_refused_at_once():
    out = replay('20261108-081345')
    clean = [b for t, flying, on, b in out if on and t < 23.4]
    assert clean and all(b.trusted for b in clean)
    after = [b for t, flying, on, b in out if t >= 23.75]                  # caught on the next sample
    assert after and all(not b.trusted for b in after)                    # a 37 m jump is never flown,
                                                                          # nor trusted again once EKF2
                                                                          # follows it (latched)
