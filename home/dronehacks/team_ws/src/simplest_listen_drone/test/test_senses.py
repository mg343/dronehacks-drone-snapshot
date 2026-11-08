"""senses.py: messages in, plain words out."""
import json
import math
import pathlib
import re

from simplest_listen_drone.senses import camera_from_odometry, describe, px4_from_state

STATE = {'px4_link': True, 'armed': False, 'landed': True, 'nav_state': 2, 'nav_state_name': 'POSCTL',
         'offboard_active': False, 'local_enu': [0.01, 0.0, 0.0], 'clamped': [], 'frozen': False,
         'envelope_ok': True}


def cam(x=0.0, y=0.0, z=0.0, yaw_deg=0.0):
    h = math.radians(yaw_deg) / 2
    return camera_from_odometry(x, y, z, 0.0, 0.0, math.sin(h), math.cos(h))


def test_heading_from_quaternion():
    assert round(cam(yaw_deg=90)['heading_deg']) == 90
    assert round(cam(yaw_deg=-45)['heading_deg']) == -45


def test_on_the_pad_everything_fresh():
    line = describe(10.0, cam(0.0, 0.0, 0.0), 9.9, STATE, 9.9)
    assert 'camera: +0.00 fwd +0.00 left +0.00 up, facing +0 deg' in line
    assert 'px4: disarmed, POSCTL, offboard off, on the ground' in line
    assert 'agree: px4 and camera 0.01 m apart' in line and 'over 0.3' not in line


def test_nothing_received_yet():
    assert describe(0.0, None, None, None, None) == 'camera: nothing yet | px4: nothing yet | agree: unknown'


def test_stale_inputs_are_called_out():
    line = describe(10.0, cam(), 7.5, STATE, 4.0)
    assert 'camera: no pose for 2.5 s' in line and 'px4: no state for 6.0 s' in line and 'agree: unknown' in line


def test_no_link_to_the_flight_controller():
    assert 'px4: NO LINK' in describe(1.0, None, None, dict(STATE, px4_link=False), 1.0)


def test_flying_offboard_with_a_clamp():
    s = dict(STATE, armed=True, landed=False, nav_state_name='OFFBOARD', offboard_active=True,
             clamped=['max_speed'], local_enu=[1.0, 0.0, 0.73])
    line = describe(5.0, cam(1.0, 0.0, 0.73, 90), 5.0, s, 5.0)
    assert 'px4: ARMED, OFFBOARD, offboard ON, in the air, kit clamped: max_speed' in line
    assert 'facing +90 deg' in line


def test_disagreement_is_flagged():
    line = describe(1.0, cam(0.0, 0.0, 0.0), 1.0, dict(STATE, local_enu=[0.0, 0.0, 0.73]), 1.0)
    assert '0.73 m apart  <-- over 0.3 m' in line


def test_bad_json_is_ignored():
    assert px4_from_state('not json') is None
    assert px4_from_state('[1, 2]') is None
    assert px4_from_state(json.dumps(STATE))['armed'] is False


def test_the_node_publishes_nothing_to_the_drone():
    """Its only publisher is its own status topic: no /skybeta/setpoint/*, no /fmu/in/*."""
    src = (pathlib.Path(__file__).parents[1] / 'simplest_listen_drone' / 'node.py').read_text()
    assert re.findall(r"create_publisher\(\w+, '([^']+)'", src) == ['/simplest_listen_drone/status']
    assert '/skybeta/setpoint' not in src and '/fmu/in' not in src


def test_the_launch_never_lets_offboard_engage():
    src = (pathlib.Path(__file__).parents[1] / 'launch' / 'simplest_listen_drone.launch.py').read_text()
    assert "'auto_offboard_on_arm': False" in src and "'allow_mode_request': False" in src
