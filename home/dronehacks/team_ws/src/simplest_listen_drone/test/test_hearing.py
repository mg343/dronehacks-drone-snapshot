"""hearing.py and subscriptions.py: name matching, rates, the table, and the list itself."""
from simplest_listen_drone.hearing import Ear, extras, resolve, short, table
from simplest_listen_drone.subscriptions import NEVER, TOPICS


def test_resolve_exact_and_versioned():
    graph = {'/fmu/out/vehicle_status_v1': 0, '/fmu/out/vehicle_status_v3': 0, '/skybeta/state': 0}
    assert resolve('/skybeta/state', graph) == '/skybeta/state'
    assert resolve('/fmu/out/vehicle_status', graph) == '/fmu/out/vehicle_status_v3'
    assert resolve('/fmu/out/vehicle_attitude', graph) is None
    assert resolve('/fmu/out/vehicle', {'/fmu/out/vehicle_status_v1': 0}) is None   # no prefix matches


def test_catch_all_skips_encoded_images_and_our_own_topics():
    graph = ['/oak/left/image_raw', '/oak/left/image_raw/compressed', '/oak/left/image_raw/theora',
             '/visual_slam/vis/landmarks_cloud', '/simplest_listen_drone/status', '/parameter_events']
    assert extras(graph, {'/oak/left/image_raw'}, NEVER) == ['/visual_slam/vis/landmarks_cloud']


def test_rate_counts_the_last_five_seconds():
    e = Ear('/x', 'std_msgs/msg/String')
    for i in range(50):
        e.heard(i * 0.1)                   # 10 Hz for 5 s
    assert abs(e.rate(4.9) - 10.0) < 0.5
    assert e.rate(20.0) == 0.0


def test_images_are_heavy_and_heard_raw():
    assert Ear('/oak/left/image_raw', 'sensor_msgs/msg/Image').heavy
    assert not Ear('/skybeta/state', 'std_msgs/msg/String').heavy


def test_table_lists_heard_and_missing():
    e = Ear('/skybeta/state', 'std_msgs/msg/String')
    e.heard(9.0, preview='data={...}')
    out = table(10.0, [e], ['/fmu/out/battery_status'])
    assert '/skybeta/state' in out and '1.0s' in out and 'data={...}' in out
    assert out.splitlines()[-1] == 'not on the graph: /fmu/out/battery_status'


def test_short_keeps_lines_short():
    line = short({'a': 1.23456, 'b': [1, 2, 3, 4, 5, 6], 'c': {'d': 'x'}, 'e': 'y' * 200})
    assert line.startswith('a=1.235 b=[6 items] c=d=x') and len(line) <= 90


def test_every_topic_is_listed_once_with_a_note():
    names = [t for t, _ in TOPICS]
    assert len(names) == len(set(names)) and all(n.startswith('/') and note for n, note in TOPICS)
