"""The kit's flight interface (unchanged settings) plus simplest_course_drone.

For COMPETITOR_LAUNCH=simplest_course_drone.launch.py with COMPETITOR_PKG=simplest_course_drone.
Fly one piece at a time:  until:=cp2_gate_a, then until:=tunnel, until:=gate_b, then the whole course.
Another map:              map:=/root/team_ws/course_map.yaml
Bench test (shorter settle): settle_s:=5.0
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

INTERFACE = [  # the kit example's own defaults
    ('max_altitude_m', '3.0'), ('min_altitude_m', '0.3'), ('max_speed_mps', '1.5'),
    ('setpoint_rate_hz', '10.0'), ('allow_mode_request', 'false'), ('auto_offboard_on_arm', 'true'),
    ('odometry_topic', '/visual_slam/tracking/odometry'),
]
COURSE = [('map', ''), ('until', ''), ('rest_height_m', '0.10'), ('allow_windows', 'false'),
          ('settle_s', '30.0'), ('heading_tol_deg', '2.0'), ('drift_tol_m', '0.2'), ('height_tol_m', '0.25'), ('flags_timeout_s', '2.5'), ('coast_s', '6.0')]


def generate_launch_description():
    args = [DeclareLaunchArgument(n, default_value=d) for n, d in INTERFACE + COURSE]
    interface = Node(
        package='skybeta_flight_interface', executable='flight_interface', name='flight_interface',
        output='screen', emulate_tty=True,
        parameters=[{n: LaunchConfiguration(n) for n, _ in INTERFACE}])
    course = Node(
        package='simplest_course_drone', executable='simplest_course_drone', name='simplest_course_drone',
        output='screen', emulate_tty=True,
        parameters=[{n: LaunchConfiguration(n) for n, _ in COURSE}])
    return LaunchDescription(args + [interface, course])
