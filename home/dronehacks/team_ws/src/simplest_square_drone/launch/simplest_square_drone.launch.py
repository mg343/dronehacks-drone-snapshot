"""The kit's flight interface (unchanged settings) plus simplest_square_drone.

For COMPETITOR_LAUNCH=simplest_square_drone.launch.py with COMPETITOR_PKG=simplest_square_drone.
The flight itself (where to go, how long to hover) is in simplest_square_drone/steps.py, not here.
``record:=false`` turns off the node's recording (it records from arming; see node.py).
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
MISSION = [('speed_mps', '0.3'), ('yaw_rate_dps', '20.0'), ('record', 'true')]


def generate_launch_description():
    args = [DeclareLaunchArgument(n, default_value=d) for n, d in INTERFACE + MISSION]
    interface = Node(
        package='skybeta_flight_interface', executable='flight_interface', name='flight_interface',
        output='screen', emulate_tty=True,
        parameters=[{n: LaunchConfiguration(n) for n, _ in INTERFACE}])
    mission = Node(
        package='simplest_square_drone', executable='simplest_square_drone', name='simplest_square_drone',
        output='screen', emulate_tty=True,
        parameters=[{n: LaunchConfiguration(n) for n, _ in MISSION}])
    return LaunchDescription(args + [interface, mission])
