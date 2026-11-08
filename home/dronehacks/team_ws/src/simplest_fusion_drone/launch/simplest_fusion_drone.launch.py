"""The kit's flight interface (unchanged settings) plus simplest_fusion_drone.

For COMPETITOR_LAUNCH=simplest_fusion_drone.launch.py with COMPETITOR_PKG=simplest_fusion_drone.
The flight is simplest_square_drone/steps.py; the trust rules are simplest_fusion_drone/fusion.py.
Fusion settings can be overridden here, e.g. settle_s:=5.0 for a bench test, heading_tol_deg:=3.0.
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
FUSION = [('settle_s', '30.0'), ('heading_tol_deg', '2.0'), ('drift_tol_m', '0.2'), ('height_tol_m', '0.25'), ('flags_timeout_s', '2.5'),
          ('min_texture', '0.01'), ('min_brightness', '15.0'), ('require_image', 'false'),
          ('coast_s', '6.0')]


def generate_launch_description():
    args = [DeclareLaunchArgument(n, default_value=d) for n, d in INTERFACE + FUSION]
    interface = Node(
        package='skybeta_flight_interface', executable='flight_interface', name='flight_interface',
        output='screen', emulate_tty=True,
        parameters=[{n: LaunchConfiguration(n) for n, _ in INTERFACE}])
    fusion = Node(
        package='simplest_fusion_drone', executable='simplest_fusion_drone', name='simplest_fusion_drone',
        output='screen', emulate_tty=True,
        parameters=[{n: LaunchConfiguration(n) for n, _ in FUSION}])
    return LaunchDescription(args + [interface, fusion])
