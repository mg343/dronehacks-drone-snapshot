"""The kit's flight interface plus simplest_map_drone, which only listens.

For COMPETITOR_LAUNCH=simplest_map_drone.launch.py with COMPETITOR_PKG=simplest_map_drone.
auto_offboard_on_arm is false: if the pilot arms, the drone stays in Position under the pilot.
Another map: map:=/root/team_ws/course_map.yaml
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    args = [DeclareLaunchArgument('map', default_value=''), DeclareLaunchArgument('rest_height_m', default_value='0.07')]
    interface = Node(
        package='skybeta_flight_interface', executable='flight_interface', name='flight_interface',
        output='screen', emulate_tty=True,
        parameters=[{'auto_offboard_on_arm': False, 'allow_mode_request': False,
                     'odometry_topic': '/visual_slam/tracking/odometry'}])
    listener = Node(
        package='simplest_map_drone', executable='simplest_map_drone', name='simplest_map_drone',
        output='screen', emulate_tty=True,
        parameters=[{'map': LaunchConfiguration('map'), 'rest_height_m': LaunchConfiguration('rest_height_m')}])
    return LaunchDescription(args + [interface, listener])
