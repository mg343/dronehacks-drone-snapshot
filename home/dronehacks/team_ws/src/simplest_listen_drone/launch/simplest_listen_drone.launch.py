"""The kit's flight interface plus simplest_listen_drone, which only listens.

For COMPETITOR_LAUNCH=simplest_listen_drone.launch.py with COMPETITOR_PKG=simplest_listen_drone.
The flight interface is here only because it publishes /skybeta/state. auto_offboard_on_arm is
false, so if the pilot arms, the drone stays in Position under the pilot: nothing of ours flies.
"""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    interface = Node(
        package='skybeta_flight_interface', executable='flight_interface', name='flight_interface',
        output='screen', emulate_tty=True,
        parameters=[{'auto_offboard_on_arm': False, 'allow_mode_request': False,
                     'odometry_topic': '/visual_slam/tracking/odometry'}])
    listener = Node(
        package='simplest_listen_drone', executable='simplest_listen_drone', name='simplest_listen_drone',
        output='screen', emulate_tty=True)
    return LaunchDescription([interface, listener])
