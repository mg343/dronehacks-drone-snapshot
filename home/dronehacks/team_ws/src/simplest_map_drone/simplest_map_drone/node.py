"""Where are we on the course? Listens only: publishes nothing to the drone, so it cannot fly.

Every 1 s, one line: the drone's position in course (east, north) metres, the next checkpoint and
its distance, the nearest wall, and the checkpoints passed so far. Also on /simplest_map_drone/status.

The map is a parameter (`map`); the default is maps/course_arena_2026-11-08_flown.yaml in this package
(the course as taped, corrected by what the team's flights hit).
Setup rule (course.py): power the drone on ON the launch mark, FACING the launch heading.
"""
import json
import os

import rclpy
from ament_index_python.packages import get_package_share_directory
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import String

from dronehacks_planner.course_map import load_map
from simplest_listen_drone.senses import camera_from_odometry
from simplest_map_drone.course import CourseFrame, Progress, nearest_wall


def default_map():
    return os.path.join(get_package_share_directory('simplest_map_drone'), 'maps', 'course_arena_2026-11-08_flown.yaml')


class SimplestMapDrone(Node):
    def __init__(self):
        super().__init__('simplest_map_drone')
        self.declare_parameter('map', '')                  # empty: the packaged default
        self.declare_parameter('rest_height_m', 0.10)
        path = self.get_parameter('map').value or default_map()
        self.cmap = load_map(path)
        self.frame = CourseFrame.from_map(self.cmap, float(self.get_parameter('rest_height_m').value))
        self.progress = Progress(self.cmap)
        self.camera = None
        self.landed = self.armed = None
        self.create_subscription(Odometry, '/visual_slam/tracking/odometry', self.on_odometry, 10)
        self.create_subscription(String, '/skybeta/state', self.on_state, 10)
        self.status_pub = self.create_publisher(String, '/simplest_map_drone/status', 10)
        self.create_timer(1.0, self.say)
        self.get_logger().info('map {} ({} walls, {} boulders, goals {})'.format(
            path, len(self.cmap.segments), len(self.cmap.boulders),
            ' > '.join(self.progress.label(s) for s in self.progress.stages)))

    def on_odometry(self, msg):
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        self.camera = camera_from_odometry(p.x, p.y, p.z, q.x, q.y, q.z, q.w)
        e, n = self.frame.to_course(self.camera['x'], self.camera['y'])
        passed = self.progress.update(e, n, self.landed, self.armed)
        if passed:
            self.get_logger().info('PASSED {}'.format(passed))

    def on_state(self, msg):
        try:
            s = json.loads(msg.data)
        except ValueError:
            return
        self.landed, self.armed = s.get('landed'), s.get('armed')

    def say(self):
        if self.camera is None:
            line = 'no camera pose yet'
        else:
            e, n = self.frame.to_course(self.camera['x'], self.camera['y'])
            d, what = nearest_wall(self.cmap, e, n)
            line = 'course ({:+.2f} E, {:+.2f} N) | {} | nearest: {} {:.2f} m'.format(
                e, n, self.progress.words(e, n), what, d)
        self.get_logger().info(line)
        self.status_pub.publish(String(data=line))


def main(args=None):
    rclpy.init(args=args)
    node = SimplestMapDrone()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
