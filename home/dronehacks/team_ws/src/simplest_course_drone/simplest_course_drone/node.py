"""Fly the course from the map: the planner writes the step list, fusion and the engine fly it.

    map --dronehacks_planner--> route --route_steps--> fly_to / hover / land lines
        --simplest_fusion_drone's node (trust gate + tunnel coast)--> simplest_square_drone's engine
        --> /skybeta/setpoint/* --> the kit --> PX4

Everything flying is the fusion node, unchanged; this file only supplies the steps and reports
checkpoint progress (simplest_map_drone.course.Progress) on /simplest_course_drone/progress.

Parameters:
    map            course map file; empty = simplest_map_drone's maps/course_arena_2026-11-08_flown.yaml
    until          fly only up to this goal, then land there (e.g. cp2_gate_a, tunnel, gate_b);
                   empty = the whole course to the pad
    rest_height_m  the odom origin's height above the floor on the launch mark (skybeta-11: 0.10)
    allow_windows  let the planner choose the window over Gate B (off: its clearance is unreviewed)
Setup rule (simplest_map_drone.course): power on ON the launch mark, FACING the launch heading.
"""
import dataclasses

import rclpy
from std_msgs.msg import String

from dronehacks_planner.course_map import load_map
from dronehacks_planner.route import PlannerConfig, plan
from simplest_course_drone.route_steps import course_steps, describe
from simplest_fusion_drone.node import SimplestFusionDrone
from simplest_map_drone.course import CourseFrame, Progress, nearest_wall
from simplest_map_drone.node import default_map


class SimplestCourseDrone(SimplestFusionDrone):
    def __init__(self):
        super().__init__('simplest_course_drone', make_steps=self.make_steps)
        self.progress_pub = self.create_publisher(String, '/simplest_course_drone/progress', 10)
        self.create_timer(1.0, self.report)

    def make_steps(self, node):
        node.declare_parameter('map', '')
        node.declare_parameter('until', '')
        node.declare_parameter('rest_height_m', 0.10)
        node.declare_parameter('allow_windows', False)
        path = node.get_parameter('map').value or default_map()
        until = node.get_parameter('until').value or None
        self.cmap = load_map(path)
        self.frame = CourseFrame.from_map(self.cmap, float(node.get_parameter('rest_height_m').value))
        config = dataclasses.replace(PlannerConfig(), allow_windows=bool(node.get_parameter('allow_windows').value))
        route = plan(self.cmap, config)
        steps = course_steps(route, self.frame, self.cmap, until)
        self.progress = Progress(self.cmap)
        node.get_logger().info('map {}: route {:.1f} m through {}{}\n{}'.format(
            path, route.length_m, ' > '.join(route.goals), ', until ' + until if until else '', describe(steps)))
        return steps

    def tick(self):
        super().tick()
        b = self.belief
        if b is not None and b.pose is not None:
            e, n = self.frame.to_course(b.pose[0], b.pose[1])
            passed = self.progress.update(e, n, self.landed, self.armed)
            if passed:
                self.get_logger().info('PASSED {}'.format(passed))

    def report(self):
        b = self.belief
        if b is None or b.pose is None:
            line = 'progress: no trusted pose | passed: {}'.format(', '.join(self.progress.passed) or 'none')
        else:
            e, n = self.frame.to_course(b.pose[0], b.pose[1])
            d, what = nearest_wall(self.cmap, e, n)
            line = 'course ({:+.2f} E, {:+.2f} N) | {} | nearest: {} {:.2f} m'.format(
                e, n, self.progress.words(e, n), what, d)
        self.progress_pub.publish(String(data=line))


def main(args=None):
    rclpy.init(args=args)
    node = SimplestCourseDrone()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
