"""Sensor fusion in front of the flight-tested step engine.

    the relevant subset  (simplest_listen_drone.subscriptions.RELEVANT)
          -> fusion.Fusion   one Belief: where am I, can I trust it        (fusion.py)
          -> the rule        a pose (camera, or EKF2 while coasting) / none (tick, below)
          -> engine.Mission  simplest_square_drone's steps, unchanged      (steps.py, engine.py)
          -> /skybeta/setpoint/{pose,twist} -> the kit's flight interface -> PX4

Never touches /fmu/in/*, never arms or changes mode (the kit's contract). Status on
/simplest_fusion_drone/status: the engine's state plus the belief in words.
"""
import json
import math

import rclpy
from geometry_msgs.msg import PoseStamped, TwistStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.serialization import deserialize_message
from rcl_interfaces.msg import ParameterDescriptor
from rosidl_runtime_py.utilities import get_message
from sensor_msgs.msg import Image
from std_msgs.msg import String

from simplest_fusion_drone.fusion import (Config, Fusion, Readings, ned_to_enu, px4_yaw_to_enu,
                                          yaw_from_quaternion)
from simplest_fusion_drone.image import from_ros_image, seeing
from simplest_listen_drone.hearing import resolve
from simplest_listen_drone.subscriptions import RELEVANT
from simplest_square_drone.engine import Config as EngineConfig, Mission
from simplest_square_drone.steps import STEPS

QOS = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.VOLATILE)
IMAGE_EVERY = 6                     # decode every 6th frame: 5 Hz of 30, enough to notice blank or dark


class SimplestFusionDrone(Node):
    def __init__(self, name='simplest_fusion_drone', make_steps=None):
        """make_steps(node) -> a step list, for nodes that fly something other than the square
        (simplest_course_drone builds its steps from the course map). Default: the square."""
        super().__init__(name)
        cfg = {}
        for name, value in vars(Config()).items():
            self.declare_parameter(name, value, ParameterDescriptor(dynamic_typing=True))
            v = self.get_parameter(name).value
            cfg[name] = (str(v).lower() in ('true', '1')) if isinstance(value, bool) else float(v)
        self.fusion = Fusion(Config(**cfg))
        self.steps = make_steps(self) if make_steps else STEPS
        self.mission = Mission(self.steps, EngineConfig())
        self.r = Readings()
        self.offboard = False
        self.landed = self.armed = self.px4 = None
        self.frames = 0
        self.subs = {}
        self.last_status = None
        self.belief = None
        self.handlers = {
            '/visual_slam/tracking/odometry': self.on_odometry,
            '/visual_slam/status': self.on_vo_status,
            '/skybeta/state': self.on_state,
            '/fmu/out/vehicle_local_position': self.on_local_position,
            '/fmu/out/vehicle_attitude': self.on_attitude,
            '/fmu/out/estimator_status_flags': self.on_flags,
            '/oak/left/image_mono': self.on_image,
            # '/oak/left/camera_info' is in RELEVANT for the step that turns pixels into directions
        }
        assert set(self.handlers) <= set(RELEVANT), 'fusion may only read the relevant subset'
        self.pose_pub = self.create_publisher(PoseStamped, '/skybeta/setpoint/pose', 10)
        self.twist_pub = self.create_publisher(TwistStamped, '/skybeta/setpoint/twist', 10)
        self.status_pub = self.create_publisher(String, '/{}/status'.format(name), 10)
        self.create_timer(1.0, self.discover)
        self.create_timer(0.05, self.tick)                 # 20 Hz, as the square
        self.get_logger().info('{}: {} steps behind a trust gate. Arm, then Position.'.format(name, len(self.steps)))

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    # ---- subscriptions: names from RELEVANT, types from the live graph ----
    def discover(self):
        graph = dict(self.get_topic_names_and_types())
        for listed, handler in self.handlers.items():
            live = resolve(listed, graph)
            if live is None or listed in self.subs or not graph[live]:
                continue
            raw = listed == '/oak/left/image_mono'
            self.subs[listed] = self.create_subscription(get_message(graph[live][0]), live, handler, QOS, raw=raw)
            self.get_logger().info('fusion hearing {} [{}]'.format(live, graph[live][0]))

    def on_odometry(self, msg):
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        self.r.camera = (p.x, p.y, p.z, yaw_from_quaternion(q.w, q.x, q.y, q.z))
        self.r.camera_t = self.now()

    def on_vo_status(self, msg):
        self.r.vo_state, self.r.vo_t = int(msg.vo_state), self.now()

    def on_state(self, msg):
        try:
            s = json.loads(msg.data)
        except ValueError:
            return
        self.offboard = bool(s.get('offboard_active', False))
        self.landed, self.armed = s.get('landed'), s.get('armed')
        local = s.get('local_enu')
        self.px4 = tuple(local) if isinstance(local, list) and len(local) == 3 else None

    def on_local_position(self, msg):
        self.r.ekf = ned_to_enu(msg.x, msg.y, msg.z)
        self.r.ekf_xy_valid = bool(msg.xy_valid)
        self.r.dist_bottom = float(msg.dist_bottom) if msg.dist_bottom_valid else None
        self.r.ekf_t = self.now()

    def on_attitude(self, msg):
        w, x, y, z = msg.q
        self.r.px4_yaw_enu, self.r.att_t = px4_yaw_to_enu(yaw_from_quaternion(w, x, y, z)), self.now()

    def on_flags(self, msg):
        self.r.ev_pos, self.r.ev_yaw, self.r.rng_hgt = bool(msg.cs_ev_pos), bool(msg.cs_ev_yaw), bool(msg.cs_rng_hgt)
        self.r.flags_t = self.now()

    def on_image(self, data):
        self.frames += 1
        if self.frames % IMAGE_EVERY:
            return
        m = deserialize_message(data, Image)
        pixels = from_ros_image(m.height, m.width, m.step, m.encoding, m.data)
        if pixels is not None:
            self.r.brightness, self.r.texture = seeing(pixels)
            self.r.image_t = self.now()

    # ---- the rule ----
    def tick(self):
        now = self.now()
        flying = self.mission.state != 'WAIT'
        b = self.belief = self.fusion.update(now, self.r, flying)
        pose_t = self.r.camera_t if b.pose is not None else None
        cmd = self.mission.step(now, b.pose, pose_t, self.offboard, self.landed, self.armed, b.yaw, self.px4)
        self.publish(cmd)
        status = '{}: {} || {}'.format(self.mission.state, self.mission.reason, b.words())
        if status != self.last_status:
            self.get_logger().info(status)
            self.last_status = status
        self.status_pub.publish(String(data=status))

    def publish(self, cmd):
        stamp = self.get_clock().now().to_msg()
        if cmd.kind == 'pose':
            m = PoseStamped()
            m.header.stamp, m.header.frame_id = stamp, 'odom'
            m.pose.position.x, m.pose.position.y, m.pose.position.z = cmd.x, cmd.y, cmd.z
            if cmd.yaw is None:
                m.pose.orientation.w = 0.0          # all-zero quaternion: do not command yaw
            else:
                m.pose.orientation.z, m.pose.orientation.w = math.sin(cmd.yaw / 2.0), math.cos(cmd.yaw / 2.0)
            self.pose_pub.publish(m)
        elif cmd.kind == 'twist':
            m = TwistStamped()
            m.header.stamp, m.header.frame_id = stamp, 'odom'
            m.twist.linear.x, m.twist.linear.y, m.twist.linear.z = cmd.x, cmd.y, cmd.z
            self.twist_pub.publish(m)


def main(args=None):
    rclpy.init(args=args)
    node = SimplestFusionDrone()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
