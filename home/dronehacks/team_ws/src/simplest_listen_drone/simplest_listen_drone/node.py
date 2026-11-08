"""Listen to every topic in subscriptions.py (and anything else on the graph) and report it.
Publishes nothing to the drone, so it cannot fly.

Every 1 s: one plain line from the camera and PX4 (senses.py).
Every 5 s: a table of every topic heard: rate, age of the last message, and its latest fields.
Both also go out on /simplest_listen_drone/status.

All subscriptions are BEST_EFFORT / VOLATILE, depth 1: that matches every publisher (PX4's
best-effort /fmu/out writers included; a default RELIABLE subscriber hears nothing from them).
Images and point clouds are taken raw (bytes, counted, never decoded) to keep Python off the CPU
cuVSLAM needs.
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rosidl_runtime_py import message_to_ordereddict
from rosidl_runtime_py.utilities import get_message
from std_msgs.msg import String

from simplest_listen_drone.hearing import Ear, extras, resolve, short, table
from simplest_listen_drone.senses import camera_from_odometry, describe, px4_from_state
from simplest_listen_drone.subscriptions import CATCH_ALL, NEVER, TOPICS

QOS = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.VOLATILE)


class SimplestListenDrone(Node):
    def __init__(self):
        super().__init__('simplest_listen_drone')
        self.ears = {}                     # live topic name -> Ear
        self.subs = []
        self.camera = self.camera_t = self.px4 = self.px4_t = None
        self.status_pub = self.create_publisher(String, '/simplest_listen_drone/status', 10)
        self.create_timer(1.0, self.say)
        self.create_timer(5.0, self.report)
        self.create_timer(2.0, self.discover)     # topics appear as the stack comes up
        self.get_logger().info('listening to {} listed topics{} (publishes nothing to the drone)'.format(
            len(TOPICS), ' and anything else on the graph' if CATCH_ALL else ''))

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def discover(self):
        graph = dict(self.get_topic_names_and_types())
        wanted = {}
        for listed, note in TOPICS:
            live = resolve(listed, graph)
            if live:
                wanted[live] = note
        if CATCH_ALL:
            for topic in extras(graph, wanted, NEVER):
                wanted[topic] = '(not on our list)'
        for topic, note in wanted.items():
            if topic not in self.ears and graph[topic]:
                self.listen(topic, graph[topic][0], note)

    def listen(self, topic, type_name, note):
        ear = Ear(topic, type_name, note)
        try:
            msg_type = get_message(type_name)
        except (AttributeError, ModuleNotFoundError, ValueError) as exc:
            self.get_logger().warn('cannot load {} for {}: {}'.format(type_name, topic, exc))
            return
        self.ears[topic] = ear
        raw = ear.heavy
        self.subs.append(self.create_subscription(
            msg_type, topic, lambda msg, e=ear, r=raw: self.on_message(e, msg, r), QOS, raw=raw))
        self.get_logger().info('now hearing {} [{}]'.format(topic, type_name))

    def on_message(self, ear, msg, raw):
        t = self.now()
        if raw:
            ear.heard(t, size=len(msg))
            return
        if ear.topic == '/visual_slam/tracking/odometry':
            p, q = msg.pose.pose.position, msg.pose.pose.orientation
            self.camera, self.camera_t = camera_from_odometry(p.x, p.y, p.z, q.x, q.y, q.z, q.w), t
        elif ear.topic == '/skybeta/state':
            s = px4_from_state(msg.data)
            if s is not None:
                self.px4, self.px4_t = s, t
        # previews cost a dict conversion: only for slow topics or every 10th fast message
        preview = short(message_to_ordereddict(msg)) if ear.count % 10 == 0 else ''
        ear.heard(t, preview=preview)

    def say(self):
        line = describe(self.now(), self.camera, self.camera_t, self.px4, self.px4_t)
        self.get_logger().info(line)
        self.status_pub.publish(String(data=line))

    def report(self):
        graph = dict(self.get_topic_names_and_types())
        missing = [listed for listed, _ in TOPICS if resolve(listed, graph) is None]
        block = table(self.now(), self.ears.values(), missing)
        self.get_logger().info('\n' + block)
        self.status_pub.publish(String(data=block))


def main(args=None):
    rclpy.init(args=args)
    node = SimplestListenDrone()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
