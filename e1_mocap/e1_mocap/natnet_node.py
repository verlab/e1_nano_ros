"""e1_mocap/natnet: OptiTrack rigid bodies -> one geometry_msgs/PoseStamped per E1 Nano.

Every rigid body whose name matches ``name_pattern`` (default ``e1_<id>``) is published on
``/<prefix><id>/mocap/pose`` in a Z-up world frame; ``e1_bridge`` relays it to the drone
(``mocap_topic:=/e1_{id}/mocap/pose``). Diagnostics: ``/e1/mocap/status`` (std_msgs/String, 1 Hz).

    ros2 run e1_mocap natnet --ros-args -p server_ip:=192.168.1.10 -p local_ip:=192.168.1.20
"""
from __future__ import annotations

import json
import re
import threading
import time

import rclpy
from rcl_interfaces.msg import ParameterDescriptor
from e1_core.mocap.frames import PoseFilter, to_z_up
from e1_core.mocap.natnet import NatNetClient, Version
from e1_core.mocap.relay import drone_id
from geometry_msgs.msg import PoseStamped, TransformStamped
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from tf2_ros import TransformBroadcaster

SENSOR_QOS = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=5)


class NatNetNode(Node):
    def __init__(self):
        super().__init__('e1_mocap')
        defaults = dict(server_ip='127.0.0.1', local_ip='0.0.0.0', multicast=True, multicast_group='239.255.42.99',
                        command_port=1510, data_port=1511, natnet_version='', up_axis='z', name_pattern=r'e1_(\d+)',
                        by_id=False, namespace_prefix='e1_', topic='mocap/pose', world_frame='world', publish_tf=False,
                        rate=120.0)
        for k, v in defaults.items():
            # YAML 1.1 reads a bare `y` as true: let up_axis take any type and normalise it below
            desc = ParameterDescriptor(dynamic_typing=True) if k == 'up_axis' else ParameterDescriptor()
            self.declare_parameter(k, v, desc)
        g = lambda k: self.get_parameter(k).value
        up = g('up_axis')
        self.up = 'y' if up is True or str(up).lower() == 'y' else 'z'
        self.prefix, self.topic, self.world = g('namespace_prefix'), g('topic'), g('world_frame')
        self.pattern = None if g('by_id') else re.compile(g('name_pattern'))
        self.pubs, self.filters, self.t_sent, self.seen, self.latest = {}, {}, {}, {}, {}
        self.lock = threading.Lock()
        self.tf = TransformBroadcaster(self) if g('publish_tf') else None
        self.status = self.create_publisher(String, '/e1/mocap/status', 10)
        ver = g('natnet_version')
        version = Version(*map(int, ver.split('.')[:2])) if ver else None
        self.client = NatNetClient(g('server_ip'), g('local_ip'), g('multicast'), g('multicast_group'),
                                   g('command_port'), g('data_port'), version)
        self.client.on_frame = self.on_frame
        ok = self.client.start()
        v = self.client.version
        self.get_logger().info(f'NatNet {v.major}.{v.minor} from {g("server_ip")} ({self.client.server_name or "no reply"}), '
                               f'{"multicast" if g("multicast") else "unicast"}, rigid bodies {self.client.names}')
        if not ok:
            self.get_logger().warn('Motive did not answer NAT_CONNECT; check IPs, firewall and that streaming is on')
        self.frames0 = 0
        self.create_timer(1.0 / max(g('rate'), 1.0), self.publish)
        self.create_timer(1.0, self.report)
        self.create_timer(5.0, lambda: self.client.refresh_names(0.5))

    def on_frame(self, frame, bodies, t):
        """NatNet thread: keep only the newest valid pose per drone; publishing happens in the executor."""
        with self.lock:
            for b in bodies:
                did = drone_id(b, self.pattern)
                if did is None:
                    continue
                self.seen[did] = (t, b.valid)
                if not b.valid:
                    continue
                pos, quat = to_z_up(b.pos, b.quat, self.up)
                if self.filters.setdefault(did, PoseFilter()).update(t, pos, quat) is None:
                    continue
                self.latest[did] = (t, pos, quat, b.name or str(b.id))

    def publish(self):
        """Executor thread (timer at `rate` Hz): publish each pose that is new since the last tick."""
        with self.lock:
            fresh = {d: v for d, v in self.latest.items() if v[0] > self.t_sent.get(d, -1.0)}
            for d, v in fresh.items():
                self.t_sent[d] = v[0]
        stamp = self.get_clock().now().to_msg()
        for did, (_, pos, quat, name) in fresh.items():
            if did not in self.pubs:
                self.pubs[did] = self.create_publisher(PoseStamped, f'/{self.prefix}{did}/{self.topic}', SENSOR_QOS)
                self.get_logger().info(f'tracking {name} -> /{self.prefix}{did}/{self.topic}')
            m = PoseStamped()
            m.header.stamp, m.header.frame_id = stamp, self.world
            m.pose.position.x, m.pose.position.y, m.pose.position.z = map(float, pos)
            m.pose.orientation.w, m.pose.orientation.x, m.pose.orientation.y, m.pose.orientation.z = map(float, quat)
            self.pubs[did].publish(m)
            if self.tf:
                tr = TransformStamped()
                tr.header = m.header
                tr.child_frame_id = f'{self.prefix}{did}/mocap'
                tr.transform.translation.x, tr.transform.translation.y, tr.transform.translation.z = map(float, pos)
                tr.transform.rotation = m.pose.orientation
                self.tf.sendTransform(tr)

    def report(self):
        now, n = time.monotonic(), self.client.stats.frames
        with self.lock:
            drones = {str(d): {'age_s': round(now - t, 3), 'valid': v} for d, (t, v) in sorted(self.seen.items())}
        msg = {'frames_per_s': n - self.frames0, 'drones': drones, 'decode_errors': len(self.client.stats.errors)}
        self.frames0 = n
        self.status.publish(String(data=json.dumps(msg)))
        lost = [d for d, s in drones.items() if s['age_s'] > 0.2 or not s['valid']]
        if lost:
            self.get_logger().warn(f'not tracked: {lost}', throttle_duration_sec=2.0)

    def destroy_node(self):
        self.client.stop()
        super().destroy_node()


def main():
    rclpy.init()
    node = NatNetNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
