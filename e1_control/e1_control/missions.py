"""Example missions that drive e1_swarm_controller through its ROS 2 services.

    ros2 run e1_control square_mission --ros-args -p id:=1 -p side:=0.6 -p height:=0.4
    ros2 run e1_control formation_mission --ros-args -p n:=6 -p radius:=0.6
"""
from __future__ import annotations

import math
import time

import rclpy
from e1_msgs.msg import SwarmStatus
from e1_msgs.srv import GoTo, Land, Takeoff
from rclpy.node import Node


class Mission(Node):
    def __init__(self, name):
        super().__init__(name)
        self.declare_parameter('namespace_prefix', 'e1_')
        self.prefix = self.get_parameter('namespace_prefix').value
        self.status = None
        self.create_subscription(SwarmStatus, '/e1/swarm_status', lambda m: setattr(self, 'status', m), 10)
        self._svc_clients = {}

    def call(self, srv_type, name, **fields):
        cli = self._svc_clients.get(name) or self.create_client(srv_type, name)
        self._svc_clients[name] = cli
        if not cli.wait_for_service(timeout_sec=10.0):
            raise RuntimeError(f'service {name} not available (is e1_swarm_controller running?)')
        req = srv_type.Request()
        for k, v in fields.items():
            setattr(req, k, v)
        fut = cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=5.0)
        res = fut.result()
        self.get_logger().info(f'{name}: {res.message}')
        return res

    def goto(self, did, x, y, z, duration, yaw=0.0, relative=False):
        from geometry_msgs.msg import Point
        return self.call(GoTo, f'/{self.prefix}{did}/goto', position=Point(x=float(x), y=float(y), z=float(z)),
                         yaw=float(yaw), duration=float(duration), relative=relative)

    def wait(self, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.05)

    def wait_drones(self, n, timeout=20.0):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.status:
                ids = [i for i, on in zip(self.status.ids, self.status.online) if on]
                if len(ids) >= n:
                    return sorted(ids)[:n]
        raise RuntimeError('not enough drones online')


def square_main(args=None):
    rclpy.init(args=args)
    m = Mission('e1_square_mission')
    for k, v in (('id', 1), ('side', 0.6), ('height', 0.4), ('leg_time', 2.5)):
        m.declare_parameter(k, v)
    g = lambda k: m.get_parameter(k).value
    did, side, h, leg = g('id'), float(g('side')), float(g('height')), float(g('leg_time'))
    m.wait_drones(1)
    m.call(Takeoff, f'/{m.prefix}{did}/takeoff', height=h, duration=2.5)
    m.wait(3.0)
    for dx, dy in ((side, 0), (0, side), (-side, 0), (0, -side)):
        m.goto(did, dx, dy, 0.0, leg, relative=True)
        m.wait(leg + 0.5)
    m.call(Land, f'/{m.prefix}{did}/land', duration=2.5)
    m.wait(3.0)
    m.destroy_node()
    rclpy.shutdown()


def formation_main(args=None):
    rclpy.init(args=args)
    m = Mission('e1_formation_mission')
    for k, v in (('n', 4), ('radius', 0.6), ('height', 0.5), ('turns', 2)):
        m.declare_parameter(k, v)
    g = lambda k: m.get_parameter(k).value
    n, r, h = int(g('n')), float(g('radius')), float(g('height'))
    ids = m.wait_drones(n)
    m.get_logger().info(f'formation with drones {ids}')
    m.call(Takeoff, '/e1/takeoff_all', height=h, duration=3.0)
    m.wait(3.5)
    for k in range(int(g('turns')) * 4 + 1):
        ph = k * math.pi / 4
        for j, did in enumerate(ids):
            a = ph + 2 * math.pi * j / n
            m.goto(did, r * math.cos(a), r * math.sin(a), h + 0.1 * math.sin(2 * a), 2.5)
        m.wait(3.0)
    m.call(Land, '/e1/land_all', duration=3.0)
    m.wait(3.5)
    m.destroy_node()
    rclpy.shutdown()
