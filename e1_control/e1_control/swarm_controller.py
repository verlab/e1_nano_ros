"""e1_swarm_controller: position control for every E1 Nano of the swarm in one node.

Per drone (namespace /<prefix><id>):
  sub  pose (PoseStamped, from e1_bridge), odom (Odometry), state (FlightState)
  sub  setpoint (e1_msgs/PositionTarget)   streaming setpoints (overrides trajectories)
  srv  takeoff (e1_msgs/Takeoff), land (e1_msgs/Land), goto (e1_msgs/GoTo)
  pub  cmd/attitude (e1_msgs/AttitudeTarget) at `rate` Hz while flying
Global:
  srv  /e1/takeoff_all, /e1/land_all
  pub  /e1/setpoints (visualization_msgs/MarkerArray)
"""
from __future__ import annotations

import math

import numpy as np
import rclpy
from e1_core.control import Gains, PositionController, VehicleModel, min_jerk, separation_acc
from e1_core.math3d import yaw_of
from e1_msgs.msg import AttitudeTarget, FlightState, PositionTarget, SwarmStatus
from e1_msgs.srv import GoTo, Land, Takeoff
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from visualization_msgs.msg import Marker, MarkerArray

SENSOR_QOS = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=5)
IDLE, FLYING, LANDING = 'idle', 'flying', 'landing'


class Vehicle:
    def __init__(self, node: 'SwarmController', did: int):
        self.n, self.id = node, did
        ns = f'/{node.prefix}{did}'
        self.ctrl = PositionController(node.model, node.gains)
        self.pos = self.vel = None
        self.quat = (1.0, 0.0, 0.0, 0.0)
        self.battery = 4.2
        self.mode = IDLE
        self.traj = None              # callable(t) -> (p, v, a)
        self.yaw_sp = 0.0
        self.t_land_end = None
        node.create_subscription(PoseStamped, f'{ns}/pose', self.on_pose, SENSOR_QOS)
        node.create_subscription(Odometry, f'{ns}/odom', self.on_odom, SENSOR_QOS)
        node.create_subscription(FlightState, f'{ns}/state', self.on_state, 10)
        node.create_subscription(PositionTarget, f'{ns}/setpoint', self.on_setpoint, 10)
        self.pub = node.create_publisher(AttitudeTarget, f'{ns}/cmd/attitude', 10)
        node.create_service(Takeoff, f'{ns}/takeoff', lambda rq, rs: self.takeoff(rq.height, rq.duration, rs))
        node.create_service(Land, f'{ns}/land', lambda rq, rs: self.land(rq.duration, rs))
        node.create_service(GoTo, f'{ns}/goto', self.srv_goto)
        node.get_logger().info(f'controlling {ns}')

    # ------------------------------------------------------------ inputs
    def on_pose(self, m):
        p = m.pose.position
        q = m.pose.orientation
        self.pos = np.array([p.x, p.y, p.z])
        self.quat = (q.w, q.x, q.y, q.z)

    def on_odom(self, m):
        v = m.twist.twist.linear
        self.vel = np.array([v.x, v.y, v.z])

    def on_state(self, m):
        self.battery = m.battery_voltage or 4.2

    def on_setpoint(self, m: PositionTarget):
        p, v, a = m.position, m.velocity, m.acceleration
        sp = (np.array([p.x, p.y, p.z]), np.array([v.x, v.y, v.z]), np.array([a.x, a.y, a.z]))
        self.traj = lambda t, sp=sp: sp
        self.yaw_sp = m.yaw
        if self.mode == IDLE:
            self.ctrl.reset()
        self.mode = FLYING

    # ------------------------------------------------------------ high level
    def current_sp(self, t):
        if self.traj is not None:
            return np.array(self.traj(t)[0])
        return None if self.pos is None else self.pos.copy()

    def _segment(self, target, duration):
        t0 = self.n.now()
        p0 = self.current_sp(t0)
        p1 = np.asarray(target, float)
        self.traj = lambda t: min_jerk(p0, p1, duration, t - t0)

    def takeoff(self, height, duration, res=None):
        if self.pos is None:
            if res:
                res.success, res.message = False, 'no pose yet'
            return res
        self.ctrl.reset()
        self.traj = None
        self._segment(self.pos + [0, 0, height or 0.4], duration or 2.5)
        self.yaw_sp = yaw_of(self.quat)
        self.mode = FLYING
        if res:
            res.success, res.message = True, f'taking off to {height:.2f} m'
        return res

    def land(self, duration, res=None):
        if self.mode == IDLE or self.pos is None:
            if res:
                res.success, res.message = False, 'not flying'
            return res
        sp = self.current_sp(self.n.now())
        self._segment([sp[0], sp[1], -0.02], duration or 2.5)
        self.mode = LANDING
        self.t_land_end = self.n.now() + (duration or 2.5)
        if res:
            res.success, res.message = True, 'landing'
        return res

    def srv_goto(self, req, res):
        if self.mode != FLYING:
            res.success, res.message = False, 'take off first'
            return res
        tgt = np.array([req.position.x, req.position.y, req.position.z])
        if req.relative:
            tgt += self.current_sp(self.n.now())
        self._segment(tgt, req.duration or 2.0)
        self.yaw_sp = req.yaw
        res.success, res.message = True, f'goto {np.round(tgt, 2).tolist()}'
        return res

    # ------------------------------------------------------------ control tick
    def tick(self, t, dt, others):
        if self.mode == IDLE or self.pos is None or self.traj is None:
            return None
        p, v, a = self.traj(t)
        if self.mode == LANDING and (t > self.t_land_end and self.pos[2] < 0.04):
            self.send((1.0, 0.0, 0.0, 0.0), 0.0)    # thrust 0 = disarm
            self.mode, self.traj = IDLE, None
            return None
        extra = separation_acc(self.id, others, self.n.min_sep) if self.n.min_sep > 0 and self.mode == FLYING else None
        q, duty = self.ctrl.update(t, self.pos, self.vel, yaw_of(self.quat), p, v, a, self.yaw_sp, self.battery, dt, extra)
        self.send(q, duty)
        return p

    def send(self, q, thrust):
        m = AttitudeTarget()
        m.header.stamp = self.n.get_clock().now().to_msg()
        m.header.frame_id = self.n.world
        m.orientation.w, m.orientation.x, m.orientation.y, m.orientation.z = map(float, q)
        m.thrust = float(thrust)
        self.pub.publish(m)


class SwarmController(Node):
    def __init__(self):
        super().__init__('e1_swarm_controller')
        dp = self.declare_parameter
        dp('ids', [0]); dp('namespace_prefix', 'e1_'); dp('rate', 50.0); dp('world_frame', 'world')
        dp('mass', 0.0251); dp('max_thrust', 0.216); dp('kp', [4.0, 4.0, 5.0]); dp('kd', [3.0, 3.0, 3.5])
        dp('ki', [0.4, 0.4, 1.2]); dp('tilt_max_deg', 25.0); dp('min_separation', 0.25)
        g = lambda k: self.get_parameter(k).value
        self.prefix, self.world, self.rate, self.min_sep = g('namespace_prefix'), g('world_frame'), g('rate'), g('min_separation')
        self.model = VehicleModel(g('mass'), g('max_thrust'))
        self.gains = Gains(np.array(g('kp')), np.array(g('kd')), np.array(g('ki')), tilt_max=math.radians(g('tilt_max_deg')))
        self.v: dict[int, Vehicle] = {}
        ids = [i for i in g('ids') if i > 0]
        for i in ids:
            self.v[i] = Vehicle(self, i)
        if not ids:
            self.create_subscription(SwarmStatus, '/e1/swarm_status', self.on_status, 10)
        self.create_service(Takeoff, '/e1/takeoff_all', self.takeoff_all)
        self.create_service(Land, '/e1/land_all', self.land_all)
        self.p_markers = self.create_publisher(MarkerArray, '/e1/setpoints', 10)
        self.t0 = self.get_clock().now()
        self.create_timer(1.0 / self.rate, self.loop)

    def now(self):
        return (self.get_clock().now() - self.t0).nanoseconds * 1e-9

    def on_status(self, m: SwarmStatus):
        for did, online in zip(m.ids, m.online):
            if online and did not in self.v:
                self.v[did] = Vehicle(self, did)

    def takeoff_all(self, req, res):
        n = sum(1 for v in self.v.values() if v.takeoff(req.height, req.duration) is None and v.mode == FLYING)
        res.success, res.message = n > 0, f'{n} drones taking off'
        return res

    def land_all(self, req, res):
        for v in self.v.values():
            v.land(req.duration)
        res.success, res.message = True, 'landing all'
        return res

    def loop(self):
        t, dt = self.now(), 1.0 / self.rate
        others = {i: v.pos for i, v in self.v.items() if v.pos is not None and v.mode != IDLE}
        ma = MarkerArray()
        for did, v in self.v.items():
            sp = v.tick(t, dt, others)
            if sp is not None:
                mk = Marker()
                mk.header.frame_id, mk.header.stamp = self.world, self.get_clock().now().to_msg()
                mk.ns, mk.id, mk.type, mk.action = 'setpoints', did, Marker.SPHERE, Marker.ADD
                mk.pose.position.x, mk.pose.position.y, mk.pose.position.z = map(float, sp)
                mk.pose.orientation.w = 1.0
                mk.scale.x = mk.scale.y = mk.scale.z = 0.03
                mk.color.r, mk.color.g, mk.color.b, mk.color.a = 1.0, 0.45, 0.1, 0.9
                ma.markers.append(mk)
        if ma.markers:
            self.p_markers.publish(ma)


def main(args=None):
    rclpy.init(args=args)
    node = SwarmController()
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
