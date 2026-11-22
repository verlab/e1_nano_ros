"""e1_bridge: ROS 2 driver for E1 Nano drones (real or simulated) over Zenoh.

One node serves the whole swarm. Drones are discovered through Zenoh liveliness tokens
(``e1/<id>/alive``) and get a namespace ``/<prefix><id>`` with:

Published
    state        e1_msgs/FlightState          50 Hz telemetry
    imu          sensor_msgs/Imu              (frame <ns>/base_link)
    battery      sensor_msgs/BatteryState
    pose         geometry_msgs/PoseStamped    (from e1/<id>/pose: simulator or mocap relay)
    odom         nav_msgs/Odometry
    joint_states sensor_msgs/JointState       rotor angles for robot_state_publisher
    TF           <world> -> <ns>/base_link
Subscribed
    cmd/attitude e1_msgs/AttitudeTarget
    cmd/manual   e1_msgs/ManualControl
    cmd/motors   e1_msgs/MotorCommand
Services
    arm (std_srvs/SetBool), set_mode (e1_msgs/SetMode), set_param (e1_msgs/SetParam), console (e1_msgs/Console)
Global
    /e1/emergency_stop (std_srvs/Trigger)  -> e1/all/cmd/arm (disarm everybody)
    /e1/swarm_status   (e1_msgs/SwarmStatus, 1 Hz)
"""
from __future__ import annotations

import math
import queue
import time

import rclpy
from e1_core import protocol as P
from e1_core.client import connect
from e1_msgs.msg import AttitudeTarget, FlightState, ManualControl, MotorCommand, SwarmStatus
from e1_msgs.srv import Console, SetMode, SetParam
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import BatteryState, Imu, JointState
from std_srvs.srv import SetBool, Trigger
from tf2_ros import TransformBroadcaster

ROTOR_JOINTS = ['rotor_rear_left_joint', 'rotor_rear_right_joint', 'rotor_front_right_joint', 'rotor_front_left_joint']
ROTOR_SIGN = [1, -1, 1, -1]      # joint axis already encodes the spin; sign kept for clarity
MAX_RAD_S = 3980.0 / 20.0        # visual rotor speed (slowed down 20x for RViz)

SENSOR_QOS = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=5)


class DroneBridge:
    def __init__(self, node: 'E1Bridge', did: int):
        self.n, self.id = node, did
        self.ns = f'/{node.prefix}{did}'
        self.frame = f'{node.prefix}{did}/base_link'
        self.seq = 0
        self.t0 = time.monotonic()
        self.last_state = None
        self.last_seen = 0.0
        self.rotor_angle = [0.0] * 4
        self.last_js = time.monotonic()
        mk = node.create_publisher
        self.p_state = mk(FlightState, f'{self.ns}/state', 10)
        self.p_imu = mk(Imu, f'{self.ns}/imu', SENSOR_QOS)
        self.p_batt = mk(BatteryState, f'{self.ns}/battery', 10)
        self.p_pose = mk(PoseStamped, f'{self.ns}/pose', SENSOR_QOS)
        self.p_odom = mk(Odometry, f'{self.ns}/odom', SENSOR_QOS)
        self.p_js = mk(JointState, f'{self.ns}/joint_states', 10)
        node.create_subscription(AttitudeTarget, f'{self.ns}/cmd/attitude', self.on_attitude, 10)
        node.create_subscription(ManualControl, f'{self.ns}/cmd/manual', self.on_manual, 10)
        node.create_subscription(MotorCommand, f'{self.ns}/cmd/motors', self.on_motors, 10)
        node.create_service(SetBool, f'{self.ns}/arm', self.srv_arm)
        node.create_service(SetMode, f'{self.ns}/set_mode', self.srv_mode)
        node.create_service(SetParam, f'{self.ns}/set_param', self.srv_param)
        node.create_service(Console, f'{self.ns}/console', self.srv_console)
        z = node.z
        self.zpub_att = z.declare_publisher(P.key(did, 'cmd', 'attitude'))
        self.zsubs = [z.declare_subscriber(P.key(did, 'state'), self.z_state),
                      z.declare_subscriber(P.key(did, 'pose'), self.z_pose),
                      z.declare_subscriber(P.key(did, 'log'), self.z_log)]
        self.log_q: 'queue.Queue[str]' = queue.Queue()
        if node.mocap_topic:
            node.create_subscription(PoseStamped, node.mocap_topic.format(id=did), self.on_mocap, SENSOR_QOS)
        node.get_logger().info(f'drone {did} -> {self.ns}')

    # ------------------------------------------------------------ helpers
    def _hdr(self, msg_header, frame):
        msg_header.stamp = self.n.get_clock().now().to_msg()
        msg_header.frame_id = frame
        return msg_header

    def _cmd_meta(self):
        self.seq = (self.seq + 1) & 0xFFFF
        return self.seq, int((time.monotonic() - self.t0) * 1000)

    def put(self, sub, payload: bytes):
        self.n.z.put(P.key(self.id, *sub.split('/')), payload)

    # ------------------------------------------------------------ zenoh -> ros
    def z_state(self, sample):
        try:
            s = P.State.decode(sample.payload.to_bytes())
        except ValueError:
            return
        self.last_state, self.last_seen = s, time.monotonic()
        m = FlightState()
        self._hdr(m.header, self.frame)
        m.drone_id, m.mode, m.armed = self.id, s.mode, s.armed
        m.landed = bool(s.flags & P.F_LANDED)
        m.low_battery = bool(s.flags & P.F_LOW_BATTERY)
        m.cmd_timeout = bool(s.flags & P.F_CMD_TIMEOUT)
        m.failsafe = bool(s.flags & P.F_FAILSAFE)
        m.attitude.w, m.attitude.x, m.attitude.y, m.attitude.z = map(float, s.quat)
        m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z = map(float, s.rates)
        m.linear_acceleration.x, m.linear_acceleration.y, m.linear_acceleration.z = map(float, s.acc)
        m.motors = [float(x) for x in s.motors]
        m.thrust, m.battery_voltage, m.seq = float(s.thrust), float(s.battery_v), s.seq
        self.p_state.publish(m)
        imu = Imu()
        self._hdr(imu.header, self.frame)
        imu.orientation = m.attitude
        imu.angular_velocity = m.angular_velocity
        imu.linear_acceleration = m.linear_acceleration
        imu.orientation_covariance[0] = imu.angular_velocity_covariance[0] = imu.linear_acceleration_covariance[0] = -1.0
        self.p_imu.publish(imu)
        b = BatteryState()
        self._hdr(b.header, self.frame)
        b.voltage = float(s.battery_v)
        b.percentage = float(min(max((s.battery_v - 3.3) / 0.9, 0.0), 1.0))
        b.present = s.battery_v > 1.0
        b.power_supply_technology = BatteryState.POWER_SUPPLY_TECHNOLOGY_LIPO
        self.p_batt.publish(b)
        if self.n.publish_js:
            now = time.monotonic()
            dt, self.last_js = now - self.last_js, now
            for k in range(4):
                self.rotor_angle[k] = (self.rotor_angle[k] + s.motors[k] * MAX_RAD_S * dt) % (2 * math.pi)
            js = JointState()
            js.header.stamp = self.n.get_clock().now().to_msg()
            js.name = ROTOR_JOINTS
            js.position = list(self.rotor_angle)
            self.p_js.publish(js)

    def z_pose(self, sample):
        try:
            p = P.Pose.decode(sample.payload.to_bytes())
        except ValueError:
            return
        ps = PoseStamped()
        self._hdr(ps.header, self.n.world)
        ps.pose.position.x, ps.pose.position.y, ps.pose.position.z = map(float, p.pos)
        q = ps.pose.orientation
        q.w, q.x, q.y, q.z = map(float, p.quat)
        self.p_pose.publish(ps)
        od = Odometry()
        od.header = ps.header
        od.child_frame_id = self.frame
        od.pose.pose = ps.pose
        if not any(math.isnan(v) for v in p.vel):
            od.twist.twist.linear.x, od.twist.twist.linear.y, od.twist.twist.linear.z = map(float, p.vel)
        self.p_odom.publish(od)
        if self.n.publish_tf:
            tf = TransformStamped()
            tf.header = ps.header
            tf.child_frame_id = self.frame
            tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z = map(float, p.pos)
            tf.transform.rotation = q
            self.n.tf.sendTransform(tf)

    def z_log(self, sample):
        txt = sample.payload.to_bytes().decode(errors='replace')
        self.log_q.put(txt)
        self.n.get_logger().info(f'[e1/{self.id}] {txt.rstrip()}')

    # ------------------------------------------------------------ ros -> zenoh
    def on_attitude(self, m: AttitudeTarget):
        seq, t = self._cmd_meta()
        q = m.orientation
        mask = 0 if m.use_body_rate else P.IGNORE_RATES
        cmd = P.CmdAttitude((q.w, q.x, q.y, q.z), (m.body_rate.x, m.body_rate.y, m.body_rate.z), float(m.thrust), mask, seq, t)
        self.zpub_att.put(cmd.encode())

    def on_manual(self, m: ManualControl):
        seq, t = self._cmd_meta()
        self.put('cmd/manual', P.CmdManual(m.roll, m.pitch, m.yaw, m.throttle, math.nan, seq, t).encode())

    def on_mocap(self, m: PoseStamped):
        seq, t = self._cmd_meta()
        p, q = m.pose.position, m.pose.orientation
        self.put('pose', P.Pose((p.x, p.y, p.z), (q.w, q.x, q.y, q.z), (math.nan,) * 3, seq, t).encode())

    def on_motors(self, m: MotorCommand):
        seq, t = self._cmd_meta()
        self.put('cmd/motors', P.CmdMotors(tuple(float(x) for x in m.duty), seq, t).encode())

    def srv_arm(self, req, res):
        seq, t = self._cmd_meta()
        self.put('cmd/arm', P.CmdArm(bool(req.data), seq, t).encode())
        res.success, res.message = True, ('arm' if req.data else 'disarm') + ' sent'
        return res

    def srv_mode(self, req, res):
        if req.mode not in P.MODE_NAMES:
            res.success, res.message = False, f'invalid mode {req.mode}'
            return res
        seq, t = self._cmd_meta()
        self.put('cmd/mode', P.CmdMode(req.mode, seq, t).encode())
        res.success, res.message = True, P.MODE_NAMES[req.mode]
        return res

    def srv_param(self, req, res):
        if not req.name or len(req.name) > 16:
            res.success, res.message = False, 'parameter name must have 1..16 chars'
            return res
        seq, t = self._cmd_meta()
        self.put('cmd/param', P.CmdParam(req.name, float(req.value), seq, t).encode())
        res.success, res.message = True, f'{req.name} = {req.value:g} sent'
        return res

    def srv_console(self, req, res):
        while not self.log_q.empty():
            self.log_q.get_nowait()
        self.put('cmd/console', req.command.encode())
        out, end = [], time.monotonic() + 1.0
        while time.monotonic() < end:
            try:
                out.append(self.log_q.get(timeout=0.1))
            except queue.Empty:
                if out:
                    break
        res.success, res.message = bool(out), ''.join(out) or '(no reply)'
        return res


class E1Bridge(Node):
    def __init__(self):
        super().__init__('e1_bridge')
        self.declare_parameter('zenoh_connect', 'tcp/127.0.0.1:7447')
        self.declare_parameter('ids', [0])
        self.declare_parameter('namespace_prefix', 'e1_')
        self.declare_parameter('world_frame', 'world')
        self.declare_parameter('publish_tf', True)
        self.declare_parameter('publish_joint_states', True)
        self.declare_parameter('mocap_topic', '')
        g = lambda n: self.get_parameter(n).value
        self.prefix, self.world = g('namespace_prefix'), g('world_frame')
        self.publish_tf, self.publish_js, self.mocap_topic = g('publish_tf'), g('publish_joint_states'), g('mocap_topic')
        self.tf = TransformBroadcaster(self)
        self.z = connect(g('zenoh_connect'))
        self.drones: dict[int, DroneBridge] = {}
        self._new: 'queue.Queue[int]' = queue.Queue()
        ids = [i for i in g('ids') if i > 0]
        for i in ids:
            self._new.put(i)
        self.liv = None
        if not ids:
            self.liv = self.z.liveliness().declare_subscriber(f'{P.PREFIX}/*/alive', self._on_alive, history=True)
        self.create_timer(0.1, self._spawn)
        self.p_status = self.create_publisher(SwarmStatus, '/e1/swarm_status', 10)
        self.create_timer(1.0, self._status)
        self.create_service(Trigger, '/e1/emergency_stop', self._estop)
        self.get_logger().info(f"Zenoh connected to {g('zenoh_connect')}; "
                               f"{'auto-discovery' if not ids else 'ids ' + str(ids)}")

    def _on_alive(self, sample):
        import zenoh
        did = P.parse_id(str(sample.key_expr))
        if isinstance(did, int) and sample.kind == zenoh.SampleKind.PUT:
            self._new.put(did)

    def _spawn(self):
        while not self._new.empty():
            did = self._new.get_nowait()
            if did not in self.drones:
                self.drones[did] = DroneBridge(self, did)

    def _status(self):
        m = SwarmStatus()
        m.header.stamp = self.get_clock().now().to_msg()
        now = time.monotonic()
        for did, d in sorted(self.drones.items()):
            s = d.last_state
            m.ids.append(did)
            m.online.append(now - d.last_seen < 1.0)
            m.armed.append(bool(s and s.armed))
            m.battery_voltage.append(float(s.battery_v) if s else 0.0)
        self.p_status.publish(m)

    def _estop(self, req, res):
        self.z.put(P.key(P.BROADCAST, 'cmd', 'arm'), P.CmdArm(False).encode())
        res.success, res.message = True, 'disarm broadcast to e1/all'
        return res


def main(args=None):
    rclpy.init(args=args)
    node = E1Bridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.z.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
