"""Pure-Python (no ROS) Zenoh client for one E1 Nano or a whole swarm.

    from e1_core.client import connect, Swarm
    s = connect('tcp/127.0.0.1:7447')
    swarm = Swarm(s)                 # discovers e1/*/alive
    swarm.wait_for(3)
    swarm.takeoff(0.4); swarm.goto_formation(...); swarm.land()

Works identically against the simulator (``e1-sim``) and real drones.
"""
from __future__ import annotations

import json
import threading
import time

import numpy as np
import zenoh

from . import protocol as P
from .control import PositionController, VehicleModel, Waypoints, YawAlign, min_jerk
from .math3d import yaw_of


def make_config(endpoint='tcp/127.0.0.1:7447', mode='client', listen=None):
    conf = zenoh.Config()
    conf.insert_json5('mode', json.dumps(mode))
    if endpoint:
        conf.insert_json5('connect/endpoints', json.dumps([endpoint] if isinstance(endpoint, str) else list(endpoint)))
    if listen:
        conf.insert_json5('listen/endpoints', json.dumps([listen] if isinstance(listen, str) else list(listen)))
    conf.insert_json5('scouting/multicast/enabled', 'false')
    return conf


def connect(endpoint='tcp/127.0.0.1:7447', mode='client', listen=None):
    return zenoh.open(make_config(endpoint, mode, listen))


class Drone:
    """Handle to one drone: telemetry cache + command helpers + optional position loop."""

    def __init__(self, session, drone_id, model: VehicleModel | None = None, rate_hz=50.0):
        self.s, self.id = session, int(drone_id)
        self.state: P.State | None = None
        self.pose: P.Pose | None = None
        self.t_state = self.t_pose = 0.0
        self.logs = []
        self.ctrl = PositionController(model)
        self.rate_hz = rate_hz
        self.setpoint = None          # callable t -> (pos, vel, acc) or None
        self.sp_yaw = 0.0
        self.yaw_align = YawAlign()   # mocap heading -> firmware heading (no magnetometer onboard)
        self._t0 = time.monotonic()
        self._seq = 0
        self._lock = threading.Lock()
        self._subs = [
            session.declare_subscriber(P.key(self.id, 'state'), self._on_state),
            session.declare_subscriber(P.key(self.id, 'pose'), self._on_pose),
            session.declare_subscriber(P.key(self.id, 'log'), lambda s: self.logs.append(s.payload.to_bytes().decode(errors='replace'))),
        ]
        self._pub_att = session.declare_publisher(P.key(self.id, 'cmd', 'attitude'))

    # ---------------------------------------------------------------- telemetry
    def _on_state(self, sample):
        try:
            self.state = P.State.decode(sample.payload.to_bytes())
            self.t_state = time.monotonic()
        except ValueError:
            pass

    def _on_pose(self, sample):
        try:
            self.pose = P.Pose.decode(sample.payload.to_bytes())
            self.t_pose = time.monotonic()
        except ValueError:
            pass

    @property
    def position(self):
        return None if self.pose is None else np.array(self.pose.pos)

    @property
    def online(self):
        return time.monotonic() - self.t_state < 1.0

    # ---------------------------------------------------------------- raw commands
    def _now_ms(self):
        return int((time.monotonic() - self._t0) * 1000)

    def _next(self):
        self._seq = (self._seq + 1) & 0xFFFF
        return self._seq

    def put(self, sub, payload):
        self.s.put(P.key(self.id, *sub.split('/')), payload)

    def arm(self, on=True):
        self.put('cmd/arm', P.CmdArm(on, self._next(), self._now_ms()).encode())

    def disarm(self):
        self.setpoint = None
        self.arm(False)

    def set_mode(self, mode):
        self.put('cmd/mode', P.CmdMode(mode, self._next(), self._now_ms()).encode())

    def set_param(self, name, value):
        self.put('cmd/param', P.CmdParam(name, float(value), self._next(), self._now_ms()).encode())

    def console(self, text):
        self.put('cmd/console', text.encode())

    def attitude(self, quat, thrust, rates=(0, 0, 0), use_rates=False):
        m = P.CmdAttitude(tuple(map(float, quat)), tuple(map(float, rates)), float(thrust),
                          0 if use_rates else P.IGNORE_RATES, self._next(), self._now_ms())
        self._pub_att.put(m.encode())

    def manual(self, roll=0.0, pitch=0.0, yaw=0.0, throttle=0.0):
        self.put('cmd/manual', P.CmdManual(roll, pitch, yaw, throttle, float('nan'), self._next(), self._now_ms()).encode())

    def motors(self, duty):
        self.put('cmd/motors', P.CmdMotors(tuple(map(float, duty)), self._next(), self._now_ms()).encode())

    # ---------------------------------------------------------------- position loop (one tick)
    def tick(self, t, dt):
        """Compute and send one attitude command toward the current setpoint."""
        if self.setpoint is None or self.pose is None:
            return
        pos, vel, acc = self.setpoint(t)
        bv = self.state.battery_v if self.state else 4.2
        yaw = yaw_of(self.pose.quat)
        q, duty = self.ctrl.update(t, self.pose.pos, self.pose.vel, yaw, pos, vel, acc, self.sp_yaw, bv, dt)
        if self.yaw_align is not None and self.state is not None:
            self.yaw_align.update(t, yaw_of(self.state.quat), yaw)
            q = self.yaw_align.apply(q)
        self.attitude(q, duty)

    def close(self):
        for s in self._subs:
            s.undeclare()
        self._pub_att.undeclare()


class Swarm:
    """Set of drones flown from one control thread (one Zenoh session for all)."""

    def __init__(self, session, ids=None, discover=True, rate_hz=50.0, model: VehicleModel | None = None):
        self.s = session
        self.rate_hz = rate_hz
        self.model = model
        self.drones: dict[int, Drone] = {}
        self._lock = threading.Lock()
        for i in ids or []:
            self.add(i)
        self._liv = None
        if discover:
            self._liv = session.liveliness().declare_subscriber(f'{P.PREFIX}/*/alive', self._on_alive, history=True)
        self._stop = threading.Event()
        self.t0 = time.monotonic()
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def _on_alive(self, sample):
        did = P.parse_id(str(sample.key_expr))
        if isinstance(did, int) and sample.kind == zenoh.SampleKind.PUT:
            self.add(did)

    def add(self, did):
        with self._lock:
            if did not in self.drones:
                self.drones[did] = Drone(self.s, did, self.model, self.rate_hz)
        return self.drones[did]

    def __getitem__(self, did):
        return self.drones[did]

    @property
    def ids(self):
        return sorted(self.drones)

    def now(self):
        return time.monotonic() - self.t0

    def wait_for(self, n, timeout=10.0):
        """Wait until n drones are discovered and publishing pose + state."""
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            ready = [d for d in self.drones.values() if d.online and d.pose is not None]
            if len(ready) >= n:
                return sorted(d.id for d in ready)
            time.sleep(0.05)
        raise TimeoutError(f'only {len(self.drones)} drones discovered')

    def _loop(self):
        dt = 1.0 / self.rate_hz
        nxt = time.monotonic()
        while not self._stop.is_set():
            t = self.now()
            with self._lock:
                ds = list(self.drones.values())
            for d in ds:
                d.tick(t, dt)
            nxt += dt
            time.sleep(max(0.0, nxt - time.monotonic()))

    # ---------------------------------------------------------------- high-level helpers
    def fly(self, targets: dict, duration, yaw=0.0, wait=True):
        """Move each drone id -> xyz target with a min-jerk segment of `duration` seconds."""
        t_start = self.now()
        for did, p1 in targets.items():
            d = self.drones[did]
            cur = d.setpoint(t_start)[0] if d.setpoint else np.array(d.pose.pos)
            p0 = np.array(cur, float)
            d.sp_yaw = yaw
            d.setpoint = (lambda p0, p1: (lambda t: min_jerk(p0, p1, duration, t - t_start)))(p0, np.asarray(p1, float))
        if wait:
            self.sleep(duration)

    def follow(self, did, waypoints: Waypoints, wait=True):
        t_start = self.now()
        self.drones[did].setpoint = lambda t: waypoints(t - t_start)
        if wait:
            self.sleep(waypoints.t_end)

    def takeoff(self, z=0.4, duration=2.5, ids=None):
        ids = ids or self.ids
        self.fly({i: np.array(self.drones[i].pose.pos) * [1, 1, 0] + [0, 0, z] for i in ids}, duration)

    def land(self, duration=2.5, ids=None):
        ids = ids or self.ids
        targets = {}
        for i in ids:
            d = self.drones[i]
            cur = d.setpoint(self.now())[0] if d.setpoint else np.array(d.pose.pos)
            targets[i] = np.asarray(cur) * [1, 1, 0] + [0, 0, -0.02]
        self.fly(targets, duration)
        for i in ids:
            self.drones[i].setpoint = None
            self.drones[i].disarm()

    def emergency_stop(self):
        for d in self.drones.values():
            d.setpoint = None
        self.s.put(P.key(P.BROADCAST, 'cmd', 'arm'), P.CmdArm(False).encode())

    def sleep(self, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end and not self._stop.is_set():
            time.sleep(0.02)

    def close(self):
        self._stop.set()
        self._thread.join(timeout=1)
        for d in self.drones.values():
            d.close()
        if self._liv is not None:
            self._liv.undeclare()
