"""MuJoCo software-in-the-loop simulator for one or many E1 Nano drones.

Each simulated drone runs the Python port of the CF-Drone firmware (``e1_core.firmware``),
a first-order motor model and a battery model, and talks the real Zenoh protocol:
it publishes ``e1/<id>/state`` + ``e1/<id>/pose`` (ground truth, like mocap) and obeys
``e1/<id>/cmd/*`` and ``e1/all/cmd/*``. Ground software (ROS 2 bridge, controllers, examples)
is therefore identical in simulation and on hardware.
"""
from __future__ import annotations

import collections
import copy
import math
import os
import threading
import time
import xml.etree.ElementTree as ET

import mujoco
import numpy as np

from .. import protocol as P
from ..firmware import Firmware
from ..math3d import qmul, yaw_quat
from .description import find_mjcf

MAX_THRUST = 0.216     # N per motor at duty 1, 4.2 V
MOTOR_TAU = 0.02       # s, coreless motor + prop spin-up time constant


def build_swarm_xml(positions, base_xml=None, floor=True):
    """Replicate the single-drone MJCF for every (id, xyz, yaw) in `positions`."""
    base_xml = base_xml or find_mjcf()
    tree = ET.parse(base_xml)
    root = tree.getroot()
    comp = root.find('compiler')
    comp.set('meshdir', os.path.join(os.path.dirname(os.path.abspath(base_xml)), comp.get('meshdir')))
    wb = root.find('worldbody')
    tmpl = wb.find("body[@name='base_link']")
    wb.remove(tmpl)
    act_root, sen_root = root.find('actuator'), root.find('sensor')
    acts, sens = list(act_root), list(sen_root)
    for a in acts:
        act_root.remove(a)
    for s in sens:
        sen_root.remove(s)
    for did, xyz, yaw in positions:
        pre = f'd{did}_'
        b = copy.deepcopy(tmpl)
        for el in b.iter():
            if el.get('name'):
                el.set('name', pre + el.get('name'))
        b.set('pos', f'{xyz[0]} {xyz[1]} {xyz[2]}')
        b.set('euler', f'0 0 {yaw}')
        wb.append(b)
        for a in acts:
            a2 = copy.deepcopy(a)
            a2.set('name', pre + a.get('name'))
            a2.set('site', pre + a.get('site'))
            act_root.append(a2)
        for s in sens:
            s2 = copy.deepcopy(s)
            s2.set('name', pre + s.get('name'))
            for k in ('site', 'objname'):
                if s.get(k):
                    s2.set(k, pre + s.get(k))
            sen_root.append(s2)
    if not floor:
        f = wb.find("geom[@name='floor']")
        if f is not None:
            wb.remove(f)
    return ET.tostring(root, encoding='unicode')


def build_cargo_xml(positions, base_xml=None, box_mass=0.1, tether_length=0.3, floor=True):
    """Build a four-drone scene with a free box and unilateral, length-limited tethers."""
    if len(positions) != 4:
        raise ValueError('the cargo scene requires exactly four drones')
    if not math.isfinite(box_mass) or box_mass <= 0:
        raise ValueError('box_mass must be a positive finite value')
    if not math.isfinite(tether_length) or tether_length <= 0:
        raise ValueError('tether_length must be a positive finite value')

    root = ET.fromstring(build_swarm_xml(positions, base_xml=base_xml, floor=floor))
    world = root.find('worldbody')
    tendons = ET.Element('tendon')
    root.insert(list(root).index(world) + 1, tendons)
    payload_sites = []
    quadrants = set()
    half_box = 0.06
    for did, xyz, _ in positions:
        sx = 1 if xyz[0] >= 0 else -1
        sy = 1 if xyz[1] >= 0 else -1
        if (sx, sy) in quadrants:
            raise ValueError('cargo scene drones must occupy the four grid quadrants')
        quadrants.add((sx, sy))

        prefix = f'd{did}_'
        drone = world.find(f"body[@name='{prefix}base_link']")
        ET.SubElement(drone, 'site', name=prefix + 'tether_attach', pos='0 0 0.004', size='0.003',
                      rgba='0.95 0.65 0.15 1')
        payload_sites.append((did, sx, sy))
    if len(quadrants) != 4:
        raise ValueError('cargo scene drones must occupy the four grid quadrants')

    box = ET.SubElement(world, 'body', name='payload', pos='0 0 0.041')
    ET.SubElement(box, 'freejoint', name='payload_root')
    ET.SubElement(box, 'inertial', pos='0 0 0', mass=f'{box_mass:g}',
                  diaginertia=f'{box_mass * (0.12**2 + 0.08**2) / 12:g} '
                              f'{box_mass * (0.12**2 + 0.08**2) / 12:g} '
                              f'{box_mass * (0.12**2 + 0.12**2) / 12:g}')
    ET.SubElement(box, 'geom', name='payload_box', type='box', size='0.06 0.06 0.04',
                  rgba='0.75 0.42 0.18 1', mass='0')
    for did, sx, sy in payload_sites:
        site_name = f'payload_attach_{did}'
        ET.SubElement(box, 'site', name=site_name, pos=f'{sx * half_box:g} {sy * half_box:g} 0.035',
                      size='0.003', rgba='0.95 0.65 0.15 1')
        tendon = ET.SubElement(tendons, 'spatial', name=f'tether_{did}', width='0.002',
                               rgba='0.95 0.65 0.15 1', limited='true',
                               range=f'0 {tether_length:g}')
        ET.SubElement(tendon, 'site', site=f'd{did}_tether_attach')
        ET.SubElement(tendon, 'site', site=site_name)
    return ET.tostring(root, encoding='unicode')


def layout(n, kind='grid', spacing=0.5, z=None):
    """Default start positions on the floor."""
    out = []
    cols = int(math.ceil(math.sqrt(n)))
    rows = int(math.ceil(n / cols))
    for k in range(n):
        if kind == 'line':
            x, y = 0.0, (k - (n - 1) / 2) * spacing
        elif kind == 'circle':
            r = max(spacing * n / (2 * math.pi), spacing) if n > 1 else 0.0
            x, y = r * math.cos(2 * math.pi * k / n), r * math.sin(2 * math.pi * k / n)
        else:
            r, c = divmod(k, cols)
            x, y = (c - (cols - 1) / 2) * spacing, (r - (rows - 1) / 2) * spacing
        out.append((x, y))
    return out


class SimDrone:
    def __init__(self, sim, did, prefix):
        m = sim.model
        self.id, self.pre = did, prefix
        self.fw = Firmware(did)
        self.body = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, prefix + 'base_link')
        jid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, prefix + 'root')
        self.qadr, self.vadr = m.jnt_qposadr[jid], m.jnt_dofadr[jid]
        self.act = [mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_ACTUATOR, prefix + n)
                    for n in ('rear_left', 'rear_right', 'front_right', 'front_left')]
        self.s_gyro = m.sensor_adr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_SENSOR, prefix + 'gyro')]
        self.s_acc = m.sensor_adr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_SENSOR, prefix + 'accel')]
        self.spin = np.zeros(4)             # normalized rotor speed (0..1)
        self.used_mah = 0.0
        self.capacity_mah = 250.0
        self.battery_v = 4.15
        self.cmds = collections.deque(maxlen=64)
        self.seq = 0
        self.yaw_offset = 0.0           # rad, onboard heading error at t=0 (no magnetometer)
        self.yaw_drift = 0.0            # rad/s, onboard heading drift (gyro z bias)

    def onboard_attitude(self, q, t):
        """Attitude as the firmware estimates it: true attitude seen from a yaw-shifted frame."""
        if self.yaw_offset == 0.0 and self.yaw_drift == 0.0:
            return q
        return qmul(yaw_quat(self.yaw_offset + self.yaw_drift * t), q)

    def battery(self, duty, dt):
        current = 1.6 * float(np.sum(duty ** 2))            # A (≈1.8 A at hover)
        self.used_mah += current * dt / 3.6
        soc = max(0.0, 1.0 - self.used_mah / self.capacity_mah)
        self.battery_v = 3.3 + 0.85 * soc - 0.05 * current  # crude 1S LiPo curve with sag
        return self.battery_v


class SwarmSim:
    """N E1 Nano drones in one MuJoCo world."""

    def __init__(self, n=1, ids=None, kind='grid', spacing=0.5, starts=None, fw_rate=500.0, seed=0,
                 gyro_noise=0.01, acc_noise=0.05, yaw_offset=0.0, yaw_drift=0.0,
                 cargo_mass=None, tether_length=0.3):
        ids = list(ids or range(1, n + 1))
        starts = starts or layout(len(ids), kind, spacing)
        self.z0 = 0.0198
        positions = [(i, (x, y, self.z0), 0.0) for i, (x, y) in zip(ids, starts)]
        if cargo_mass is None:
            self.xml = build_swarm_xml(positions)
        else:
            self.xml = build_cargo_xml(positions, box_mass=cargo_mass, tether_length=tether_length)
        self.model = mujoco.MjModel.from_xml_string(self.xml)
        self.data = mujoco.MjData(self.model)
        self.dt = self.model.opt.timestep
        self.fw_every = max(1, int(round(1.0 / (fw_rate * self.dt))))
        self.drones = {i: SimDrone(self, i, f'd{i}_') for i in ids}
        self.rng = np.random.default_rng(seed)
        self.gyro_noise, self.acc_noise = gyro_noise, acc_noise
        self.k = 0
        for d in self.drones.values():
            d.yaw_offset, d.yaw_drift = float(yaw_offset), float(yaw_drift)
        mujoco.mj_forward(self.model, self.data)

    @property
    def t(self):
        return self.data.time

    def pose(self, did):
        d = self.drones[did]
        q = self.data.qpos[d.qadr:d.qadr + 7]
        v = self.data.qvel[d.vadr:d.vadr + 3]
        return P.Pose(tuple(q[:3]), tuple(q[3:7]), tuple(v), d.seq, int(self.t * 1000))

    def push(self, did, kind, msg):
        """Queue a decoded command for drone `did` ('all' = every drone)."""
        targets = self.drones.values() if did == P.BROADCAST else [self.drones[did]] if did in self.drones else []
        for d in targets:
            d.cmds.append((kind, msg))

    def step(self):
        m, dat = self.model, self.data
        fw_tick = (self.k % self.fw_every) == 0
        for d in self.drones.values():
            if fw_tick:
                while d.cmds:
                    kind, msg = d.cmds.popleft()
                    d.fw.handle(kind, msg)
                q = dat.qpos[d.qadr + 3:d.qadr + 7].copy()
                gyro = dat.sensordata[d.s_gyro:d.s_gyro + 3] + self.rng.normal(0, self.gyro_noise, 3)
                acc = dat.sensordata[d.s_acc:d.s_acc + 3] + self.rng.normal(0, self.acc_noise, 3)
                bv = d.battery(d.fw.motors, self.dt * self.fw_every)
                d.fw.step(self.t, d.onboard_attitude(q, self.t), gyro, acc, bv)
            duty = np.asarray(d.fw.motors, float)
            d.spin += (duty * (d.battery_v / 4.2) - d.spin) * (self.dt / MOTOR_TAU)
            thrust = MAX_THRUST * np.clip(d.spin, 0, 1.2) ** 2
            for a, f in zip(d.act, thrust):
                dat.ctrl[a] = f
        mujoco.mj_step(m, dat)
        self.k += 1


class ZenohSimServer:
    """Runs a SwarmSim in real time and bridges it to Zenoh."""

    def __init__(self, sim: SwarmSim, session, state_hz=50.0, pose_hz=100.0, speed=1.0, publish_pose=True):
        self.sim, self.s = sim, session
        self.publish_pose = publish_pose  # False: poses come from a mocap relay (e.g. ``e1-sim --natnet``)
        self.state_every = max(1, int(round(1 / (state_hz * sim.dt))))
        self.pose_every = max(1, int(round(1 / (pose_hz * sim.dt))))
        self.speed = speed
        self.pub_state, self.pub_pose, self.tokens, self.subs = {}, {}, [], []
        for did in sim.drones:
            self.pub_state[did] = session.declare_publisher(P.key(did, 'state'))
            self.pub_pose[did] = session.declare_publisher(P.key(did, 'pose'))
            self.tokens.append(session.liveliness().declare_token(P.key(did, 'alive')))
            self.subs.append(session.declare_subscriber(P.key(did, 'cmd', '**'), self._on_cmd))
        self.subs.append(session.declare_subscriber(P.key(P.BROADCAST, 'cmd', '**'), self._on_cmd))
        self._stop = threading.Event()
        self.lock = threading.Lock()

    def _on_cmd(self, sample):
        k = str(sample.key_expr)
        did = P.parse_id(k)
        kind = '/'.join(k.split('/')[2:])
        raw = sample.payload.to_bytes()
        try:
            if kind == 'cmd/console':
                self._console(did, raw.decode(errors='replace').strip())
                return
            msg = P.DECODERS[kind](raw)
        except (KeyError, ValueError):
            return
        with self.lock:
            self.sim.push(did, kind, msg)

    def _console(self, did, line):
        parts = line.split()
        if not parts:
            return
        if parts[0] == 'p' and len(parts) == 3:
            self.sim.push(did, 'cmd/param', P.CmdParam(parts[1], float(parts[2])))
        elif parts[0] in ('arm', 'disarm'):
            self.sim.push(did, 'cmd/arm', P.CmdArm(parts[0] == 'arm'))
        reply = f'[sim e1/{did}] {line}\n'
        for d in (self.sim.drones if did == P.BROADCAST else [did]):
            self.s.put(P.key(d, 'log'), reply.encode())

    def publish(self):
        sim = self.sim
        if self.publish_pose and sim.k % self.pose_every == 0:
            for did in sim.drones:
                pose = sim.pose(did)
                self.pub_pose[did].put(pose.encode())
        if sim.k % self.state_every == 0:
            for did, d in sim.drones.items():
                d.seq += 1
                st = d.fw.state(d.seq)
                self.pub_state[did].put(st.encode())
                while d.fw.log:
                    self.s.put(P.key(did, 'log'), (d.fw.log.pop(0) + '\n').encode())

    def run(self, duration=math.inf, on_step=None, render_every=None):
        t_wall0, t_sim0 = time.monotonic(), self.sim.t
        while not self._stop.is_set() and self.sim.t - t_sim0 < duration:
            with self.lock:
                self.sim.step()
            self.publish()
            if on_step:
                on_step(self.sim)
            if self.speed > 0:
                lag = (self.sim.t - t_sim0) / self.speed - (time.monotonic() - t_wall0)
                if lag > 0.002:
                    time.sleep(lag)

    def stop(self):
        self._stop.set()

    def close(self):
        for p in list(self.pub_state.values()) + list(self.pub_pose.values()) + self.subs + self.tokens:
            p.undeclare()
