"""Genesis backend for the E1 SITL (same interface as ``mujoco_sim.SwarmSim``).

Use it when you want Genesis' GPU-parallel physics / photo-realistic rendering, or to share
assets with RL pipelines (``gs.morphs.Drone``). ``ZenohSimServer`` accepts either backend:

    e1-sim -n 4 --backend genesis
"""
from __future__ import annotations

import collections

import numpy as np

from .. import protocol as P
from ..firmware import Firmware
from ..math3d import qrot_inv
from .description import find_urdf
from .mujoco_sim import MOTOR_TAU, layout

KF = 1.496e-10          # N / rpm^2   (0.216 N at 38 000 rpm)
MAX_RPM = 38000.0
PROPS = ('rotor_rear_left', 'rotor_rear_right', 'rotor_front_right', 'rotor_front_left')
SPIN = (-1, 1, -1, 1)   # Genesis applies +spin*km*rpm^2 on the body = reaction torque sign


class _GDrone:
    def __init__(self, did, entity):
        self.id, self.e = did, entity
        self.fw = Firmware(did)
        self.spin = np.zeros(4)
        self.cmds = collections.deque(maxlen=64)
        self.seq = 0
        self.battery_v = 4.15
        self.used_mah = 0.0
        self.prev_v = np.zeros(3)


class GenesisSwarm:
    def __init__(self, n=1, ids=None, kind='grid', spacing=0.5, starts=None, dt=0.002, backend='gpu', show_viewer=False):
        import genesis as gs
        if not getattr(gs, '_initialized', False):
            gs.init(backend=gs.gpu if backend == 'gpu' else gs.cpu, logging_level='warning')
        ids = list(ids or range(1, n + 1))
        starts = starts or layout(len(ids), kind, spacing)
        self.gs = gs
        self.dt = dt
        self.scene = gs.Scene(sim_options=gs.options.SimOptions(dt=dt), show_viewer=show_viewer)
        self.scene.add_entity(gs.morphs.Plane())
        self.drones = {}
        for did, (x, y) in zip(ids, starts):
            e = self.scene.add_entity(gs.morphs.Drone(file=find_urdf('genesis'), pos=(x, y, 0.02),
                                                      propellers_link_name=PROPS, propellers_spin=SPIN))
            self.drones[did] = _GDrone(did, e)
        self.scene.build()
        self.k = 0
        self.t = 0.0

    @staticmethod
    def _np(x):
        return x.detach().cpu().numpy().ravel() if hasattr(x, 'detach') else np.asarray(x).ravel()

    def pose(self, did):
        d = self.drones[did]
        return P.Pose(tuple(self._np(d.e.get_pos())), tuple(self._np(d.e.get_quat())), tuple(self._np(d.e.get_vel())),
                      d.seq, int(self.t * 1000))

    def push(self, did, kind, msg):
        targets = self.drones.values() if did == P.BROADCAST else [self.drones[did]] if did in self.drones else []
        for d in targets:
            d.cmds.append((kind, msg))

    def step(self):
        for d in self.drones.values():
            while d.cmds:
                d.fw.handle(*d.cmds.popleft())
            q = self._np(d.e.get_quat())
            w_world = self._np(d.e.get_ang())
            v = self._np(d.e.get_vel())
            acc_world = (v - d.prev_v) / self.dt + np.array([0, 0, 9.81])
            d.prev_v = v
            gyro = qrot_inv(q, w_world)
            acc = qrot_inv(q, acc_world)
            current = 1.6 * float(np.sum(d.fw.motors ** 2))
            d.used_mah += current * self.dt / 3.6
            d.battery_v = 3.3 + 0.85 * max(0.0, 1 - d.used_mah / 250.0) - 0.05 * current
            duty = d.fw.step(self.t, q, gyro, acc, d.battery_v)
            d.spin += (np.asarray(duty) * d.battery_v / 4.2 - d.spin) * (self.dt / MOTOR_TAU)
            d.e.set_propellers_rpm(np.clip(d.spin, 0, 1.2) * MAX_RPM)
        self.scene.step()
        self.k += 1
        self.t += self.dt
