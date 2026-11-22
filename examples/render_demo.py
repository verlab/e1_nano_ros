#!/usr/bin/env python3
"""Offline (lock-step) swarm demo rendered to mp4 — no Zenoh, no real time; good for docs/CI.

    MUJOCO_GL=egl python3 examples/render_demo.py --n 6 --out docs/assets/swarm_demo.mp4
"""
import argparse
import math

import imageio.v2 as iio
import mujoco
import numpy as np

from e1_core import protocol as P
from e1_core.control import PositionController, circle_formation, grid_formation, min_jerk, separation_acc
from e1_core.math3d import yaw_of
from e1_core.sim.mujoco_sim import SwarmSim

ap = argparse.ArgumentParser()
ap.add_argument('--n', type=int, default=6)
ap.add_argument('--out', default='swarm_demo.mp4')
ap.add_argument('--fps', type=int, default=30)
ap.add_argument('--width', type=int, default=960)
ap.add_argument('--height', type=int, default=540)
a = ap.parse_args()

sim = SwarmSim(n=a.n, kind='grid', spacing=0.5)
ids = list(sim.drones)
home = {i: np.array(sim.pose(i).pos) for i in ids}
z = 0.5
# mission: takeoff -> stacked grid -> circle rotating 3 x 45 deg -> home -> land
plan = [({i: home[i] * [1, 1, 0] + [0, 0, z] for i in ids}, 3.0)]
grid = grid_formation(a.n, 0.45, z)
plan.append(({i: grid[k] + [0, 0, 0.15 * (k % 2)] for k, i in enumerate(ids)}, 3.0))
for k in range(4):
    plan.append((dict(zip(ids, circle_formation(a.n, 0.6, z=z, phase=k * math.pi / 4))), 3.0))
plan.append(({i: home[i] * [1, 1, 0] + [0, 0, z] for i in ids}, 3.0))
plan.append(({i: home[i] * [1, 1, 0] + [0, 0, -0.02] for i in ids}, 3.0))

segs, t0, prev = [], 0.0, {i: home[i].copy() for i in ids}
for targets, dur in plan:
    segs.append((t0, dur, dict(prev), targets))
    prev, t0 = targets, t0 + dur + 0.4
T_END = t0 + 0.5


def setpoint(i, t):
    for s0, d, p0, p1 in reversed(segs):
        if t >= s0:
            return min_jerk(p0[i], p1[i], d, t - s0)
    return home[i], np.zeros(3), np.zeros(3)


ctl = {i: PositionController() for i in ids}
r = mujoco.Renderer(sim.model, a.height, a.width)
cam = mujoco.MjvCamera()
cam.lookat[:] = [0, 0, 0.35]
cam.distance, cam.azimuth, cam.elevation = 1.9, -130.0, -22.0
w = iio.get_writer(a.out, fps=a.fps, codec='libx264', quality=8, macro_block_size=8)
every_render = int(round(1 / (a.fps * sim.dt)))
landed = False
for k in range(int(T_END / sim.dt)):
    if k % 20 == 0:
        t = sim.t
        pos = {i: np.array(sim.pose(i).pos) for i in ids}
        for i in ids:
            if t > segs[-1][0] + segs[-1][1]:
                sim.push(i, 'cmd/arm', P.CmdArm(False))
                continue
            p = sim.pose(i)
            sp, sv, sa = setpoint(i, t)
            q, duty = ctl[i].update(t, p.pos, p.vel, yaw_of(p.quat), sp, sv, sa, 0.0, sim.drones[i].battery_v, 0.02,
                                    separation_acc(i, pos, 0.25))
            sim.push(i, 'cmd/attitude', P.CmdAttitude(tuple(q), (0, 0, 0), duty))
    sim.step()
    if k % every_render == 0:
        cam.azimuth = -130.0 + 8.0 * sim.t
        r.update_scene(sim.data, camera=cam)
        w.append_data(r.render())
w.close()
print('written', a.out)
